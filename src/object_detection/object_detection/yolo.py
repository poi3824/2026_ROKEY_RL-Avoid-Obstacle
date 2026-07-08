########## YoloModel ##########
import os
import json
import math
import time
from collections import Counter

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import numpy as np


PACKAGE_NAME = "object_detection"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

YOLO_MODEL_FILENAME = "best_seg1.pt"
YOLO_CLASS_NAME_JSON = "class_name_tool.json"

YOLO_MODEL_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_MODEL_FILENAME)
YOLO_JSON_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_CLASS_NAME_JSON)

# 2026-07-07: seg 모델(best_seg.pt)은 이 장비(GPU 없음, CPU 추론)에서 1프레임에
# ~1초 걸린다(기존 detect 모델 대비 ~5배). 기존처럼 "1초 동안 모은 프레임 전부"를
# 배치 추론하면 프레임 수(카메라 fps에 비례)가 그대로 추론 시간이 되어 5초
# 타임아웃(GET_TARGET_TIMEOUT)을 훌쩍 넘긴다. 그래서 시간 기반이 아니라 개수
# 기반으로 캡을 걸어 배치 크기(=추론 시간)를 예측 가능하게 만든다.
FUSION_FRAME_COUNT = 3
GET_FRAMES_MAX_WAIT_SEC = 5.0  # 카메라가 멈춰있는 경우를 대비한 안전장치


class YoloModel:
    def __init__(self):
        self.model = YOLO(YOLO_MODEL_PATH)
        with open(YOLO_JSON_PATH, "r", encoding="utf-8") as file:
            class_dict = json.load(file)
            self.reversed_class_dict = {v: int(k) for k, v in class_dict.items()}

    def get_frames(self, img_node, count=FUSION_FRAME_COUNT, max_wait_sec=GET_FRAMES_MAX_WAIT_SEC):
        """count장을 채울 때까지 프레임을 모은다(카메라가 멈춰있으면 max_wait_sec에서 포기)."""
        end_time = time.time() + max_wait_sec
        frames = {}

        while time.time() < end_time and len(frames) < count:
            rclpy.spin_once(img_node)
            frame = img_node.get_color_frame()
            stamp = img_node.get_color_frame_stamp()
            if frame is not None:
                frames[stamp] = frame
            time.sleep(0.01)

        if not frames:
            print("No frames captured in %.2f seconds", max_wait_sec)

        print("%d frames captured", len(frames))
        return list(frames.values())

    def has_label(self, frame, target, confidence_threshold=0.6):
        """단일 프레임 1장만으로 target 라벨이 있는지 빠르게 확인한다 (True/False).

        2026-07-07: hand 안전 감지용. get_best_detection은 pick 신뢰도를 위해
        ~1초짜리 멀티프레임 융합을 쓰는데, 안전 감지는 그 정도 견고함보다
        반응 속도가 중요하고, pick이 쓰는 자원(락/서비스)과 경합하면 안 되므로
        여기서는 프레임 1장만 돌리는 훨씬 가벼운 경로를 따로 둔다.
        """
        if frame is None:
            return False
        label_id = self.reversed_class_dict[target]
        results = self.model([frame], verbose=False)
        detected = False
        for res in results:
            for score, label in zip(res.boxes.conf.tolist(), res.boxes.cls.tolist()):
                if int(label) != label_id:
                    continue
                # 2026-07-08: 파지 직후 hand_detected 오탐 진단용 임시 로그.
                # pick()이 물체를 쥐고 hover로 복귀할 때 닫힌 그리퍼(+쥔 물체)를
                # 모델이 hand로 오인식하는지 확인하려고 threshold 미만인 것도 찍는다.
                print(f"[has_label] target='{target}' confidence={score:.3f} (threshold={confidence_threshold})")
                if score >= confidence_threshold:
                    detected = True
        return detected

    def get_best_detection(self, img_node, target):
        """bbox/score에 더해, seg 모델이면 grasp용 짧은 변 각도(angle_deg)도 반환한다.

        각도는 물체가 세그멘테이션 안 되거나(아직 detect 전용 모델이거나 마스크가
        안 잡힌 경우) None을 반환하므로, 호출부에서 None -> 0.0(회전 없음)으로
        처리한다.
        """
        rclpy.spin_once(img_node)
        frames = self.get_frames(img_node)
        if not frames:  # Check if frames are empty
            return None, None, None

        results = self.model(frames, verbose=False)
        print("classes: ")
        print(results[0].names)
        detections = self._aggregate_detections(results)
        label_id = self.reversed_class_dict[target]
        print("label_id: ", label_id)
        print("detections: ", detections)

        matches = [d for d in detections if d["label"] == label_id]
        if not matches:
            print("No matches found for the target label.")
            return None, None, None
        best_det = max(matches, key=lambda x: x["score"])
        angle_deg = self._find_matching_mask_angle(results, label_id, best_det["box"])
        return best_det["box"], best_det["score"], angle_deg

    def _find_matching_mask_angle(self, results, label_id, box, iou_threshold=0.3):
        """box와 IoU가 가장 높은 마스크 하나를 골라 짧은 변(그립 대상 면) 각도를 반환한다.

        여러 프레임에 걸쳐 박스는 평균으로 fuse하지만, 각도는 사각형 대칭성(mod 180)
        때문에 단순 평균이 의미가 없어서 fuse하지 않고 best match 프레임 하나만 쓴다.
        """
        best_iou = iou_threshold
        best_angle = None
        for res in results:
            if res.masks is None:
                continue
            boxes = res.boxes.xyxy.tolist()
            labels = res.boxes.cls.tolist()
            polys = res.masks.xy
            for det_box, label, poly in zip(boxes, labels, polys):
                if int(label) != label_id:
                    continue
                iou = self._iou(box, det_box)
                if iou > best_iou:
                    best_iou = iou
                    best_angle = self._short_axis_angle_deg(poly)
        return best_angle

    def _short_axis_angle_deg(self, polygon_xy):
        """마스크 폴리곤(픽셀 좌표)에서 최소외접사각형의 짧은 변 방향 각도(0~180도)를 구한다.

        카메라가 물체를 수직으로 내려다보는 구도라 원근 왜곡이 거의 없으므로,
        픽셀상 짧은 변 = 실제(56mm) 짧은 변으로 봐도 된다. cv2.minAreaRect의 angle
        필드는 OpenCV 버전마다 관례가 달라 직접 신뢰하지 않고, boxPoints로 얻은
        네 꼭짓점에서 더 짧은 변의 방향 벡터를 계산한다. 그리퍼가 대칭(핑거 2개)이라
        180도 반대 방향은 같은 축이므로 mod 180으로 정규화한다.
        """
        pts = np.asarray(polygon_xy, dtype=np.float32)
        if pts.shape[0] < 3:
            return None
        rect = cv2.minAreaRect(pts)
        box_pts = cv2.boxPoints(rect)
        edge1 = box_pts[1] - box_pts[0]
        edge2 = box_pts[2] - box_pts[1]
        short_edge = edge1 if np.linalg.norm(edge1) < np.linalg.norm(edge2) else edge2
        return math.degrees(math.atan2(short_edge[1], short_edge[0])) % 180.0

    def _aggregate_detections(self, results, confidence_threshold=0.5, iou_threshold=0.5):
        """
        Fuse raw detection boxes across frames using IoU-based grouping
        and majority voting for robust final detections.
        """
        raw = []
        for res in results:
            for box, score, label in zip(
                res.boxes.xyxy.tolist(),
                res.boxes.conf.tolist(),
                res.boxes.cls.tolist(),
            ):
                if score >= confidence_threshold:
                    raw.append({"box": box, "score": score, "label": int(label)})

        final = []
        used = [False] * len(raw)

        for i, det in enumerate(raw):
            if used[i]:
                continue
            group = [det]
            used[i] = True
            for j, other in enumerate(raw):
                if not used[j] and other["label"] == det["label"]:
                    if self._iou(det["box"], other["box"]) >= iou_threshold:
                        group.append(other)
                        used[j] = True

            boxes = np.array([g["box"] for g in group])
            scores = np.array([g["score"] for g in group])
            labels = [g["label"] for g in group]

            final.append(
                {
                    "box": boxes.mean(axis=0).tolist(),
                    "score": float(scores.mean()),
                    "label": Counter(labels).most_common(1)[0][0],
                }
            )

        return final

    def _iou(self, box1, box2):
        """
        Compute Intersection over Union (IoU) between two boxes [x1, y1, x2, y2].
        """
        x1, y1 = max(box1[0], box2[0]), max(box1[1], box2[1])
        x2, y2 = min(box1[2], box2[2]), min(box1[3], box2[3])
        inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter
        return inter / union if union > 0 else 0.0

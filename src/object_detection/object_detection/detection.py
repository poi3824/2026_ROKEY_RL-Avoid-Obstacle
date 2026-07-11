import json
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from typing import Any, Callable, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition, SrvVisibilityCheck
from std_msgs.msg import Bool, String
from object_detection.realsense import ImgNode
from object_detection.yolo import YoloModel


PACKAGE_NAME = 'object_detection'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

DEPTH_ROI_HALF = 3  # (2*n+1)x(2*n+1) 정사각형 영역에서 median 계산

# 2026-07-07: robot_action_node의 응급/일시정지 로직이 참고하는 hand 안전 감지.
# get_3d_position 서비스(픽 본연의 멀티프레임 감지)와 자원을 공유하면 pick이
# 느려지므로, 완전히 별도인 타이머 + 단일 프레임 감지 + 토픽 발행으로 분리한다.
#
# 2026-07-08: 이전엔 "이 장비(CPU 추론)는 1프레임에 ~1초 걸린다"고 가정해서
# 1.5초까지 늘렸는데, 실측(순수 모델 추론만 벤치마크)해보니 프레임당 ~0.1초로
# 훨씬 빨랐다(그 1초 추정은 카메라 프레임 획득 등 다른 오버헤드가 섞여있었던
# 것으로 보임). 그래서 반응 속도를 다시 되찾기 위해 간격을 줄인다 — 0.1초
# 추론 + 여유를 감안해 0.3초.
#
# 2026-07-12: 위 "~0.1초"도 CPU 가정이었다 - 실제로는 이 장비에 GPU(RTX 4050)가
# 있고, ultralytics가 device 미지정이어도 추론 시점에 자동으로 CUDA를 쓰고
# 있었다(실측: self.model([frame])이 이미 cuda:0에서 38.8ms/frame로 돎 - 코드
# 변경 없이 원래부터 이 속도였다). 0.3초 주기에 38.8ms만 쓰고 260ms 넘게
# 노는 셈이라, hand 판정 반응 속도(=PAUSE 트리거 지연)와 HMI 오버레이 지연
# (vision_stream_node._draw_overlay 참고, 최신 detections를 최신 프레임에
# 얹는 구조라 이 주기만큼 지연이 생김)을 같이 줄이기 위해 0.1초로 재단축한다.
# HAND_BUFFER_SIZE(다수결 표본 개수)는 그대로 둔다 - 아래 흔들림 방지 로직은
# "표본 개수" 기반이라 체크 간격과 무관하게 노이즈 필터링 강도가 동일하고,
# 다수결이 완성되는 실제 시간만 1.5초->0.5초로 줄어 진짜 손이 들어왔을 때
# 더 빨리 멈춘다(안전성 저하가 아니라 반응성 개선).
HAND_CHECK_INTERVAL_SEC = 0.1
HAND_LABEL = "hand"
# YoloModel.has_label()의 기존 기본값(0.9)과 정확히 동일해야 한다 - hand_detected는
# safety_monitor_node가 ESTOP/PAUSE 판단에 직접 쓰는 안전 신호라, 이 리팩토링으로
# 판정 값이 단 하나도 달라지면 안 된다(아래 _extract_hand_detected 참고).
HAND_CONFIDENCE_THRESHOLD = 0.9

# 2026-07-11 (HMI 재구축 Phase 4): hmi/vision_detections에 publish할 때 쓰는
# 하한선. hand 판정(HAND_CONFIDENCE_THRESHOLD)과는 별개 - 화면 표시용이라 더
# 낮게 잡아도 되고, HMI 쪽(vision_bridge.py가 쓰던 0.6)에서 한 번 더 필터링한다.
DETECTIONS_PUBLISH_CONFIDENCE_THRESHOLD = 0.5

# 2026-07-08: threshold(0.6) 근처에서 confidence가 흔들려서(실측: 같은 손이
# 0.35~0.96 사이를 왔다갔다) 단일 프레임 판정만으로는 감지->해제->감지가
# 반복돼 실제로는 못 움직이는데 "완전 정지"처럼 보이는 문제가 있었다. 그래서
# 매 체크마다 찍는 프레임 수(1장)는 그대로 두고, 최근 HAND_BUFFER_SIZE 번의
# 단일 프레임 판정 이력을 다수결로 스무딩한다 - 표본 개수 기반이라 체크
# 간격과 무관하게 흔들림엔 여전히 강하다. 2026-07-12: HAND_CHECK_INTERVAL_SEC이
# 0.3->0.1초로 줄면서 이 5표 다수결이 완성되는 실제 시간도 1.5초->0.5초로
# 줄었다(표본 개수는 그대로라 노이즈 필터링 강도는 동일, 반응 시간만 빨라짐).
HAND_BUFFER_SIZE = 5


def _extract_hand_detected(detections, threshold=HAND_CONFIDENCE_THRESHOLD):
    """detect_frame()이 반환한 감지 목록에서 hand_detected 단일 프레임 판정을
    뽑아낸다. YoloModel.has_label(frame, HAND_LABEL, confidence_threshold=0.9)와
    의미상 동일해야 한다(라벨 일치 + score >= threshold인 감지가 하나라도
    있으면 True) - 순수 함수라 실제 모델/카메라 없이도 단위 테스트 가능하다.
    """
    return any(d["label"] == HAND_LABEL and d["score"] >= threshold for d in detections)


class ObjectDetectionNode(Node):
    def __init__(self, model_name = 'yolo'):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        # 2026-07-08: 손 감지 타이머와 get_3d_position 서비스가 하나의 싱글스레드
        # executor를 나눠 쓰던 게 문제였다 — 손 감지가 자주 돌 때 get_3d_position
        # 요청이 뒤에 밀려 GET_TARGET_TIMEOUT(motion_node 기준)을 넘겨 pick이
        # 계속 실패했다(실측: 6.0~6.02초 만에 정확히 타임아웃). 그래서 둘을
        # 별도 콜백그룹으로 나누고 MultiThreadedExecutor로 진짜 동시에 돌린다.
        self._depth_cbg = MutuallyExclusiveCallbackGroup()
        self._hand_cbg = MutuallyExclusiveCallbackGroup()
        # 2026-07-09: 스캔 스윕(motion_executor.sweep_to_detect) 중 반복 호출되는
        # 가벼운 단일 프레임 가시성 체크. get_3d_position(무거운 8프레임 융합)이나
        # hand 감지 타이머와 자원 경합하면 안 되므로 별도 콜백그룹으로 분리한다.
        self._visibility_cbg = MutuallyExclusiveCallbackGroup()
        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth,
            callback_group=self._depth_cbg,
        )
        self.create_service(
            SrvVisibilityCheck,
            'check_visibility',
            self.handle_check_visibility,
            callback_group=self._visibility_cbg,
        )
        self.hand_detected_pub = self.create_publisher(Bool, 'hand_detected', 10)
        # 2026-07-11 (HMI 재구축 Phase 4): hand 체크 타이머가 이미 0.3초마다 단일
        # 프레임 추론을 하고 있어서(예전엔 has_label()이 결과를 hand 여부 bool
        # 하나로 접어버리고 나머지는 버렸음), 그 결과를 hmi_ros_bridge가 재사용할
        # 수 있게 전체 detection도 같이 발행한다 - 추가 추론 없음(자원 사용량
        # 불변, 아래 _hand_check_timer_callback 참고). hmi_interface/
        # vision_bridge.py가 지금 자체 YOLO 모델을 또 로드하는 중복을 없애기
        # 위한 것 - 그 파일은 이 커밋에서 건드리지 않는다(다음 단계 작업).
        self.detections_pub = self.create_publisher(String, 'hmi/vision_detections', 10)
        self._hand_history = deque(maxlen=HAND_BUFFER_SIZE)
        self.create_timer(
            HAND_CHECK_INTERVAL_SEC, self._hand_check_timer_callback,
            callback_group=self._hand_cbg,
        )
        self.get_logger().info("ObjectDetectionNode initialized.")

    def _hand_check_timer_callback(self):
        """단일 프레임 1장으로 hand 여부를 확인하고, 최근 판정 이력의 다수결로
        스무딩해 /hand_detected로 발행한다.

        get_3d_position 서비스(픽용 멀티프레임 감지)와 완전히 분리된 경로라
        pick 흐름과 자원 경합이 없다. img_node는 여기서 직접 spin해서 프레임을
        최신 상태로 유지한다(그 전엔 실제 detect 요청이 있을 때만 spin됐음).

        다수결(HAND_BUFFER_SIZE 참고)은 매 체크마다 새로 여러 프레임을 찍는 게
        아니라, 지금까지처럼 1장씩 찍은 결과를 이력에 쌓아 판단만 스무딩한다 —
        체크 속도/자원 사용량은 그대로다. 이력이 아직 안 찼을 때(막 시작 시)도
        같은 식(과반)으로 계산되므로 별도 예외 처리가 필요 없다.

        2026-07-11 (HMI 재구축 Phase 4): 예전엔 여기서 self.model.has_label()을
        불러 hand 여부 bool 하나만 얻고 나머지 감지 결과는 버렸다. 이제
        self.model.detect_frame()으로 같은 추론 1회 호출에서 전체 감지 목록을
        받아, hand 판정은 _extract_hand_detected()로 예전과 동일하게(threshold
        0.9) 뽑아내고, 그 목록 전체를 hmi/vision_detections로도 발행한다 -
        추론 횟수/주기/hand_detected 판정 로직은 전혀 바뀌지 않았다.
        """
        try:
            with self.img_node.spin_lock:
                rclpy.spin_once(self.img_node, timeout_sec=0)
            frame = self.img_node.get_color_frame()
            detections = self.model.detect_frame(
                frame, confidence_threshold=DETECTIONS_PUBLISH_CONFIDENCE_THRESHOLD
            )
            detected_this_frame = _extract_hand_detected(detections)
            self._hand_history.append(detected_this_frame)
            detected = sum(self._hand_history) > len(self._hand_history) / 2
            self.hand_detected_pub.publish(Bool(data=detected))

            self.detections_pub.publish(String(data=json.dumps({
                "stamp": time.time(),
                "detections": detections,
            })))
        except Exception as e:
            self.get_logger().error(f"hand 체크 타이머 오류, 계속 진행함: {e}")

    def _load_model(self, name):
        """모델 이름에 따라 인스턴스를 반환합니다."""
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request, response):
        """클라이언트 요청을 처리해 3D 좌표와 grasp 각도를 반환합니다."""
        self.get_logger().info(f"Received request: {request}")
        x, y, z, angle_deg = self._compute_position(request.target)
        response.depth_position = [float(x), float(y), float(z)]
        response.angle_deg = float(angle_deg)
        return response

    def handle_check_visibility(self, request, response):
        """스캔 스윕 중 반복 호출되는 가벼운 단일 프레임 가시성 체크.

        get_3d_position(8프레임 융합)과 달리 프레임 1장만 돌려서 "물체가 잘리지
        않고 온전히 보이는지"만 빠르게 확인한다 — 실제 정밀 좌표/각도 계산은
        스윕이 멈춘 뒤 get_3d_position으로 따로 한다.
        """
        with self.img_node.spin_lock:
            rclpy.spin_once(self.img_node, timeout_sec=0)
        frame = self.img_node.get_color_frame()
        response.visible = self.model.is_fully_visible(frame, request.target)
        return response

    def _compute_position(self, target):
        """이미지를 처리해 객체의 카메라 좌표와 grasp용 짧은 변 각도(image-plane, deg)를 계산합니다.

        angle_deg는 seg 모델이 아니거나 마스크를 못 찾은 경우 0.0(회전 없음)이 된다.
        """
        with self.img_node.spin_lock:
            rclpy.spin_once(self.img_node)

        angle_deg = 0.0
        if target == "":
            color = self._wait_for_valid_data(self.img_node.get_color_frame, "color frame")
            h, w = color.shape[:2]
            cx, cy = w//2, h//2
        else:
            box, score, angle_deg, mask_center = self.model.get_best_detection(self.img_node, target)
            if box is None or score is None:
                self.get_logger().warn(f"No detection for '{target}'")
                return 0.0, 0.0, 0.0, 0.0
            self.get_logger().info(
                f"Detected: box={box}, score={score:.2f}, angle={angle_deg}, mask_center={mask_center}"
            )
            # 2026-07-08: depth 샘플링 지점은 bbox 중심보다 마스크 중심(무게중심)이
            # 낫다 — 손잡이가 있거나 일부만 보이는 물체는 bbox 중심이 실제 물체
            # 바깥(빈 공간)에 떨어질 수 있는데, 마스크 중심은 항상 물체 내부다.
            # 마스크 매칭에 실패하면(detect 전용 모델이거나 IoU 미달) bbox 중심으로
            # 폴백한다.
            if mask_center is not None:
                cx, cy = int(mask_center[0]), int(mask_center[1])
            else:
                cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
            if angle_deg is None:
                angle_deg = 0.0

        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return 0.0, 0.0, 0.0, 0.0

        x, y, z = self._pixel_to_camera_coords(cx, cy, cz)
        return x, y, z, angle_deg

    def _get_depth(self, x, y):
        """(x,y) 주변 ROI에서 유효한(0이 아닌) depth 값들 중 가장 가까운(min) 값을 반환합니다.

        ROI 안에 물체와 배경(바닥 등)이 같이 걸리면 median은 둘 중 어느 쪽이
        많이 잡히느냐에 따라 값이 튀므로, 항상 가장 가까운 표면(물체 쪽)을
        고르도록 min을 사용한다.
        """
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        h, w = frame.shape[:2]
        x0, x1 = max(0, x - DEPTH_ROI_HALF), min(w, x + DEPTH_ROI_HALF + 1)
        y0, y1 = max(0, y - DEPTH_ROI_HALF), min(h, y + DEPTH_ROI_HALF + 1)
        roi = frame[y0:y1, x0:x1]
        valid = roi[roi > 0]
        if valid.size == 0:
            self.get_logger().warn(f"No valid depth around ({x},{y}).")
            return None
        return float(np.min(valid))

    def _wait_for_valid_data(self, getter, description):
        """getter 함수가 유효한 데이터를 반환할 때까지 spin 하며 재시도합니다."""
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            with self.img_node.spin_lock:
                rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        """픽셀 좌표와 intrinsics를 이용해 카메라 좌표계로 변환합니다."""
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from typing import Any, Callable, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition
from std_msgs.msg import Bool
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
HAND_CHECK_INTERVAL_SEC = 0.3
HAND_LABEL = "hand"

# 2026-07-08: threshold(0.6) 근처에서 confidence가 흔들려서(실측: 같은 손이
# 0.35~0.96 사이를 왔다갔다) 단일 프레임 판정만으로는 감지->해제->감지가
# 반복돼 실제로는 못 움직이는데 "완전 정지"처럼 보이는 문제가 있었다. 그래서
# 매 체크마다 찍는 프레임 수(1장, ~1초)는 그대로 두고, 최근 HAND_BUFFER_SIZE
# 번의 단일 프레임 판정 이력을 다수결로 스무딩한다 — 체크 속도/자원 사용량은
# 그대로면서 흔들림에는 강해진다.
HAND_BUFFER_SIZE = 5


class ObjectDetectionNode(Node):
    def __init__(self, model_name = 'yolo'):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth
        )
        self.hand_detected_pub = self.create_publisher(Bool, 'hand_detected', 10)
        self._hand_history = deque(maxlen=HAND_BUFFER_SIZE)
        self.create_timer(HAND_CHECK_INTERVAL_SEC, self._hand_check_timer_callback)
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
        """
        try:
            rclpy.spin_once(self.img_node, timeout_sec=0)
            frame = self.img_node.get_color_frame()
            detected_this_frame = self.model.has_label(frame, HAND_LABEL)
            self._hand_history.append(detected_this_frame)
            detected = sum(self._hand_history) > len(self._hand_history) / 2
            self.hand_detected_pub.publish(Bool(data=detected))
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

    def _compute_position(self, target):
        """이미지를 처리해 객체의 카메라 좌표와 grasp용 짧은 변 각도(image-plane, deg)를 계산합니다.

        angle_deg는 seg 모델이 아니거나 마스크를 못 찾은 경우 0.0(회전 없음)이 된다.
        """
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
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

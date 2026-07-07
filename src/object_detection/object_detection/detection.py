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
# get_3d_position 서비스(픽 본연의 ~1초짜리 멀티프레임 감지)와 자원을 공유하면
# pick이 느려지므로, 완전히 별도인 타이머 + 단일 프레임 감지 + 토픽 발행으로 분리한다.
HAND_CHECK_INTERVAL_SEC = 0.2  # 5Hz
HAND_LABEL = "hand"


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
        self.create_timer(HAND_CHECK_INTERVAL_SEC, self._hand_check_timer_callback)
        self.get_logger().info("ObjectDetectionNode initialized.")

    def _hand_check_timer_callback(self):
        """단일 프레임 1장으로 hand 여부만 빠르게 확인해 /hand_detected로 발행한다.

        get_3d_position 서비스(픽용 멀티프레임 감지)와 완전히 분리된 경로라
        pick 흐름과 자원 경합이 없다. img_node는 여기서 직접 spin해서 프레임을
        최신 상태로 유지한다(그 전엔 실제 detect 요청이 있을 때만 spin됐음).
        """
        try:
            rclpy.spin_once(self.img_node, timeout_sec=0)
            frame = self.img_node.get_color_frame()
            detected = self.model.has_label(frame, HAND_LABEL)
            self.hand_detected_pub.publish(Bool(data=detected))
        except Exception as e:
            self.get_logger().error(f"hand 체크 타이머 오류, 계속 진행함: {e}")

    def _load_model(self, name):
        """모델 이름에 따라 인스턴스를 반환합니다."""
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request, response):
        """클라이언트 요청을 처리해 3D 좌표를 반환합니다."""
        self.get_logger().info(f"Received request: {request}")
        coords = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]
        return response

    def _compute_position(self, target):
        """이미지를 처리해 객체의 카메라 좌표를 계산합니다."""
        rclpy.spin_once(self.img_node)

        if target == "":
            color = self._wait_for_valid_data(self.img_node.get_color_frame, "color frame")
            h, w = color.shape[:2]
            cx, cy = w//2, h//2
        else:
            box, score = self.model.get_best_detection(self.img_node, target)
            if box is None or score is None:
                self.get_logger().warn(f"No detection for '{target}'")
                return 0.0, 0.0, 0.0
            self.get_logger().info(f"Detected: box={box}, score={score:.2f}")
            cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return 0.0, 0.0, 0.0

        return self._pixel_to_camera_coords(cx, cy, cz)

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

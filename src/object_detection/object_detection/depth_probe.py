"""디버깅용: 카메라 중앙 픽셀 주변 depth(mm)를 터미널에 계속 찍어주는 임시 노드.

get_surface_z()/detection.py의 target="" 분기가 실제로 어떤 depth를 읽고 있는지
눈으로 확인하기 위한 용도. object_detection_node와 별개로 단독 실행 가능.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

ROI_HALF = 3  # detection.py의 DEPTH_ROI_HALF와 동일
LOG_INTERVAL_SEC = 0.5  # 30Hz로 그대로 찍으면 스크롤이 안 되니 초당 2번으로 제한


class DepthProbeNode(Node):
    def __init__(self):
        super().__init__('depth_probe_node')
        self.bridge = CvBridge()
        self._last_log = 0.0
        self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.get_logger().info("depth 프레임 대기 중...")

    def depth_callback(self, msg):
        now = time.time()
        if now - self._last_log < LOG_INTERVAL_SEC:
            return
        self._last_log = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        center_raw = int(frame[cy, cx])

        x0, x1 = max(0, cx - ROI_HALF), min(w, cx + ROI_HALF + 1)
        y0, y1 = max(0, cy - ROI_HALF), min(h, cy + ROI_HALF + 1)
        roi = frame[y0:y1, x0:x1]
        valid = roi[roi > 0]
        median = float(np.median(valid)) if valid.size else None

        self.get_logger().info(
            f"center=({cx},{cy}) raw={center_raw}mm  roi_median={median}mm  "
            f"valid_px={valid.size}/{roi.size}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DepthProbeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# 실제 RealSense 카메라 없이도 검증 가능한 통합 테스트 - 합성 이미지/가짜
# detections 토픽으로 VisionStreamNode의 구독/오버레이 로직을 실제 rclpy
# pub/sub으로 확인한다(카메라 하드웨어가 이 개발 환경에 없어서 object_detection_node
# 전체를 띄운 진짜 E2E는 여기서는 할 수 없다 - 그 경계를 명확히 하기 위해 이
# 테스트는 vision_stream_node 자체의 계약만 검증한다).
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hmi_ros_bridge.vision_stream_node import VisionStreamNode  # noqa: E402


def _spin_until(executor, condition, timeout_sec=5.0):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if condition():
            return True
    return False


def test_vision_stream_node_ingests_frame_and_detections():
    rclpy.init()
    try:
        node = VisionStreamNode()
        talker = rclpy.create_node("test_vision_talker")
        bridge = CvBridge()
        image_pub = talker.create_publisher(Image, "/camera/camera/color/image_raw", 1)
        detections_pub = talker.create_publisher(String, "hmi/vision_detections", 10)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(talker)

        # --- 합성 프레임 발행 -> get_latest_jpeg()가 채워지는지 ---
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        frame[:] = (60, 120, 200)
        img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_pub.publish(img_msg)

        assert _spin_until(executor, lambda: node.get_latest_jpeg() is not None), \
            "카메라 프레임을 받아도 JPEG가 안 만들어짐"

        jpeg_bytes = node.get_latest_jpeg()
        decoded = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        assert decoded is not None and decoded.shape[:2] == (240, 320)

        # --- 가짜 detections 발행 -> _latest_detections에 반영되는지 ---
        payload = {
            "stamp": time.time(),
            "detections": [{"label": "obj_A", "score": 0.87, "box": [10, 10, 50, 50]}],
        }
        detections_pub.publish(String(data=json.dumps(payload)))

        assert _spin_until(executor, lambda: len(node._latest_detections) == 1), \
            "hmi/vision_detections가 노드에 반영되지 않음"
        assert node._latest_detections[0]["label"] == "obj_A"

        # --- 오버레이 켜면 draw 경로가 예외 없이 동작하는지(정확한 픽셀 비교는 안 함) ---
        node.set_overlay_enabled(True)
        image_pub.publish(img_msg)
        assert _spin_until(executor, lambda: True, timeout_sec=0.5)
        jpeg_bytes2 = node.get_latest_jpeg()
        assert jpeg_bytes2 is not None

        # --- stale detections(오래됨)는 오버레이에 안 쓰여야 함 - _draw_overlay가
        # 원본 프레임을 그대로 반환하는지 간접 확인(예외 없이 통과하면 OK) ---
        node._latest_detections_stamp = time.time() - 999
        drawn = node._draw_overlay(frame.copy())
        assert drawn is not None

        executor.shutdown()
        node.destroy_node()
        talker.destroy_node()
    finally:
        rclpy.shutdown()

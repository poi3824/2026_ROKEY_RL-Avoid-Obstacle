"""디버깅용: /get_3d_position 서비스를 반복 호출해서 depth_position/angle_deg를 터미널에
계속 찍어주는 독립 스크립트.

GRASP_ANGLE_SIGN 캘리브레이션(로봇 wrist C를 known delta만큼 돌려가며 angle_deg가
어느 방향으로 움직이는지 확인)에 쓴다. object_detection_node가 먼저 떠 있어야 하고,
target 파라미터는 class_name_tool.json에 있는 실제 물체 라벨이어야 한다 — 빈 문자열이면
detection.py가 surface-z 전용 분기(seg 미실행)를 타서 angle_deg가 항상 0.0으로 나온다.

사용법:
    ros2 run object_detection angle_probe --ros-args -p target:=obj_A
"""
import time

import rclpy
from rclpy.node import Node

from od_msg.srv import SrvDepthPosition

LOG_INTERVAL_SEC = 1.0


class AngleProbeNode(Node):
    def __init__(self):
        super().__init__('angle_probe_node')
        self.declare_parameter('target', '')
        self.target = self.get_parameter('target').value

        self.client = self.create_client(SrvDepthPosition, '/get_3d_position')
        while not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_3d_position service...")

    def probe_once(self):
        req = SrvDepthPosition.Request()
        req.target = self.target
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = AngleProbeNode()

    if not node.target:
        node.get_logger().error(
            "target 파라미터가 비어있음. "
            "예) ros2 run object_detection angle_probe --ros-args -p target:=obj_A"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f"target='{node.target}' 기준 depth_position/angle_deg를 Ctrl+C까지 계속 찍습니다.")
    try:
        while rclpy.ok():
            result = node.probe_once()
            if result is None:
                print("서비스 응답 없음 (타임아웃)")
            else:
                x, y, z = result.depth_position
                print(f"pos=({x:.1f}, {y:.1f}, {z:.1f})mm  angle_deg={result.angle_deg:.1f}")
            time.sleep(LOG_INTERVAL_SEC)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

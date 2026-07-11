# WorldMapNode._publish_task_status()가 실제 rclpy 그래프로 hmi/task_status/
# world_map을 올바른 형태로 발행하는지 검증한다.
#
# 2026-07-11 (HMI 재구축 Phase 5): handle_update() 전체(MoveLine/point cloud/TF
# 필요)를 실제로 돌리는 통합 테스트는 로봇/카메라 하드웨어 없이는 너무 무거워서
# (my_robot_pkg의 test_brain_node_task_status.py처럼 fake 서비스 전체를 새로
# 만들어야 함) 여기서는 WorldMapNode 생성자(ScanWorker 포함)가 실제로 블로킹
# 없이 뜨는지 + _publish_task_status()가 실제 토픽에 올바른 스키마로 발행하는지
# 만 검증한다. on_pose_progress 콜백 자체는 run_scan()의 for 루프 안에서 바로
# 호출되는 한 줄짜리 순수 위임이라 코드 리뷰로 충분하다고 판단했다.
import json
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pointcloud_perception.world_map_node import WorldMapNode  # noqa: E402


def _spin_until(executor, condition, timeout_sec=5.0):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if condition():
            return True
    return False


def test_world_map_node_constructs_without_blocking_and_publishes_task_status():
    rclpy.init()
    try:
        node = WorldMapNode()  # ScanWorker 생성 포함 - MoveLine 서비스 없이도 안 막혀야 함

        listener = rclpy.create_node("task_status_listener")
        received = []
        listener.create_subscription(
            String, "hmi/task_status/world_map",
            lambda msg: received.append(json.loads(msg.data)), 10,
        )

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(listener)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()

        time.sleep(0.3)  # discovery

        task_id = "test-task-1"
        node._publish_task_status(task_id, "RUNNING", title="월드맵 스캔", phase="starting")
        node._publish_task_status(
            task_id, "RUNNING", step_index=2, step_total=10,
            title="월드맵 스캔", detail="pose 2/10 (flat)", phase="scanning",
        )
        node._publish_task_status(
            task_id, "COMPLETED", title="월드맵 스캔 완료", detail="3개 장애물 감지", phase="done",
        )

        assert _spin_until(executor, lambda: len(received) >= 3), f"발행 못 받음: {received}"

        assert received[0]["status"] == "RUNNING"
        assert received[0]["mode"] == "world_map_scan"
        assert received[0]["task_id"] == task_id

        assert received[1]["step_index"] == 2
        assert received[1]["step_total"] == 10
        assert received[1]["progress"] == 0.2

        assert received[2]["status"] == "COMPLETED"
        assert received[2]["detail"] == "3개 장애물 감지"

        executor.shutdown()
        node.destroy_node()
        listener.destroy_node()
    finally:
        rclpy.shutdown()

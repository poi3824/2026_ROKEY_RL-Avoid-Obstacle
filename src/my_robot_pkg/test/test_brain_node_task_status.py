# 실제 rclpy 그래프(가짜 get_keyword/safety_reset/MoveTo/Pick/Place 서버)로
# brain_node.py가 hmi/task_status/manipulation을 실제로 발행하는지 검증한다.
#
# 2026-07-11 (HMI 재구축 Phase 5): brain_node.py에 task_status 발행을 추가했다 -
# BrainNode의 생성자가 get_keyword/motion 서버들을 blocking wait하므로, 이
# 테스트는 실제 서비스/액션 서버를 흉내 내는 fake 노드를 먼저 띄운 뒤에만
# BrainNode를 생성할 수 있다(그렇지 않으면 생성자에서 무한 대기).
#
# 범위: 성공 경로(1개 물체 pick+place+home) 하나만 검증한다 - 실패/ESTOP 경로는
# _safety_state를 ESTOP으로 바꿔야 하는 등 시나리오가 더 복잡해 이 커밋에서는
# 다루지 않는다(코드 리뷰로는 로직 확인했으나 실제 rclpy 그래프 테스트는 아직 없음).
import json
import os
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_interfaces.action import MoveTo, Pick, Place
from robot_interfaces.msg import SafetyState

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from my_robot_pkg.brain_node import BrainNode  # noqa: E402


def _safety_qos():
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE
    return qos


class _FakeSupportNode(Node):
    """brain_node가 생성자에서 기다리는 모든 외부 서비스/액션을 성공 응답으로 흉내 낸다."""

    def __init__(self):
        super().__init__("fake_support_node")
        self.create_service(Trigger, "get_keyword", self._on_trigger_ok)
        self.create_service(Trigger, "/safety/reset", self._on_trigger_ok)
        self.create_service(Trigger, "update_world_map", self._on_trigger_ok)

        self._moveto_server = ActionServer(self, MoveTo, "motion/move_to", self._exec_moveto)
        self._pick_server = ActionServer(self, Pick, "motion/pick", self._exec_pick)
        self._place_server = ActionServer(self, Place, "motion/place", self._exec_place)

        self.safety_pub = self.create_publisher(SafetyState, "/safety/state", _safety_qos())

    def _on_trigger_ok(self, request, response):
        response.success = True
        response.message = "ok"
        return response

    def publish_run_state(self):
        msg = SafetyState()
        msg.state = SafetyState.RUN
        msg.reason = ""
        self.safety_pub.publish(msg)

    def _exec_moveto(self, goal_handle):
        goal_handle.succeed()
        result = MoveTo.Result()
        result.success = True
        result.message = "ok"
        return result

    def _exec_pick(self, goal_handle):
        goal_handle.succeed()
        result = Pick.Result()
        result.success = True
        result.picked_pose = []
        result.message = "ok"
        return result

    def _exec_place(self, goal_handle):
        goal_handle.succeed()
        result = Place.Result()
        result.success = True
        result.message = "ok"
        return result


def test_execute_command_publishes_task_status_sequence():
    rclpy.init()
    try:
        fake = _FakeSupportNode()
        fake_executor = MultiThreadedExecutor(num_threads=4)
        fake_executor.add_node(fake)
        fake_thread = threading.Thread(target=fake_executor.spin, daemon=True)
        fake_thread.start()

        fake.publish_run_state()
        time.sleep(0.3)  # TRANSIENT_LOCAL 전파 시간

        listener = rclpy.create_node("task_status_listener")
        received = []
        listener.create_subscription(
            String, "hmi/task_status/manipulation",
            lambda msg: received.append(json.loads(msg.data)), 10,
        )
        listener_executor = SingleThreadedExecutor()
        listener_executor.add_node(listener)
        listener_thread = threading.Thread(target=listener_executor.spin, daemon=True)
        listener_thread.start()

        time.sleep(0.3)  # BrainNode 생성 전, listener가 discovery될 시간

        brain = BrainNode()  # 생성자에서 robot_init() -> MoveTo(home) 1회 발생(task_status 없음)

        deadline = time.time() + 10.0
        while time.time() < deadline:
            if brain._safety_state == SafetyState.RUN:
                break
            time.sleep(0.05)
        assert brain._safety_state == SafetyState.RUN, "safety_state가 brain_node에 반영되지 않음"

        brain.execute_command("obj_A / scan / target1 / home")

        deadline = time.time() + 10.0
        while time.time() < deadline and not any(m["status"] == "COMPLETED" for m in received):
            time.sleep(0.05)

        statuses = [m["status"] for m in received]
        assert "RUNNING" in statuses, f"RUNNING 이벤트 없음: {statuses}"
        assert statuses[-1] == "COMPLETED", f"마지막 이벤트가 COMPLETED가 아님: {statuses}"

        task_ids = {m["task_id"] for m in received}
        assert len(task_ids) == 1, f"한 명령 실행 중 task_id가 여러 개 나옴: {task_ids}"

        modes = {m["mode"] for m in received}
        assert modes == {"pick_place"}

        final = received[-1]
        assert final["step_index"] == final["step_total"] == 1
        assert final["progress"] == 1.0

        brain.destroy_node()
        listener.destroy_node()
        fake.destroy_node()
    finally:
        rclpy.shutdown()

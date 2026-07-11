# rclpy가 필요한 통합 테스트 - 실제 ROS 그래프(fake publisher/service 노드)를 띄워서
# BridgeNode가 진짜 토픽/서비스와 맞물려 동작하는지 확인한다. mock 없이 실제
# rclpy pub/sub, 실제 서비스 호출로 검증한다(이 세션 전체에서 지켜온 "실데이터로
# 검증" 원칙 - 다만 여기서는 로봇 하드웨어가 없으니 fake talker/service 노드가
# 그 역할을 한다).
import os
import sys
import time

import json

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool
from rcl_interfaces.msg import Log
from robot_interfaces.msg import SafetyState

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hmi_ros_bridge.bridge_node import BridgeNode  # noqa: E402
from hmi_ros_bridge.emit_channel import EmitChannel  # noqa: E402

_LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class _FakeManualRecordService(Node):
    def __init__(self):
        super().__init__("fake_manual_record_service")
        self.received = []
        self.create_service(SetBool, "/voice/manual_record", self._on_request)

    def _on_request(self, request, response):
        self.received.append(request.data)
        response.success = True
        response.message = "ok"
        return response


def _spin_until(executor, condition, timeout_sec=5.0):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if condition():
            return True
    return False


def test_full_bridge_node_behavior():
    rclpy.init()
    try:
        emit_channel = EmitChannel()
        bridge = BridgeNode(emit_channel)
        fake_service = _FakeManualRecordService()

        talker = rclpy.create_node("test_talker")
        state_pub = talker.create_publisher(String, "/voice/state", 10)
        level_pub = talker.create_publisher(Float32, "/voice/level", 10)
        rosout_pub = talker.create_publisher(Log, "/rosout", 50)
        safety_pub = talker.create_publisher(SafetyState, "/safety/state", _LATCHED_QOS)
        manip_task_pub = talker.create_publisher(String, "hmi/task_status/manipulation", _LATCHED_QOS)
        worldmap_task_pub = talker.create_publisher(String, "hmi/task_status/world_map", _LATCHED_QOS)

        executor = SingleThreadedExecutor()
        for n in (bridge, fake_service, talker):
            executor.add_node(n)

        # --- voice_status: /voice/state, /voice/level -> latest-value 상태 ---
        captured = {}

        def state_received():
            for name, payload in emit_channel.drain_dirty_states():
                captured[name] = payload
            return "voice_status" in captured and captured["voice_status"].get("state") == "recording"

        msg = String()
        msg.data = "recording"
        state_pub.publish(msg)
        lvl = Float32()
        lvl.data = 0.75
        level_pub.publish(lvl)
        assert _spin_until(executor, state_received), "voice_status가 latest-value 슬롯에 반영되지 않음"

        # --- voice_log: /rosout(get_keyword_node 필터) -> ordered 이벤트 ---
        log_events = []

        def log_received():
            log_events.extend(emit_channel.drain_events())
            return any(name == "voice_log" for name, _payload in log_events)

        log_msg = Log()
        log_msg.name = "get_keyword_node"
        log_msg.msg = "웨이크워드 감지"
        log_msg.level = 20  # INFO
        rosout_pub.publish(log_msg)

        other_log = Log()
        other_log.name = "some_other_node"
        other_log.msg = "무시되어야 함"
        other_log.level = 20
        rosout_pub.publish(other_log)

        assert _spin_until(executor, log_received), "voice_log가 이벤트 큐에 반영되지 않음"
        voice_logs = [p for name, p in log_events if name == "voice_log"]
        assert any(p["text"] == "웨이크워드 감지" for p in voice_logs)
        assert not any(p["text"] == "무시되어야 함" for p in voice_logs), \
            "get_keyword_node가 아닌 노드의 로그는 필터링되어야 함"

        # --- 서비스 discovery: fake_manual_record_service가 그래프에 보이는지 ---
        assert _spin_until(
            executor, lambda: bridge._manual_record_client.service_is_ready(),
        ), "/voice/manual_record fake 서비스가 discovery되지 않음"

        # --- command 처리: voice.start_record -> 실제 서비스 호출 ---
        ack = bridge.handle_command({"command_id": "cmd-1", "action": "voice.start_record"})
        assert ack["ok"] is True
        assert ack["task_id"] is None  # Phase 5 전까지는 nullable

        assert _spin_until(executor, lambda: len(fake_service.received) == 1), \
            "fake 서비스가 실제로 호출되지 않음"
        assert fake_service.received == [True]

        # --- command_id 중복 방지: 같은 command_id 재요청 시 서비스 재호출 안 함 ---
        ack2 = bridge.handle_command({"command_id": "cmd-1", "action": "voice.start_record"})
        assert ack2 == ack
        executor.spin_once(timeout_sec=0.2)
        assert len(fake_service.received) == 1, "중복 command_id인데 서비스가 다시 호출됨"

        # --- 알 수 없는 action ---
        ack3 = bridge.handle_command({"command_id": "cmd-2", "action": "unknown.thing"})
        assert ack3["ok"] is False

        # --- safety_status: /safety/state(무수정 토픽) -> latest-value 상태 ---
        safety_msg = SafetyState()
        safety_msg.state = SafetyState.ESTOP
        safety_msg.reason = "hand detected"
        safety_pub.publish(safety_msg)

        safety_captured = {}

        def safety_received():
            for name, payload in emit_channel.drain_dirty_states():
                if name == "safety_status":
                    safety_captured["payload"] = payload
            return "payload" in safety_captured

        assert _spin_until(executor, safety_received), "safety_status가 relay되지 않음"
        assert safety_captured["payload"]["state"] == "ESTOP"
        assert safety_captured["payload"]["reason"] == "hand detected"

        # --- task_status: manipulation/world_map 두 토픽이 서로 안 밀어내고
        # 둘 다 event_name="task_status"로 나가는지(source는 payload 안에서 구분) ---
        manip_msg = String(data=json.dumps({
            "task_id": "t-1", "mode": "pick_place", "status": "RUNNING",
            "phase": "", "title": "", "detail": "",
            "step_index": 0, "step_total": 1, "progress": 0.0, "timestamp": 0,
        }))
        world_msg = String(data=json.dumps({
            "task_id": "t-2", "mode": "world_map_scan", "status": "RUNNING",
            "phase": "", "title": "", "detail": "",
            "step_index": None, "step_total": None, "progress": None, "timestamp": 0,
        }))
        manip_task_pub.publish(manip_msg)
        worldmap_task_pub.publish(world_msg)

        task_events = []

        def task_status_received():
            task_events.extend(emit_channel.drain_dirty_states())
            sources = {p["source"] for n, p in task_events if n == "task_status"}
            return {"manipulation", "world_map"} <= sources

        assert _spin_until(executor, task_status_received), \
            f"task_status 두 소스가 모두 relay되지 않음: {task_events}"
        assert all(name == "task_status" for name, _p in task_events), \
            "task_status는 슬롯 키와 무관하게 항상 event_name='task_status'로 나가야 함"

        executor.shutdown()
        for n in (bridge, fake_service, talker):
            n.destroy_node()
    finally:
        rclpy.shutdown()

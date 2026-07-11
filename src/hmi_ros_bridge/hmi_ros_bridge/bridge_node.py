# /voice/state, /voice/level, /rosout(get_keyword_node 필터)을 구독하고
# /voice/manual_record 서비스를 호출하는 rclpy 노드.
#
# src/hmi_interface/hmi_interface/voice_bridge.py의 ROS 쪽 로직을 그대로 계승하되
# (같은 토픽/서비스, 같은 필터링 방식), 전송 계층만 raw `websockets` -> EmitChannel
# 경유 Flask-SocketIO로 바꿨다. hmi_interface의 voice_bridge.py는 원본 그대로
# 남겨두고(deprecated 처리 전까지 병행 운영), 여기서는 로직만 재사용한다.
#
# 콜백은 절대 emit_channel 밖으로 직접 소켓 I/O를 하지 않는다 - publish_state/
# publish_event로 큐에 넣기만 하고 바로 리턴한다(emit_channel.py 주석 참고).
import time

from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool

VOICE_STATE_TOPIC = "/voice/state"
VOICE_LEVEL_TOPIC = "/voice/level"
VOICE_MANUAL_RECORD_SERVICE = "/voice/manual_record"
ROSOUT_TOPIC = "/rosout"
LOG_SOURCE_NODE = "get_keyword_node"
_LOG_LEVEL_NAMES = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

# command_id 중복 방지 (Bridge 쪽 방어 계층 - Flask 쪽에도 별도로 있다).
COMMAND_DEDUP_TTL_SEC = 60.0


def _log_level_to_int(level):
    if isinstance(level, (bytes, bytearray)):
        return int.from_bytes(level, "little")
    return int(level)


class BridgeNode(Node):
    def __init__(self, emit_channel):
        super().__init__("hmi_ros_bridge")
        self._emit = emit_channel
        self._state = "idle"
        self._level = 0.0
        self._seen_commands = {}  # command_id -> (expire_at, ack_payload)

        self.create_subscription(String, VOICE_STATE_TOPIC, self._on_voice_state, 10)
        self.create_subscription(Float32, VOICE_LEVEL_TOPIC, self._on_voice_level, 10)
        self.create_subscription(Log, ROSOUT_TOPIC, self._on_rosout, 50)
        self._manual_record_client = self.create_client(SetBool, VOICE_MANUAL_RECORD_SERVICE)

        self.get_logger().info("hmi_ros_bridge 시작 - voice 구독/서비스 준비 완료")

    def _on_voice_state(self, msg):
        self._state = msg.data
        self._emit.publish_state("voice_status", {"state": self._state, "level": self._level})

    def _on_voice_level(self, msg):
        self._level = round(float(msg.data), 4)
        self._emit.publish_state("voice_status", {"state": self._state, "level": self._level})

    def _on_rosout(self, msg):
        if msg.name != LOG_SOURCE_NODE:
            return
        stamp = msg.stamp.sec + msg.stamp.nanosec / 1e9
        level_name = _LOG_LEVEL_NAMES.get(_log_level_to_int(msg.level), "INFO")
        self._emit.publish_event("voice_log", {"level": level_name, "text": msg.msg, "stamp": stamp})

    def _sweep_seen_commands(self, now):
        expired = [cid for cid, (expire_at, _ack) in self._seen_commands.items() if expire_at < now]
        for cid in expired:
            del self._seen_commands[cid]

    def handle_command(self, data):
        """command.schema.json 형태의 dict를 받아 command_ack.schema.json 형태의
        dict를 반환한다. 순수 함수에 가깝게 만들어(소켓 I/O 없음) 단위 테스트가
        실제 소켓 연결 없이도 이 메서드만 호출해서 검증할 수 있게 한다."""
        now = time.time()
        self._sweep_seen_commands(now)

        command_id = data.get("command_id")
        action = data.get("action")

        cached = self._seen_commands.get(command_id)
        if cached is not None:
            return cached[1]

        if action == "voice.start_record":
            ack = self._request_manual_record(command_id, True)
        elif action == "voice.stop_record":
            ack = self._request_manual_record(command_id, False)
        else:
            ack = {
                "command_id": command_id, "ok": False, "task_id": None,
                "error": f"알 수 없는 action: {action}", "timestamp": now,
            }

        self._seen_commands[command_id] = (now + COMMAND_DEDUP_TTL_SEC, ack)
        return ack

    def _request_manual_record(self, command_id, start):
        now = time.time()
        if not self._manual_record_client.service_is_ready():
            self.get_logger().warn(
                f"{VOICE_MANUAL_RECORD_SERVICE} 서비스 없음 - get_keyword_node가 안 떠 있는 듯"
            )
            return {
                "command_id": command_id, "ok": False, "task_id": None,
                "error": f"{VOICE_MANUAL_RECORD_SERVICE} 서비스를 사용할 수 없습니다",
                "timestamp": now,
            }

        req = SetBool.Request()
        req.data = start
        self._manual_record_client.call_async(req)  # fire-and-forget - 결과는 /voice/state로 확인됨
        return {"command_id": command_id, "ok": True, "task_id": None, "error": None, "timestamp": now}

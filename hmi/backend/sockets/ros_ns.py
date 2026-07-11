# hmi_ros_bridge(python-socketio Client) <-> Flask, "/ros" 네임스페이스.
#
# /safety/state, /hmi/task_status/* 까지 이 채널로 흐르므로 Bridge Token으로 인증한다.
# 원칙: 이 클래스는 순수 relay만 한다 - ROS 쪽에서 온 걸 브라우저로, 브라우저 명령을
# ROS 쪽으로 그대로 전달만 하고 성패를 스스로 판단하지 않는다. 유일한 예외가
# command_result 합성인데, 그것도 "동일 task_id의 terminal task_status만 정본"이라는
# 원칙을 지키기 위해 여기서 이미 relay하는 terminal task_status를 그대로 복사해
# 만드는 것뿐이다(별도 판단 로직 없음) - Bridge는 command_result를 아예 만들지 않는다.
import logging
import time

from flask import request
from flask_socketio import Namespace, emit
from jsonschema import ValidationError

from schema_registry import validate

logger = logging.getLogger("hmi.ros_ns")


class RosNamespace(Namespace):
    def __init__(self, namespace, state, bridge_token):
        super().__init__(namespace)
        self.state = state
        self.bridge_token = bridge_token

    def on_connect(self, auth):
        token = (auth or {}).get("token")
        if not self.bridge_token or token != self.bridge_token:
            logger.warning("hmi_ros_bridge connect 거부 - 토큰 불일치/누락")
            raise ConnectionRefusedError("invalid bridge token")
        self.state.set_bridge_connected(request.sid)
        logger.info("hmi_ros_bridge 연결됨 sid=%s", request.sid)
        emit("bridge_status", {"connected": True}, namespace="/", broadcast=True)

    def on_disconnect(self):
        self.state.clear_bridge(request.sid)
        logger.info("hmi_ros_bridge 연결 끊김 sid=%s", request.sid)
        emit("bridge_status", {"connected": False}, namespace="/", broadcast=True)

    def _relay(self, event, data, cache_key=None):
        """cache_key가 있으면 HmiState에 마지막 값으로 기록해둔다 - 새 브라우저
        탭이 붙었을 때(browser_ns.on_connect) 다음 갱신을 기다리지 않고 즉시
        재전송하기 위함이다(Socket.IO에는 ROS TRANSIENT_LOCAL 같은 재전송이
        없어서, 이게 없으면 늦게 연결한 탭은 다음 이벤트가 올 때까지 빈 상태로
        보인다 - 실기로 헤드리스 브라우저 스크린샷에서 확인한 문제)."""
        if cache_key is not None:
            self.state.record_last_known(cache_key, event, data)
        emit(event, data, namespace="/", broadcast=True)

    def on_voice_status(self, data):
        self._relay("voice_status", data, cache_key="voice_status")

    def on_voice_log(self, data):
        self._relay("voice_log", data)  # 로그 스트림은 "상태"가 아니라 캐시 안 함

    def on_rl_reach_progress(self, data):
        # move_via_rl() 한 에피소드의 스텝 스트림 - voice_log와 동일하게 "최신값"이
        # 아니라 누적해서 봐야 하는 데이터라 cache_key로 캐싱하지 않는다(새로 붙은
        # 탭은 진행 중이던 에피소드의 과거 스텝을 못 보고 다음 스텝부터 보게 됨 -
        # voice_log와 같은 한계, 허용 가능한 트레이드오프).
        self._relay("rl_reach_progress", data)

    def on_safety_status(self, data):
        try:
            validate("safety_status.schema.json", data)
        except ValidationError as e:
            logger.warning("safety_status 스키마 불일치(그래도 relay): %s", e.message)
        self._relay("safety_status", data, cache_key="safety_status")

    def on_task_status(self, data):
        """data는 task_status_event.schema.json 형태({source, status}) - Bridge가
        구독 토픽 기준으로 source를 이미 태깅해서 보낸다. manipulation/world_map
        각각 별도 cache_key로 저장해 서로 안 덮어쓴다(emit_channel.py의
        slot_key/event_name 분리와 동일한 이유)."""
        try:
            validate("task_status_event.schema.json", data)
        except ValidationError as e:
            logger.warning("task_status 스키마 불일치(그래도 relay): %s", e.message)
        source = (data or {}).get("source", "unknown")
        self._relay("task_status", data, cache_key=f"task_status:{source}")

        status = (data or {}).get("status") or {}
        if status.get("status") in ("COMPLETED", "FAILED"):
            entry = self.state.pop_pending_control(status.get("task_id"))
            if entry:
                emit("command_result", {
                    "command_id": entry["command_id"],
                    "task_id": status.get("task_id"),
                    "terminal_status": status["status"],
                    "error": None,
                    "timestamp": time.time(),
                }, namespace="/", room=entry["sid"])

    def on_command_ack(self, data):
        data = data or {}
        try:
            validate("command_ack.schema.json", data)
        except ValidationError as e:
            logger.warning("command_ack 스키마 불일치, 무시: %s", e.message)
            return

        command_id = data["command_id"]
        sid = self.state.pop_pending_ack(command_id)
        if sid is None:
            logger.warning("command_ack에 대응하는 pending 요청 없음: %s", command_id)
            return

        emit("command_ack", data, namespace="/", room=sid)

        task_id = data.get("task_id")
        if task_id:
            self.state.register_pending_control(task_id, command_id, sid)

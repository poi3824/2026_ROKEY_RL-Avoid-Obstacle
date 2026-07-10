# React(socket.io-client) <-> Flask, default 네임스페이스("/").
#
# 브라우저는 이 네임스페이스로만 붙는다 - "/ros"는 hmi_ros_bridge 전용(토큰 인증)이라
# 브라우저 쪽에는 아예 노출하지 않는다.
import logging
import time

from flask import request
from flask_socketio import Namespace, emit
from jsonschema import ValidationError

from schema_registry import validate

logger = logging.getLogger("hmi.browser_ns")


class BrowserNamespace(Namespace):
    def __init__(self, namespace, state):
        super().__init__(namespace)
        self.state = state

    def on_connect(self):
        logger.info("browser connected sid=%s", request.sid)
        emit("bridge_status", {"connected": self.state.bridge_connected})

    def on_disconnect(self):
        logger.info("browser disconnected sid=%s", request.sid)

    def on_command(self, data):
        data = data or {}
        try:
            validate("command.schema.json", data)
        except ValidationError as e:
            emit("command_ack", {
                "command_id": data.get("command_id") or "unknown",
                "ok": False,
                "task_id": None,
                "error": f"invalid command payload: {e.message}",
                "timestamp": time.time(),
            })
            return

        command_id = data["command_id"]

        if self.state.check_and_mark_duplicate(command_id):
            emit("command_ack", {
                "command_id": command_id, "ok": False, "task_id": None,
                "error": "duplicate command_id", "timestamp": time.time(),
            })
            return

        if not self.state.bridge_connected:
            emit("command_ack", {
                "command_id": command_id, "ok": False, "task_id": None,
                "error": "hmi_ros_bridge not connected", "timestamp": time.time(),
            })
            return

        self.state.register_pending_ack(command_id, request.sid)
        emit("command", data, namespace="/ros", room=self.state.bridge_sid)

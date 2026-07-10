# Phase 0 스모크 테스트 - 실제 hmi_ros_bridge/React 없이 Flask-SocketIO test_client
# 두 개(브라우저 역할 하나, bridge 역할 하나)로 전체 계약을 검증한다.
#
# 검증 대상: health API, /ros 토큰 인증(거부/승인), command -> command_ack
# targeted 전달, command_ack.task_id -> terminal task_status -> command_result
# 합성, command_id 중복 방지(Flask 쪽).
import os
import sys
import uuid

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import create_app  # noqa: E402
from config import Config  # noqa: E402


@pytest.fixture
def app_and_sio():
    app, socketio = create_app()
    app.config["TESTING"] = True
    return app, socketio


def test_health_endpoint(app_and_sio):
    app, _socketio = app_and_sio
    client = app.test_client()
    res = client.get("/api/health")
    assert res.status_code == 200
    body = res.get_json()
    assert body["status"] == "ok"
    assert body["bridge_connected"] is False


def test_ros_namespace_rejects_bad_token(app_and_sio):
    app, socketio = app_and_sio
    bridge = socketio.test_client(app, namespace="/ros", auth={"token": "wrong-token"})
    assert bridge.is_connected("/ros") is False


def test_full_command_roundtrip(app_and_sio):
    app, socketio = app_and_sio

    browser = socketio.test_client(app, namespace="/")
    bridge = socketio.test_client(app, namespace="/ros", auth={"token": Config.BRIDGE_TOKEN})
    assert bridge.is_connected("/ros") is True

    # bridge 연결 시 browser에 bridge_status broadcast
    browser_events = browser.get_received("/")
    assert any(e["name"] == "bridge_status" and e["args"][0]["connected"] for e in browser_events)

    command_id = str(uuid.uuid4())
    task_id = "task-" + command_id[:8]
    browser.emit("command", {"command_id": command_id, "action": "voice.start_record"}, namespace="/")

    bridge_events = bridge.get_received("/ros")
    relayed = [e for e in bridge_events if e["name"] == "command"]
    assert len(relayed) == 1
    assert relayed[0]["args"][0]["command_id"] == command_id

    # bridge가 접수 확인(ack) + task_id 발급
    bridge.emit("command_ack", {
        "command_id": command_id, "ok": True, "task_id": task_id, "timestamp": 0,
    }, namespace="/ros")

    browser_events = browser.get_received("/")
    acks = [e for e in browser_events if e["name"] == "command_ack"]
    assert len(acks) == 1
    assert acks[0]["args"][0]["task_id"] == task_id

    # bridge가 terminal task_status를 relay -> Flask가 command_result를 합성해야 함
    bridge.emit("task_status", {
        "source": "manipulation",
        "status": {
            "task_id": task_id, "mode": "pick_place", "status": "COMPLETED",
            "timestamp": 1,
        },
    }, namespace="/ros")

    browser_events = browser.get_received("/")
    results = [e for e in browser_events if e["name"] == "command_result"]
    assert len(results) == 1
    assert results[0]["args"][0]["terminal_status"] == "COMPLETED"
    assert results[0]["args"][0]["command_id"] == command_id


def test_duplicate_command_id_rejected(app_and_sio):
    app, socketio = app_and_sio
    browser = socketio.test_client(app, namespace="/")
    bridge = socketio.test_client(app, namespace="/ros", auth={"token": Config.BRIDGE_TOKEN})
    assert bridge.is_connected("/ros") is True

    command_id = str(uuid.uuid4())
    browser.emit("command", {"command_id": command_id, "action": "voice.start_record"}, namespace="/")
    browser.emit("command", {"command_id": command_id, "action": "voice.start_record"}, namespace="/")

    events = browser.get_received("/")
    acks = [e for e in events if e["name"] == "command_ack"]
    assert len(acks) == 1
    assert acks[0]["args"][0]["ok"] is False
    assert "duplicate" in acks[0]["args"][0]["error"]

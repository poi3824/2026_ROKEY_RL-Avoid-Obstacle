# Phase 2 검증 - 이 개발 머신에 실제로 존재하는 데이터(~/.ros/my_robot_pkg/pick_log.db,
# data/world_maps/world_map_update_*)에 대해 REST API가 실제로 동작하는지 확인한다.
# voice_log.db는 이 머신에 없으므로(get_keyword_node를 여기서 띄운 적 없음) "빈 리스트"
# 경로를 검증한다 - 이것도 실제 운영에서 흔한 정상 케이스(HMI가 로봇 노드 없이도 떠야 함).
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import create_app  # noqa: E402
from readers import pick_log_reader, voice_log_reader, worldmap_reader  # noqa: E402


def _client():
    app, _socketio = create_app()
    app.config["TESTING"] = True
    return app.test_client()


def test_db_summary_real_data():
    assert pick_log_reader.db_exists(), "이 머신에 실제 pick_log.db가 있어야 하는 테스트"
    client = _client()
    res = client.get("/api/db/summary")
    assert res.status_code == 200
    body = res.get_json()
    assert "total" in body and "success" in body and "success_rate" in body


def test_db_pick_attempts_real_data():
    client = _client()
    res = client.get("/api/db/pick_attempts?limit=5")
    assert res.status_code == 200
    rows = res.get_json()["rows"]
    assert isinstance(rows, list)
    if rows:
        assert set(rows[0].keys()) >= {"id", "ts", "obj_label", "success"}


def test_db_voice_events_missing_db_returns_empty():
    assert not voice_log_reader.db_exists()
    client = _client()
    res = client.get("/api/db/voice_events")
    assert res.status_code == 200
    assert res.get_json()["rows"] == []


def test_worldmap_scans_real_data():
    scan_ids = worldmap_reader.list_scan_ids()
    assert len(scan_ids) > 0, "이 머신에 실제 world_map_update_* 스캔이 있어야 하는 테스트"

    client = _client()
    res = client.get("/api/db/worldmap_scans?limit=5")
    assert res.status_code == 200
    rows = res.get_json()["rows"]
    assert len(rows) > 0
    assert rows[0]["scan_id"] == scan_ids[0]


def test_worldmap_api_list_and_latest():
    client = _client()
    res = client.get("/api/worldmap/list")
    assert res.status_code == 200
    scan_ids = res.get_json()["scan_ids"]
    assert len(scan_ids) > 0

    res = client.get("/api/worldmap/latest")
    assert res.status_code == 200
    assert res.get_json()["scan_id"] == scan_ids[0]


def test_worldmap_api_obstacles_for_latest_scan_with_summary():
    scan_ids = worldmap_reader.list_scan_ids()
    scan_with_summary = next(
        (s for s in scan_ids if os.path.exists(
            os.path.join(worldmap_reader.RECORD_DIR, s, "world_map_summary.json")
        )),
        None,
    )
    if scan_with_summary is None:
        return  # 이 머신의 스캔들 중 summary가 있는 게 하나도 없으면 스킵

    client = _client()
    res = client.get(f"/api/worldmap/{scan_with_summary}/obstacles")
    assert res.status_code == 200
    body = res.get_json()
    assert body["scan_id"] == scan_with_summary
    assert isinstance(body["obstacles"], list)


def test_worldmap_api_rejects_path_traversal():
    client = _client()
    res = client.get("/api/worldmap/..%2F..%2F..%2Fetc%2Fpasswd/obstacles")
    assert res.status_code in (400, 404)


def test_robot_control_returns_501_not_200():
    client = _client()
    res = client.post("/api/robot_control", json={"cmd": "noop"})
    assert res.status_code == 501

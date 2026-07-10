# World Map 3D 뷰어용 API - hmi_bridge/app.py의 /api/worldmap/* 그대로 이관.
# Phase 6에서 React RobotViewer(@react-three/fiber)가 이 API를 직접 붙인다.
from flask import Blueprint, jsonify

from readers import worldmap_reader as wm

worldmap_bp = Blueprint("worldmap", __name__)


@worldmap_bp.route("/api/worldmap/list")
def api_list():
    return jsonify({"scan_ids": wm.list_scan_ids()})


@worldmap_bp.route("/api/worldmap/latest")
def api_latest():
    scan_id = wm.latest_scan_id()
    if scan_id is None:
        return jsonify({"error": "no scans found under RECORD_DIR"}), 404
    return jsonify({"scan_id": scan_id})


@worldmap_bp.route("/api/worldmap/<scan_id>/points")
def api_points(scan_id):
    try:
        points = wm.load_points(scan_id)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400
    except FileNotFoundError as e:
        return jsonify({"error": str(e)}), 404
    return jsonify({
        "scan_id": scan_id,
        "frame_id": "base_link",
        "num_points": int(points.shape[0]),
        "points": points.tolist(),
    })


@worldmap_bp.route("/api/worldmap/<scan_id>/obstacles")
def api_obstacles(scan_id):
    try:
        obstacles = wm.load_obstacles(scan_id)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400
    except FileNotFoundError as e:
        return jsonify({"error": str(e)}), 404
    return jsonify({
        "scan_id": scan_id,
        "frame_id": "base_link",
        "obstacles": obstacles,
    })

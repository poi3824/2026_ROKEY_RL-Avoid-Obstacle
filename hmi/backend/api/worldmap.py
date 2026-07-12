# World Map 3D 뷰어용 API - hmi_bridge/app.py의 /api/worldmap/* 그대로 이관.
# Phase 6에서 React RobotViewer(@react-three/fiber)가 이 API를 직접 붙인다.
#
# 2026-07-12: ICP 적용/미적용, Hough 분리/DBSCAN만 비교를 3D로 보기 위해
# points/obstacles에 ?variant= 쿼리 파라미터를 추가(worldmap_reader.load_points/
# load_obstacles 참고). 기존 호출부(쿼리 없음)는 이전과 동일하게 동작한다.
from flask import Blueprint, jsonify, request

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


@worldmap_bp.route("/api/worldmap/<scan_id>/variants")
def api_variants(scan_id):
    return jsonify({"scan_id": scan_id, **wm.get_variants(scan_id)})


@worldmap_bp.route("/api/worldmap/<scan_id>/points")
def api_points(scan_id):
    variant = request.args.get("variant", "raw")
    try:
        points = wm.load_points(scan_id, variant=variant)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400
    except FileNotFoundError as e:
        return jsonify({"error": str(e)}), 404
    return jsonify({
        "scan_id": scan_id,
        "frame_id": "base_link",
        "variant": variant,
        "num_points": int(points.shape[0]),
        "points": points.tolist(),
    })


@worldmap_bp.route("/api/worldmap/<scan_id>/obstacles")
def api_obstacles(scan_id):
    variant = request.args.get("variant", "hough")
    try:
        obstacles = wm.load_obstacles(scan_id, variant=variant)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400
    except FileNotFoundError as e:
        return jsonify({"error": str(e)}), 404
    return jsonify({
        "scan_id": scan_id,
        "frame_id": "base_link",
        "variant": variant,
        "obstacles": obstacles,
    })

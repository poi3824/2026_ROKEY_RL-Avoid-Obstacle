# DB(SQLite) 읽기 전용 API - hmi_interface/app.py의 /api/db/* 그대로 이관.
from flask import Blueprint, jsonify, request

from readers import pick_log_reader as pl
from readers import voice_log_reader as vl
from readers import worldmap_reader as wm

db_bp = Blueprint("db", __name__)


@db_bp.route("/api/db/summary")
def api_db_summary():
    return jsonify(pl.fetch_summary())


@db_bp.route("/api/db/pick_attempts")
def api_db_pick_attempts():
    limit = request.args.get("limit", default=50, type=int)
    return jsonify({"rows": pl.fetch_recent_attempts(limit=limit)})


@db_bp.route("/api/db/voice_events")
def api_db_voice_events():
    limit = request.args.get("limit", default=50, type=int)
    return jsonify({"rows": vl.fetch_recent_events(limit=limit)})


@db_bp.route("/api/db/worldmap_scans")
def api_db_worldmap_scans():
    limit = request.args.get("limit", default=30, type=int)
    return jsonify({"rows": wm.fetch_recent_scans(limit=limit)})

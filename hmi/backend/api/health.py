import time

from flask import Blueprint, current_app, jsonify

health_bp = Blueprint("health", __name__)


@health_bp.route("/api/health")
def get_health():
    ros_state = current_app.config["ROS_NAMESPACE_STATE"]
    return jsonify({
        "status": "ok",
        "timestamp": time.time(),
        "bridge_connected": ros_state.bridge_connected,
    })

# /api/robot_control - 합의된 원칙: hmi_ros_bridge가 실제로 연결되기 전까지는
# 정상 동작처럼 보이는 200 응답을 반환하지 않는다(과거 hmi_interface/app.py의
# stub은 200 + "not_implemented" 메시지를 줬는데, 이러면 프론트가 상태 코드만
# 보고 성공으로 오인하기 쉽다) - 명시적으로 501을 반환한다.
from flask import Blueprint, jsonify, request

robot_control_bp = Blueprint("robot_control", __name__)


@robot_control_bp.route("/api/robot_control", methods=["POST"])
def api_robot_control():
    data = request.get_json(silent=True) or {}
    return jsonify({
        "status": "not_implemented",
        "message": "hmi_ros_bridge가 아직 연결되지 않았습니다 (Phase 3+ 예정).",
        "received": data,
    }), 501

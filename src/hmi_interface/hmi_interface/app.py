# 통합 HMI Flask 백엔드 (구상 단계).
#
# hmi_bridge(월드맵 3D 뷰어, read-only 파일 뷰어)와는 별개 패키지다 - 서로
# 건드리지 않는다. 이 앱은 STT/TTS·Vision·RL·파라미터·DB·로봇 제어 설정을
# 탭 하나에 모으는 통합 관제 화면 자리를 잡는 역할이고, 지금 실제로 연결된
# 데이터는 pick_logger(SQLite)뿐이다 - 나머지 탭은 백엔드가 아직 없어서
# 화면에 "추후 연동" 표시만 해둔다.
#
# hmi_bridge와 같은 원칙을 따른다: 이 Flask 앱은 로봇을 직접 제어하지 않는다.
# /api/robot_control은 자리만 잡아둔 stub이고, 실제로 뭔가 시키려면 brain_node
# 같은 액션 클라이언트를 가진 rclpy 노드를 별도로 만들어 조합해야 한다(hmi_bridge/
# app.py의 같은 주석 참고).
import os

from flask import Flask, jsonify, render_template, request

from hmi_interface import pick_log_reader as pl


def create_app():
    share_dir = _share_dir_or_source_dir()
    app = Flask(
        __name__,
        template_folder=os.path.join(share_dir, "templates"),
        static_folder=os.path.join(share_dir, "static"),
    )

    @app.route("/")
    def index():
        return render_template("index.html")

    # ---- DB (SQLite) 탭 - pick_logger.py가 쓰는 DB를 읽기 전용으로 조회 ----
    @app.route("/api/db/summary")
    def api_db_summary():
        return jsonify(pl.fetch_summary())

    @app.route("/api/db/pick_attempts")
    def api_db_pick_attempts():
        limit = request.args.get("limit", default=50, type=int)
        return jsonify({"rows": pl.fetch_recent_attempts(limit=limit)})

    # ---- 로봇 제어 설정 탭 - 아직 stub. TODO 참고 ----
    @app.route("/api/robot_control", methods=["POST"])
    def api_robot_control():
        data = request.get_json(silent=True) or {}
        # TODO: 실제로 로봇에 뭔가 시키려면 이 Flask 프로세스가 아니라 별도
        # rclpy 노드(액션 클라이언트)를 만들어 조합해야 한다 - 이 앱이 dsr_node를
        # 직접 붙잡으면 motion_node의 dsr_lock 직렬화를 우회하게 돼 위험하다.
        app.logger.info(f"[stub] robot_control 요청 수신(미구현): {data}")
        return jsonify({"status": "not_implemented", "message": "아직 로봇 제어와 연결되지 않았습니다."})

    return app


def _share_dir_or_source_dir():
    """설치된 share 디렉토리를 우선 쓰되, 아직 colcon build 전이면(개발 중)
    소스 트리의 templates/static을 바로 쓴다 - 매번 빌드 안 해도 index.html
    수정이 바로 보이게 하기 위함."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("hmi_interface")
    except Exception:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    app = create_app()
    app.run(host="0.0.0.0", port=5050, debug=True)


if __name__ == "__main__":
    main()

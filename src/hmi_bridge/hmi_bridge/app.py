# World Map / Obstacle 3D viewer 모듈의 Flask 백엔드.
#
# 지금은 로봇/ROS 그래프와 분리된 read-only 뷰어다 - 저장된
# data/world_maps/world_map_update_*/ 스캔 결과 파일만 읽어서 보여준다. 그래서
# world_map_node가 실행 중이 아니어도(지난 스캔 결과를 보는 용도로) 그냥
# `ros2 run hmi_bridge hmi_bridge_server`로 띄울 수 있다.
#
# 나중에 라이브 진행 상황(/world_map/progress)이나 명령 전달(/command/dispatch)이
# 필요해지면 rclpy 노드를 별도로 만들어 이 Flask 앱과 조합하는 방향으로 확장한다 -
# Flask가 로봇을 직접 제어하지 않는다는 원칙은 유지한다.
import os

from ament_index_python.packages import get_package_share_directory
from flask import Flask, jsonify, render_template

from hmi_bridge import worldmap_loader as wm


def create_app():
    share_dir = get_package_share_directory("hmi_bridge")
    app = Flask(
        __name__,
        template_folder=os.path.join(share_dir, "templates"),
        static_folder=os.path.join(share_dir, "static"),
    )

    @app.route("/")
    def index():
        return render_template("world_map.html")

    @app.route("/api/worldmap/list")
    def api_list():
        return jsonify({"scan_ids": wm.list_scan_ids()})

    @app.route("/api/worldmap/latest")
    def api_latest():
        scan_id = wm.latest_scan_id()
        if scan_id is None:
            return jsonify({"error": "no scans found under RECORD_DIR"}), 404
        return jsonify({"scan_id": scan_id})

    @app.route("/api/worldmap/<scan_id>/points")
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

    @app.route("/api/worldmap/<scan_id>/obstacles")
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

    return app


def main():
    app = create_app()
    app.run(host="0.0.0.0", port=5000, debug=False)


if __name__ == "__main__":
    main()

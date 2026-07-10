# Flask 앱 팩토리 - REST API + Flask-SocketIO. ROS는 절대 직접 붙잡지 않는다
# (hmi_bridge/hmi_interface와 동일 원칙) - ROS와의 유일한 접점은 hmi_ros_bridge가
# "/ros" 네임스페이스에 python-socketio Client로 접속하는 것뿐이다.
import logging

from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO

from api.db import db_bp
from api.health import health_bp
from api.robot_control import robot_control_bp
from api.worldmap import worldmap_bp
from config import Config
from sockets.browser_ns import BrowserNamespace
from sockets.ros_ns import RosNamespace
from sockets.state import HmiState

logging.basicConfig(level=logging.INFO)


def create_app():
    app = Flask(__name__)
    app.config.from_object(Config)
    # 개발 단계에서 React(Vite, :5173)와 Flask(:5100)가 서로 다른 origin이라 필요.
    # 프로덕션에서는 Flask가 React 빌드 결과물을 같은 origin으로 서빙하므로 불필요해진다.
    CORS(app, resources={r"/api/*": {"origins": "*"}})

    state = HmiState(
        dedup_ttl_sec=Config.COMMAND_DEDUP_TTL_SEC,
        pending_ttl_sec=Config.PENDING_CONTROL_TTL_SEC,
    )
    app.config["ROS_NAMESPACE_STATE"] = state

    app.register_blueprint(health_bp)
    app.register_blueprint(db_bp)
    app.register_blueprint(worldmap_bp)
    app.register_blueprint(robot_control_bp)

    # async_mode="threading": 이 프로세스는 rclpy를 안 쓰지만, eventlet의
    # monkey-patch가 sqlite3 리더 모듈 등과 얽히는 걸 피하려고 가장 단순한
    # threading 모드를 쓴다 - 로컬 단일 프로세스 배포라 성능상 문제 없다.
    socketio = SocketIO(app, async_mode="threading", cors_allowed_origins="*")
    socketio.on_namespace(BrowserNamespace("/", state))
    socketio.on_namespace(RosNamespace("/ros", state, bridge_token=Config.BRIDGE_TOKEN))

    return app, socketio

# 실행 진입점: ros2 run hmi_ros_bridge hmi_ros_bridge_server
#
# rclpy.spin()은 반드시 메인 스레드에서 돈다 - 이 저장소의 hmi_interface/
# voice_bridge.py, vision_bridge.py가 이미 실기로 겪은 버그(SIGINT 처리가 메인
# 스레드의 spin을 전제로 함, 아니면 "terminate called without an active exception"으로
# 죽음)와 동일한 함정이라 같은 패턴을 그대로 따른다.
import logging
import os

import rclpy
from dotenv import load_dotenv

from hmi_ros_bridge.bridge_node import BridgeNode
from hmi_ros_bridge.emit_channel import EmitChannel
from hmi_ros_bridge.socketio_worker import SocketioWorker

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("hmi_ros_bridge.main")

# hmi/backend/.env를 공유 설정으로 그대로 읽는다 - HMI_BRIDGE_TOKEN은 백엔드와
# 반드시 같은 값이어야 하므로, 별도 .env 파일 두 개를 사람이 수동으로 맞추게
# 하지 않는다. 이미 셸에 export된 값이 있으면 그게 우선한다(override=False 기본값).
#
# __file__ 기준 상대경로는 못 쓴다 - `ros2 run`은 소스 트리가 아니라 colcon이
# install/hmi_ros_bridge/lib/python3.10/site-packages/ 밑에 복사해둔 사본을
# 실행하므로 디렉토리 깊이가 소스 트리와 다르다(실기로 겪음: 이 경로 계산이
# 틀려서 .env를 못 찾아 토큰이 빈 채로 시작되고, hmi/backend가 연결을 조용히
# 거부하는 문제가 있었다). 대신 HMI_ROS_BRIDGE_ENV_FILE로 명시 지정하거나, 이
# 저장소가 항상 ~/RL-Avoid-Obstacle에 있다는 이 프로젝트의 기존 관행(예:
# pointcloud_perception.world_map_algo.RECORD_DIR도 홈 기준 절대경로를 그대로
# 하드코딩)을 그대로 따른다.
_DEFAULT_BACKEND_ENV_PATH = os.path.expanduser("~/RL-Avoid-Obstacle/hmi/backend/.env")
_BACKEND_ENV_PATH = os.environ.get("HMI_ROS_BRIDGE_ENV_FILE", _DEFAULT_BACKEND_ENV_PATH)
if os.path.exists(_BACKEND_ENV_PATH):
    load_dotenv(_BACKEND_ENV_PATH)
    logger.info(f".env 로드: {_BACKEND_ENV_PATH}")
else:
    logger.warning(
        f"{_BACKEND_ENV_PATH}를 못 찾음 - 이미 export된 환경변수만 사용 "
        f"(HMI_ROS_BRIDGE_ENV_FILE로 다른 위치를 지정할 수 있음)"
    )

DEFAULT_BACKEND_URL = "http://localhost:5100"


def main():
    backend_host = os.environ.get("HMI_BACKEND_HOST", "localhost")
    backend_host = "localhost" if backend_host == "0.0.0.0" else backend_host
    backend_port = os.environ.get("HMI_BACKEND_PORT", "5100")
    backend_url = os.environ.get(
        "HMI_BACKEND_SOCKET_URL", f"http://{backend_host}:{backend_port}"
    )
    token = os.environ.get("HMI_BRIDGE_TOKEN", "")
    if not token:
        logger.warning("HMI_BRIDGE_TOKEN이 비어 있음 - hmi/backend가 연결을 거부할 것")

    rclpy.init()
    emit_channel = EmitChannel()
    node = BridgeNode(emit_channel)

    worker = SocketioWorker(
        url=backend_url, token=token, emit_channel=emit_channel,
        command_handler=node.handle_command,
    )
    worker.start()
    node.get_logger().info(f"hmi/backend 연결 대상: {backend_url}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        worker.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

# /voice/state, /voice/level(voice_interface.robot_get_keyword_node가 발행)을
# 구독해서 그대로 websocket으로 흘려보내는 작은 rclpy 노드.
#
# hmi_interface의 Flask 앱(app.py)은 ROS를 직접 붙잡지 않는다는 원칙을 그대로
# 지킨다 - 이 브릿지가 그 원칙의 "필요해지면 별도 rclpy 노드를 만들어 조합한다"
# 부분이다. 브라우저는 Flask가 서빙한 페이지에서 이 노드가 여는 websocket
# (기본 8765 포트)에 직접 붙어서 값을 받는다 - Flask 프로세스는 이 데이터
# 경로에 아예 관여하지 않는다.
import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

import websockets

VOICE_STATE_TOPIC = "/voice/state"
VOICE_LEVEL_TOPIC = "/voice/level"
WS_HOST = "0.0.0.0"
WS_PORT = 8765


class VoiceBridge(Node):
    def __init__(self, loop, clients):
        super().__init__("hmi_voice_bridge")
        self._loop = loop
        self._clients = clients
        self._state = "idle"
        self._level = 0.0

        self.create_subscription(String, VOICE_STATE_TOPIC, self._on_state, 10)
        self.create_subscription(Float32, VOICE_LEVEL_TOPIC, self._on_level, 10)

    def _on_state(self, msg):
        self._state = msg.data
        self._broadcast()

    def _on_level(self, msg):
        self._level = round(float(msg.data), 4)
        self._broadcast()

    def _broadcast(self):
        payload = json.dumps({"state": self._state, "level": self._level})
        # rclpy 콜백은 ROS spin 스레드에서 도는데, websocket 전송은 asyncio
        # 이벤트루프(별도 스레드) 소관이라 스레드 안전하게 넘겨준다.
        asyncio.run_coroutine_threadsafe(_send_all(self._clients, payload), self._loop)


async def _send_all(clients, payload):
    dead = []
    for ws in list(clients):
        try:
            await ws.send(payload)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.discard(ws)


async def _handler(websocket, clients):
    clients.add(websocket)
    try:
        # 이 채널은 서버(ROS)->브라우저 단방향이라 클라이언트가 보내는 건 그냥 버린다.
        async for _ in websocket:
            pass
    finally:
        clients.discard(websocket)


def main():
    rclpy.init()

    clients = set()
    loop = asyncio.new_event_loop()
    node = VoiceBridge(loop, clients)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    async def _serve():
        async with websockets.serve(lambda ws: _handler(ws, clients), WS_HOST, WS_PORT):
            node.get_logger().info(f"voice_bridge websocket 서버 시작: ws://{WS_HOST}:{WS_PORT}")
            await asyncio.Future()  # 영구 실행

    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(_serve())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

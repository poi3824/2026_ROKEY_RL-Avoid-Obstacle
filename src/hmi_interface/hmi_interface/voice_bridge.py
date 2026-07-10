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
from std_srvs.srv import SetBool
from rcl_interfaces.msg import Log

import websockets

VOICE_STATE_TOPIC = "/voice/state"
VOICE_LEVEL_TOPIC = "/voice/level"
VOICE_MANUAL_RECORD_SERVICE = "/voice/manual_record"
WS_HOST = "0.0.0.0"
WS_PORT = 8765

# get_keyword_node의 self.get_logger()가 이미 찍는 로그를 그대로 HMI로 보여준다
# (새 토픽/코드 추가 없이 재활용) - 모든 노드 로그가 /rosout(rcl_interfaces/msg/Log)에
# 모이므로 여기서 이름으로 필터링만 한다.
ROSOUT_TOPIC = "/rosout"
LOG_SOURCE_NODE = "get_keyword_node"
_LOG_LEVEL_NAMES = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


def _log_level_to_int(level):
    """rcl_interfaces/msg/Log.level은 이 rclpy 빌드에서 1바이트 bytes로 온다
    (예: Log.INFO == b'\\x14') - int로 정규화한다."""
    if isinstance(level, (bytes, bytearray)):
        return int.from_bytes(level, "little")
    return int(level)


class VoiceBridge(Node):
    def __init__(self, loop, ws_clients):
        super().__init__("hmi_voice_bridge")
        self._loop = loop
        # 2026-07-10 버그 수정: rclpy.node.Node가 내부적으로 서비스 클라이언트
        # 목록을 self._clients(리스트)로 관리한다 - 여기서 같은 이름을 쓰면
        # __init__에서 그걸 덮어써서, 이후 spin/destroy_node가 ROS Client 객체인
        # 줄 알고 이 안의 websocket 객체에 접근하다 죽는다(실기로 확인:
        # AttributeError: 'ServerConnection' object has no attribute 'handle').
        # Node의 내부 속성과 절대 겹치지 않도록 접두어를 붙인다.
        self._ws_clients = ws_clients
        self._state = "idle"
        self._level = 0.0

        self.create_subscription(String, VOICE_STATE_TOPIC, self._on_state, 10)
        self.create_subscription(Float32, VOICE_LEVEL_TOPIC, self._on_level, 10)
        # 2026-07-10: /rosout 구독 - get_keyword_node 로그를 STT-TTS 탭 로그
        # 블록에 그대로 흘려보낸다. /rosout은 TRANSIENT_LOCAL(과거분 재생)이지만
        # 여기선 기본(VOLATILE) 구독으로 충분하다 - 그 시점부터의 실시간 로그만
        # 필요하고, DDS QoS 규칙상 VOLATILE 구독자는 TRANSIENT_LOCAL 발행자와도
        # 호환된다(과거분만 못 받을 뿐).
        self.create_subscription(Log, ROSOUT_TOPIC, self._on_rosout, 50)

        # 2026-07-10: HMI 수동 녹음 토글 버튼 - 브라우저 -> 이 노드(websocket
        # 수신) -> get_keyword_node(서비스 호출) 방향. Flask는 여전히 이 경로에
        # 관여하지 않는다(브라우저가 이 노드의 websocket에 직접 붙어서 명령을 보냄).
        self._manual_record_client = self.create_client(SetBool, VOICE_MANUAL_RECORD_SERVICE)

    def request_manual_record(self, start):
        if not self._manual_record_client.service_is_ready():
            self.get_logger().warn(
                f"{VOICE_MANUAL_RECORD_SERVICE} 서비스 없음 - get_keyword_node가 안 떠 있는 듯"
            )
            return
        req = SetBool.Request()
        req.data = start
        self._manual_record_client.call_async(req)  # fire-and-forget - 결과는 /voice/state로 확인됨

    def _on_state(self, msg):
        self._state = msg.data
        self._broadcast({"state": self._state, "level": self._level})

    def _on_level(self, msg):
        self._level = round(float(msg.data), 4)
        self._broadcast({"state": self._state, "level": self._level})

    def _on_rosout(self, msg):
        if msg.name != LOG_SOURCE_NODE:
            return
        stamp = msg.stamp.sec + msg.stamp.nanosec / 1e9
        level_name = _LOG_LEVEL_NAMES.get(_log_level_to_int(msg.level), "INFO")
        self._broadcast({"log": {"level": level_name, "text": msg.msg, "stamp": stamp}})

    def _broadcast(self, payload_dict):
        payload = json.dumps(payload_dict)
        # rclpy 콜백은 ROS spin 스레드에서 도는데, websocket 전송은 asyncio
        # 이벤트루프(별도 스레드) 소관이라 스레드 안전하게 넘겨준다.
        asyncio.run_coroutine_threadsafe(_send_all(self._ws_clients, payload), self._loop)


async def _send_all(clients, payload):
    dead = []
    for ws in list(clients):
        try:
            await ws.send(payload)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.discard(ws)


async def _handler(websocket, clients, node):
    clients.add(websocket)
    try:
        # 2026-07-10: 수동 녹음 버튼 추가 전엔 이 채널이 서버->브라우저 단방향이라
        # 들어오는 메시지를 그냥 버렸다. 이제 {"cmd": "start_record"|"stop_record"}를
        # 받아 get_keyword_node의 서비스를 호출한다(node.request_manual_record).
        async for message in websocket:
            try:
                data = json.loads(message)
            except (ValueError, TypeError):
                continue
            cmd = data.get("cmd")
            if cmd == "start_record":
                node.request_manual_record(True)
            elif cmd == "stop_record":
                node.request_manual_record(False)
    finally:
        clients.discard(websocket)


def main():
    rclpy.init()

    clients = set()
    loop = asyncio.new_event_loop()
    node = VoiceBridge(loop, clients)

    # 2026-07-10 버그 수정: 이전엔 asyncio 루프를 메인 스레드에서 돌리고
    # rclpy.spin()을 백그라운드 스레드로 돌렸는데, 그 상태에서 SIGINT(Ctrl+C)를
    # 받으면 "terminate called without an active exception"(core dump)으로
    # 죽는 걸 asyncio 시그널 핸들러를 넣은 뒤에도 실기로 재확인했다. asyncio
    # signal handler로는 못 고치는 문제였다 - rclpy.init()이 내부적으로 등록하는
    # SIGINT 처리가 "메인 스레드에서 spin"을 전제하는 것으로 보인다(이 코드베이스의
    # 다른 모든 노드처럼). 그래서 반대로 asyncio 루프를 백그라운드 스레드로 돌리고
    # rclpy.spin()을 메인 스레드에 둔다(vision_bridge.py와 동일한 수정).
    def _run_loop():
        asyncio.set_event_loop(loop)
        loop.run_forever()

    loop_thread = threading.Thread(target=_run_loop, daemon=True)
    loop_thread.start()

    server_holder = {}

    async def _start_server():
        server = await websockets.serve(lambda ws: _handler(ws, clients, node), WS_HOST, WS_PORT)
        server_holder["server"] = server
        node.get_logger().info(f"voice_bridge websocket 서버 시작: ws://{WS_HOST}:{WS_PORT}")

    asyncio.run_coroutine_threadsafe(_start_server(), loop).result()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server = server_holder.get("server")
        if server is not None:
            async def _close_server():
                server.close()
                await server.wait_closed()
            asyncio.run_coroutine_threadsafe(_close_server(), loop).result()
        loop.call_soon_threadsafe(loop.stop)
        loop_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

# python-socketio Client 래퍼 - hmi/backend Flask-SocketIO의 "/ros" 네임스페이스에
# Bridge Token으로 접속한다.
#
# sio.emit()을 호출하는 건 이 파일의 전용 워커 스레드(_drain_loop) 하나뿐이다.
# rclpy 콜백(bridge_node.py)은 절대 이 클래스를 직접 건드리지 않고 EmitChannel에만
# 쓴다 - ROS 콜백 스레드가 네트워크 I/O로 블록되는 걸 막는다(설계 원칙).
#
# 재연결: python-socketio의 내장 reconnection(핑/퐁 heartbeat 포함)은 "한 번 연결된
# 뒤 끊어졌을 때"만 자동으로 동작하고, 맨 처음 connect() 자체가 실패하면(예: Flask가
# 아직 안 떠 있음) 예외를 던지고 끝난다 - 그래서 최초 연결은 별도 스레드에서 지수
# 백오프로 재시도한다(cold-start 시나리오 커버).
import logging
import threading
import time

import socketio

logger = logging.getLogger("hmi_ros_bridge.socketio_worker")

NAMESPACE = "/ros"
WORKER_TICK_SEC = 0.05
INITIAL_CONNECT_MAX_DELAY_SEC = 30.0


class SocketioWorker:
    def __init__(self, url, token, emit_channel, command_handler):
        self._url = url
        self._token = token
        self._emit_channel = emit_channel
        self._command_handler = command_handler
        self._stop_event = threading.Event()
        self._worker_thread = None
        self._connect_thread = None

        self.client = socketio.Client(
            reconnection=True, reconnection_delay=1, reconnection_delay_max=10,
        )

        @self.client.event(namespace=NAMESPACE)
        def connect():
            logger.info("hmi/backend(%s)에 연결됨", self._url)

        @self.client.event(namespace=NAMESPACE)
        def disconnect():
            logger.warning("hmi/backend 연결 끊김 - 내장 reconnection이 재시도함")

        @self.client.event(namespace=NAMESPACE)
        def connect_error(data):
            logger.error("hmi/backend 연결 실패(HMI_BRIDGE_TOKEN 확인 필요): %s", data)

        @self.client.on("command", namespace=NAMESPACE)
        def on_command(data):
            ack = self._command_handler(data)
            if ack is not None:
                self._emit_channel.publish_event("command_ack", ack)

    def start(self):
        self._worker_thread = threading.Thread(target=self._drain_loop, daemon=True)
        self._worker_thread.start()
        self._connect_thread = threading.Thread(target=self._connect_with_retry, daemon=True)
        self._connect_thread.start()

    def stop(self):
        self._stop_event.set()
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)
        if self.client.connected:
            self.client.disconnect()

    def _connect_with_retry(self):
        delay = 1.0
        while not self._stop_event.is_set():
            try:
                self.client.connect(self._url, namespaces=[NAMESPACE], auth={"token": self._token})
                return
            except Exception as e:
                logger.warning("hmi/backend 최초 연결 실패, %.1f초 후 재시도: %s", delay, e)
                time.sleep(delay)
                delay = min(delay * 2, INITIAL_CONNECT_MAX_DELAY_SEC)

    def _drain_loop(self):
        while not self._stop_event.is_set():
            if not self.client.connected:
                time.sleep(WORKER_TICK_SEC)
                continue
            for name, payload in self._emit_channel.drain_dirty_states():
                self._safe_emit(name, payload)
            for name, payload in self._emit_channel.drain_events():
                self._safe_emit(name, payload)
            time.sleep(WORKER_TICK_SEC)

    def _safe_emit(self, event_name, payload):
        try:
            self.client.emit(event_name, payload, namespace=NAMESPACE)
        except Exception as e:
            logger.warning("emit 실패(%s): %s", event_name, e)

# Flask 백엔드가 들고 있는 실시간 relay 상태.
#
# browser_ns.py/ros_ns.py 두 네임스페이스 핸들러가 서로 상태를 직접 주고받지 않고
# 이 객체 하나를 통해서만 조율한다 - dedup/pending 로직이 여러 파일에 흩어지는 걸 막는다.
#
# command_id 중복 방지는 Flask(여기)와 hmi_ros_bridge 양쪽에서 각각 수행한다(방어
# 계층 분리) - 여기 dedup은 "브라우저가 같은 명령을 두 번 보냈을 때 Flask가 두 번
# relay하지 않는다"만 책임지고, Bridge 쪽 dedup은 "Flask가 재시도 등으로 같은 명령을
# 두 번 전달했을 때 ROS 호출을 두 번 하지 않는다"를 책임진다.
import threading
import time


class HmiState:
    def __init__(self, dedup_ttl_sec, pending_ttl_sec):
        self._lock = threading.Lock()
        self._dedup_ttl = dedup_ttl_sec
        self._pending_ttl = pending_ttl_sec

        self.bridge_sid = None

        # command_id -> expire_at(float)
        self._seen_commands = {}
        # command_id -> {"sid":..., "expire_at":...}  (command_ack를 요청자에게 돌려주기 위함)
        self._pending_ack = {}
        # task_id -> {"command_id":..., "sid":..., "expire_at":...}
        # (terminal task_status를 만났을 때 command_result를 합성해 보낼 대상 찾기용)
        self._pending_control = {}

        # cache_key -> (event_name, payload) : ROS 쪽에서 온 "상태성" 이벤트의
        # 마지막 값. Socket.IO 자체에는 ROS TRANSIENT_LOCAL 같은 재전송 기능이
        # 없어서, 새 브라우저 탭이 붙었을 때(또는 재연결) 마지막 상태를 즉시
        # 못 받고 다음 갱신까지 빈 화면으로 보이는 문제가 있었다(실기 확인:
        # 헤드리스 브라우저로 대시보드를 열었더니 Safety/Task가 계속
        # "알 수 없음"으로 나왔음 - fake talker가 브라우저 접속보다 먼저 발행을
        # 끝내버린 상황). voice_log처럼 "각 항목이 독립적인 이벤트 스트림"은
        # 여기 캐시하지 않는다(cache_key는 record_last_known 호출부가 직접 고른다).
        self._last_known = {}

    def record_last_known(self, cache_key, event_name, payload):
        with self._lock:
            self._last_known[cache_key] = (event_name, payload)

    def all_last_known(self):
        with self._lock:
            return list(self._last_known.values())

    @property
    def bridge_connected(self):
        with self._lock:
            return self.bridge_sid is not None

    def set_bridge_connected(self, sid):
        with self._lock:
            self.bridge_sid = sid

    def clear_bridge(self, sid):
        with self._lock:
            if self.bridge_sid == sid:
                self.bridge_sid = None

    def check_and_mark_duplicate(self, command_id):
        """True를 반환하면 이미 처리한 command_id(중복) - 호출부는 relay하면 안 된다."""
        now = time.time()
        with self._lock:
            self._sweep_locked(self._seen_commands, now)
            if command_id in self._seen_commands:
                return True
            self._seen_commands[command_id] = now + self._dedup_ttl
            return False

    def register_pending_ack(self, command_id, sid):
        now = time.time()
        with self._lock:
            self._sweep_locked(self._pending_ack, now)
            self._pending_ack[command_id] = {"sid": sid, "expire_at": now + self._pending_ttl}

    def pop_pending_ack(self, command_id):
        with self._lock:
            entry = self._pending_ack.pop(command_id, None)
        return entry["sid"] if entry else None

    def register_pending_control(self, task_id, command_id, sid):
        if not task_id:
            return
        now = time.time()
        with self._lock:
            self._sweep_locked(self._pending_control, now)
            self._pending_control[task_id] = {
                "command_id": command_id,
                "sid": sid,
                "expire_at": now + self._pending_ttl,
            }

    def pop_pending_control(self, task_id):
        if not task_id:
            return None
        with self._lock:
            return self._pending_control.pop(task_id, None)

    @staticmethod
    def _sweep_locked(store, now):
        expired = [
            key for key, val in store.items()
            if (val["expire_at"] if isinstance(val, dict) else val) < now
        ]
        for key in expired:
            del store[key]

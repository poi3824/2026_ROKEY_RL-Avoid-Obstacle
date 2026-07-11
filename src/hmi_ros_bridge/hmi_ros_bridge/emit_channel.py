# ROS 콜백 스레드가 sio.emit()을 직접 호출하지 않게 하는 채널.
#
# 두 가지 종류의 트래픽을 구분해서 다룬다(둘 다 하나의 공유 FIFO에 넣지 않는다):
#
# 1. "상태"(latest-value) - voice_status처럼 "지금 값이 뭐냐"만 의미 있고 중간값은
#    버려도 되는 데이터. 토픽별로 독립된 슬롯을 두어, 한 토픽이 고빈도로 갱신돼도
#    다른(저빈도·중요) 토픽 슬롯을 밀어내지 못하게 한다 - 공유 bounded FIFO에서
#    drop-oldest를 쓰면 이 starvation이 생길 수 있어서 아예 구조를 분리했다.
# 2. "이벤트"(ordered, 드롭 최소화) - voice_log, command_ack처럼 각 항목이 고유한
#    의미를 갖고 순서/완전성이 중요한 데이터. bounded Queue를 쓰되 용량을 넉넉히
#    잡아(기본 500) 정상적인 부하에서는 절대 안 찬다 - 그래도 꽉 차면 가장 오래된
#    항목을 버리고 경고 로그를 남긴다(무한정 쌓아서 메모리를 못 먹게 함).
#
# 실제 소켓 전송(emit)은 이 모듈이 하지 않는다 - socketio_worker.py의 전용 워커
# 스레드 하나만 이 채널에서 꺼내 emit을 직렬 호출한다.
import logging
import queue
import threading

logger = logging.getLogger("hmi_ros_bridge.emit_channel")


class EmitChannel:
    def __init__(self, event_queue_maxsize=500):
        self._lock = threading.Lock()
        self._latest = {}  # slot_key -> (event_name, payload)
        self._dirty = set()  # slot_key들 - drain 대상
        self._events = queue.Queue(maxsize=event_queue_maxsize)

    def publish_state(self, slot_key, event_name, payload):
        """slot_key는 순수 내부 식별자(starvation 방지용 슬롯 분리에만 쓰임),
        event_name이 실제 Socket.IO에 나가는 이벤트 이름이다 - 이 둘이 다를 수
        있다(예: task_status:manipulation, task_status:world_map 두 슬롯이 전부
        event_name="task_status"로 나가야 하는 경우, source는 payload 안에서
        구분). 슬롯 키를 그대로 이벤트 이름으로 쓰면 여러 소스가 같은 이벤트
        이름 하나로 통합 발행돼야 하는 경우를 표현할 수 없다."""
        with self._lock:
            self._latest[slot_key] = (event_name, payload)
            self._dirty.add(slot_key)

    def publish_event(self, event_name, payload):
        try:
            self._events.put_nowait((event_name, payload))
        except queue.Full:
            try:
                self._events.get_nowait()  # 가장 오래된 것 버림
            except queue.Empty:
                pass
            try:
                self._events.put_nowait((event_name, payload))
            except queue.Full:
                pass
            logger.warning("event queue full - 오래된 %s 이벤트 드롭됨", event_name)

    def drain_dirty_states(self):
        with self._lock:
            items = [self._latest[key] for key in self._dirty]
            self._dirty.clear()
        return items

    def drain_events(self, max_items=50):
        items = []
        for _ in range(max_items):
            try:
                items.append(self._events.get_nowait())
            except queue.Empty:
                break
        return items

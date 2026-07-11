# 순수 Python 로직 테스트 - ROS 불필요. latest-value 상태와 ordered 이벤트 큐가
# 서로 분리되어(한쪽이 다른쪽을 밀어내지 않음) 의도한 대로 동작하는지 확인한다.
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from hmi_ros_bridge.emit_channel import EmitChannel  # noqa: E402


def test_latest_state_coalesces_to_last_value():
    ch = EmitChannel()
    ch.publish_state("voice_status", "voice_status", {"state": "idle", "level": 0.1})
    ch.publish_state("voice_status", "voice_status", {"state": "recording", "level": 0.5})
    ch.publish_state("voice_status", "voice_status", {"state": "recording", "level": 0.9})

    drained = ch.drain_dirty_states()
    assert drained == [("voice_status", {"state": "recording", "level": 0.9})]
    # 두 번째 drain은 dirty한 게 없어야 함
    assert ch.drain_dirty_states() == []


def test_latest_state_does_not_starve_other_topics():
    ch = EmitChannel()
    for i in range(1000):
        ch.publish_state("voice_status", "voice_status", {"level": i})
    ch.publish_state("safety_status", "safety_status", {"state": "ESTOP"})

    drained = dict(ch.drain_dirty_states())
    assert drained["safety_status"] == {"state": "ESTOP"}
    assert drained["voice_status"] == {"level": 999}


def test_different_slot_keys_can_share_one_event_name():
    """task_status:manipulation / task_status:world_map처럼 슬롯 키는 다르지만
    Socket.IO에는 같은 event_name("task_status")으로 나가야 하는 경우."""
    ch = EmitChannel()
    ch.publish_state("task_status:manipulation", "task_status", {"source": "manipulation"})
    ch.publish_state("task_status:world_map", "task_status", {"source": "world_map"})

    drained = ch.drain_dirty_states()
    assert len(drained) == 2
    event_names = {name for name, _payload in drained}
    assert event_names == {"task_status"}
    sources = {payload["source"] for _name, payload in drained}
    assert sources == {"manipulation", "world_map"}


def test_events_are_delivered_in_order_not_coalesced():
    ch = EmitChannel()
    ch.publish_event("voice_log", {"text": "line1"})
    ch.publish_event("voice_log", {"text": "line2"})
    ch.publish_event("voice_log", {"text": "line3"})

    drained = ch.drain_events(max_items=10)
    assert [p["text"] for _name, p in drained] == ["line1", "line2", "line3"]


def test_event_queue_drops_oldest_when_full():
    ch = EmitChannel(event_queue_maxsize=3)
    for i in range(5):
        ch.publish_event("voice_log", {"i": i})

    drained = ch.drain_events(max_items=10)
    # 오래된 것부터 버려지므로 마지막 3개(2,3,4)만 남아야 함
    assert [p["i"] for _name, p in drained] == [2, 3, 4]

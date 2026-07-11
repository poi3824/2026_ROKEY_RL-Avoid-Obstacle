# 순수 로직 테스트 - 실제 카메라/YOLO 모델/rclpy 없이도 돌아간다.
#
# 2026-07-11 (HMI 재구축 Phase 4): detection.py의 _hand_check_timer_callback을
# has_label() 직접 호출에서 detect_frame() + _extract_hand_detected()로
# 리팩토링했다 - hand_detected는 safety_monitor_node가 ESTOP/PAUSE 판단에
# 직접 쓰는 안전 신호라, 이 테스트로 "판정 값이 하나도 안 바뀌었는지"를
# has_label(threshold=0.9)과 동일한 의미(라벨 일치 + score>=threshold가
# 하나라도 있으면 True)로 명시 검증한다.
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from object_detection.detection import (  # noqa: E402
    _extract_hand_detected,
    HAND_CONFIDENCE_THRESHOLD,
)


def test_default_threshold_matches_has_label_default():
    assert HAND_CONFIDENCE_THRESHOLD == 0.9


def test_no_detections_means_no_hand():
    assert _extract_hand_detected([]) is False


def test_hand_above_threshold_detected():
    detections = [{"label": "hand", "score": 0.95, "box": [0, 0, 1, 1]}]
    assert _extract_hand_detected(detections) is True


def test_hand_below_threshold_not_detected():
    # has_label(threshold=0.9)도 0.5짜리는 무시했을 것 - 동일해야 함
    detections = [{"label": "hand", "score": 0.5, "box": [0, 0, 1, 1]}]
    assert _extract_hand_detected(detections) is False


def test_other_labels_ignored_even_if_high_confidence():
    detections = [{"label": "obj_A", "score": 0.99, "box": [0, 0, 1, 1]}]
    assert _extract_hand_detected(detections) is False


def test_mixed_detections_any_matching_hand_triggers():
    detections = [
        {"label": "obj_B", "score": 0.99, "box": [0, 0, 1, 1]},
        {"label": "hand", "score": 0.91, "box": [2, 2, 3, 3]},
    ]
    assert _extract_hand_detected(detections) is True


def test_exact_threshold_boundary_is_inclusive():
    # has_label의 `score >= confidence_threshold`와 동일해야 함 (>가 아니라 >=)
    detections = [{"label": "hand", "score": 0.9, "box": [0, 0, 1, 1]}]
    assert _extract_hand_detected(detections) is True

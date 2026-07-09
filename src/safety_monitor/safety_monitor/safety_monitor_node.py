# 안전 감시 전담 노드.
#
# 이전 구조에서는 robot_action_node(=두뇌)가 /hand_detected를 직접 구독하고,
# 음성 '정지'는 STT가 /dsr01/emergency_stop 서비스를 직접 호출했으며, 두뇌 안의
# emergency_stop 서비스 핸들러가 dsr_lock을 다시 잡아 재진입 데드락 위험이 있었다.
#
# 여기서는 안전 신호(손 감지, 음성 정지, 향후 충돌/힘)를 이 노드가 모아서 판단하고,
# 결과를 /safety/state(SafetyState)로 방송한다. brain은 고수준 goal 제어에,
# motion은 move_linear 폴링 루프의 실시간 정지·재개에 이 상태를 쓴다.
# 하드 정지(ESTOP)는 두뇌 상태와 무관하게 로봇 컨트롤러의 motion/move_stop을
# 직접 호출하므로, 두뇌가 바쁘거나 멈춰 있어도 로봇을 세울 수 있다.
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from dsr_msgs2.srv import MoveStop

from robot_interfaces.msg import SafetyState

# ESTOP 시 로봇 컨트롤러에 보낼 하드 정지 강도.
# 0=QSTOP_STO(서보 토크 차단, 가장 강력), 1=QSTOP, 2=SSTOP, 3=HOLD.
# 2026-07-09: QSTOP_STO(0)는 서보 토크를 차단해서, 재개하려면 컨트롤러가
# 재서보/재초기화하는 과정에서 홈으로 이동하는 것 같은 부작용이 있었다.
# HOLD(3)는 서보 토크를 유지한 채 그 자리에서 멈추므로, RESUME 시 같은
# 목적지로 movel을 다시 issue하는 것만으로 하던 동작을 이어갈 수 있다
# (motion_executor._handle_interrupts의 hand_pause와 동일한 재개 방식).
ESTOP_STOP_MODE = 3

# /safety/state를 주기적으로도 재발행해서(하트비트) 늦게 뜬 구독자나 메시지 유실에
# 대비한다. 상태가 바뀌는 순간에는 이 타이머와 별개로 즉시 발행한다.
HEARTBEAT_SEC = 0.5

# 2026-07-09: YOLO hand 감지 오탐(그리퍼+쥔 물체를 손으로 오인식하는 문제)에 대한
# 임시 완화책 — 모델을 다시 학습시키는 대신, 음성으로 "손 아니야"라고 하면 이
# 시간(초) 동안만 /hand_detected를 무시한다. hand 감지는 래치가 아니라 실시간
# 값이라(_current_state 참고) 그냥 한 번 클리어하는 걸로는 안 되고, 시간 기반으로
# 억제해야 한다. 너무 길게 잡으면 그 사이 진짜 손이 들어와도 못 걸러서 짧게 둔다.
IGNORE_HAND_DURATION_SEC = 1.0


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__("safety_monitor_node")

        self.declare_parameter("robot_id", "dsr01")
        robot_id = self.get_parameter("robot_id").value
        move_stop_service = f"/{robot_id}/motion/move_stop"

        self._hand_detected = False
        self._estop_latched = False  # ESTOP은 래치된다 — /safety/reset 전까지 유지
        self._hand_ignore_until = 0.0  # 이 시각(monotonic) 전까지는 hand 감지를 무시
        self._last_state = None

        # 상태는 마지막 값이 중요하므로 transient_local로 발행 → 나중에 뜬
        # brain/motion 구독자도 최신 상태를 즉시 받는다.
        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        self.state_pub = self.create_publisher(SafetyState, "/safety/state", state_qos)

        self.create_subscription(Bool, "/hand_detected", self._on_hand_detected, 10)
        self.create_subscription(Bool, "/voice/estop", self._on_voice_estop, 10)

        # 버튼/외부 트리거용 서비스도 열어둔다.
        self.create_service(Trigger, "/safety/estop", self._on_estop_request)
        self.create_service(Trigger, "/safety/reset", self._on_reset_request)
        self.create_service(Trigger, "/safety/ignore_hand", self._on_ignore_hand_request)

        self.move_stop_client = self.create_client(MoveStop, move_stop_service)

        self.create_timer(HEARTBEAT_SEC, self._publish_state)

        self.get_logger().info(
            f"safety_monitor_node 시작. move_stop 대상: {move_stop_service}"
        )
        self._publish_state()

    # ---- 입력 콜백 ----
    def _on_hand_detected(self, msg):
        prev = self._hand_detected
        self._hand_detected = bool(msg.data)
        if self._hand_detected != prev:
            self._publish_state()

    def _on_voice_estop(self, msg):
        if msg.data:
            self._trigger_estop("음성 정지 키워드")

    def _on_estop_request(self, request, response):
        self._trigger_estop("외부 estop 서비스")
        response.success = True
        response.message = "estop triggered"
        return response

    def _on_reset_request(self, request, response):
        was = self._estop_latched
        self._estop_latched = False
        self.get_logger().info(f"ESTOP 리셋 (이전 래치={was})")
        self._publish_state()
        response.success = True
        response.message = "safety reset"
        return response

    def _on_ignore_hand_request(self, request, response):
        self._hand_ignore_until = time.monotonic() + IGNORE_HAND_DURATION_SEC
        self.get_logger().warn(f"손 감지 {IGNORE_HAND_DURATION_SEC}초간 무시(음성 정정)")
        self._publish_state()
        response.success = True
        response.message = "hand detection ignored temporarily"
        return response

    # ---- 정지 실행 ----
    def _trigger_estop(self, reason):
        if not self._estop_latched:
            self.get_logger().warn(f"ESTOP 발동: {reason}")
        self._estop_latched = True
        self._call_move_stop()
        self._publish_state()

    def _call_move_stop(self):
        """로봇 컨트롤러의 motion/move_stop을 직접 호출한다(두뇌 경유 없음)."""
        if not self.move_stop_client.service_is_ready():
            # 논블로킹으로 한 번만 확인 — 서비스가 아직 없으면 상태 발행만 하고 넘어간다.
            if not self.move_stop_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().error("move_stop 서비스가 아직 없음 — 상태만 발행")
                return
        req = MoveStop.Request()
        req.stop_mode = ESTOP_STOP_MODE
        self.move_stop_client.call_async(req)  # fire-and-forget (spin이 future 처리)

    # ---- 상태 계산/발행 ----
    def _current_state(self):
        if self._estop_latched:
            return SafetyState.ESTOP, "estop latched"
        if self._hand_detected and time.monotonic() >= self._hand_ignore_until:
            return SafetyState.PAUSE, "hand detected"
        return SafetyState.RUN, "normal"

    def _publish_state(self):
        state, reason = self._current_state()
        msg = SafetyState()
        msg.state = state
        msg.reason = reason
        self.state_pub.publish(msg)
        if state != self._last_state:
            self.get_logger().info(f"safety state → {reason} ({state})")
            self._last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

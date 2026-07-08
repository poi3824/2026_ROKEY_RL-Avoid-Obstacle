# 중앙 두뇌 / 통신 허브 (개편 v2에서 robot_action_node를 경량화한 노드).
#
# 로봇/DSR·Vision을 직접 건드리지 않는다. 하는 일:
#   1) STT(get_keyword)로부터 명령 텍스트 수신
#   2) POSITION_COORDS로 이름있는 위치(scan/targetN/home)를 좌표로 풀기
#   3) motion_node에 Action goal(Pick → Place → MoveTo home) 전송, 결과 추적
#   4) /safety/state 구독 — 새 명령 시작 시 ESTOP 래치를 /safety/reset으로 해제
#
# 무거운 멀티스레드(로봇 제어)는 전부 motion_node에 있다. 여기는 상태머신 + Action
# client + 단일 executor spin 스레드만 있어 가볍고, motion에 goal을 보낸 뒤에는
# 비동기로 결과를 기다리므로(폴링) blocking 되지 않는다.
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_srvs.srv import Trigger

from robot_interfaces.action import MoveTo, Pick, Place
from robot_interfaces.msg import SafetyState

POSITION_COORDS = {
    "home": [417.61, -0.76, 477.45, 174.25, 179.99, -7.65],
    "scan": [603.65, 117.06, 466.15, 96.74, -179.75, -85.08],
    "target1": [200.0, 100.0, 466.058, 138.332, -179.994, -43.561],
    "target2": [199.91, 0.066, 466.172, 177.529, 179.942, -2.8],
    "target3": [199.93, 100.092, 466.217, 174.166, 179.947, -6.174],
}


class BrainNode(Node):
    def __init__(self):
        super().__init__("brain_node")

        # 상시 spin 스레드 — 서비스/액션 client의 future를 처리한다.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        # get_keyword (STT)
        self.get_keyword_client = self.create_client(Trigger, "get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

        # safety reset (ESTOP 래치 해제)
        self.safety_reset_client = self.create_client(Trigger, "/safety/reset")

        # /safety/state 구독 (고수준 상태 추적)
        safety_qos = QoSProfile(depth=1)
        safety_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        safety_qos.reliability = ReliabilityPolicy.RELIABLE
        self._safety_state = SafetyState.RUN
        self.create_subscription(
            SafetyState, "/safety/state", self._on_safety_state, safety_qos
        )

        # motion_node Action clients
        self.moveto_client = ActionClient(self, MoveTo, "motion/move_to")
        self.pick_client = ActionClient(self, Pick, "motion/pick")
        self.place_client = ActionClient(self, Place, "motion/place")
        for name, client in (
            ("move_to", self.moveto_client),
            ("pick", self.pick_client),
            ("place", self.place_client),
        ):
            while not client.wait_for_server(timeout_sec=3.0):
                self.get_logger().info(f"Waiting for motion/{name} action server...")

        self.robot_init()

    def _on_safety_state(self, msg):
        self._safety_state = msg.state

    def robot_init(self):
        self.get_logger().info("Initializing robot: home 이동")
        self._send_move_to(POSITION_COORDS["home"], "home")

    # ---- 메인 루프 ----
    def run_once(self):
        self.get_logger().info("call get_keyword service")
        result = self._call_service(self.get_keyword_client, self.get_keyword_request)

        if result is None or not result.success:
            message = result.message if result else "no response"
            self.get_logger().warn(f"get_keyword failed: {message}")
            return

        try:
            self.execute_command(result.message)
        except Exception as e:
            # 파싱 실패 등 예외가 main 루프를 뚫고 나가 노드가 죽지 않도록 방어.
            self.get_logger().error(f"명령 처리 중 오류, 이번 명령은 무시함: {e}")
            self._send_move_to(POSITION_COORDS["home"], "home")

    def execute_command(self, message):
        """message 형식: "object / source / target / return_pos" (각 구간 공백 구분)."""
        obj_part, source_part, target_part, _return_part = message.split("/")
        objects = obj_part.split()
        sources = source_part.split()
        targets = target_part.split()

        self.get_logger().info(
            f"object: {objects}\n source: {sources}\n target: {targets}"
        )

        # 음성 정지는 STT→safety_monitor 경로로 이미 처리되므로 여기 STOP이 올 일은
        # 거의 없지만, 방어적으로 무시한다.
        if "STOP" in objects:
            self.get_logger().warn("STOP 명령 수신(무시) — 안전 정지는 safety_monitor가 처리")
            return

        if not (len(objects) == len(sources) == len(targets)):
            self.get_logger().warn(
                f"물체/위치 개수 불일치, 이번 명령은 무시함: "
                f"{len(objects)} / {len(sources)} / {len(targets)}"
            )
            return

        # 새 명령 시작 — 이전 ESTOP 래치가 있으면 해제한다.
        self._reset_safety()

        for obj, source, target in zip(objects, sources, targets):
            scan_pose = POSITION_COORDS.get(source)
            target_pose = POSITION_COORDS.get(target)
            if scan_pose is None:
                self.get_logger().warn(f"'{source}' 스캔 위치가 아직 채워지지 않음")
                continue
            if target_pose is None:
                self.get_logger().warn(f"'{target}' 좌표가 아직 채워지지 않음")
                continue

            pick_res = self._send_pick(obj, scan_pose)
            if pick_res is None or not pick_res.success:
                reason = pick_res.message if pick_res else "no result"
                self.get_logger().warn(f"'{obj}' Pick 실패({reason}), 이번 물체 건너뜀")
                continue

            place_res = self._send_place(target_pose)
            if place_res is None or not place_res.success:
                reason = place_res.message if place_res else "no result"
                self.get_logger().warn(f"'{obj}' Place 실패({reason})")

        self._send_move_to(POSITION_COORDS["home"], "home")

    # ---- Action / service 헬퍼 ----
    def _reset_safety(self):
        if not self.safety_reset_client.service_is_ready():
            if not self.safety_reset_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("/safety/reset 서비스 없음 — 리셋 생략")
                return
        self._call_service(self.safety_reset_client, Trigger.Request(), timeout_sec=2.0)

    def _send_move_to(self, pose, label=""):
        goal = MoveTo.Goal()
        goal.pose = [float(v) for v in pose]
        goal.label = label
        return self._send_goal_and_wait(self.moveto_client, goal, "MoveTo")

    def _send_pick(self, obj, scan_pose):
        goal = Pick.Goal()
        goal.object_label = obj
        goal.scan_pose = [float(v) for v in scan_pose]
        return self._send_goal_and_wait(self.pick_client, goal, "Pick")

    def _send_place(self, target_pose):
        goal = Place.Goal()
        goal.target_pose = [float(v) for v in target_pose]
        return self._send_goal_and_wait(self.place_client, goal, "Place")

    def _send_goal_and_wait(self, client, goal, name):
        """goal 전송 후 결과를 폴링으로 기다린다(spin은 상시 스레드가 처리)."""
        goal_future = client.send_goal_async(
            goal, feedback_callback=lambda fb: self._on_feedback(name, fb)
        )
        goal_handle = self._wait_future(goal_future)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f"{name} goal 거부됨")
            return None

        result_future = goal_handle.get_result_async()
        wrapped = self._wait_future(result_future)
        return wrapped.result if wrapped is not None else None

    def _on_feedback(self, name, feedback_msg):
        fb = feedback_msg.feedback
        phase = getattr(fb, "phase", "")
        self.get_logger().info(f"[{name}] {phase}")

    def _call_service(self, client, request, timeout_sec=None):
        future = client.call_async(request)
        return self._wait_future(future, timeout_sec)

    def _wait_future(self, future, timeout_sec=None):
        start = time.time()
        while not future.done():
            if timeout_sec is not None and (time.time() - start) > timeout_sec:
                return None
            time.sleep(0.01)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        while rclpy.ok():
            node.run_once()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

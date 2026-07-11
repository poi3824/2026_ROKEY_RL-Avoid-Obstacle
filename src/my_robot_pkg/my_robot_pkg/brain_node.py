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
import json
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_srvs.srv import Trigger
from std_msgs.msg import String

from robot_interfaces.action import MoveTo, Pick, Place
from robot_interfaces.msg import SafetyState

# Pick/Place/MoveTo goal 결과 대기 타임아웃.
# get_keyword(웨이크워드 대기)와 달리 이건 "사람이 언제 말할지 모르는" 무한 대기가
# 아니라 물리 동작 + 정해진 서비스 타임아웃들의 조합이므로 상한을 걸 수 있고, 걸어야
# 한다 — 이게 없으면 motion_node 쪽에서 처리되지 않은 예외로 goal이 영원히
# EXECUTING 상태에 머물 때 brain도 결과를 무한정 기다리며 그대로 멈춰버린다
# (오늘 겪은 "다음 동작으로 안 넘어감" 증상의 원인).
#
# motion_node.py 기준 최악 케이스 역산(Pick 기준):
# 2026-07-08: "seg 추론 프레임당 ~1초" 가정이 실측(~0.1초)과 안 맞았던 걸 바로잡아
# GET_TARGET_TIMEOUT을 12s->6s로 줄인 김에 이 값도 같이 재계산한다.
#   get_3d_position 왕복 GET_TARGET_TIMEOUT=6s
#   get_surface_z()는 GET_SURFACE_Z_SAMPLES=5회 샘플 × 6s = 최대 30s
#   PICK_MAX_ATTEMPTS=3회 시도, 매 시도 hover 이동 + get_surface_z(최대 30s)
#     + 하강 + gripper(GRIPPER_TIMEOUT_SEC=3s) + 실패 시 redetect(최대 6s)
#   => 3 * (30 + 3) + 2 * 6 ≈ 111s. 여유를 더해 130s로 잡는다.
# MoveTo/Place는 이보다 훨씬 짧게 끝나지만(get_surface_z 호출이 0~1회) 같은 상수를
# 공용으로 써도 실패 감지가 최대 130s 늦어질 뿐 안전엔 문제없어 하나로 통일한다.
#
# 2026-07-09: ESTOP이 이제 액션을 중단(abort)하지 않고 그 자리에서 대기하다가
# RESUME 시 하던 동작을 이어간다(motion_executor._handle_interrupts 참고). 즉
# 정지 상태로 있는 시간도 이 타임아웃에 그대로 들어간다 — 사용자가 "다시 시작해"를
# 말하기까지 4분 이상은 안 걸릴 거라는 판단 하에 여유를 두고 360s로 늘린다.
ACTION_RESULT_TIMEOUT_SEC = 360.0

# world_map_node의 update_world_map 서비스 타임아웃. run_scan()은 지그재그 그리드
# 전체를 MoveLine으로 훑는 구조라 pose마다 이동 + settle(1.2~3s)이 누적돼 실측상
# 수 분이 걸린다(world_map_node.py 주석 참고). ACTION_RESULT_TIMEOUT_SEC(Pick/Place
# 기준 130s)보다 훨씬 크게 잡아야 정상 스캔 도중 끊기지 않는다.
WORLD_MAP_SCAN_TIMEOUT_SEC = 600.0

# POSITION_COORDS = {
#     "home": [417.61, -0.76, 477.45, 174.25, 179.99, -7.65],
#     "scan": [560.37, 256.18, 460.56, 44.09, 175.79, -130.28],
#     # 2026-07-09: 스캔 스윕(scan -> scan_b) 범위. 실측 완료 — Y축으로만 이동.
#     "scan_b": [560.37, -121.8, 460.56, 44.09, 175.79, -130.28],
#     "target1": [200.0, 100.0, 466.058, 138.332, -179.994, -43.561],
#     "target2": [199.91, 0.066, 466.172, 177.529, 179.942, -2.8],
#     "target3": [199.93, 100.092, 466.217, 174.166, 179.947, -6.174],
# }
POSITION_COORDS = {
    "home": [417.61, -0.76, 477.45, 174.25, 179.99, -7.65],
    "scan": [560.37, 256.18, 460.56, 44.09, 175.79, -130.28],
    # 2026-07-09: 스캔 스윕(scan -> scan_b) 범위. 실측 완료 — Y축으로만 이동.
    "scan_b": [560.37, -121.8, 460.56, 44.09, 175.79, -130.28],
    "target1": [200.0, 100.0, 271.058, 138.332, -179.994, -43.561],
    "target2": [199.91, 0.066, 271.172, 177.529, 179.942, -2.8],
    "target3": [199.93, -100.092, 271.217, 174.166, 179.947, -6.174],
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

        # world map (STT "월드맵 업데이트" 트리거). get_keyword_client와 달리
        # 생성자에서 blocking 대기하지 않는다 — world_map_node가 안 떠 있어도
        # pick-and-place만으로 brain_node가 뜰 수 있어야 하므로, 서비스 준비
        # 여부는 호출 시점에 확인한다(safety_reset_client와 동일한 패턴).
        self.world_map_client = self.create_client(Trigger, "update_world_map")

        # 2026-07-08: TTS. 상태 전환 시 get_keyword_node로 말할 텍스트를 던진다
        # (get_keyword_node가 재생 + 웨이크워드 피드백 루프 차단까지 담당).
        self.tts_pub = self.create_publisher(String, "/tts/speak", 10)

        # /safety/state 구독 (고수준 상태 추적)
        safety_qos = QoSProfile(depth=1)
        safety_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        safety_qos.reliability = ReliabilityPolicy.RELIABLE
        # 2026-07-09 버그 수정: 처음엔 RUN이 아니라 None(미확인)으로 시작한다.
        # 이전엔 기본이 RUN이라, safety_monitor가 이미 ESTOP/PAUSE를 래치한 상태에서
        # brain_node가 재시작되면 TRANSIENT_LOCAL 구독이 실제 상태를 받기 전까지의
        # 짧은 창 동안 execute_command의 안전 체크(`!= SafetyState.RUN`)가 실제로는
        # 안전하지 않은데도 통과했다. None은 RUN과 항상 다르므로 그 체크가 그대로
        # "안전 미확인 = 정지 상태"로 처리해준다.
        self._safety_state = None
        self.create_subscription(
            SafetyState, "/safety/state", self._on_safety_state, safety_qos
        )

        # 2026-07-11 (HMI 재구축 Phase 5): HMI 대시보드용 조작 태스크 진행 상태.
        # /hmi/task_status/world_map(world_map_node가 발행)과 별개 토픽 -
        # hmi_ros_bridge가 두 토픽을 구독 소스 기준으로 태깅해서 하나의
        # Socket.IO task_status 이벤트로 통합한다(motion_node는 발행하지
        # 않음 - Action feedback은 _on_feedback을 통해 이미 brain_node가
        # 소비하는 보조 정보일 뿐, 정본은 여기서 발행하는 이 상태다).
        # safety_qos와 동일하게 TRANSIENT_LOCAL - 늦게 붙는 hmi_ros_bridge도
        # 마지막 상태를 바로 받게 한다.
        self.task_status_pub = self.create_publisher(
            String, "hmi/task_status/manipulation", safety_qos
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

    # --- safety state 구독 콜백 ----
    def _on_safety_state(self, msg):
        self._safety_state = msg.state

    def _say(self, text):
        """로봇 음성 응답을 요청한다(fire-and-forget). get_keyword_node가 재생한다."""
        self.tts_pub.publish(String(data=text))

    def _publish_task_status(
        self, task_id, status, step_index=None, step_total=None,
        title="", detail="", phase="",
    ):
        """hmi/task_status/manipulation으로 task_status.schema.json 형태 JSON을
        발행한다. status는 IDLE/WAITING/RUNNING/COMPLETED/FAILED만 쓴다(Safety
        RUN/PAUSE/ESTOP과 절대 섞지 않는다 - 별도 모델)."""
        self.task_status_pub.publish(String(data=json.dumps({
            "task_id": task_id,
            "mode": "pick_place",
            "phase": phase,
            "title": title,
            "detail": detail,
            "step_index": step_index,
            "step_total": step_total,
            "progress": (step_index / step_total) if step_total else None,
            "status": status,
            "timestamp": time.time(),
        })))

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

    # get_keword 서비스 호출 후 받아온 message를 리스트로 정리
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

        # 월드맵 스캔 명령. 프롬프트 규칙상 STOP과 겹치면 STOP이 우선이라
        # 이 시점에는 이미 STOP이 아님이 보장된다.
        if "WORLD_MAP" in objects:
            self.get_logger().warn("월드맵 업데이트 명령 수신")
            self._update_world_map()
            return

        # 2026-07-09: RESUME은 이제 get_keyword_node가 브레인을 거치지 않고
        # 여기서 바로 처리한다(_call_safety_reset 참고) — ESTOP이 액션을
        # 중단시키지 않고 그 자리에서 대기만 하는 구조가 되면서, 브레인의
        # 메인 루프가 진행 중인 액션 결과를 기다리며 블로킹돼 있어 RESUME이
        # 여기로 절대 전달될 수 없기 때문이다(실측: "다시 시작해"를 해도
        # 하강이 재개 안 됨). STOP과 마찬가지로 여기 올 일은 거의 없지만,
        # 방어적으로만 남겨둔다.
        if "RESUME" in objects:
            self.get_logger().warn("RESUME 명령 수신(무시) — get_keyword_node가 이미 처리함")
            return

        # 리스트의 크기 맞으면 작동, 맞지 않으면 크기가 작은 리스트는 무시함으로
        if not (len(objects) == len(sources) == len(targets)):
            self.get_logger().warn(
                f"물체/위치 개수 불일치, 이번 명령은 무시함: "
                f"{len(objects)} / {len(sources)} / {len(targets)}"
            )
            return

        # 새 명령 시작 — 이전 ESTOP 래치가 있으면 해제한다.
        self._reset_safety()
        self._say("명령을 확인했습니다")

        # 2026-07-11 (HMI 재구축 Phase 5): task_id는 voice 명령 1회 실행(여러
        # 물체를 순차 처리할 수 있음) 단위로 여기서 새로 발급한다 - Action goal
        # UUID를 재사용하지 않는 이유는 하나의 명령이 Pick/Place/MoveTo 여러
        # 개로 이루어져 goal이 여러 개 생기기 때문에, "이 명령 전체"를 가리키는
        # 상위 식별자가 필요해서다.
        task_id = str(uuid.uuid4())
        step_total = len(objects)
        self._publish_task_status(
            task_id, "RUNNING", step_index=0, step_total=step_total,
            title=f"{step_total}개 물체 처리", phase="starting",
        )

        for step_index, (obj, source, target) in enumerate(zip(objects, sources, targets)):
            scan_pose = POSITION_COORDS.get(source)
            target_pose = POSITION_COORDS.get(target)
            if scan_pose is None:
                self.get_logger().warn(f"'{source}' 스캔 위치가 아직 채워지지 않음")
                continue
            if target_pose is None:
                self.get_logger().warn(f"'{target}' 좌표가 아직 채워지지 않음")
                continue

            self._publish_task_status(
                task_id, "RUNNING", step_index=step_index, step_total=step_total,
                title=f"'{obj}'를 '{target}'로 이동", phase="picking",
            )

            # source(예: "scan")의 짝(예: "scan_b")이 POSITION_COORDS에 있으면 그 사이를
            # 스윕하며 탐색한다(2026-07-09). 없으면 scan_pose 한 지점만 보는 기존 동작.
            scan_pose_b = POSITION_COORDS.get(f"{source}_b")
            pick_res = self._send_pick(obj, scan_pose, scan_pose_b)
            if pick_res is None or not pick_res.success:
                reason = pick_res.message if pick_res else "no result"
                self.get_logger().warn(f"'{obj}' Pick 실패({reason}), 이번 물체 건너뜀")
                if self._safety_state != SafetyState.RUN:
                    # ESTOP 래치가 아직 안 풀렸으면 남은 물체를 계속 시도해봐야
                    # 전부 즉시 emergency stop으로 실패할 뿐이다. 여기서 멈추고
                    # RESUME 음성이 올 때까지 기다린다 (home 복귀도 시도 안 함 —
                    # 그것도 바로 emergency stop으로 실패하므로).
                    self.get_logger().warn("안전 정지 상태 감지 — 남은 물체 처리 중단, RESUME 대기")
                    self._say("정지했습니다")
                    self._publish_task_status(
                        task_id, "FAILED", step_index=step_index, step_total=step_total,
                        title=f"'{obj}' 처리 중 안전 정지", detail=reason, phase="estopped",
                    )
                    return
                self._say("물체를 잡지 못해 건너뜁니다")
                continue

            self._say("잡았습니다")
            self._publish_task_status(
                task_id, "RUNNING", step_index=step_index, step_total=step_total,
                title=f"'{obj}'를 '{target}'로 이동", phase="placing",
            )

            place_res = self._send_place(target_pose)
            if place_res is None or not place_res.success:
                reason = place_res.message if place_res else "no result"
                self.get_logger().warn(f"'{obj}' Place 실패({reason})")
                if self._safety_state != SafetyState.RUN:
                    self.get_logger().warn("안전 정지 상태 감지 — 남은 물체 처리 중단, RESUME 대기")
                    self._say("정지했습니다")
                    self._publish_task_status(
                        task_id, "FAILED", step_index=step_index, step_total=step_total,
                        title=f"'{obj}' 처리 중 안전 정지", detail=reason, phase="estopped",
                    )
                    return
            else:
                self._say("놓았습니다")

        # 2026-07-09 버그 수정: 이전엔 이 마지막 home 복귀만 결과를 확인 안 하고
        # 바로 "완료했습니다"를 말했다 — 복귀 도중 손 감지/ESTOP이 걸려도 사용자는
        # 정상 종료로 안내받았다. pick/place와 동일하게 결과를 확인한다.
        move_res = self._send_move_to(POSITION_COORDS["home"], "home")
        if move_res is None or not move_res.success:
            reason = move_res.message if move_res else "no result"
            self.get_logger().warn(f"home 복귀 실패({reason})")
            if self._safety_state != SafetyState.RUN:
                self.get_logger().warn("안전 정지 상태 감지 — RESUME 대기")
                self._say("정지했습니다")
            else:
                self._say("복귀에 실패했습니다")
            self._publish_task_status(
                task_id, "FAILED", step_index=step_total, step_total=step_total,
                title="home 복귀 실패", detail=reason, phase="returning_home",
            )
            return
        self._say("작업을 완료했습니다")
        self._publish_task_status(
            task_id, "COMPLETED", step_index=step_total, step_total=step_total,
            title=f"{step_total}개 물체 처리 완료", phase="done",
        )

    # ---- Action / service 헬퍼 ----
    def _reset_safety(self):
        if not self.safety_reset_client.service_is_ready():
            if not self.safety_reset_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("/safety/reset 서비스 없음 — 리셋 생략")
                return
        self._call_service(self.safety_reset_client, Trigger.Request(), timeout_sec=2.0)

    def _update_world_map(self):
        # 2026-07-11: 실패 경로 3곳 전부 home 복귀 추가 - 원래는 성공했을 때만
        # 복귀시켰는데(스캔 그리드 마지막 pose에서 그냥 끝나는 문제 때문에 추가된
        # 로직), 타임아웃/스캔 실패로 끝나도 로봇이 스캔 중이던 임의의 포즈에
        # 그대로 남아있어 다음 명령이나 사람 접근 시 위험할 수 있었다. 서비스가
        # 아예 준비 안 된 경우(스캔을 시작도 안 함)까지 포함해 세 분기 모두
        # 일관되게 home으로 보낸다(성공 분기와 동일하게 결과 확인 없이 fire-and-forget).
        if not self.world_map_client.service_is_ready():
            if not self.world_map_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("update_world_map 서비스 없음 — 생략")
                self._say("월드맵 서비스에 연결할 수 없습니다")
                self._send_move_to(POSITION_COORDS["home"], "home")
                return

        self._say("월드맵 스캔을 시작합니다")
        result = self._call_service(
            self.world_map_client, Trigger.Request(), timeout_sec=WORLD_MAP_SCAN_TIMEOUT_SEC
        )

        if result is None:
            self.get_logger().error(f"world map 스캔 타임아웃({WORLD_MAP_SCAN_TIMEOUT_SEC}s)")
            self._say("월드맵 스캔이 시간 초과되었습니다")
            self._send_move_to(POSITION_COORDS["home"], "home")
            return

        if not result.success:
            self.get_logger().warn(f"world map 스캔 실패: {result.message}")
            self._say("월드맵 스캔에 실패했습니다")
            self._send_move_to(POSITION_COORDS["home"], "home")
            return

        self.get_logger().info(f"world map 스캔 완료: {result.message}")
        # 2026-07-10: run_scan()이 스캔 그리드 마지막 pose(코너 근처)에서 그냥 끝나서
        # home으로 안 돌아오는 문제 - pick/place 루프 종료 시와 동일하게 여기서도 명시적으로 복귀시킨다.
        self._send_move_to(POSITION_COORDS["home"], "home")
        self._say("월드맵 스캔을 완료했습니다")

    def _send_move_to(self, pose, label=""):
        goal = MoveTo.Goal()
        goal.pose = [float(v) for v in pose]
        goal.label = label
        return self._send_goal_and_wait(self.moveto_client, goal, "MoveTo")

    def _send_pick(self, obj, scan_pose, scan_pose_b=None):
        goal = Pick.Goal()
        goal.object_label = obj
        goal.scan_pose = [float(v) for v in scan_pose]
        goal.scan_pose_b = [float(v) for v in scan_pose_b] if scan_pose_b else []
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
        wrapped = self._wait_future(result_future, timeout_sec=ACTION_RESULT_TIMEOUT_SEC)
        if wrapped is None:
            self.get_logger().error(
                f"{name} 결과 대기 타임아웃({ACTION_RESULT_TIMEOUT_SEC}s) — goal 취소 시도"
            )
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_future(cancel_future, timeout_sec=5.0)
            return None
        return wrapped.result

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

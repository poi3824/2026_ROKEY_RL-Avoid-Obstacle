# 로봇 제어 전담 노드 (개편 v2에서 robot_action_node로부터 분리).
#
# 이전엔 robot_action_node 하나가 오케스트레이션 + 로봇 제어를 다 했다. 개편 후
# 두뇌(brain_node)는 상태머신/시퀀싱만 맡고, 실제 로봇 제어(dsr_node, MotionExecutor,
# 좌표변환, 멀티스레드 폴링)는 전부 이 노드로 옮겼다. 두뇌와는 Action으로만 통신한다:
#   MoveTo : 주어진 pose로 이동
#   Pick   : scan_pose에서 물체 detect → 집기(재시도 포함)
#   Place  : target_pose로 이동 → depth로 내려놓기
#
# 멀티스레드 구조는 없앤 게 아니라 그대로 이관했다:
#   • 모든 콜백을 하나의 ReentrantCallbackGroup + MultiThreadedExecutor로 돌린다.
#     Action 실행 콜백이 blocking(amovel 폴링)인 동안에도 result/cancel 응답과
#     /safety/state 구독, get_3d_position 응답이 병렬로 처리돼야 하기 때문이다.
#     (MutuallyExclusive로 묶으면 execute가 도는 동안 result 응답이 막혀 클라이언트가
#      결과를 못 받는다.) 손 감지/정지 상태는 여기서 estop_event/hand_pause_event에
#     실시간 반영되고, move_linear()의 폴링 루프가 그 이벤트를 본다.
#   • dsr_lock — amovel/check_motion/get_current_posx/stop 등 dsr_node를 건드리는
#     호출을 직렬화(global executor 재진입 방지).
import os
import sys
import threading
import time
import traceback

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import DR_init

from std_msgs.msg import Bool
from od_msg.srv import SrvDepthPosition, SrvVisibilityCheck
from dsr_msgs2.srv import MoveStop
from ament_index_python.packages import get_package_share_directory

from robot_interfaces.action import MoveTo, Pick, Place
from robot_interfaces.msg import SafetyState

from my_robot_pkg.gripper import RG2Gripper
from my_robot_pkg.motion_executor import MotionExecutor, EmergencyStop
from my_robot_pkg.pick_logger import PickLogger

PACKAGE_PATH = get_package_share_directory("my_robot_pkg")

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

DEPTH_OFFSET = 5
MIN_DEPTH = 40.0

# get_3d_position 응답 대기 타임아웃.
# 2026-07-08: "seg 추론이 프레임당 ~1초"라는 가정으로 12초까지 늘렸었는데,
# 실측해보니 프레임당 ~0.1초라 FUSION_FRAME_COUNT=8장을 배치 추론해도 1초
# 안팎이다. hand 감지 타이머와의 경합, 카메라/네트워크 지연 여유를 감안해
# 6초로 줄인다.
GET_TARGET_TIMEOUT = 6.0
# 2026-07-09: /check_visibility는 단일 프레임(~0.1초)만 보는 가벼운 경로라
# GET_TARGET_TIMEOUT보다 훨씬 짧게 잡는다 — 스윕 폴링 루프 안에서 반복 호출되므로
# 오래 걸리면 스윕 반응성 자체가 떨어진다.
CHECK_VISIBILITY_TIMEOUT = 2.0
GET_SURFACE_Z_SAMPLES = 5  # 팔이 settle 중이거나 depth가 순간적으로 튀는 프레임을 걸러내기 위한 샘플 수
GET_SURFACE_Z_SAMPLE_INTERVAL = 0.1  # 샘플 사이 간격(초)

# 세그멘테이션 기반 grasp yaw 계산용 캘리브레이션 상수 (robot_action_node에서 이관).
# 카메라와 그리퍼가 손목에 함께 고정돼 회전하므로, 그리퍼 손가락이 닫히는 축은
# 이미지 평면 기준 항상 같은 각도(GRASP_AXIS_IMG_ANGLE_DEG)에 보인다.
# 2026-07-09: 실제 장비로 캘리브레이션 완료 — 물체를 그리퍼 축에 맞춰 놓고 읽은
# 각도가 0도(GRASP_AXIS_IMG_ANGLE_DEG), wrist를 +로 돌렸을 때 angle_deg도 +로
# 움직이는 것 확인(GRASP_ANGLE_SIGN). 둘 다 기존 placeholder 값과 우연히 일치.
GRASP_AXIS_IMG_ANGLE_DEG = 0.0
GRASP_ANGLE_SIGN = 1.0


def compute_grasp_c(current_c, angle_deg):
    """마스크 짧은 변의 이미지 각도(angle_deg)로부터 새 wrist yaw(C)를 계산한다.

    그리퍼가 대칭(핑거 2개)이라 delta를 [-90, 90)로 정규화해 항상 최소 회전만 적용.
    """
    delta = GRASP_ANGLE_SIGN * (angle_deg - GRASP_AXIS_IMG_ANGLE_DEG)
    delta = ((delta + 90.0) % 180.0) - 90.0
    return current_c + delta


# ---- DSR / dsr_node 부트스트랩 (robot_action_node와 동일 패턴) ----
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import (
        amovel, movej, mwait, get_current_posx, posx, check_motion,
        amovej, get_current_posj, DR_BASE,
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

gripper = RG2Gripper(TOOLCHARGER_IP, TOOLCHARGER_PORT)
stop_client = dsr_node.create_client(MoveStop, "motion/move_stop")

# move_linear()가 amovel + check_motion 폴링 도중 확인하는 플래그.
# 이제 세팅/해제는 /safety/state 구독 콜백(_on_safety_state)에서 이뤄진다.
estop_event = threading.Event()
hand_pause_event = threading.Event()

# amovel/movej/mwait/check_motion/stop/get_current_posx 전부 dsr_node(global executor)를
# spin하므로, 동시 호출을 막기 위해 이 락으로 직렬화한다.
dsr_lock = threading.Lock()


def stop(stop_mode=1):
    """motion/move_stop을 호출한다. move_linear가 EmergencyStop 처리 시 사용."""
    req = MoveStop.Request()
    req.stop_mode = stop_mode
    with dsr_lock:
        future = stop_client.call_async(req)
        rclpy.spin_until_future_complete(dsr_node, future, timeout_sec=1.0)
    return future.result()


class MotionNode(Node):
    def __init__(self):
        super().__init__("motion_node")

        self.declare_parameter("grip_min_width_mm", 30.0)
        grip_min_width = self.get_parameter("grip_min_width_mm").value

        self.pick_logger = PickLogger()

        # Action 서버의 goal/result/cancel 서비스가 blocking execute 콜백과 "동시에"
        # 처리돼야 클라이언트(brain)가 결과를 받는다. MutuallyExclusive로 묶으면
        # execute가 도는 동안 같은 그룹인 result 응답 서비스가 막혀서 brain이 결과를
        # 영영 못 받고 robot_init의 MoveTo home에서 멈춘다(→ 이후 명령을 못 받던 원인).
        # 그래서 전부 Reentrant + MultiThreadedExecutor로 돌린다. brain은 goal을
        # 순차로만(결과 받고 다음) 보내므로 동시 모션 위험은 없다.
        self.cbg = ReentrantCallbackGroup()

        # 세 액션 서버(MoveTo/Pick/Place)가 물리적으로 로봇 하나를 공유한다.
        # ReentrantCallbackGroup은 goal/result/cancel "서비스"가 execute_callback
        # 실행 중에도 응답할 수 있게 해줄 뿐이다 — execute_callback 자체는
        # executor.create_task()로 콜백그룹과 무관하게 스케줄되므로, 콜백그룹
        # 설정만으로는 두 goal의 execute_callback이 동시에 도는 걸 막지 못한다
        # (brain이 항상 순차로만 보내서 지금까진 안 터졌을 뿐). 그래서 goal
        # 수락 여부를 결정하는 goal_callback에서 직접 "이미 실행 중이면 거부"를
        # 강제한다.
        self._goal_lock = threading.Lock()
        self._goal_active = False

        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position", callback_group=self.cbg
        )
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_3d_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        # 2026-07-09: 스캔 스윕(sweep_to_detect) 중 반복 호출하는 가벼운 가시성 체크.
        self.check_visibility_client = self.create_client(
            SrvVisibilityCheck, "/check_visibility", callback_group=self.cbg
        )
        while not self.check_visibility_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for check_visibility service...")
        self.check_visibility_request = SrvVisibilityCheck.Request()

        # /safety/state 구독 — 발행자(safety_monitor)와 QoS(transient_local/reliable) 일치.
        safety_qos = QoSProfile(depth=1)
        safety_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        safety_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            SafetyState, "/safety/state", self._on_safety_state, safety_qos,
            callback_group=self.cbg,
        )

        self.motion = MotionExecutor(
            amovel, movej, mwait, gripper, VELOCITY, ACC, stop,
            self.get_surface_z, self.get_target_pos, grip_min_width, self.pick_logger,
            check_motion, estop_event, hand_pause_event, dsr_lock,
            # 2026-07-10: move_via_rl()/move_joint() 전용 (dsr_policy_path 통합).
            # get_current_posx는 ref=DR_BASE로 고정해서 넘긴다 — dsr_policy_path.py가
            # "클라이언트 라이브러리의 _g_coord 기본값에 의존하지 말라"고 명시적으로
            # 강조한 부분(세션 간 값이 남아있을 수 있음)을 그대로 지킨다.
            amovej=amovej, get_current_posj=get_current_posj,
            get_current_posx=lambda: get_current_posx(ref=DR_BASE),
        )

        # Action servers
        self._moveto_srv = ActionServer(
            self, MoveTo, "motion/move_to", self._execute_move_to,
            callback_group=self.cbg, cancel_callback=self._on_cancel,
            goal_callback=self._on_goal,
        )
        self._pick_srv = ActionServer(
            self, Pick, "motion/pick", self._execute_pick,
            callback_group=self.cbg, cancel_callback=self._on_cancel,
            goal_callback=self._on_goal,
        )
        self._place_srv = ActionServer(
            self, Place, "motion/place", self._execute_place,
            callback_group=self.cbg, cancel_callback=self._on_cancel,
            goal_callback=self._on_goal,
        )

        self.get_logger().info("motion_node 준비 완료 (MoveTo / Pick / Place)")

    # ---- 안전 상태 반영 ----
    def _on_safety_state(self, msg):
        if msg.state == SafetyState.ESTOP:
            estop_event.set()
        elif msg.state == SafetyState.PAUSE:
            hand_pause_event.set()
        else:  # RUN
            estop_event.clear()
            hand_pause_event.clear()

    def _on_cancel(self, goal_handle):
        # 취소 요청도 응급 정지에 준해 처리한다(진행 중 move_linear가 멈추도록).
        estop_event.set()
        return CancelResponse.ACCEPT

    def _on_goal(self, goal_request):
        """MoveTo/Pick/Place 3개가 공유하는 goal 수락 게이트.

        로봇/그리퍼는 물리적으로 하나뿐이라 두 goal의 execute_callback이 동시에
        도는 걸 절대 허용하면 안 된다(그리퍼는 Modbus TCP 소켓 하나를 공유하는데,
        요청-응답에 순서 검증이 없어 두 스레드가 동시에 쓰면 응답이 뒤바뀌어도
        코드가 알아챌 방법이 없다). 이미 실행 중인 goal이 있으면 무조건 거부한다.
        """
        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warn("이미 실행 중인 goal이 있어 새 goal을 거부함")
                return GoalResponse.REJECT
            self._goal_active = True
            return GoalResponse.ACCEPT

    def _release_goal_slot(self):
        with self._goal_lock:
            self._goal_active = False

    def _safe_terminate(self, goal_handle):
        """未처리 예외로 execute 콜백이 죽을 때 goal을 안전하게 종료 상태로 옮긴다.

        succeed()/abort()/canceled()는 goal이 이미 다른 상태(예: 취소 처리 중)면
        rclpy가 잘못된 상태 전이로 보고 예외를 또 던진다. 여기서 그 예외까지
        잡아야 브레인의 get_result_async()가 영원히 기다리는 걸 막을 수 있다.
        """
        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif goal_handle.is_active:
                goal_handle.abort()
        except Exception as e:
            self.get_logger().error(f"goal 상태 정리 중 추가 오류(무시): {e}")

    # ---- Action 실행 콜백 ----
    # 각 _execute_*는 _goal_active를 반드시 해제해야 하므로(안 그러면 로봇이
    # 영구적으로 새 goal을 못 받음) 실제 로직은 _do_*로 옮기고 try/finally로 감싼다.
    def _execute_move_to(self, goal_handle):
        try:
            return self._do_move_to(goal_handle)
        finally:
            self._release_goal_slot()

    def _do_move_to(self, goal_handle):
        pose = list(goal_handle.request.pose)
        label = goal_handle.request.label
        self.get_logger().info(f"[MoveTo] {label or ''} -> {pose}")

        fb = MoveTo.Feedback()
        fb.phase = "moving"
        goal_handle.publish_feedback(fb)

        result = MoveTo.Result()
        try:
            if label == "home":
                # home은 대기 상태라 그리퍼가 열려 있어야 한다. 이전 pick이
                # 실패로 끝나거나 중간에 중단돼 그리퍼가 닫힌 채 남아있을 수
                # 있어서, home으로 갈 때는 항상 그리퍼를 열어준다.
                self.motion.go_home(pose)
            else:
                self.motion.move_linear(pose)
        except EmergencyStop:
            goal_handle.abort()
            result.success = False
            result.message = "emergency stop"
            return result
        except Exception as e:
            self.get_logger().error(f"[MoveTo] 처리 중 예외 발생, goal 중단: {e}\n{traceback.format_exc()}")
            self._safe_terminate(goal_handle)
            result.success = False
            result.message = f"internal error: {e}"
            return result

        goal_handle.succeed()
        result.success = True
        result.message = "done"
        return result

    def _execute_pick(self, goal_handle):
        try:
            return self._do_pick(goal_handle)
        finally:
            self._release_goal_slot()

    def _do_pick(self, goal_handle):
        obj = goal_handle.request.object_label
        scan_pose = list(goal_handle.request.scan_pose)
        scan_pose_b = list(goal_handle.request.scan_pose_b)
        self.get_logger().info(f"[Pick] object={obj} scan_pose={scan_pose} scan_pose_b={scan_pose_b}")

        result = Pick.Result()
        fb = Pick.Feedback()

        def send_feedback(phase, attempt=0):
            fb.phase = phase
            fb.attempt = attempt
            goal_handle.publish_feedback(fb)

        try:
            send_feedback("scanning", 0)
            # 2026-07-09: scan_pose_b가 있으면 scan_pose->scan_pose_b로 스윕하며 물체가
            # 온전히(잘리지 않고) 보일 때까지 탐색한다 — 고정 스캔 위치 하나로는 물체가
            # 프레임 가장자리에 걸려 반만 보여서 grasp 각도가 틀리는 경우가 있었다.
            # scan_pose_b가 없으면(하위 호환) 기존처럼 scan_pose 한 지점만 본다.
            if scan_pose_b:
                found = self.motion.sweep_to_detect(
                    scan_pose, scan_pose_b, lambda: self._check_visible(obj),
                )
                if not found:
                    goal_handle.abort()
                    result.success = False
                    result.picked_pose = []
                    result.message = f"스캔 스윕 실패: {obj}"
                    return result
            else:
                self.motion.move_to_scan(scan_pose)

            source_pos = self.get_target_pos(obj)
            if source_pos is None:
                goal_handle.abort()
                result.success = False
                result.picked_pose = []
                result.message = f"detect 실패: {obj}"
                return result

            send_feedback("detected", 0)

            success = self.motion.pick(source_pos, obj, feedback_cb=send_feedback)
        except EmergencyStop:
            goal_handle.abort()
            result.success = False
            result.picked_pose = []
            result.message = "emergency stop"
            return result
        except Exception as e:
            self.get_logger().error(f"[Pick] 처리 중 예외 발생, goal 중단: {e}\n{traceback.format_exc()}")
            self._safe_terminate(goal_handle)
            result.success = False
            result.picked_pose = []
            result.message = f"internal error: {e}"
            return result

        result.success = bool(success)
        result.picked_pose = source_pos if success else []
        result.message = "gripped" if success else "grip 실패"
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _execute_place(self, goal_handle):
        try:
            return self._do_place(goal_handle)
        finally:
            self._release_goal_slot()

    def _do_place(self, goal_handle):
        target_pose = list(goal_handle.request.target_pose)
        self.get_logger().info(f"[Place] target_pose={target_pose}")

        fb = Place.Feedback()
        fb.phase = "moving"
        goal_handle.publish_feedback(fb)

        def send_feedback(phase):
            fb.phase = phase
            goal_handle.publish_feedback(fb)

        result = Place.Result()
        try:
            # 2026-07-10: RL로 target 상단까지 이동 + 정렬한 뒤, depth 기반 하강/
            # 그리퍼 개방/후퇴까지 마친다(motion_executor.MotionExecutor.place_via_rl()
            # docstring 참고). RL이 목표에 수렴하지 못하면 하강 자체를 시도하지 않는다.
            placed = self.motion.place_via_rl(target_pose, feedback_cb=send_feedback)
        except EmergencyStop:
            goal_handle.abort()
            result.success = False
            result.message = "emergency stop"
            return result
        except Exception as e:
            self.get_logger().error(f"[Place] 처리 중 예외 발생, goal 중단: {e}\n{traceback.format_exc()}")
            self._safe_terminate(goal_handle)
            result.success = False
            result.message = f"internal error: {e}"
            return result

        result.success = bool(placed)
        result.message = "placed" if placed else "RL reach 실패 (목표 미도달)"
        if placed:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ---- 검출 / 좌표변환 (robot_action_node에서 이관) ----
    def _wait_for(self, future, timeout_sec=None):
        """future 완료까지 폴링만 한다 — spin은 MultiThreadedExecutor가 담당."""
        start = time.time()
        while not future.done():
            if timeout_sec is not None and (time.time() - start) > timeout_sec:
                return None
            time.sleep(0.01)
        return future.result()

    def _check_visible(self, label):
        """스캔 스윕 중 반복 호출되는 가벼운 단일 프레임 가시성 체크(/check_visibility).

        get_target_pos(8프레임 융합, ~1초)와 달리 프레임 1장만 보는 빠른 경로라
        sweep_to_detect의 폴링 루프 안에서 반복 호출해도 부담이 적다.
        """
        self.check_visibility_request.target = label
        future = self.check_visibility_client.call_async(self.check_visibility_request)
        result = self._wait_for(future, timeout_sec=CHECK_VISIBILITY_TIMEOUT)
        return bool(result is not None and result.visible)

    def get_target_pos(self, label):
        """realsense로 label의 depth를 찍어 베이스 좌표 + grasp orientation을 반환한다."""
        self.get_position_request.target = label
        self.get_logger().info(f"call get_3d_position service with target={label}")
        future = self.get_position_client.call_async(self.get_position_request)
        result = self._wait_for(future, timeout_sec=GET_TARGET_TIMEOUT)

        if result is None:
            return None

        camera_coords = result.depth_position.tolist()
        if sum(camera_coords) == 0:
            self.get_logger().warn(f"No detection for '{label}'")
            return None

        gripper2cam_path = os.path.join(PACKAGE_PATH, "resource", "T_gripper2camera.npy")
        with dsr_lock:  # get_current_posx도 dsr_node를 spin하므로 직렬화
            robot_posx = get_current_posx()[0]
        base_coords = self.transform_to_base(camera_coords, gripper2cam_path, robot_posx)

        if base_coords[2] and sum(base_coords) != 0:
            base_coords[2] += DEPTH_OFFSET
            base_coords[2] = max(base_coords[2], MIN_DEPTH)

        grasp_c = compute_grasp_c(robot_posx[5], result.angle_deg)
        orientation = list(robot_posx[3:5]) + [grasp_c]
        return list(base_coords[:3]) + orientation

    def get_surface_z(self):
        """카메라 중앙 픽셀 depth를 여러 번 읽어 base z의 median을 반환한다 (YOLO 미사용)."""
        zs = []
        for i in range(GET_SURFACE_Z_SAMPLES):
            z = self._read_surface_z_once()
            if z is not None:
                zs.append(z)
            if i < GET_SURFACE_Z_SAMPLES - 1:
                time.sleep(GET_SURFACE_Z_SAMPLE_INTERVAL)

        if not zs:
            self.get_logger().warn("get_surface_z: 유효한 depth 샘플이 하나도 없음")
            return None

        return float(np.median(zs))

    def _read_surface_z_once(self):
        """카메라 중앙 픽셀 depth 1회를 읽어 base z로 변환한다. 실패하면 None."""
        self.get_position_request.target = ""
        future = self.get_position_client.call_async(self.get_position_request)
        result = self._wait_for(future, timeout_sec=GET_TARGET_TIMEOUT)

        if result is None:
            return None

        camera_coords = result.depth_position.tolist()
        if sum(camera_coords) == 0:
            return None

        gripper2cam_path = os.path.join(PACKAGE_PATH, "resource", "T_gripper2camera.npy")
        with dsr_lock:
            robot_posx = get_current_posx()[0]
        base_coords = self.transform_to_base(camera_coords, gripper2cam_path, robot_posx)
        return base_coords[2]

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        """카메라 좌표계 3D 좌표를 로봇 베이스 좌표계로 변환한다."""
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def destroy_node(self):
        self.pick_logger.close()
        super().destroy_node()


def main(args=None):
    node = MotionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

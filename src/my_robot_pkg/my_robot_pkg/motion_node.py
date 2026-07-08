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
from od_msg.srv import SrvDepthPosition
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

# get_3d_position 응답 대기 타임아웃. object_detection이 CPU seg 추론(프레임당 ~1초,
# FUSION_FRAME_COUNT=3장)에 더해 단일 스레드 executor에서 hand 감지 타이머와 경합하므로
# 5초로는 빠듯해 검출이 타임아웃으로 실패했다. 여유를 준다.
GET_TARGET_TIMEOUT = 12.0
GET_SURFACE_Z_SAMPLES = 5  # 팔이 settle 중이거나 depth가 순간적으로 튀는 프레임을 걸러내기 위한 샘플 수
GET_SURFACE_Z_SAMPLE_INTERVAL = 0.1  # 샘플 사이 간격(초)

# 세그멘테이션 기반 grasp yaw 계산용 캘리브레이션 상수 (robot_action_node에서 이관).
# 카메라와 그리퍼가 손목에 함께 고정돼 회전하므로, 그리퍼 손가락이 닫히는 축은
# 이미지 평면 기준 항상 같은 각도(GRASP_AXIS_IMG_ANGLE_DEG)에 보인다 — 실측 필요.
# GRASP_ANGLE_SIGN은 이미지 각도가 +로 늘 때 C를 +/-로 돌려야 하는지 — 실측 필요.
# TODO(hardware calibration): 실제 장비에서 측정해 채운다.
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
    from DSR_ROBOT2 import amovel, movej, mwait, get_current_posx, posx, check_motion
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

        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position", callback_group=self.cbg
        )
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_3d_position service...")
        self.get_position_request = SrvDepthPosition.Request()

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
        )

        # Action servers
        self._moveto_srv = ActionServer(
            self, MoveTo, "motion/move_to", self._execute_move_to,
            callback_group=self.cbg, cancel_callback=self._on_cancel,
        )
        self._pick_srv = ActionServer(
            self, Pick, "motion/pick", self._execute_pick,
            callback_group=self.cbg, cancel_callback=self._on_cancel,
        )
        self._place_srv = ActionServer(
            self, Place, "motion/place", self._execute_place,
            callback_group=self.cbg, cancel_callback=self._on_cancel,
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

    # ---- Action 실행 콜백 ----
    def _execute_move_to(self, goal_handle):
        pose = list(goal_handle.request.pose)
        label = goal_handle.request.label
        self.get_logger().info(f"[MoveTo] {label or ''} -> {pose}")

        fb = MoveTo.Feedback()
        fb.phase = "moving"
        goal_handle.publish_feedback(fb)

        result = MoveTo.Result()
        try:
            self.motion.move_linear(pose)
            self.motion.wait()
        except EmergencyStop:
            goal_handle.abort()
            result.success = False
            result.message = "emergency stop"
            return result

        goal_handle.succeed()
        result.success = True
        result.message = "done"
        return result

    def _execute_pick(self, goal_handle):
        obj = goal_handle.request.object_label
        scan_pose = list(goal_handle.request.scan_pose)
        self.get_logger().info(f"[Pick] object={obj} scan_pose={scan_pose}")

        result = Pick.Result()
        fb = Pick.Feedback()

        try:
            fb.phase = "scanning"; fb.attempt = 0
            goal_handle.publish_feedback(fb)
            self.motion.move_to_scan(scan_pose)

            source_pos = self.get_target_pos(obj)
            if source_pos is None:
                goal_handle.abort()
                result.success = False
                result.picked_pose = []
                result.message = f"detect 실패: {obj}"
                return result

            fb.phase = "detected"
            goal_handle.publish_feedback(fb)

            success = self.motion.pick(source_pos, obj)
        except EmergencyStop:
            goal_handle.abort()
            result.success = False
            result.picked_pose = []
            result.message = "emergency stop"
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
        target_pose = list(goal_handle.request.target_pose)
        self.get_logger().info(f"[Place] target_pose={target_pose}")

        fb = Place.Feedback()
        fb.phase = "moving"
        goal_handle.publish_feedback(fb)

        result = Place.Result()
        try:
            self.motion.place(target_pose)
        except EmergencyStop:
            goal_handle.abort()
            result.success = False
            result.message = "emergency stop"
            return result

        goal_handle.succeed()
        result.success = True
        result.message = "placed"
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

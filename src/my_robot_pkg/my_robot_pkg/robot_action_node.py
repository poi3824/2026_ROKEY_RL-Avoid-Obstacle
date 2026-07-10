# B-2 STT 노드(get_keyword_node)가 제공하는 /get_keyword 서비스를 호출해서
# 음성 인식 결과를 받아오고, 그 결과로 로봇 동작을 수행하는 노드.
#
# 통신 구조:
#   이 노드(client) --- /get_keyword (std_srvs/srv/Trigger) ---> get_keyword_node(server)
#   request  : 없음
#   response : success(bool), message(string, "object / source / target / return_pos" 형식)
import os
import sys
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import DR_init

from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from od_msg.srv import SrvDepthPosition
from dsr_msgs2.srv import MoveStop
from ament_index_python.packages import get_package_share_directory

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


GET_TARGET_TIMEOUT = 5.0
GET_SURFACE_Z_SAMPLES = 5  # 팔이 settle 중이거나 depth가 순간적으로 튀는 프레임을 걸러내기 위한 샘플 수
GET_SURFACE_Z_SAMPLE_INTERVAL = 0.1  # 샘플 사이 간격(초)

# 세그멘테이션 기반 grasp yaw 계산용 캘리브레이션 상수.
# 카메라와 그리퍼가 손목(wrist)에 함께 고정되어 회전하고(B~180도로 항상 수직 하방을
# 봄), C(wrist yaw)만 돌리면 카메라도 같이 돌아간다. 따라서 그리퍼 손가락이 닫히는
# 축은 이미지 평면 기준으로 항상 같은 각도(GRASP_AXIS_IMG_ANGLE_DEG)에 보인다 —
# 이 값은 실제 그리퍼를 카메라 화면에 비춰서 측정해야 하는 하드웨어 캘리브레이션값.
# GRASP_ANGLE_SIGN은 이미지 각도가 +로 늘 때 C를 +로 돌려야 하는지 -로 돌려야
# 하는지(카메라 장착 방향에 따라 다름) — 실제 로봇에서 한 번 테스트해서 결정한다.
# TODO(hardware calibration): 아래 두 값을 실제 장비에서 측정해 채운다.
GRASP_AXIS_IMG_ANGLE_DEG = 0.0
GRASP_ANGLE_SIGN = 1.0

# POSITION_COORDS = {
#     "home": [417.61, -0.76, 477.45, 174.25, 179.99, -7.65],
#     "scan": [603.65, 117.06, 466.15, 96.74, -179.75, -85.08],
#     "target1": [200.0, 100.0, 466.058, 138.332, -179.994, -43.561],
#     "target2": [199.91, 0.066, 466.172, 177.529, 179.942, -2.8],
#     "target3": [199.93, 100.092, 466.217, 174.166, 179.947, -6.174],
# }
POSITION_COORDS = {
    "home": [417.61, -0.76, 477.45, 174.25, 179.99, -7.65],
    "scan": [603.65, 117.06, 466.15, 96.74, -179.75, -85.08],
    "target1": [200.0, 100.0, 271.058, 138.332, -179.994, -43.561],
    "target2": [199.91, 0.066, 271.172, 177.529, 179.942, -2.8],
    "target3": [199.93, 100.092, 271.217, 174.166, 179.947, -6.174],
}
# tcp Z 위치 195.0 씩 빼기

def compute_grasp_c(current_c, angle_deg):
    """마스크 짧은 변의 이미지 각도(angle_deg)로부터 새 wrist yaw(C)를 계산한다.

    GRASP_AXIS_IMG_ANGLE_DEG 주석 참고: 물체가 이미지에서 기준각 대비 angle_deg만큼
    돌아가 있으면, C를 그만큼 돌려서 그리퍼 손가락 축과 물체의 짧은 변을 맞춘다.
    그리퍼가 대칭(핑거 2개)이라 delta가 90도를 넘어가면 반대쪽(긴 변)을 잡게 되므로
    [-90, 90) 범위로 정규화해 항상 최소 회전만 적용한다.
    """
    delta = GRASP_ANGLE_SIGN * (angle_deg - GRASP_AXIS_IMG_ANGLE_DEG)
    delta = ((delta + 90.0) % 180.0) - 90.0
    return current_c + delta


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

home = posx(POSITION_COORDS["home"])
gripper = RG2Gripper(TOOLCHARGER_IP, TOOLCHARGER_PORT)

stop_client = dsr_node.create_client(MoveStop, "motion/move_stop")

# move_linear()가 amovel + check_motion 폴링 도중 이 플래그를 확인한다.
# 세팅되는 곳은 trigger_emergency_stop() 하나뿐이고(음성 STOP, /emergency_stop
# 서비스 모두 이걸 거쳐간다), heard()가 새 명령을 시작할 때 clear한다.
estop_event = threading.Event()

# 2026-07-07: amovel/movej/mwait/check_motion/stop/get_current_posx 전부
# dsr_node(DR_init.__dsr__node = g_node)를 spin한다. 예전엔 hand 감지를 이
# 프로세스 안 백그라운드 스레드에서 get_target_pos -> get_current_posx로 했는데,
# 그게 move_linear의 check_motion 폴링과 동시에 dsr_node를 spin하면서 충돌한
# 적이 있어(손을 넣어도 반응이 없던 원인) 이 락을 추가했다. hand 감지는 이제
# object_detection_node의 /hand_detected 토픽 구독으로 바뀌어 dsr_node를 아예
# 안 건드리지만, dsr_node를 건드리는 지점들을 계속 안전하게 직렬화하려고 남겨둔다.
dsr_lock = threading.Lock()


def stop(stop_mode=1):
    req = MoveStop.Request()
    req.stop_mode = stop_mode
    with dsr_lock:
        future = stop_client.call_async(req)
        rclpy.spin_until_future_complete(dsr_node, future, timeout_sec=1.0)
    return future.result()


def emergency_stop():
    """stop_mode=0(QSTOP_STO)으로 서보 토크를 즉시 끊는다.

    일반 stop()의 기본값인 QSTOP(1, 감속 후 정지)보다 더 즉각적이고
    안전 등급이 높은 정지다. estop_event는 건드리지 않는다 — 그건
    trigger_emergency_stop()이 담당한다.
    """
    return stop(stop_mode=0)


def trigger_emergency_stop():
    """음성 STOP 키워드, /emergency_stop 서비스, (나중에) 버튼이 전부 이 함수 하나를 호출한다.

    emergency_stop()으로 실제 로봇을 멈추는 것과 별개로, estop_event를 세팅해서
    move_linear()의 폴링 루프가 진행 중인 이동을 중단하고 EmergencyStop을
    올리게 만든다.
    """
    emergency_stop()
    estop_event.set()


def _handle_emergency_stop_request(request, response):
    trigger_emergency_stop()
    response.success = True
    response.message = "emergency stop triggered"
    return response


# dsr_node는 RobotActionNode(아래)와 별개의 노드다. 버튼을 누르는 스크립트나
# 나중에 붙일 다른 노드가 `ros2 service call /emergency_stop std_srvs/srv/Trigger "{}"`로
# 호출하면 된다 — robot_action_node가 pick/place로 바쁘게 blocking 중이어도
# 이 서비스만 별도로 호출할 수 있게 하려는 목적.
#
# motion_executor.move_linear()가 movel(sync) 대신 amovel(async)+check_motion()
# 폴링을 쓰도록 바꿔서, 로봇이 움직이는 도중에는 폴링 간격(0.05s)마다 dsr_node가
# spin되어 이 서비스 요청도 그때 처리된다. 다만 로봇이 완전히 정지해 다음 음성
# 명령을 기다리는 동안(=애초에 멈춰있는 동안)에는 dsr_node를 spin하는 호출이
# 없으므로, 그 사이 버튼을 눌러도 다음 movel/mwait/stop 호출 전까지는 처리가
# 지연될 수 있다 — 다만 그 시점엔 로봇이 이미 안 움직이고 있으니 안전상 문제는
# 아니다.
emergency_stop_srv = dsr_node.create_service(
    Trigger, "emergency_stop", _handle_emergency_stop_request
)


class RobotActionNode(Node):
    def __init__(self):
        super().__init__("robot_action_node")

        # 2026-07-07: 이전엔 호출마다 rclpy.spin_until_future_complete(self, ...)로
        # 그때그때 임시 executor를 만들어 썼는데, 손 감지 기능으로 호출 빈도가
        # 크게 늘면서 "wait set index too big" 에러가 반복적으로 났다.
        #
        # 처음엔 rclpy.spin(self)(executor 인자 없이)로 상시 스레드를 만들었는데,
        # rclpy 소스 확인 결과 spin()/spin_until_future_complete()가 executor를
        # 안 넘기면 프로세스 전체가 공유하는 get_global_executor() 하나를 쓴다.
        # 그런데 DSR_ROBOT2 벤더 코드(amovel/mwait/check_motion/get_current_posx)와
        # 우리 stop()도 dsr_node에 대해 전부 이 global executor를 쓰기 때문에,
        # 내 상시 spin 스레드와 메인 스레드의 dsr_node 호출이 같은 executor를
        # 두고 충돌해서 "generator already executing"이 났다.
        #
        # 그래서 self 전용 SingleThreadedExecutor를 따로 만들어서 그것만
        # spin한다 — global executor(= dsr_node 쪽)는 이제 전혀 안 건드린다.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self.get_keyword_client = self.create_client(Trigger, "get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_3d_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        # 통 크기가 바뀌면 코드/재빌드 없이
        # `ros2 run my_robot_pkg robot_action_node --ros-args -p grip_min_width_mm:=25.0`
        # 처럼 파라미터만 바꿔서 대응한다.
        self.declare_parameter("grip_min_width_mm", 30.0)
        grip_min_width = self.get_parameter("grip_min_width_mm").value

        self.pick_logger = PickLogger()

        # 2026-07-07: hand 감지는 이제 object_detection_node가 /hand_detected
        # (Bool)로 독립적으로(5Hz, 단일 프레임) 발행한다 — pick이 쓰는
        # get_3d_position 서비스/get_position_request와 자원을 안 나눠 쓰므로
        # 여기서는 구독만 하면 된다. 예전엔 이 프로세스 안에서 백그라운드
        # 스레드가 get_target_pos("hand")를 반복 호출했는데, 그게 pick 본연의
        # 카메라 호출과 계속 경합해서 전체가 눈에 띄게 느려졌었다.
        self.hand_pause_event = threading.Event()
        self.hand_detected_sub = self.create_subscription(
            Bool, "hand_detected", self._on_hand_detected, 10
        )

        self.motion = MotionExecutor(
            amovel, movej, mwait, gripper, VELOCITY, ACC, stop,
            self.get_surface_z, self.get_target_pos, grip_min_width, self.pick_logger,
            check_motion, estop_event, self.hand_pause_event,
            dsr_lock,
        )

        self.robot_init()

    def _on_hand_detected(self, msg):
        if msg.data:
            if not self.hand_pause_event.is_set():
                self.get_logger().warn("작업 공간에 손 감지, 일시정지")
            self.hand_pause_event.set()
        else:
            self.hand_pause_event.clear()

    def robot_init(self):
        self.get_logger().info("Initializing robot")
        self.motion.go_home(home)

    def run_once(self):
        self.get_logger().info("call get_keyword service")
        result = self.request_keyword()

        if result is None or not result.success:
            message = result.message if result else "no response"
            self.get_logger().warn(f"get_keyword failed: {message}")
            return

        try:
            self.execute_robot_action(result.message)
        except EmergencyStop:
            # 응급 정지 직후에는 return_home()으로 바로 움직이면 안 된다 —
            # emergency_stop()이 QSTOP_STO로 서보 토크 자체를 끊었을 수 있어서,
            # 사람이 확인하고 재활성화하기 전까지는 그대로 멈춰있어야 한다.
            self.get_logger().error("응급 정지로 동작 중단. 다음 명령을 기다립니다.")
        except Exception as e:
            # message 파싱 실패(형식이 안 맞는 LLM 응답 등)나 다른 예외가 나면
            # 여기서 안 잡고 그냥 흘려보내면 main()의 while 루프까지 뚫고 나가서
            # 노드 자체가 죽는다. 그러면 로봇은 죽기 직전 위치(대개 home)에
            # 그대로 멈춰서 더 이상 아무 명령에도 반응하지 않게 된다.
            self.get_logger().error(f"명령 처리 중 오류, 이번 명령은 무시함: {e}")
            self.motion.return_home(home)

    def request_keyword(self):
        """/get_keyword 서비스를 호출하고 응답(Trigger.Response)을 반환한다."""
        future = self.get_keyword_client.call_async(self.get_keyword_request)
        return self._wait_for(future)

    def _wait_for(self, future, timeout_sec=None):
        """future가 끝날 때까지 폴링만 한다 — spin은 __init__에서 시작한 상시
        spin 스레드(self._spin_thread)가 계속 처리하므로 여기서는 직접 spin하지 않는다.
        timeout_sec을 넘기면 그 시간 안에 안 끝날 경우 None을 반환한다.
        """
        start = time.time()
        while not future.done():
            if timeout_sec is not None and (time.time() - start) > timeout_sec:
                return None
            time.sleep(0.01)
        return future.result()

    def execute_robot_action(self, message):
        """get_keyword_node가 보낸 message를 파싱한다.

        message 형식: "object / source / target / return_pos"
        각 구간은 다시 공백으로 구분된 토큰들일 수 있음 (예: "obj_A obj_B").
        """
        obj_part, source_part, target_part, return_part = message.split("/")

        objects = obj_part.split()
        sources = source_part.split()
        targets = target_part.split()
        return_pos = return_part.split()

        self.get_logger().info(
            f"object: {objects}\n source: {sources}\n target: {targets}\n return: {return_pos}"
        )

        if "STOP" in objects:
            self.get_logger().warn("정지 명령 수신: 응급 정지")
            trigger_emergency_stop()
            return objects, sources, targets, return_pos

        self.heard(objects, sources, targets)

        return objects, sources, targets, return_pos

    def heard(self, objects, sources, targets):
        """source 라벨의 고정(스캔) 위치로 먼저 이동해서 카메라로 물체를
        detect한 뒤, 그 좌표로 집고, target 라벨의 고정 좌표로 옮긴다.

        scan_pos:   말한 source 라벨의 고정 좌표. 카메라가 물체를 내려다볼 수
                    있는 위치일 뿐, 물체의 정확한 위치는 아니다.
        source_pos: scan_pos에 도착한 뒤 get_target_pos(obj)로 detect한
                    물체의 실제 좌표.
        target_pos: 말한 target 라벨 그대로 POSITION_COORDS에서 찾은 고정 좌표.
        """
        # 새 명령을 시작하니 이전에 응급 정지가 걸려 있었더라도 여기서 해제한다.
        # (STOP 명령 자체는 이 함수를 타지 않으므로 상관없다.)
        estop_event.clear()

        if not (len(objects) == len(sources) == len(targets)):
            self.get_logger().warn(
                f"물체 및 위치 개수 불일치, 이번 명령은 무시함: "
                f"{len(objects)} / {len(sources)} / {len(targets)}"
            )
            return

        for obj, source, target in zip(objects, sources, targets):
            scan_pos = POSITION_COORDS.get(source)
            target_pos = POSITION_COORDS.get(target)

            if scan_pos is None:
                self.get_logger().warn(f"'{source}' 스캔 위치가 아직 채워지지 않음")
                continue
            if target_pos is None:
                self.get_logger().warn(f"'{target}' 좌표가 아직 채워지지 않음")
                continue

            self.motion.move_to_scan(scan_pos)

            source_pos = self.get_target_pos(obj)
            if source_pos is None:
                self.get_logger().warn(f"'{obj}' 카메라 detect 실패, 건너뜀")
                continue

            self.get_logger().info(f"Moving '{obj}': detected {source_pos} -> {target}")

            if not self.motion.pick(source_pos, obj):
                self.get_logger().warn(f"'{obj}' 파지 실패(grip_detected=0), 이번 물체는 건너뜀")
                continue
            self.motion.place(target_pos)

        self.motion.return_home(home)

    def get_target_pos(self, label):
        """realsense로 label(물체 클래스명)의 depth를 찍어 베이스 좌표를 반환한다.

        /get_3d_position 서비스(object_detection 노드)가 YOLO로 물체를 찾아
        카메라 좌표계 3D 좌표를 주면, 캘리브레이션 행렬(T_gripper2camera.npy)로
        로봇 베이스 좌표계로 변환한다.
        """
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
        with dsr_lock:  # get_current_posx도 dsr_node를 spin하므로 move_linear의 폴링과 직렬화
            robot_posx = get_current_posx()[0]
        base_coords = self.transform_to_base(camera_coords, gripper2cam_path, robot_posx)

        if base_coords[2] and sum(base_coords) != 0:
            base_coords[2] += DEPTH_OFFSET
            base_coords[2] = max(base_coords[2], MIN_DEPTH)

        grasp_c = compute_grasp_c(robot_posx[5], result.angle_deg)
        orientation = list(robot_posx[3:5]) + [grasp_c]
        return list(base_coords[:3]) + orientation

    def get_surface_z(self):
        """카메라 중앙 픽셀의 depth를 여러 번 읽어 base 좌표계 z값의 median을 반환한다 (YOLO 미사용).

        팔이 아직 완전히 settle되지 않았거나 depth가 순간적으로 튀는/끊기는
        프레임 하나 때문에 잘못된 높이로 계산되는 걸 막기 위해, 한 프레임이 아니라
        GET_SURFACE_Z_SAMPLES번 읽어서 그중 유효한 값들의 median을 쓴다.
        하나도 유효한 값이 없으면 None을 반환해서 호출부(motion_executor.pick/place)가
        fallback을 쓰도록 한다.
        """
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


def main(args=None):
    node = RobotActionNode()
    try:
        while rclpy.ok():
            node.run_once()
    finally:
        node.pick_logger.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

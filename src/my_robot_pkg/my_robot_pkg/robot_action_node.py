# B-2 STT 노드(get_keyword_node)가 제공하는 /get_keyword 서비스를 호출해서
# 음성 인식 결과를 받아오고, 그 결과로 로봇 동작을 수행하는 노드.
#
# 통신 구조:
#   이 노드(client) --- /get_keyword (std_srvs/srv/Trigger) ---> get_keyword_node(server)
#   request  : 없음
#   response : success(bool), message(string, "object / source / target / return_pos" 형식)
import os
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
import DR_init

from std_srvs.srv import Trigger
from od_msg.srv import SrvDepthPosition
from dsr_msgs2.srv import MoveStop
from ament_index_python.packages import get_package_share_directory

from my_robot_pkg.gripper import RG2Gripper
from my_robot_pkg.motion_executor import MotionExecutor

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

POSITION_COORDS = {
    "home": [417.61, -0.76, 477.45, 174.25, 179.99, -7.65],
    "scan": [603.65, 117.06, 466.15, 96.74, -179.75, -85.08],
    "target1": [200.0, 100.0, 466.058, 138.332, -179.994, -43.561],
    "target2": [199.91, 0.066, 466.172, 177.529, 179.942, -2.8],
    "target3": [199.93, 100.092, 466.217, 174.166, 179.947, -6.174],
}


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movel, movej, mwait, get_current_posx, posx
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

home = posx(POSITION_COORDS["home"])
gripper = RG2Gripper(TOOLCHARGER_IP, TOOLCHARGER_PORT)

stop_client = dsr_node.create_client(MoveStop, "motion/move_stop")


def stop(stop_mode=1):
    req = MoveStop.Request()
    req.stop_mode = stop_mode
    future = stop_client.call_async(req)
    rclpy.spin_until_future_complete(dsr_node, future, timeout_sec=1.0)
    return future.result()


class RobotActionNode(Node):
    def __init__(self):
        super().__init__("robot_action_node")

        self.get_keyword_client = self.create_client(Trigger, "get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_3d_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.motion = MotionExecutor(movel, movej, mwait, gripper, VELOCITY, ACC, stop, self.get_surface_z)
        self.robot_init()

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
        rclpy.spin_until_future_complete(self, future)
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
            self.get_logger().warn("정지 명령 수신: 즉시 정지")
            self.motion.stop()
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

            if not self.motion.pick(source_pos):
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=GET_TARGET_TIMEOUT)

        result = future.result()
        if result is None:
            return None

        camera_coords = result.depth_position.tolist()
        if sum(camera_coords) == 0:
            self.get_logger().warn(f"No detection for '{label}'")
            return None

        gripper2cam_path = os.path.join(PACKAGE_PATH, "resource", "T_gripper2camera.npy")
        robot_posx = get_current_posx()[0]
        base_coords = self.transform_to_base(camera_coords, gripper2cam_path, robot_posx)

        if base_coords[2] and sum(base_coords) != 0:
            base_coords[2] += DEPTH_OFFSET
            base_coords[2] = max(base_coords[2], MIN_DEPTH)

        return list(base_coords[:3]) + robot_posx[3:]

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
        rclpy.spin_until_future_complete(self, future, timeout_sec=GET_TARGET_TIMEOUT)

        result = future.result()
        if result is None:
            return None

        camera_coords = result.depth_position.tolist()
        if sum(camera_coords) == 0:
            return None

        gripper2cam_path = os.path.join(PACKAGE_PATH, "resource", "T_gripper2camera.npy")
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

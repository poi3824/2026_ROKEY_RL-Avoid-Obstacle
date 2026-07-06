# "World Map Update" 음성 명령으로 트리거되는 1회성 전체 스캔 노드.
#
# pointcloud_node.py(실시간 매 프레임 obstacle_state)와는 별개로, 넓은 영역을
# MoveLine으로 훑으면서 point cloud를 base_link 기준으로 merge한 뒤 DBSCAN으로
# 군집화해서 장애물 위치를 뽑아낸다. cobot_ws의 ~/cobot_scan/world_map_scan_capture_ranged.py
# (지그재그 스캔 경로 + TF 변환)에서 이미 검증된 로직을 그대로 포팅했다.
#
# 애초에 그리드의 각 코너에서 제자리 회전(A/B만 변경)으로 중심을 바라보게
# 만들려 했었는데, 카메라가 flange 원점에서 T_gripper2camera만큼 떨어져 있어서
# 제자리 회전만으로는 원하는 옆면 뷰가 안 나온다는 걸 실제 로봇으로 확인함
# (2026-07-06). 대신 스캔 시작/끝 지점에서만 중심 반대쪽으로 더 물러나면서
# 살짝 내려간 위치로 이동해, 손으로 직접 찾은 자세로 옆면을 찍고 다시 원래
# 그리드로 돌아오는 방식으로 변경했다.
#
# service/topic 콜백(handle_update) 안에서 MoveLine 서비스 호출과 point cloud
# 대기를 위해 spin_until_future_complete/spin_once를 반복 호출해야 하는데,
# 이걸 self(서비스를 제공 중인 바로 그 노드)에 대고 호출하면 재귀적으로 꼬일 수
# 있다. detection.py(ObjectDetectionNode가 서비스 콜백 안에서 자기 자신이 아닌
# 별도 ImgNode를 spin하는 패턴)를 그대로 따라서, MoveLine/point cloud/TF는
# 별도의 ScanWorker 노드에서 처리한다.
import json
import math
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from dsr_msgs2.srv import MoveLine

from obstacle_avoidance_msgs.msg import WorldMapObstacle, WorldMapUpdate

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except Exception:
    OPEN3D_AVAILABLE = False


# =========================
# 스캔 설정 (cobot_scan/world_map_scan_capture_ranged.py 와 동일한 캘리브레이션)
# =========================

POINTCLOUD_TOPIC = "/camera/camera/depth/color/points"
MOVE_LINE_SERVICE = "/dsr01/motion/move_line"
TARGET_FRAME = "base_link"

START_POINT = {"x": 283.81, "y": 370.71}
END_POINT = {"x": 563.54, "y": -271.25}

X_GAP_MM = 140.0
Y_GAP_MM = 140.0

FIXED_Z_MM = 466.13
FIXED_A_DEG = 1.488
FIXED_B_DEG = -180.0
FIXED_C_DEG = -178.73

# 스캔 시작/끝 지점에서 옆면을 찍기 위한 "사이드뷰" 여분 포즈.
# 실제 로봇을 손으로 움직여 찾은 값이라 정밀하지 않음 (2026-07-06) - 실측 후 조정 필요.
SIDE_VIEW_Y_OFFSET_MM = 50.0  # 중심 반대쪽으로 이만큼 더 물러난다
SIDE_VIEW_Z_DROP_MM = 30.0    # FIXED_Z_MM에서 이만큼 낮춘다 (정확한 값 아님, placeholder)
SIDE_VIEW_START_ABC_DEG = (98.7, -154.71, -82.0)
SIDE_VIEW_END_ABC_DEG = (92.3, 142.3, -90.13)

VEL = [20.0, 5.0]
ACC = [20.0, 5.0]

SETTLE_SEC = 0.8
FRAMES_PER_POSE = 1
VOXEL_SIZE_M = 0.005

ROI_MARGIN_MM = 80.0
ROI_Z_MIN_M = -0.05
ROI_Z_MAX_M = 0.60

# DBSCAN 클러스터링 파라미터. 실제 스캔 밀도 보고 튜닝 필요.
CLUSTER_EPS_M = 0.03
CLUSTER_MIN_POINTS = 15

RECORD_DIR = os.path.expanduser("~/cobot_scan")


# =========================
# 스캔 경로 생성 (지그재그 그리드 + 시작/끝 사이드뷰)
# =========================

def make_inclusive_range(start: float, end: float, step_abs: float):
    if step_abs <= 0:
        raise ValueError("step_abs must be positive")

    dist = abs(end - start)
    n = int(math.ceil(dist / step_abs)) + 1

    if n <= 1:
        return np.array([start], dtype=float)

    return np.linspace(start, end, n)


def generate_scan_columns():
    """[(x, [y0, y1, ...]), ...] 형태로 column별 지그재그 순서의 y 리스트를 생성."""
    x_values = make_inclusive_range(START_POINT["x"], END_POINT["x"], X_GAP_MM)
    y_values = make_inclusive_range(START_POINT["y"], END_POINT["y"], Y_GAP_MM)

    columns = []
    for ix, x in enumerate(x_values):
        ys = y_values if ix % 2 == 0 else y_values[::-1]
        columns.append((float(x), [float(y) for y in ys]))

    return columns


def generate_flat_scan_poses():
    """평범한 탑다운 지그재그 그리드 포즈 (틸트 없음)."""
    poses = []
    for x, ys in generate_scan_columns():
        for y in ys:
            poses.append([
                float(x), float(y), float(FIXED_Z_MM),
                float(FIXED_A_DEG), float(FIXED_B_DEG), float(FIXED_C_DEG),
            ])
    return poses


def generate_scan_poses():
    """시작/끝에 사이드뷰 포즈를 덧붙인 전체 스캔 포즈 리스트.

    순서: [시작-사이드뷰] -> [평범한 탑다운 그리드 (첫 포즈=START_POINT)]
         -> [끝-사이드뷰 (그리드 마지막 포즈=END_POINT 다음)]
    """
    start_side = [
        START_POINT["x"], START_POINT["y"] + SIDE_VIEW_Y_OFFSET_MM,
        FIXED_Z_MM - SIDE_VIEW_Z_DROP_MM,
    ] + list(SIDE_VIEW_START_ABC_DEG)

    end_side = [
        END_POINT["x"], END_POINT["y"] - SIDE_VIEW_Y_OFFSET_MM,
        FIXED_Z_MM - SIDE_VIEW_Z_DROP_MM,
    ] + list(SIDE_VIEW_END_ABC_DEG)

    poses = [start_side] + generate_flat_scan_poses() + [end_side]
    return [[float(v) for v in pose] for pose in poses]


# =========================
# TF / point cloud 유틸리티
# =========================

def quaternion_to_matrix(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return np.eye(3)

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def transform_to_matrix(transform_stamped):
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation

    R = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def apply_transform(points_xyz, T):
    if points_xyz.size == 0:
        return points_xyz

    ones = np.ones((points_xyz.shape[0], 1), dtype=np.float64)
    homo = np.hstack([points_xyz.astype(np.float64), ones])
    transformed = (T @ homo.T).T
    return transformed[:, :3]


def get_roi_bounds_m():
    xs = [START_POINT["x"], END_POINT["x"]]
    ys = [START_POINT["y"], END_POINT["y"]]

    x_min_mm = min(xs) - ROI_MARGIN_MM
    x_max_mm = max(xs) + ROI_MARGIN_MM
    y_min_mm = min(ys) - ROI_MARGIN_MM
    y_max_mm = max(ys) + ROI_MARGIN_MM

    return {
        "x_min_m": x_min_mm / 1000.0,
        "x_max_m": x_max_mm / 1000.0,
        "y_min_m": y_min_mm / 1000.0,
        "y_max_m": y_max_mm / 1000.0,
        "z_min_m": ROI_Z_MIN_M,
        "z_max_m": ROI_Z_MAX_M,
    }


def crop_roi(points_base):
    roi = get_roi_bounds_m()
    mask = (
        (points_base[:, 0] >= roi["x_min_m"]) & (points_base[:, 0] <= roi["x_max_m"]) &
        (points_base[:, 1] >= roi["y_min_m"]) & (points_base[:, 1] <= roi["y_max_m"]) &
        (points_base[:, 2] >= roi["z_min_m"]) & (points_base[:, 2] <= roi["z_max_m"])
    )
    return points_base[mask]


def pointcloud_msg_to_xyz(msg):
    points = [
        [p[0], p[1], p[2]]
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ]
    if not points:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(points, dtype=np.float64)


def voxel_downsample(points_xyz, voxel_size_m):
    if not OPEN3D_AVAILABLE or points_xyz.shape[0] == 0 or voxel_size_m <= 0:
        return points_xyz
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz)
    pcd = pcd.voxel_down_sample(voxel_size_m)
    return np.asarray(pcd.points)


def cluster_points(points_xyz, eps=CLUSTER_EPS_M, min_points=CLUSTER_MIN_POINTS):
    """DBSCAN으로 point cloud를 군집화해서 장애물 목록을 만든다.

    반환: [{"id", "centroid": [x,y,z], "radius", "num_points"}, ...]
    """
    if points_xyz.shape[0] == 0:
        return []

    if not OPEN3D_AVAILABLE:
        raise RuntimeError("open3d가 없어서 클러스터링을 할 수 없습니다.")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz)
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points))

    clusters = []
    for label in sorted(set(labels.tolist())):
        if label < 0:
            continue  # noise
        member_points = points_xyz[labels == label]
        centroid = member_points.mean(axis=0)
        radius = float(np.linalg.norm(member_points - centroid, axis=1).max())
        clusters.append({
            "id": int(label),
            "centroid": centroid.tolist(),
            "radius": radius,
            "num_points": int(member_points.shape[0]),
        })

    return clusters


def save_record(merged_points, clusters, poses):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(RECORD_DIR, f"world_map_update_{timestamp}")
    os.makedirs(out_dir, exist_ok=True)

    np.save(os.path.join(out_dir, "merged_base_roi.npy"), merged_points)

    summary = {
        "start_point_xy_mm": START_POINT,
        "end_point_xy_mm": END_POINT,
        "x_gap_mm": X_GAP_MM,
        "y_gap_mm": Y_GAP_MM,
        "side_view_y_offset_mm": SIDE_VIEW_Y_OFFSET_MM,
        "side_view_z_drop_mm": SIDE_VIEW_Z_DROP_MM,
        "side_view_start_abc_deg": SIDE_VIEW_START_ABC_DEG,
        "side_view_end_abc_deg": SIDE_VIEW_END_ABC_DEG,
        "cluster_eps_m": CLUSTER_EPS_M,
        "cluster_min_points": CLUSTER_MIN_POINTS,
        "scan_poses": poses,
        "clusters": clusters,
    }
    with open(os.path.join(out_dir, "world_map_summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    return out_dir


def build_world_map_update_msg(clusters, scan_dir, stamp):
    msg = WorldMapUpdate()
    msg.header.stamp = stamp
    msg.header.frame_id = TARGET_FRAME
    msg.scan_dir = scan_dir

    obstacles = []
    for c in clusters:
        obs = WorldMapObstacle()
        obs.id = c["id"]
        obs.centroid.x, obs.centroid.y, obs.centroid.z = c["centroid"]
        obs.radius = c["radius"]
        obs.num_points = c["num_points"]
        obstacles.append(obs)
    msg.obstacles = obstacles

    return msg


# =========================
# 스캔 워커 (MoveLine + point cloud + TF, 별도 노드)
# =========================

class ScanWorker(Node):
    def __init__(self):
        super().__init__("world_map_scan_worker")

        self.latest_cloud_msg = None
        self.latest_cloud_stamp_key = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cloud_sub = self.create_subscription(
            PointCloud2, POINTCLOUD_TOPIC, self.cloud_callback, 10
        )
        self.move_line_client = self.create_client(MoveLine, MOVE_LINE_SERVICE)

    def cloud_callback(self, msg):
        self.latest_cloud_msg = msg
        self.latest_cloud_stamp_key = (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def wait_for_move_line_service(self, timeout_sec=5.0):
        return self.move_line_client.wait_for_service(timeout_sec=timeout_sec)

    def move_line(self, pose):
        req = MoveLine.Request()
        req.pos = pose
        req.vel = [float(VEL[0]), float(VEL[1])]
        req.acc = [float(ACC[0]), float(ACC[1])]
        req.time = 0.0
        req.radius = 0.0

        if hasattr(req, "ref"):
            req.ref = 0
        if hasattr(req, "mode"):
            req.mode = 0
        if hasattr(req, "blend_type"):
            req.blend_type = 0
        if hasattr(req, "sync_type"):
            req.sync_type = 0

        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            return False

        result = future.result()
        if hasattr(result, "success"):
            return bool(result.success)
        return True

    def wait_for_new_cloud(self, previous_stamp_key, timeout_sec=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_cloud_msg is None:
                continue
            if self.latest_cloud_stamp_key != previous_stamp_key:
                return self.latest_cloud_msg
        raise TimeoutError("point cloud 수신 타임아웃")

    def transform_cloud_to_base(self, msg):
        source_frame = msg.header.frame_id
        try:
            stamp = Time.from_msg(msg.header.stamp)
            tf = self.tf_buffer.lookup_transform(
                TARGET_FRAME, source_frame, stamp, timeout=Duration(seconds=1.0)
            )
        except Exception:
            tf = self.tf_buffer.lookup_transform(
                TARGET_FRAME, source_frame, Time(), timeout=Duration(seconds=1.0)
            )

        T = transform_to_matrix(tf)
        xyz_camera = pointcloud_msg_to_xyz(msg)
        return apply_transform(xyz_camera, T)

    def capture_cloud_at_pose(self):
        collected = []
        for _ in range(FRAMES_PER_POSE):
            prev_stamp = self.latest_cloud_stamp_key
            msg = self.wait_for_new_cloud(prev_stamp, timeout_sec=5.0)
            xyz_base = self.transform_cloud_to_base(msg)
            xyz_roi = crop_roi(xyz_base)
            collected.append(xyz_roi)

        if not collected:
            return np.empty((0, 3), dtype=np.float64)
        return np.vstack(collected)

    def run_scan(self):
        """스캔 경로를 훑으면서 point cloud를 base_link 기준으로 merge해서 반환한다."""
        if not self.wait_for_move_line_service(timeout_sec=5.0):
            raise RuntimeError(f"MoveLine 서비스({MOVE_LINE_SERVICE})를 찾을 수 없습니다.")

        poses = generate_scan_poses()
        merged_list = []

        for i, pose in enumerate(poses):
            ok = self.move_line(pose)
            if not ok:
                raise RuntimeError(f"MoveLine 실패 (pose {i}: {pose})")

            time.sleep(SETTLE_SEC)
            pose_points = self.capture_cloud_at_pose()
            if len(pose_points) > 0:
                merged_list.append(pose_points)

        if not merged_list:
            return np.empty((0, 3), dtype=np.float64), poses

        merged_points = np.vstack(merged_list)
        merged_points = voxel_downsample(merged_points, VOXEL_SIZE_M)
        return merged_points, poses


# =========================
# 서비스 노드
# =========================

class WorldMapNode(Node):
    def __init__(self):
        super().__init__("world_map_node")

        self.worker = ScanWorker()
        self.obstacle_pub = self.create_publisher(WorldMapUpdate, "/world_map/obstacles", 10)
        self.update_srv = self.create_service(Trigger, "update_world_map", self.handle_update)

        self.get_logger().info("world_map_node ready. waiting for /update_world_map calls...")

    def handle_update(self, request, response):
        self.get_logger().info("World map update requested.")

        try:
            merged_points, poses = self.worker.run_scan()
        except Exception as e:
            self.get_logger().error(f"scan failed: {e}")
            response.success = False
            response.message = f"scan failed: {e}"
            return response

        if merged_points.shape[0] == 0:
            self.get_logger().warn("No points captured during scan.")
            response.success = False
            response.message = "no points captured"
            return response

        try:
            clusters = cluster_points(merged_points)
        except Exception as e:
            self.get_logger().error(f"clustering failed: {e}")
            response.success = False
            response.message = f"clustering failed: {e}"
            return response

        scan_dir = save_record(merged_points, clusters, poses)

        msg = build_world_map_update_msg(clusters, scan_dir, self.get_clock().now().to_msg())
        self.obstacle_pub.publish(msg)

        self.get_logger().info(
            f"World map updated: {len(clusters)} obstacles, saved to {scan_dir}"
        )
        response.success = True
        response.message = f"{len(clusters)} obstacles detected, saved to {scan_dir}"
        return response

    def destroy_node(self):
        self.worker.destroy_node()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WorldMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

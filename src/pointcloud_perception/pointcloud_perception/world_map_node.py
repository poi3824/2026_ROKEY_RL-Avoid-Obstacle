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
from rclpy.qos import qos_profile_sensor_data

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

VEL = [60.0, 30.0]
ACC = [60.0, 30.0]

# Doosan MoveLine의 sync_type 의미는 설치된 dsr_msgs2/컨트롤러 버전에 따라 확인 필요.
# 기존 코드와 동일하게 0을 기본값으로 둔다. 만약 서비스가 "명령 접수 즉시 반환"이면
# 로봇이 아직 움직이는 중에 cloud를 찍을 수 있으므로 settle/discard를 길게 둔다.
MOVE_SYNC_TYPE = 0

# flat scan에서도 첫 프레임이 흔들릴 수 있어서 기존 0.8s보다 조금 여유를 둔다.
SETTLE_SEC = 1.2

# 사이드뷰/tilt 포즈는 그리퍼-카메라 offset 때문에 TCP 자세 변화가 크고,
# robot TF와 RealSense cloud가 안정되기까지 시간이 더 필요하다.
SIDE_VIEW_SETTLE_SEC = 3.0

# side -> flat으로 복귀하는 바로 다음 포즈는 특히 stale frame이 잡히기 쉬워서
# 별도의 settle 시간을 둔다. pose14 -> pose15가 거의 같은 cloud로 저장되는 현상 방지.
AFTER_SIDE_RETURN_SETTLE_SEC = 3.0


# settle 직후 첫 몇 장은 motion/auto-exposure/TF lag가 남을 수 있어 버린다.
DISCARD_FRAMES_AFTER_SETTLE = 5

FRAMES_PER_POSE = 1

# point cloud / TF 동기화 대기 시간.
# 중요한 점: point cloud stamp를 ROS 현재 시각과 직접 비교하지 않는다.
# RealSense header stamp가 ROS clock보다 약간 늦거나 앞설 수 있기 때문이다.
CLOUD_WAIT_TIMEOUT_SEC = 10.0
TF_WAIT_TIMEOUT_SEC = 5.0

VOXEL_SIZE_M = 0.005

# =========================
# 포즈별 ground z 정렬 옵션
# =========================
# tilt 시 +Y/-Y 진행 방향에 따라 바닥/테이블 plane이 위아래로 살짝 갈라지는 현상을
# 완화하기 위한 후처리다. 진짜 hand-eye calibration을 대체하지는 못하지만,
# map layer를 줄이는 데 가장 직접적으로 효과가 있다.
ENABLE_GROUND_Z_ALIGNMENT = True
GROUND_Z_HIST_MIN_M = -0.03
GROUND_Z_HIST_MAX_M = 0.08
GROUND_Z_HIST_BIN_M = 0.002
GROUND_Z_MIN_POINTS_IN_BIN = 1000
MAX_GROUND_Z_CORRECTION_M = 0.04

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


def side_view_pose(x, at_top):
    """x 위치에서, 스캔 영역의 위쪽(at_top=True) 또는 아래쪽 끝의 사이드뷰 포즈.

    지그재그 특성상 각 column은 항상 START_POINT.y(위) 또는 END_POINT.y(아래)
    한쪽에서 시작해 반대쪽에서 끝난다. START/END에서 실측한 (오프셋, A,B,C)를
    그 column의 x 좌표에 그대로 적용한다 (2026-07-06: 공식으로 재계산하는 대신
    실측값을 재사용하기로 함 - END쪽에서 각도 공식이 실측과 160도 이상 어긋나는
    걸 확인해서, 중간 지점도 공식보다는 검증된 실측값을 그대로 쓰는 게 안전하다고 판단).
    """
    if at_top:
        y = START_POINT["y"] + SIDE_VIEW_Y_OFFSET_MM
        abc = SIDE_VIEW_START_ABC_DEG
    else:
        y = END_POINT["y"] - SIDE_VIEW_Y_OFFSET_MM
        abc = SIDE_VIEW_END_ABC_DEG
    return [x, y, FIXED_Z_MM - SIDE_VIEW_Z_DROP_MM] + list(abc)


def generate_scan_poses():
    """지그재그 flat scan + column 경계 side view + 마지막 END_POINT side view.

    이전 코드도 논리상 마지막 side view를 append했지만, 마지막 END_POINT tilt가
    실제 로봇에서 안 보인다는 피드백이 있어 최종 side pose를 루프 밖에서 명시적으로
    한 번 더 구성한다. summary/meta에서도 마지막 pose가 side인지 쉽게 확인할 수 있다.
    """
    columns = generate_scan_columns()
    poses = []

    # scan 시작 전 START 쪽 side view
    poses.append(side_view_pose(columns[0][0], at_top=True))

    for i, (x, ys) in enumerate(columns):
        for y in ys:
            poses.append([x, y, FIXED_Z_MM, FIXED_A_DEG, FIXED_B_DEG, FIXED_C_DEG])

        # 마지막 column의 side view는 아래에서 별도로 명시적으로 추가한다.
        if i < len(columns) - 1:
            ends_at_top = (i % 2 == 1)
            poses.append(side_view_pose(x, at_top=ends_at_top))

    return [[float(v) for v in pose] for pose in poses]


def is_flat_pose(pose):
    """평범한 탑다운 그리드 포즈인지(True) 사이드뷰 포즈인지(False) 판별."""
    _, _, _, a, b, c = pose
    return (
        abs(a - FIXED_A_DEG) < 1e-6
        and abs(b - FIXED_B_DEG) < 1e-6
        and abs(c - FIXED_C_DEG) < 1e-6
    )


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


def msg_stamp_to_sec(stamp_msg):
    return stamp_msg.sec + stamp_msg.nanosec * 1e-9


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


def estimate_ground_z_mode(points_xyz):
    """포즈별 point cloud에서 바닥/테이블로 보이는 z band의 mode를 추정한다.

    RealSense tilt에서 생기는 layer는 대부분 z 방향 bias로 먼저 드러난다.
    전체 median은 장애물/옆면 point에 끌릴 수 있으므로, 낮은 z 범위에서 histogram
    peak를 ground 후보로 사용한다.
    """
    if points_xyz is None or points_xyz.shape[0] == 0:
        return None

    z = points_xyz[:, 2]
    z = z[(z >= GROUND_Z_HIST_MIN_M) & (z <= GROUND_Z_HIST_MAX_M)]
    if z.size < GROUND_Z_MIN_POINTS_IN_BIN:
        return None

    bins = np.arange(
        GROUND_Z_HIST_MIN_M,
        GROUND_Z_HIST_MAX_M + GROUND_Z_HIST_BIN_M,
        GROUND_Z_HIST_BIN_M
    )
    counts, edges = np.histogram(z, bins=bins)
    if counts.size == 0:
        return None

    idx = int(np.argmax(counts))
    if int(counts[idx]) < GROUND_Z_MIN_POINTS_IN_BIN:
        return None

    return {
        "z_mode": float((edges[idx] + edges[idx + 1]) * 0.5),
        "bin_count": int(counts[idx]),
        "bin_min": float(edges[idx]),
        "bin_max": float(edges[idx + 1]),
    }


def align_ground_z_per_pose(per_pose_points, poses):
    """포즈별 ground z mode를 공통 기준에 맞춰 z translation만 보정한다.

    이 보정은 ICP가 아니다. tilt에서 카메라 extrinsic offset 때문에 생기는
    '바닥 layer 갈라짐'을 줄이기 위한 가벼운 후처리다.
    """
    if not ENABLE_GROUND_Z_ALIGNMENT:
        return per_pose_points, {
            "enabled": False,
            "reason": "ENABLE_GROUND_Z_ALIGNMENT=False",
            "reference_ground_z": None,
            "per_pose": [],
        }

    estimates = [estimate_ground_z_mode(points) for points in per_pose_points]

    flat_ground = [
        est["z_mode"]
        for est, pose in zip(estimates, poses)
        if est is not None and is_flat_pose(pose)
    ]
    all_ground = [est["z_mode"] for est in estimates if est is not None]

    if flat_ground:
        reference_ground_z = float(np.median(flat_ground))
        reference_source = "flat_pose_median"
    elif all_ground:
        reference_ground_z = float(np.median(all_ground))
        reference_source = "all_pose_median"
    else:
        return per_pose_points, {
            "enabled": False,
            "reason": "no valid ground z mode",
            "reference_ground_z": None,
            "per_pose": [],
        }

    corrected_points = []
    per_pose_info = []

    for i, (pose, points, est) in enumerate(zip(poses, per_pose_points, estimates)):
        info = {
            "pose_index": int(i),
            "is_flat_pose": bool(is_flat_pose(pose)),
            "ground_z_mode": None,
            "ground_bin_count": 0,
            "z_correction_m": 0.0,
            "applied": False,
        }

        if est is None or points is None or points.shape[0] == 0:
            corrected_points.append(points)
            per_pose_info.append(info)
            continue

        z_mode = float(est["z_mode"])
        dz = float(reference_ground_z - z_mode)

        info["ground_z_mode"] = z_mode
        info["ground_bin_count"] = int(est["bin_count"])
        info["z_correction_m"] = dz

        if abs(dz) <= MAX_GROUND_Z_CORRECTION_M:
            p2 = points.copy()
            p2[:, 2] += dz
            corrected_points.append(p2)
            info["applied"] = True
        else:
            # 너무 큰 보정은 잘못된 plane을 잡은 것일 수 있으므로 적용하지 않는다.
            corrected_points.append(points)

        per_pose_info.append(info)

    return corrected_points, {
        "enabled": True,
        "reference_source": reference_source,
        "reference_ground_z": reference_ground_z,
        "max_ground_z_correction_m": MAX_GROUND_Z_CORRECTION_M,
        "per_pose": per_pose_info,
    }


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


def save_pose_cloud(out_dir, pose_index, points):
    if not OPEN3D_AVAILABLE or points.shape[0] == 0:
        return
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    o3d.io.write_point_cloud(
        os.path.join(out_dir, f"scan_pose_{pose_index:03d}_base_roi.ply"), pcd
    )


def save_record(merged_points, clusters, poses, per_pose_points=None, ground_z_alignment=None):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(RECORD_DIR, f"world_map_update_{timestamp}")
    os.makedirs(out_dir, exist_ok=True)

    np.save(os.path.join(out_dir, "merged_base_roi.npy"), merged_points)

    # cobot_scan/world_map_scan_capture_ranged.py도 .ply를 같이 저장했었다.
    # publish_saved_world_cloud.py는 .npy만 있으면 되지만, open3d로 .ply를
    # 직접 읽어서 draw_geometries로 보는 워크플로우도 계속 쓰려면 .ply가 필요하다.
    if OPEN3D_AVAILABLE and merged_points.shape[0] > 0:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(merged_points.astype(np.float64))
        o3d.io.write_point_cloud(os.path.join(out_dir, "merged_base_roi.ply"), pcd)

    # 포즈별로 따로 저장 - 어느 포즈가 어긋난 원인인지 하나씩 눈으로 확인할 수 있게
    # (2026-07-06: 바닥이 여러 겹으로 보이는 문제의 원인 포즈를 찾기 위해 추가).
    if per_pose_points is not None:
        for i, (pose, points) in enumerate(zip(poses, per_pose_points)):
            save_pose_cloud(out_dir, i, points)
            meta = {
                "pose_index": i,
                "pose_xyz_abc": pose,
                "is_flat_pose": is_flat_pose(pose),
                "num_points": int(points.shape[0]),
            }
            if ground_z_alignment and ground_z_alignment.get("per_pose"):
                meta["ground_z_alignment"] = ground_z_alignment["per_pose"][i]
            meta_path = os.path.join(out_dir, f"scan_pose_{i:03d}_meta.json")
            with open(meta_path, "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2)

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
        "ground_z_alignment": ground_z_alignment,
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # RealSense PointCloud2는 센서 스트림이므로 최신 프레임만 쓰는 쪽이 안전하다.
        # queue가 깊으면 이동/settle 중 쌓인 오래된 프레임을 나중에 잡을 수 있다.
        self.cloud_sub = self.create_subscription(
            PointCloud2, POINTCLOUD_TOPIC, self.cloud_callback, qos_profile_sensor_data
        )
        self.move_line_client = self.create_client(MoveLine, MOVE_LINE_SERVICE)

    def cloud_callback(self, msg):
        self.latest_cloud_msg = msg

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
            req.sync_type = int(MOVE_SYNC_TYPE)

        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            return False

        result = future.result()
        if hasattr(result, "success"):
            return bool(result.success)
        return True

    def spin_sleep(self, seconds):
        """time.sleep 대신 worker node를 계속 spin하면서 대기한다.

        TF listener와 point cloud subscriber는 spin이 돌아야 최신 데이터가 들어온다.
        time.sleep만 쓰면 settle 시간 동안 callback 처리가 멈추고, 이후 오래된 큐를
        처리하면서 정착 전 프레임/TF를 잡을 수 있다.
        """
        end_time = time.time() + float(seconds)
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def latest_cloud_stamp_sec(self):
        """현재 worker가 마지막으로 받은 point cloud stamp를 초 단위로 반환."""
        if self.latest_cloud_msg is None:
            return None
        return msg_stamp_to_sec(self.latest_cloud_msg.header.stamp)

    def wait_for_cloud_newer_than(self, min_stamp_sec=None, timeout_sec=CLOUD_WAIT_TIMEOUT_SEC):
        """특정 point cloud stamp보다 새로운 프레임이 들어올 때까지 기다린다.

        주의:
        - ROS 현재 시각(self.get_clock().now())과 cloud header stamp를 비교하지 않는다.
        - RealSense/robot TF가 같은 system clock을 쓰더라도 publish latency 때문에
          cloud stamp가 현재 시각보다 늦게 보일 수 있다.
        - 그래서 "settle 종료 시각 이후"가 아니라 "마지막으로 처리한 cloud stamp 이후"
          프레임을 잡는 방식으로 바꾼다.
        """
        t0 = time.time()
        last_seen_stamp = None

        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

            msg = self.latest_cloud_msg
            if msg is None:
                continue

            stamp_sec = msg_stamp_to_sec(msg.header.stamp)
            last_seen_stamp = stamp_sec

            if min_stamp_sec is None or stamp_sec > min_stamp_sec + 1e-9:
                return msg

        if min_stamp_sec is None:
            raise TimeoutError(
                f"새 point cloud를 받지 못했습니다. "
                f"last_seen_stamp={last_seen_stamp}"
            )

        raise TimeoutError(
            f"기준 stamp 이후의 새 point cloud를 받지 못했습니다. "
            f"min_stamp={min_stamp_sec:.6f}, last_seen_stamp={last_seen_stamp}"
        )

    def discard_cloud_frames(self, count):
        """settle 직후의 오래된/흔들린 프레임을 버리고 최신 stamp까지 진행한다."""
        min_stamp = self.latest_cloud_stamp_sec()
        discarded = 0

        for _ in range(int(count)):
            msg = self.wait_for_cloud_newer_than(
                min_stamp,
                timeout_sec=CLOUD_WAIT_TIMEOUT_SEC
            )
            min_stamp = msg_stamp_to_sec(msg.header.stamp)
            discarded += 1

        return min_stamp, discarded

    def lookup_transform_strict_spin(self, target_frame, source_frame, stamp, timeout_sec=TF_WAIT_TIMEOUT_SEC):
        """정확한 stamp의 TF가 들어올 때까지 worker node를 spin하면서 기다린다.

        lookup_transform(..., timeout=...)만 쓰면 같은 노드의 TF callback이 처리되지 않는
        구간이 생길 수 있다. 특히 RealSense cloud stamp가 최신 TF보다 30~80ms 앞서면
        "extrapolation into the future"가 발생한다. 여기서는 최신 TF fallback은 하지 않고,
        요청 stamp를 커버하는 TF가 buffer에 들어올 때까지 spin_once로 기다린다.
        """
        start = time.time()
        last_error = None

        while time.time() - start < timeout_sec:
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    stamp,
                    timeout=Duration(seconds=0.0)
                )
            except Exception as e:
                last_error = e
                rclpy.spin_once(self, timeout_sec=0.02)

        requested_sec = stamp.nanoseconds * 1e-9
        raise RuntimeError(
            f"TF lookup failed after {timeout_sec:.1f}s: "
            f"{source_frame} -> {target_frame} at {requested_sec:.6f}. "
            f"last_error={last_error}"
        )

    def transform_cloud_to_base(self, msg):
        """PointCloud2를 msg.header.stamp 시점의 TF로 base_link 기준 변환한다.

        중요:
        - Time() 최신 TF fallback은 쓰지 않는다.
        - 대신 point cloud stamp를 커버하는 TF가 들어올 때까지 spin하면서 기다린다.
        - 그래도 TF가 안 들어오면 scan fail로 드러내서 잘못된 map 누적을 막는다.
        """
        source_frame = msg.header.frame_id
        stamp = Time.from_msg(msg.header.stamp)

        tf = self.lookup_transform_strict_spin(
            TARGET_FRAME,
            source_frame,
            stamp,
            timeout_sec=TF_WAIT_TIMEOUT_SEC
        )

        T = transform_to_matrix(tf)
        xyz_camera = pointcloud_msg_to_xyz(msg)
        return apply_transform(xyz_camera, T)

    def capture_cloud_at_pose(self):
        """현재 포즈에서 point cloud를 수집한다.

        settle 직후에는 아직 motion/TF/RealSense exposure가 완전히 안정되지 않은 프레임이
        들어올 수 있으므로, 먼저 DISCARD_FRAMES_AFTER_SETTLE장 버리고 그 다음 프레임을 쓴다.
        """
        collected = []

        min_stamp, discarded = self.discard_cloud_frames(DISCARD_FRAMES_AFTER_SETTLE)
        self.get_logger().debug(
            f"discarded {discarded} frames after settle, stamp={min_stamp}"
        )

        for frame_idx in range(FRAMES_PER_POSE):
            msg = self.wait_for_cloud_newer_than(
                min_stamp,
                timeout_sec=CLOUD_WAIT_TIMEOUT_SEC
            )
            min_stamp = msg_stamp_to_sec(msg.header.stamp)

            xyz_base = self.transform_cloud_to_base(msg)
            xyz_roi = crop_roi(xyz_base)
            collected.append(xyz_roi)

            self.get_logger().debug(
                f"captured frame {frame_idx}: stamp={min_stamp:.6f}, "
                f"roi_points={xyz_roi.shape[0]}"
            )

        if not collected:
            return np.empty((0, 3), dtype=np.float64)
        return np.vstack(collected)

    def run_scan(self):
        """스캔 경로를 훑으면서 point cloud를 base_link 기준으로 merge해서 반환한다."""
        if not self.wait_for_move_line_service(timeout_sec=5.0):
            raise RuntimeError(f"MoveLine 서비스({MOVE_LINE_SERVICE})를 찾을 수 없습니다.")

        poses = generate_scan_poses()
        per_pose_points = []
        prev_pose = None

        for i, pose in enumerate(poses):
            pose_type = "flat" if is_flat_pose(pose) else "side"

            self.get_logger().info(
                f"moving pose {i:03d}/{len(poses)-1:03d}: "
                f"type={pose_type}, "
                f"pose={['%.2f' % v for v in pose]}"
            )

            ok = self.move_line(pose)
            if not ok:
                raise RuntimeError(f"MoveLine 실패 (pose {i}: {pose})")

            settle_sec = SETTLE_SEC if is_flat_pose(pose) else SIDE_VIEW_SETTLE_SEC

            # side view 직후 flat 복귀 pose는 stale side frame이 잡히는 경우가 있어 추가 대기.
            if prev_pose is not None and (not is_flat_pose(prev_pose)) and is_flat_pose(pose):
                settle_sec = max(settle_sec, AFTER_SIDE_RETURN_SETTLE_SEC)


            self.get_logger().info(
                f"pose {i:03d}/{len(poses)-1:03d} reached candidate: "
                f"type={pose_type}, settle={settle_sec:.2f}s"
            )

            # time.sleep 대신 spin_sleep을 사용해서 settle 중에도 TF/cloud callback을 처리한다.
            # 이 시간 동안 latest_cloud_msg도 계속 최신 프레임으로 갱신된다.
            self.spin_sleep(settle_sec)

            before_capture_stamp = self.latest_cloud_stamp_sec()
            if before_capture_stamp is None:
                self.get_logger().warn(
                    f"pose {i:03d}: settle 동안 point cloud를 아직 한 번도 받지 못함"
                )
            else:
                self.get_logger().info(
                    f"pose {i:03d}: latest cloud before capture stamp="
                    f"{before_capture_stamp:.6f}"
                )

            pose_points = self.capture_cloud_at_pose()
            self.get_logger().info(
                f"pose {i:03d} captured {pose_points.shape[0]} ROI points"
            )
            per_pose_points.append(pose_points)
            prev_pose = pose

        corrected_per_pose_points, ground_z_alignment = align_ground_z_per_pose(
            per_pose_points, poses
        )

        if ground_z_alignment.get("enabled"):
            ref_z = ground_z_alignment.get("reference_ground_z")
            self.get_logger().info(
                f"ground z alignment enabled: reference z={ref_z:.4f} m"
            )
            for info in ground_z_alignment.get("per_pose", []):
                if info.get("applied") and abs(info.get("z_correction_m", 0.0)) > 0.003:
                    self.get_logger().info(
                        f"pose {info['pose_index']:03d}: "
                        f"ground_z={info['ground_z_mode']:.4f}, "
                        f"z_correction={info['z_correction_m']:+.4f} m"
                    )
        else:
            self.get_logger().warn(
                f"ground z alignment disabled: {ground_z_alignment.get('reason')}"
            )

        merged_list = [p for p in corrected_per_pose_points if len(p) > 0]
        if not merged_list:
            return (
                np.empty((0, 3), dtype=np.float64),
                poses,
                corrected_per_pose_points,
                ground_z_alignment,
            )

        merged_points = np.vstack(merged_list)
        merged_points = voxel_downsample(merged_points, VOXEL_SIZE_M)
        return merged_points, poses, corrected_per_pose_points, ground_z_alignment


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
            merged_points, poses, per_pose_points, ground_z_alignment = self.worker.run_scan()
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

        scan_dir = save_record(merged_points, clusters, poses, per_pose_points, ground_z_alignment)

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

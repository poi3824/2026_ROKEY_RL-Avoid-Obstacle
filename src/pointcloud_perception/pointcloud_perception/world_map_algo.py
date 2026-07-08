# world_map_node.py에서 쓰는 순수 알고리즘 코드 (스캔 경로 생성, TF 적용, ROI/클러스터링,
# ground z 보정, ICP 잔차보정, 저장).
#
# rclpy/tf2_ros/dsr_msgs2/obstacle_avoidance_msgs를 import하지 않는다 - 그래야
# offline_icp_experiment.py가 ROS 워크스페이스를 소스/빌드하지 않고도 이 모듈만
# 불러와서 저장된 world_map_update_* 스캔으로 오프라인 튜닝을 할 수 있다.
import json
import math
import os
from datetime import datetime

import numpy as np

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except Exception:
    OPEN3D_AVAILABLE = False


# =========================
# 스캔 설정 (cobot_scan/world_map_scan_capture_ranged.py 와 동일한 캘리브레이션)
# =========================

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
CLUSTER_MIN_POINTS = 5

# 나중에 DB/UI 연동을 생각해서 cobot_scan(다른 프로젝트의 범용 스캔 폴더)이
# 아니라 이 워크스페이스 루트 하위 data/world_maps로 저장한다. src/ 밑이 아니라
# 워크스페이스 루트 형제 디렉토리인 이유는, point cloud/npy 산출물이 git으로
# 추적되지 않게 하기 위함 (.gitignore의 data/ 참고).
RECORD_DIR = os.path.expanduser("~/RL-Avoid-Obstacle/data/world_maps")


# =========================
# TF 기반 누적 + ICP 잔차보정 옵션
# =========================
# z 히스토그램 보정만으로는 한계가 뚜렷했다 (2026-07-07: 같은 top-down pose인데도
# point 수가 적은 pose에서 바닥 z가 최대 44mm까지 흔들리는 걸 실측 확인).
# ICP 잔차보정 도입 전까지는 기존 방식을 그대로 유지한다 - 로봇 스캔 없이 저장된
# world_map_update_* 폴더로 offline_icp_experiment.py에서 먼저 튜닝한다.
ENABLE_ICP_MAPPING = False

ICP_VOXEL_SIZE_M = 0.01
ICP_OUTLIER_NB_NEIGHBORS = 20
ICP_OUTLIER_STD_RATIO = 2.0
ICP_NORMAL_RADIUS_M = 0.03
ICP_NORMAL_MAX_NN = 30
ICP_MAX_CORR_DIST_M = 0.03
ICP_MAX_ITER = 50

# accept 판정은 항상 raw ICP transform 기준이다 (ICP_MODE로 일부 축을 버리더라도,
# raw solve 자체가 못 미더우면 - 예: 평면에서 xy/yaw로 미끄러짐 - 그 축이 뭐든 신뢰 불가).
MIN_ICP_FITNESS = 0.30
MAX_ICP_RMSE_M = 0.035
MAX_ICP_DELTA_XY_M = 0.025
MAX_ICP_DELTA_Z_M = 0.035
MAX_ICP_DELTA_ROLL_PITCH_DEG = 3.0
MAX_ICP_DELTA_YAW_DEG = 2.0

# flat pose 품질 게이트. compute_ground_quality()가 이 기준으로 판정한다.
MIN_POINTS_FOR_MAPPING = 3000
MAX_FLAT_GROUND_Z_DEVIATION_M = 0.025   # reference_ground_z 대비 편차 허용치

# ground_band_source()가 잘라낸 band가 이 개수보다 적으면 sparse로 본다.
# 주의: GROUND_Z_MIN_POINTS_IN_BIN(1000)과는 다른 스케일이다 - 그건 raw point cloud
# (수만 개) 히스토그램용이고, 이건 preprocess_for_mapping()으로 1cm voxel downsample된
# clean cloud(보통 수백~수천 개) 기준이다. 실측 결과 GROUND_Z_MIN_POINTS_IN_BIN을 그대로
# 재사용하면 거의 항상 fallback되어(clean 자체가 200~1000개 수준) ground band 제한이
# 사실상 no-op이 되는 걸 확인함 (2026-07-07).
ICP_MIN_BAND_POINTS = 100

# ICP correspondence용 ground band (reference_ground_z 기준 상대값).
# cloud 전체(바닥+장애물)를 그대로 ICP source로 쓰면 장애물 표면에 correspondence가
# 끌려가 바닥 정합 신호가 흐려진다 (2026-07-07 오프라인 실험에서 z_only/translation_only
# 모드가 오히려 악화되는 걸 확인). flat-flat, side-flat 정합 모두 이 밴드만 source로
# 쓰고, 구한 transform은 전체 cloud에 적용한다 (build_map_with_icp 참고).
ICP_GROUND_BAND_Z_MIN_M = -0.03
ICP_GROUND_BAND_Z_MAX_M = 0.08

# "z_only" | "translation_only" | "full_se3"
# 평면 위주 point cloud에 처음부터 full 6DoF를 믿기보다, 오프라인에서 세 모드를
# 비교해서 정한다 (constrain_transform 참고).
ICP_MODE = "full_se3"


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
    """지그재그 flat scan + 시작/중간 column 경계 side view.

    마지막 END_POINT side view는 로봇 물리 한계로 제외한다.
    따라서 마지막 pose는 flat pose로 끝난다.
    """
    columns = generate_scan_columns()
    poses = []

    # scan 시작 전 START 쪽 side view
    poses.append(side_view_pose(columns[0][0], at_top=True))

    for i, (x, ys) in enumerate(columns):
        for y in ys:
            poses.append([x, y, FIXED_Z_MM, FIXED_A_DEG, FIXED_B_DEG, FIXED_C_DEG])

        # 마지막 column의 side view는 로봇 물리 한계 때문에 추가하지 않는다.
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


def transform_stamped_to_matrix(transform_stamped):
    """TransformStamped 모양(.transform.translation/.rotation)의 객체를 4x4 행렬로 변환."""
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


def compute_ground_quality(per_pose_points, poses):
    """포즈별 ground 추정 품질(포인트 수/ground bin/기준 대비 편차)을 한 곳에서 계산한다.

    align_ground_z_per_pose(z 보정 적용)와 build_map_with_icp(flat pose 품질 게이트)가
    같은 기준(estimate, reference_ground_z)을 공유하도록 품질 판단을 이 함수로 분리했다
    - 이전에는 align_ground_z_per_pose가 품질 판단과 보정 적용을 같이 하고 있었다.
    """
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
        reference_ground_z = None
        reference_source = None

    per_pose = {}
    for i, (pose, points, est) in enumerate(zip(poses, per_pose_points, estimates)):
        pose_is_flat = is_flat_pose(pose)
        num_points = int(points.shape[0]) if points is not None else 0

        entry = {
            "pose_index": int(i),
            "is_flat_pose": bool(pose_is_flat),
            "num_points": num_points,
            "ground_z_mode": None,
            "ground_bin_count": 0,
            "passed": False,
            "reason": None,
        }

        if num_points < MIN_POINTS_FOR_MAPPING:
            entry["reason"] = (
                f"num_points({num_points}) < MIN_POINTS_FOR_MAPPING({MIN_POINTS_FOR_MAPPING})"
            )
            per_pose[str(i)] = entry
            continue

        if est is None:
            entry["reason"] = "no valid ground z estimate"
            per_pose[str(i)] = entry
            continue

        entry["ground_z_mode"] = float(est["z_mode"])
        entry["ground_bin_count"] = int(est["bin_count"])

        if pose_is_flat and reference_ground_z is not None:
            deviation = abs(entry["ground_z_mode"] - reference_ground_z)
            if deviation > MAX_FLAT_GROUND_Z_DEVIATION_M:
                entry["reason"] = (
                    f"flat pose ground_z deviation({deviation:.4f}m) > "
                    f"MAX_FLAT_GROUND_Z_DEVIATION_M({MAX_FLAT_GROUND_Z_DEVIATION_M})"
                )
                per_pose[str(i)] = entry
                continue

        entry["passed"] = True
        entry["reason"] = "ok"
        per_pose[str(i)] = entry

    return {
        "reference_ground_z": reference_ground_z,
        "reference_source": reference_source,
        "per_pose": per_pose,
    }


def align_ground_z_per_pose(per_pose_points, poses, ground_quality):
    """side-view pose에 대해서만 ground z mode를 기준으로 z translation 보정한다.

    주의:
    - flat/top-down pose는 원래 가장 신뢰도가 높은 기준 map이므로 보정하지 않는다.
    - 품질 판단(포인트 수/ground bin/편차)은 compute_ground_quality()가 전담하고,
      여기서는 그 결과(ground_quality)를 받아 실제 z 보정 적용만 한다.
    """
    if not ENABLE_GROUND_Z_ALIGNMENT:
        return per_pose_points, {
            "enabled": False,
            "reason": "ENABLE_GROUND_Z_ALIGNMENT=False",
            "reference_ground_z": None,
            "per_pose": [],
        }

    reference_ground_z = ground_quality.get("reference_ground_z")
    if reference_ground_z is None:
        return per_pose_points, {
            "enabled": False,
            "reason": "no valid ground z mode",
            "reference_ground_z": None,
            "per_pose": [],
        }

    quality_by_index = ground_quality.get("per_pose", {})
    corrected_points = []
    per_pose_info = []

    for i, (pose, points) in enumerate(zip(poses, per_pose_points)):
        pose_is_flat = is_flat_pose(pose)
        quality = quality_by_index.get(str(i), {})
        z_mode = quality.get("ground_z_mode")

        info = {
            "pose_index": int(i),
            "is_flat_pose": bool(pose_is_flat),
            "ground_z_mode": z_mode,
            "ground_bin_count": quality.get("ground_bin_count", 0),
            "z_correction_m": 0.0,
            "applied": False,
            "reason": None,
        }

        if z_mode is None or points is None or points.shape[0] == 0:
            corrected_points.append(points)
            info["reason"] = "no valid ground estimate or empty cloud"
            per_pose_info.append(info)
            continue

        dz = float(reference_ground_z - z_mode)
        info["z_correction_m"] = dz

        if pose_is_flat:
            corrected_points.append(points)
            info["reason"] = "flat pose used as reference only; not corrected"
        elif abs(dz) <= MAX_GROUND_Z_CORRECTION_M:
            p2 = points.copy()
            p2[:, 2] += dz
            corrected_points.append(p2)
            info["applied"] = True
            info["reason"] = "side pose corrected"
        else:
            corrected_points.append(points)
            info["reason"] = "correction too large; skipped"

        per_pose_info.append(info)

    return corrected_points, {
        "enabled": True,
        "mode": "side_pose_only",
        "reference_source": ground_quality.get("reference_source"),
        "reference_ground_z": reference_ground_z,
        "max_ground_z_correction_m": MAX_GROUND_Z_CORRECTION_M,
        "per_pose": per_pose_info,
    }


# =========================
# ICP 잔차보정
# =========================

def preprocess_for_mapping(points_xyz):
    """ICP 정합용 전처리: voxel downsample(성긴 해상도) + statistical outlier 제거.

    입력은 이미 ROI-crop된 상태라 재크롭하지 않는다. 여기서 만든 결과는 ICP
    correspondence 탐색에만 쓰고, 최종 merge에는 원본(full-resolution) 좌표에
    ICP transform만 적용한 값을 쓴다 (build_map_with_icp 참고).
    """
    if points_xyz is None or points_xyz.shape[0] == 0:
        return points_xyz if points_xyz is not None else np.empty((0, 3), dtype=np.float64)

    down = voxel_downsample(points_xyz, ICP_VOXEL_SIZE_M)
    if not OPEN3D_AVAILABLE or down.shape[0] == 0:
        return down

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(down)
    pcd, _ = pcd.remove_statistical_outlier(
        nb_neighbors=ICP_OUTLIER_NB_NEIGHBORS, std_ratio=ICP_OUTLIER_STD_RATIO
    )
    return np.asarray(pcd.points)


def rotation_matrix_to_euler_deg(R):
    """R(3x3, base_link 축 기준)을 ZYX(yaw-pitch-roll) 순서 euler각(deg)으로 분해.

    scipy 의존성을 추가하지 않기 위해 quaternion_to_matrix와 같은 스타일로 순수
    numpy/math만 사용한다. ICP 잔차보정은 작은 각도만 다루므로 gimbal lock
    근처(pitch~=90deg)는 실질적으로 발생하지 않는다고 가정한다.
    """
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def decompose_transform(T):
    """T(4x4)를 dx/dy/dz/roll_deg/pitch_deg/yaw_deg로 분해. raw/applied 리포트에 공용."""
    roll_deg, pitch_deg, yaw_deg = rotation_matrix_to_euler_deg(T[:3, :3])
    return {
        "dx": float(T[0, 3]),
        "dy": float(T[1, 3]),
        "dz": float(T[2, 3]),
        "roll_deg": float(roll_deg),
        "pitch_deg": float(pitch_deg),
        "yaw_deg": float(yaw_deg),
    }


def constrain_transform(T, mode):
    """ICP_MODE에 따라 적용할 자유도만 남기고 나머지는 항등으로 되돌린다.

    z_only/translation_only/full_se3 세 모드를 각각 별도로 구현하는 대신, 하나의
    ICP 계산 결과에서 신뢰하는 컴포넌트만 선택 적용하는 방식으로 통합한다.
    """
    if mode == "full_se3":
        return T.copy()

    constrained = np.eye(4, dtype=np.float64)
    if mode == "translation_only":
        constrained[:3, 3] = T[:3, 3]
    elif mode == "z_only":
        constrained[2, 3] = T[2, 3]
    else:
        raise ValueError(f"unknown ICP_MODE: {mode}")

    return constrained


def align_cloud_to_map_icp(source_np, target_np):
    """source_np를 target_np(누적 map)에 point-to-plane ICP로 정합한다.

    accept 판정은 항상 raw ICP transform 기준이다 - ICP_MODE로 특정 축만 적용할
    계획이더라도, raw solve 자체가 못 미더우면(예: 평면에서 xy/yaw로 미끄러짐)
    그 축이 뭐든 신뢰할 수 없다고 본다. 반환 리포트에는 raw/applied transform과
    delta를 모두 남겨서, "raw는 사실 컸는데 mode 때문에 안 보였다" 같은 걸 리포트
    만으로 알 수 있게 한다.
    """
    identity = np.eye(4, dtype=np.float64)
    zero_delta = {
        "dx": 0.0, "dy": 0.0, "dz": 0.0,
        "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
    }

    if not OPEN3D_AVAILABLE or source_np.shape[0] == 0 or target_np.shape[0] == 0:
        return source_np, {
            "method": "icp_point_to_plane",
            "icp_mode": ICP_MODE,
            "fitness": 0.0,
            "inlier_rmse": None,
            "raw_transform": identity.tolist(),
            "raw_delta": dict(zero_delta),
            "applied_transform": identity.tolist(),
            "applied_delta": dict(zero_delta),
            "accepted": False,
            "reason": "empty source/target or open3d unavailable",
        }

    source = o3d.geometry.PointCloud()
    target = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(source_np)
    target.points = o3d.utility.Vector3dVector(target_np)

    normal_search = o3d.geometry.KDTreeSearchParamHybrid(
        radius=ICP_NORMAL_RADIUS_M, max_nn=ICP_NORMAL_MAX_NN
    )
    source.estimate_normals(search_param=normal_search)
    target.estimate_normals(search_param=normal_search)

    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        ICP_MAX_CORR_DIST_M,
        identity,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_MAX_ITER),
    )

    T_raw = np.asarray(result.transformation, dtype=np.float64)
    raw_delta = decompose_transform(T_raw)

    accepted = (
        result.fitness >= MIN_ICP_FITNESS
        and result.inlier_rmse <= MAX_ICP_RMSE_M
        and abs(raw_delta["dx"]) <= MAX_ICP_DELTA_XY_M
        and abs(raw_delta["dy"]) <= MAX_ICP_DELTA_XY_M
        and abs(raw_delta["dz"]) <= MAX_ICP_DELTA_Z_M
        and abs(raw_delta["roll_deg"]) <= MAX_ICP_DELTA_ROLL_PITCH_DEG
        and abs(raw_delta["pitch_deg"]) <= MAX_ICP_DELTA_ROLL_PITCH_DEG
        and abs(raw_delta["yaw_deg"]) <= MAX_ICP_DELTA_YAW_DEG
    )

    if accepted:
        T_applied = constrain_transform(T_raw, ICP_MODE)
        aligned_points = apply_transform(source_np, T_applied)
        applied_delta = decompose_transform(T_applied)
    else:
        T_applied = identity
        aligned_points = source_np
        applied_delta = dict(zero_delta)

    report = {
        "method": "icp_point_to_plane",
        "icp_mode": ICP_MODE,
        "fitness": float(result.fitness),
        "inlier_rmse": float(result.inlier_rmse),
        "raw_transform": T_raw.tolist(),
        "raw_delta": raw_delta,
        "applied_transform": T_applied.tolist(),
        "applied_delta": applied_delta,
        "accepted": bool(accepted),
        "reason": "accepted" if accepted else "rejected: fitness/rmse/delta threshold",
    }

    return aligned_points, report


def ground_band_source(clean_points, reference_ground_z):
    """clean_points(전처리된 좌표)에서 ground band(ICP_GROUND_BAND_Z_*)만 잘라낸다.

    fallback 정책은 호출부(flat/side pass)마다 다르므로 여기서는 순수하게 밴드만
    잘라서 반환한다 - flat pose는 전체 cloud로 fallback해도 대체로 안전하지만(대부분
    바닥), side pose는 fallback하면 옆면 전체가 다시 ICP source에 섞여 원래 막으려던
    문제(옆면이 바닥에 끌려감)가 재발하므로 sparse하면 그냥 스킵해야 한다.
    """
    if reference_ground_z is None or clean_points.shape[0] == 0:
        return clean_points

    z = clean_points[:, 2]
    mask = (
        (z >= reference_ground_z + ICP_GROUND_BAND_Z_MIN_M)
        & (z <= reference_ground_z + ICP_GROUND_BAND_Z_MAX_M)
    )
    return clean_points[mask]


def build_map_with_icp(per_pose_points, poses, ground_quality):
    """TF로 1차 정렬된 pose cloud를 flat pose 먼저, side pose 나중에 ICP로 지도에 붙인다.

    Pass 1 (flat): compute_ground_quality()의 품질 게이트를 통과한 flat pose만 대상.
    첫 통과 pose가 누적 map을 초기화하고, 이후는 growing map에 ICP 정합 → accept면
    보정, reject면 원본(TF 결과) 유지(품질 게이트를 이미 통과했으므로 버리지 않음).

    Pass 2 (side): side cloud 전체를 ICP source로 쓰면 옆면이 flat map(대부분 바닥)에
    억지로 끌려갈 수 있어서, ground 근처 z-band(overlap 후보)만 source로 정합하고
    구한 transform은 side cloud 전체에 적용한다. reject면 해당 side pose는 제외.

    두 pass 모두 ground_band_source()로 correspondence 대상을 ground band로 제한한다 -
    flat pose도 위에서 내려다본 시야라 장애물 상단이 섞여 있으면 바닥 정합이 흐려질 수
    있어서, side pose와 동일한 방식을 적용한다 (2026-07-07: 전체 cloud로 정합했을 때
    z_only/translation_only 모드가 오히려 악화되는 걸 오프라인 실험으로 확인).

    ICP correspondence 탐색에는 성긴 preprocess_for_mapping() 결과(누적 map도 이 해상도로
    유지)를 쓰고, 최종 출력에 들어가는 점은 원본 해상도 좌표에 transform만 적용한 값이다.
    """
    reference_ground_z = ground_quality.get("reference_ground_z")
    quality_by_index = ground_quality.get("per_pose", {})

    flat_indices = [i for i, pose in enumerate(poses) if is_flat_pose(pose)]
    side_indices = [i for i, pose in enumerate(poses) if not is_flat_pose(pose)]

    global_map = None          # ICP correspondence용, ICP_VOXEL_SIZE_M 해상도로 유지
    accepted_points = []       # 최종 출력용, 원본 해상도
    mapping_report = {}

    for i in flat_indices:
        points = per_pose_points[i]
        quality = quality_by_index.get(str(i), {})

        if points is None or points.shape[0] == 0 or not quality.get("passed", False):
            mapping_report[str(i)] = {
                "type": "flat",
                "quality_gate": {
                    "passed": bool(quality.get("passed", False)),
                    "reason": quality.get("reason", "no quality info"),
                },
                "icp": None,
                "used_in_map": False,
            }
            continue

        clean = preprocess_for_mapping(points)

        if global_map is None:
            global_map = clean
            accepted_points.append(points)
            mapping_report[str(i)] = {
                "type": "flat",
                "quality_gate": {"passed": True, "reason": quality.get("reason")},
                "icp": {"method": "init_map", "accepted": True},
                "used_in_map": True,
            }
            continue

        band_source = ground_band_source(clean, reference_ground_z)
        if band_source.shape[0] < ICP_MIN_BAND_POINTS:
            # flat pose는 top-down이라 대부분 바닥이므로, 밴드가 sparse하면
            # 전체 cloud로 fallback해도 side pose만큼 위험하지 않다.
            band_source = clean

        _, icp_report = align_cloud_to_map_icp(band_source, global_map)
        T_applied = np.asarray(icp_report["applied_transform"], dtype=np.float64)

        if icp_report["accepted"]:
            full_res_out = apply_transform(points, T_applied)
            coarse_for_map = apply_transform(clean, T_applied)
        else:
            full_res_out = points
            coarse_for_map = clean

        accepted_points.append(full_res_out)
        global_map = voxel_downsample(
            np.vstack([global_map, coarse_for_map]), ICP_VOXEL_SIZE_M
        )

        mapping_report[str(i)] = {
            "type": "flat",
            "quality_gate": {"passed": True, "reason": quality.get("reason")},
            "icp": icp_report,
            "used_in_map": True,
        }

    if global_map is None:
        global_map = np.empty((0, 3), dtype=np.float64)

    for i in side_indices:
        points = per_pose_points[i]

        if points is None or points.shape[0] == 0:
            mapping_report[str(i)] = {
                "type": "side", "quality_gate": None, "icp": None,
                "used_in_map": False, "reason": "empty cloud",
            }
            continue

        if global_map.shape[0] == 0 or reference_ground_z is None:
            mapping_report[str(i)] = {
                "type": "side", "quality_gate": None, "icp": None,
                "used_in_map": False, "reason": "no flat map available for side ICP",
            }
            continue

        clean = preprocess_for_mapping(points)
        band_source = ground_band_source(clean, reference_ground_z)

        if band_source.shape[0] < ICP_MIN_BAND_POINTS:
            # side pose는 fallback하면 옆면 전체가 ICP source에 섞여 원래 막으려던
            # 문제(옆면이 바닥에 끌려감)가 재발하므로, sparse하면 그냥 스킵한다.
            mapping_report[str(i)] = {
                "type": "side", "quality_gate": None, "icp": None,
                "used_in_map": False,
                "reason": f"ground band too sparse ({band_source.shape[0]} points)",
            }
            continue

        _, icp_report = align_cloud_to_map_icp(band_source, global_map)

        if icp_report["accepted"]:
            T_applied = np.asarray(icp_report["applied_transform"], dtype=np.float64)
            full_res_out = apply_transform(points, T_applied)
            coarse_for_map = apply_transform(clean, T_applied)

            accepted_points.append(full_res_out)
            global_map = voxel_downsample(
                np.vstack([global_map, coarse_for_map]), ICP_VOXEL_SIZE_M
            )
            mapping_report[str(i)] = {
                "type": "side", "quality_gate": None, "icp": icp_report,
                "used_in_map": True,
            }
        else:
            mapping_report[str(i)] = {
                "type": "side", "quality_gate": None, "icp": icp_report,
                "used_in_map": False,
            }

    if not accepted_points:
        return np.empty((0, 3), dtype=np.float64), mapping_report

    final_map = voxel_downsample(np.vstack(accepted_points), VOXEL_SIZE_M)
    return final_map, mapping_report


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


def save_record(
    merged_points,
    clusters,
    poses,
    per_pose_points=None,
    ground_z_alignment=None,
    icp_mapping_report=None,
    out_root=None,
):
    out_root = out_root if out_root is not None else RECORD_DIR
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(out_root, f"world_map_update_{timestamp}")
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
            if icp_mapping_report:
                meta["icp_mapping"] = icp_mapping_report.get(str(i))
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
        "icp_mapping": icp_mapping_report,
        "scan_poses": poses,
        "clusters": clusters,
    }
    with open(os.path.join(out_dir, "world_map_summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    return out_dir

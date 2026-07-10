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

try:
    import cv2
    CV2_AVAILABLE = True
except Exception:
    CV2_AVAILABLE = False

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except Exception:
    MATPLOTLIB_AVAILABLE = False


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

# =========================
# 장애물(cylinder) 추출 - World Map Update 결과를 RL에 넘길 좌표로 변환
# =========================
# merged_points에는 바닥/테이블 point가 그대로 섞여 있어서, DBSCAN을 바로 돌리면
# 테이블에 맞닿은 장애물 base가 테이블 point와 이어져(density-reachable) 테이블+
# 장애물 전체가 하나의 거대 클러스터로 뭉친다 (2026-07-08: 실제 저장된 스캔으로
# 확인 - 30916 point짜리 radius 0.36m 클러스터 1개 + 파편 클러스터 여러 개).
# RANSAC으로 평면을 새로 잡는 대신 compute_ground_quality()가 이미 여러 pose의
# ground z histogram으로 추정해둔 reference_ground_z를 재사용한다 - 바닥 기준을
# 두 군데서 따로 추정하면 서로 어긋날 수 있어서 피한다.
MIN_OBSTACLE_HEIGHT_ABOVE_GROUND_M = 0.015

# 바닥 제거만으로는 서로 떨어진 물체 두 개가 한 클러스터로 남을 수 있다
# (2026-07-08: 실제 라이브 스캔에서 확인 - 밑단/윗단만 보면 완전히 분리된 원
# 두 개인데, 중간 높이(z 6~15cm)에 실제 표면과 무관한 성긴 점들이 다리를 놓아서
# DBSCAN이 하나로 이어붙임). RealSense가 depth discontinuity(물체 실루엣 경계)
# 에서 만드는 flying pixel 노이즈가 원인으로 보인다 - 여러 pose에 걸쳐 조금씩
# 나오지만, 실제 표면 점은 여러 pose가 같은 위치를 반복 관측해 국소 밀도가
# 훨씬 높은 반면 flying pixel은 pose마다 위치가 달라 국소 밀도가 낮다. 그래서
# statistical outlier removal로 이걸 걸러낸다 - std_ratio를 기존 ICP 전처리용
# (2.0)보다 세게(1.0) 줘야 실제로 갈라짐을 확인함. 다만 이걸로도 등록 오차가
# 큰(TF drift 심한) 스캔은 완전히 안 갈라질 수 있다 - 그 경우는 별개 문제(ICP
# 매핑 품질)로 다뤄야 한다.
OBSTACLE_OUTLIER_NB_NEIGHBORS = 20
OBSTACLE_OUTLIER_STD_RATIO = 1.0

# max 대신 percentile을 쓰는 이유: RealSense 노이즈로 한두 점만 튀어도 radius가
# 실제보다 크게 잡히는 걸 방지하기 위함.
CYLINDER_RADIUS_PERCENTILE = 95

# DBSCAN 자체가 통과시킨 아주 작은 파편 클러스터(노이즈에 가까움)를 한 번 더
# 걸러낸다. CLUSTER_MIN_POINTS(DBSCAN core point 기준, 5)와는 역할이 다르다.
MIN_CLUSTER_POINTS_FOR_OBSTACLE = 80

# 실측 전 placeholder (2026-07-08) - 그리퍼/툴 반지름을 포함한 안전 여유가
# 확정되면 이 값을 실측치로 교체해야 한다.
SAFETY_RADIUS_MARGIN_M = 0.04
SAFETY_HEIGHT_MARGIN_M = 0.03

# confidence = min(1.0, num_points / CONFIDENCE_NUM_POINTS_SCALE). 실제 스캔
# 밀도 보고 튜닝 필요.
CONFIDENCE_NUM_POINTS_SCALE = 1500.0

# =========================
# 디버그 시각화 / Hough 교차검증 (world_map_node.handle_update()가 스캔마다 자동 실행)
# =========================
# 스캔 자체가 로봇 이동 때문에 이미 수 분 걸리므로, PNG 저장 몇 장 추가되는 정도의
# 비용(1~2초)은 무시할 만하다. 다만 시각화가 실패해도 실제 장애물 결과 응답에는
# 영향이 없어야 하므로 호출부(world_map_node)에서 항상 try/except로 감싼다.
ENABLE_TOPVIEW_DEBUG_PNG = True
ENABLE_HOUGH_VALIDATION = True

# 2026-07-10: 서로 거의 맞닿은(수 mm 이내) 원기둥 2개를 DBSCAN이 하나의 클러스터로
# 합쳐버리는 경우가 실기에서 확인됨(eps 기반 클러스터링은 점이 실제로 연결돼 있으면
# 원리적으로 못 나눈다). Hough 원 검출(아래)은 점 연결성이 아니라 rasterize된
# height image의 원형 edge 패턴을 보므로, 같은 상황에서도 원기둥 2개를 따로
# 검출해낸다(circle_fit_vs_hough_overlay.png로 실측 확인) - cluster_points()가
# 이 정보를 병합 클러스터 분리에 실제로 쓰도록 한다. 기존에는 Hough 결과가
# save_debug_visualizations()의 교차검증 PNG에만 쓰이고 실제 장애물 목록에는
# 반영되지 않았다.
ENABLE_HOUGH_CLUSTER_SPLIT = True

# Hough 검증은 cluster_points()가 이미 걸러낸 점이 아니라, 필터링 전 raw
# point cloud를 그대로 이미지화한다 - 그래야 서로 다른 두 경로가 같은 답에
# 도달하는지 보는 교차검증이 된다 (world_map_algo.cluster_points 참고).
HOUGH_RESOLUTION_M = 0.005
HOUGH_CLOSE_ITERATIONS = 2
HOUGH_CANNY1 = 90.0
HOUGH_PARAM2 = 18.0
HOUGH_MIN_RADIUS_M = 0.02
HOUGH_MAX_RADIUS_M = 0.09
HOUGH_MATCH_MAX_DIST_M = 0.05

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


def dbscan_labels(points_xyz, eps, min_points):
    """DBSCAN 라벨(noise=-1)만 반환. cluster_points()가 이 위에서 필터/파라미터 추정을 한다."""
    if not OPEN3D_AVAILABLE:
        raise RuntimeError("open3d가 없어서 클러스터링을 할 수 없습니다.")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz)
    return np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points))


def remove_ground_band(points_xyz, ground_z, margin_m=MIN_OBSTACLE_HEIGHT_ABOVE_GROUND_M):
    """ground_z + margin 이하 point를 바닥/테이블로 보고 제거한다.

    ground_z가 없으면(품질 게이트 실패 등) 아무것도 제거하지 않고 그대로 반환한다 -
    잘못된 기준으로 장애물 point까지 잘라내는 것보다 안전하다.
    """
    if points_xyz.shape[0] == 0 or ground_z is None:
        return points_xyz
    mask = points_xyz[:, 2] > (ground_z + margin_m)
    return points_xyz[mask]


def remove_flying_pixel_outliers(
    points_xyz,
    nb_neighbors=OBSTACLE_OUTLIER_NB_NEIGHBORS,
    std_ratio=OBSTACLE_OUTLIER_STD_RATIO,
):
    """DBSCAN 전에 RealSense flying pixel(물체 경계 depth 불연속에서 생기는 노이즈)을
    제거한다. 실제 표면 점은 여러 pose가 반복 관측해 국소 밀도가 높지만, flying
    pixel은 pose마다 위치가 달라 국소 밀도가 낮다는 점을 이용한다.
    """
    if not OPEN3D_AVAILABLE or points_xyz.shape[0] == 0:
        return points_xyz
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return np.asarray(pcd.points)


def fit_circle_2d(x, y):
    """(x,y) 점들에 원을 대수적으로 피팅한다 (Kåsa method, 반복 없는 closed-form).

    x^2+y^2 = 2*a*x + 2*b*y + c 형태로 세우면 (a,b,c)에 대해 선형이라 최소자승으로
    바로 풀린다. center=(a,b), radius=sqrt(c + a^2+b^2).

    반환: (cx, cy, radius) 또는 실패 시 None.
    """
    if x.shape[0] < 3:
        return None

    A = np.column_stack([x, y, np.ones_like(x)])
    b = x ** 2 + y ** 2
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None

    a_coef, b_coef, c_coef = sol
    cx, cy = a_coef / 2.0, b_coef / 2.0
    r_sq = c_coef + cx ** 2 + cy ** 2
    if not np.isfinite(r_sq) or r_sq <= 0:
        return None

    return float(cx), float(cy), float(np.sqrt(r_sq))


def compute_cluster_params(
    member_points,
    label,
    ground_z=None,
    safety_radius_margin=SAFETY_RADIUS_MARGIN_M,
    safety_height_margin=SAFETY_HEIGHT_MARGIN_M,
):
    """클러스터 하나(원기둥 하나로 가정)를 RL/WorldMapObstacle에 넘길 파라미터로 변환한다.

    center_x/center_y는 median이 아니라 원 피팅(fit_circle_2d)으로 구한다.
    이 물체들은 위에서 보면 꽉 찬 원판이 아니라 속이 빈 링(컵 테두리 등)으로
    잡히는데, 스캔이 링 둘레를 고르게 못 찍고 한쪽으로 치우쳐 찍으면 median
    중심이 그쪽으로 쏠린다 (2026-07-08: 실제 스캔에서 각도별 반지름이 사인파
    형태로 출렁이는 걸 확인 - 중심이 밀린 원의 전형적인 signature). 이 상태로
    percentile-95를 반지름으로 쓰면 중심에서 가장 먼(반대쪽) 테두리까지 거리를
    그대로 반지름으로 채택해버려 실제보다 크게 잡힌다. 원 피팅은 각도에 따른
    반지름 편차(std)를 크게 줄여서 - 즉 더 진짜 원에 가깝게 - 중심을 잡는다.
    원 피팅이 실패하면(점이 너무 적거나 퇴화된 경우) median으로 fallback한다.

    height/center_z는 클러스터 자체 z_min이 아니라 ground_z(스캔 전체에서 이미
    추정된 바닥 높이, compute_ground_quality 참고)를 기준으로 계산한다 - 이 스캔은
    대부분 탑다운 시점이라 장애물 대부분이 상판 위주 point만 잡히고 실제 바닥까지
    이어지는 point가 거의 없어서, 클러스터 자체 z_min을 쓰면 height가 심하게
    과소추정된다. ground_z가 없으면 z_min을 그대로 fallback으로 쓴다.
    """
    x = member_points[:, 0]
    y = member_points[:, 1]
    z = member_points[:, 2]

    fit = fit_circle_2d(x, y)
    if fit is not None:
        cx, cy, _fitted_radius = fit
    else:
        cx, cy = float(np.median(x)), float(np.median(y))

    xy_dist = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    radius = float(np.percentile(xy_dist, CYLINDER_RADIUS_PERCENTILE))

    z_min = float(z.min())
    z_max = float(z.max())

    if ground_z is not None:
        height = max(0.0, z_max - float(ground_z))
        center_z = float(ground_z) + height / 2.0
    else:
        height = max(0.0, z_max - z_min)
        center_z = (z_min + z_max) / 2.0

    safety_radius = radius + safety_radius_margin
    safety_height = height + safety_height_margin

    confidence = min(1.0, member_points.shape[0] / CONFIDENCE_NUM_POINTS_SCALE)

    return {
        "id": int(label),
        "centroid": [cx, cy, center_z],
        "radius": radius,
        "height": height,
        "z_min": z_min,
        "z_max": z_max,
        "safety_radius": safety_radius,
        "safety_height": safety_height,
        "shape_type": "cylinder",
        "num_points": int(member_points.shape[0]),
        "confidence": float(confidence),
    }


def split_cluster_by_hough_circles(member_points, hough_circles, match_margin_m=HOUGH_MATCH_MAX_DIST_M):
    """member_points(DBSCAN 클러스터 하나)를 감싸는 hough_circles 중심이 2개 이상이면,
    각 점을 가장 가까운 원 중심 기준으로 나눠 서브그룹 리스트로 쪼갠다.

    이 클러스터의 XY 반경(중심에서 95퍼센타일 거리) + match_margin_m 안에 중심이
    들어오는 hough 원만 후보로 본다 - 너무 멀리 있는(다른 물체에 속하는) hough 원이
    잘못 끼어드는 걸 막는다. 후보가 1개 이하면 안 쪼개고 원본을 그대로
    [member_points] 형태로 반환한다(정상적으로 하나인 클러스터를 건드리지 않기 위함).

    2026-07-10: 서로 거의 맞닿은 원기둥 2개가 DBSCAN에서 하나로 합쳐지는 문제
    (ENABLE_HOUGH_CLUSTER_SPLIT 주석 참고)를 고치기 위해 추가.
    """
    if not hough_circles:
        return [member_points]

    x, y = member_points[:, 0], member_points[:, 1]
    cx_cluster, cy_cluster = float(np.median(x)), float(np.median(y))
    cluster_extent = float(np.percentile(np.sqrt((x - cx_cluster) ** 2 + (y - cy_cluster) ** 2), 95))

    candidate_centers = [
        h["center"] for h in hough_circles
        if math.hypot(h["center"][0] - cx_cluster, h["center"][1] - cy_cluster) <= cluster_extent + match_margin_m
    ]
    if len(candidate_centers) < 2:
        return [member_points]

    centers = np.asarray(candidate_centers)  # (K, 2)
    dists = np.linalg.norm(member_points[:, None, :2] - centers[None, :, :], axis=2)  # (N, K)
    assignment = np.argmin(dists, axis=1)

    return [member_points[assignment == k] for k in range(len(candidate_centers)) if np.any(assignment == k)]


def get_candidate_points(
    points_xyz,
    ground_z=None,
    outlier_nb_neighbors=OBSTACLE_OUTLIER_NB_NEIGHBORS,
    outlier_std_ratio=OBSTACLE_OUTLIER_STD_RATIO,
):
    """cluster_points()의 전처리(바닥 제거 -> flying pixel 제거)만 떼어낸 것.

    save_debug_visualizations()의 top-view PNG가 cluster_points()와 정확히 같은
    점 집합을 그리도록, 이 전처리를 cluster_points()와 공유한다.
    """
    candidate_points = remove_ground_band(points_xyz, ground_z)
    candidate_points = remove_flying_pixel_outliers(
        candidate_points, outlier_nb_neighbors, outlier_std_ratio
    )
    return candidate_points


def cluster_points(
    points_xyz,
    ground_z=None,
    eps=CLUSTER_EPS_M,
    min_points=CLUSTER_MIN_POINTS,
    min_cluster_points=MIN_CLUSTER_POINTS_FOR_OBSTACLE,
    safety_radius_margin=SAFETY_RADIUS_MARGIN_M,
    safety_height_margin=SAFETY_HEIGHT_MARGIN_M,
    outlier_nb_neighbors=OBSTACLE_OUTLIER_NB_NEIGHBORS,
    outlier_std_ratio=OBSTACLE_OUTLIER_STD_RATIO,
    split_merged_with_hough=ENABLE_HOUGH_CLUSTER_SPLIT,
):
    """merged_points -> (바닥 제거 -> flying pixel 제거 -> DBSCAN -> 작은 파편 제거 ->
    [Hough 기반 병합 클러스터 분리] -> cylinder 파라미터 추정).

    world_map_node가 /world_map/obstacles로 publish할 최종 장애물 목록을 만드는
    진입점. ground_z를 안 주면 바닥 제거를 생략하고 z_min/z_max로만 height를 계산한다
    (레거시 호출부 호환 - 다만 이 경우 테이블에 맞닿은 장애물이 테이블과 한 클러스터로
    뭉칠 수 있다는 점을 알고 있어야 한다. handle_update()는 항상 ground_z를 넘긴다).

    2026-07-10: split_merged_with_hough가 True면(기본값, ENABLE_HOUGH_CLUSTER_SPLIT
    주석 참고), 각 DBSCAN 클러스터를 Hough 원 검출 결과와 대조해서 서로 거의 맞닿은
    원기둥 2개가 하나로 뭉친 경우를 서브그룹으로 나눈다. Hough 검출 자체가 실패해도
    (cv2 없음, 이미지화 실패 등) 예외 없이 분리 없는 기존 DBSCAN 결과로 폴백한다 -
    이 기능이 죽어도 기존 장애물 추출 자체는 항상 응답해야 하기 때문이다.

    반환: [{"id", "centroid": [x,y,z], "radius", "height", "z_min", "z_max",
    "safety_radius", "safety_height", "shape_type", "num_points", "confidence"}, ...]
    """
    if points_xyz.shape[0] == 0:
        return []

    candidate_points = get_candidate_points(points_xyz, ground_z, outlier_nb_neighbors, outlier_std_ratio)
    if candidate_points.shape[0] == 0:
        return []

    labels = dbscan_labels(candidate_points, eps, min_points)

    raw_groups = []
    for label in sorted(set(labels.tolist())):
        if label < 0:
            continue  # noise
        member_points = candidate_points[labels == label]
        if member_points.shape[0] < min_cluster_points:
            continue
        raw_groups.append(member_points)

    hough_circles = []
    if split_merged_with_hough and CV2_AVAILABLE and raw_groups:
        try:
            # save_debug_visualizations()와 동일하게 필터링 전 raw points_xyz를 그대로
            # 이미지화한다(모듈 상단 HOUGH_RESOLUTION_M 주석 참고 - 이미 검증된 경로와
            # 동일한 입력을 써야 여기서도 같은 결과를 신뢰할 수 있다).
            _, _, blurred, x_min, y_min = build_height_image(points_xyz, ground_z)
            hough_circles = detect_circles_hough(blurred, HOUGH_RESOLUTION_M, x_min, y_min)
        except Exception:
            hough_circles = []  # Hough 실패는 분리를 포기할 뿐, DBSCAN 결과 자체는 그대로 응답한다

    clusters = []
    next_id = 0
    for member_points in raw_groups:
        sub_groups = (
            split_cluster_by_hough_circles(member_points, hough_circles)
            if hough_circles else [member_points]
        )
        for sub_points in sub_groups:
            if sub_points.shape[0] < min_cluster_points:
                continue
            clusters.append(compute_cluster_params(
                sub_points, next_id, ground_z, safety_radius_margin, safety_height_margin
            ))
            next_id += 1

    return clusters


def save_topview_debug_png(out_path, candidate_points, obstacles):
    """base_link 기준 top-view(XY) PNG - solid=radius, dashed=safety_radius.

    candidate_points/obstacles 모두 base_link 좌표를 그대로 쓰므로
    (world_map_node.transform_cloud_to_base 참고), 이 PNG의 x/y축은 카메라가
    아니라 base_link 기준이다.
    """
    if not MATPLOTLIB_AVAILABLE:
        raise RuntimeError("matplotlib이 없어서 top-view PNG를 저장할 수 없습니다.")

    fig, ax = plt.subplots(figsize=(8, 10))

    if candidate_points.shape[0] > 0:
        ax.scatter(candidate_points[:, 0], candidate_points[:, 1], s=1, c="#4C72B0", alpha=0.3)

    color_cycle = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for i, obs in enumerate(sorted(obstacles, key=lambda o: o["id"])):
        cx, cy, _ = obs["centroid"]
        color = color_cycle[i % len(color_cycle)]

        ax.plot(cx, cy, marker="x", color=color, markersize=10, mew=2)
        ax.text(cx + 0.01, cy, f"id={obs['id']}", fontsize=10)
        ax.add_patch(plt.Circle((cx, cy), obs["radius"], fill=False, linewidth=2, linestyle="-", color="black"))
        ax.add_patch(
            plt.Circle((cx, cy), obs["safety_radius"], fill=False, linewidth=1.5, linestyle="--", color="black")
        )

    ax.set_xlabel("x in base_link (m)")
    ax.set_ylabel("y in base_link (m)")
    ax.set_title("Obstacle extraction top-view debug\nsolid=radius, dashed=safety_radius")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def build_height_image(points_xyz, ground_z, resolution_m=HOUGH_RESOLUTION_M, close_iterations=HOUGH_CLOSE_ITERATIONS):
    """point cloud -> top-down 높이(z) 이미지. 픽셀값 = 그 칸의 최대 z를
    [ground_z, z_max] 범위로 0~255 정규화한 값. 점 사이 빈틈은 morphological
    closing으로 메운다.

    반환: (img_u8, closed, blurred, x_min, y_min) - x_min/y_min은 픽셀(0,0)의
    world 좌표.
    """
    if not CV2_AVAILABLE:
        raise RuntimeError("opencv(cv2)가 없어서 height image를 만들 수 없습니다.")

    x_min, x_max = points_xyz[:, 0].min(), points_xyz[:, 0].max()
    y_min, y_max = points_xyz[:, 1].min(), points_xyz[:, 1].max()
    w = int(np.ceil((x_max - x_min) / resolution_m)) + 1
    h = int(np.ceil((y_max - y_min) / resolution_m)) + 1

    height_img = np.zeros((h, w), dtype=np.float32)
    px = ((points_xyz[:, 0] - x_min) / resolution_m).astype(int)
    py = ((points_xyz[:, 1] - y_min) / resolution_m).astype(int)
    np.maximum.at(height_img, (py, px), points_xyz[:, 2])

    z_lo = ground_z if ground_z is not None else float(points_xyz[:, 2].min())
    z_hi = float(points_xyz[:, 2].max())
    span = max(z_hi - z_lo, 1e-6)
    img_u8 = np.clip((height_img - z_lo) / span * 255, 0, 255).astype(np.uint8)

    kernel = np.ones((3, 3), np.uint8)
    closed = cv2.morphologyEx(img_u8, cv2.MORPH_CLOSE, kernel, iterations=close_iterations)
    blurred = cv2.GaussianBlur(closed, (5, 5), 0)

    return img_u8, closed, blurred, x_min, y_min


def detect_circles_hough(
    blurred_img, resolution_m, x_min, y_min,
    canny1=HOUGH_CANNY1, hough_param2=HOUGH_PARAM2,
    min_radius_m=HOUGH_MIN_RADIUS_M, max_radius_m=HOUGH_MAX_RADIUS_M,
):
    """blurred height image에서 cv2.HoughCircles로 원을 검출해 world 좌표로 변환.

    반환: [{"center": [x,y], "radius": r}, ...] (world 단위, m)
    """
    if not CV2_AVAILABLE:
        raise RuntimeError("opencv(cv2)가 없어서 Hough 원 검출을 할 수 없습니다.")

    min_r_px = max(1, int(round(min_radius_m / resolution_m)))
    max_r_px = int(round(max_radius_m / resolution_m))

    circles = cv2.HoughCircles(
        blurred_img, cv2.HOUGH_GRADIENT, dp=1, minDist=min_r_px,
        param1=canny1, param2=hough_param2,
        minRadius=min_r_px, maxRadius=max_r_px,
    )
    if circles is None:
        return []

    results = []
    for cx_px, cy_px, r_px in circles[0]:
        wx = x_min + cx_px * resolution_m
        wy = y_min + cy_px * resolution_m
        wr = r_px * resolution_m
        results.append({"center": [float(wx), float(wy)], "radius": float(wr)})
    return results


def match_hough_to_clusters(hough_circles, clusters, max_dist_m=HOUGH_MATCH_MAX_DIST_M):
    """각 cluster_points() 결과에 대해 가장 가까운 hough 원을 찾아 매칭한다."""
    matches = []
    used = set()
    for c in clusters:
        ccx, ccy = c["centroid"][0], c["centroid"][1]
        best_idx, best_dist = None, None
        for i, h in enumerate(hough_circles):
            if i in used:
                continue
            d = ((h["center"][0] - ccx) ** 2 + (h["center"][1] - ccy) ** 2) ** 0.5
            if best_dist is None or d < best_dist:
                best_idx, best_dist = i, d
        if best_idx is not None and best_dist <= max_dist_m:
            used.add(best_idx)
            matches.append({"cluster": c, "hough": hough_circles[best_idx], "center_dist_m": best_dist})
        else:
            matches.append({"cluster": c, "hough": None, "center_dist_m": None})
    unmatched_hough = [h for i, h in enumerate(hough_circles) if i not in used]
    return matches, unmatched_hough


def save_hough_overlay_png(out_path, blurred_img, resolution_m, x_min, y_min, clusters, hough_circles):
    """circle-fit(파란 실선) vs hough(빨간 점선)를 height image 위에 겹쳐 그린다."""
    if not MATPLOTLIB_AVAILABLE:
        raise RuntimeError("matplotlib이 없어서 hough 오버레이 PNG를 저장할 수 없습니다.")

    fig, ax = plt.subplots(figsize=(8, 10))
    ax.imshow(blurred_img, cmap="gray", origin="lower",
              extent=[x_min, x_min + blurred_img.shape[1] * resolution_m,
                      y_min, y_min + blurred_img.shape[0] * resolution_m])

    for c in clusters:
        cx, cy = c["centroid"][0], c["centroid"][1]
        ax.add_patch(plt.Circle((cx, cy), c["radius"], fill=False, color="cyan", linewidth=2))
        ax.plot(cx, cy, marker="+", color="cyan", markersize=8)

    for h in hough_circles:
        cx, cy = h["center"]
        ax.add_patch(plt.Circle((cx, cy), h["radius"], fill=False, color="red", linewidth=1.5, linestyle="--"))
        ax.plot(cx, cy, marker="x", color="red", markersize=6)

    handles = [
        plt.Line2D([0], [0], color="cyan", lw=2, label="circle-fit (point-based)"),
        plt.Line2D([0], [0], color="red", lw=1.5, linestyle="--", label="Hough (image-based)"),
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=8)
    ax.set_xlabel("x in base_link (m)")
    ax.set_ylabel("y in base_link (m)")
    ax.set_title("circle-fit vs Hough circle cross-validation")
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def save_debug_visualizations(scan_dir, merged_points, ground_z, clusters):
    """스캔 1회당 top-view 디버그 PNG + Hough 교차검증을 scan_dir에 저장한다.

    world_map_node.handle_update()가 매 스캔마다 호출한다. ENABLE_TOPVIEW_DEBUG_PNG/
    ENABLE_HOUGH_VALIDATION로 개별 on/off 가능. 호출부에서 이미 try/except로
    감싸지만, 여기서도 단계별로 실패를 report에 기록해 하나가 실패해도 나머지는
    계속 진행한다.

    반환: {"topview_png": path 또는 None, "hough_png": path 또는 None,
    "hough_matches": [...] 또는 None, "errors": [...]}
    """
    report = {"topview_png": None, "hough_png": None, "hough_matches": None, "errors": []}

    if ENABLE_TOPVIEW_DEBUG_PNG:
        try:
            candidate_points = get_candidate_points(merged_points, ground_z)
            png_path = os.path.join(scan_dir, "obstacle_topview_debug.png")
            save_topview_debug_png(png_path, candidate_points, clusters)
            report["topview_png"] = png_path
        except Exception as e:
            report["errors"].append(f"topview_png failed: {e}")

    if ENABLE_HOUGH_VALIDATION:
        try:
            _, _, blurred, x_min, y_min = build_height_image(merged_points, ground_z)
            hough_circles = detect_circles_hough(blurred, HOUGH_RESOLUTION_M, x_min, y_min)
            matches, unmatched = match_hough_to_clusters(hough_circles, clusters)

            png_path = os.path.join(scan_dir, "circle_fit_vs_hough_overlay.png")
            save_hough_overlay_png(png_path, blurred, HOUGH_RESOLUTION_M, x_min, y_min, clusters, hough_circles)

            report["hough_png"] = png_path
            report["hough_matches"] = [
                {
                    "cluster_id": m["cluster"]["id"],
                    "circle_fit_center": m["cluster"]["centroid"][:2],
                    "circle_fit_radius_m": m["cluster"]["radius"],
                    "hough_center": m["hough"]["center"] if m["hough"] else None,
                    "hough_radius_m": m["hough"]["radius"] if m["hough"] else None,
                    "center_dist_m": m["center_dist_m"],
                }
                for m in matches
            ]
        except Exception as e:
            report["errors"].append(f"hough_validation failed: {e}")

    return report


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

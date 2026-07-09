# world_map_node가 저장한 data/world_maps/world_map_update_*/ 스캔 결과를 읽어서
# HMI(Flask API)에 넘겨주는 순수 파일 I/O 계층. ROS 노드를 안 띄우고도(rclpy 없이)
# 저장된 스캔만 있으면 동작한다 - world_map_node가 실행 중이 아니어도 지난 스캔을
# 계속 볼 수 있어야 하기 때문.
#
# clusters는 world_map_algo.cluster_points()가 만든 걸 world_map_node.save_record()가
# world_map_summary.json에 그대로 저장해둔 것이라, WorldMapObstacle.msg와 같은
# 필드(centroid/radius/height/z_min/z_max/safety_radius/safety_height/shape_type/
# num_points/confidence)를 그대로 갖고 있다 - 여기서 다시 계산하지 않는다.
import json
import os
import re

import numpy as np

from pointcloud_perception.world_map_algo import RECORD_DIR, voxel_downsample

# 브라우저로 보낼 point cloud를 더 성기게 만드는 해상도. cluster_points()가 쓰는
# 5mm보다 크게 잡아도 시각적 확인 목적으로는 충분하고 전송량이 줄어든다.
POINTS_DOWNSAMPLE_VOXEL_M = 0.01
MAX_POINTS_RETURNED = 50000

_SCAN_ID_RE = re.compile(r"^world_map_update_[0-9_]+$")


def list_scan_ids():
    """RECORD_DIR 밑의 world_map_update_* 디렉토리 이름을 최신순으로 반환한다
    (.zip 백업 파일 등은 제외 - 디렉토리만 대상)."""
    if not os.path.isdir(RECORD_DIR):
        return []
    entries = [
        name for name in os.listdir(RECORD_DIR)
        if _SCAN_ID_RE.match(name) and os.path.isdir(os.path.join(RECORD_DIR, name))
    ]
    return sorted(entries, reverse=True)


def latest_scan_id():
    scans = list_scan_ids()
    return scans[0] if scans else None


def _scan_dir(scan_id):
    """scan_id -> 실제 디렉토리 경로. scan_id는 URL에서 그대로 들어오므로, 형식을
    먼저 검증해서 path traversal(예: "../../etc")을 막는다."""
    if not _SCAN_ID_RE.match(scan_id):
        raise ValueError(f"잘못된 scan_id 형식: {scan_id}")

    scan_dir = os.path.join(RECORD_DIR, scan_id)
    if not os.path.isdir(scan_dir):
        raise FileNotFoundError(f"scan_id를 찾을 수 없습니다: {scan_id}")
    return scan_dir


def load_summary(scan_id):
    scan_dir = _scan_dir(scan_id)
    summary_path = os.path.join(scan_dir, "world_map_summary.json")
    if not os.path.exists(summary_path):
        raise FileNotFoundError(f"world_map_summary.json이 없습니다: {scan_dir}")
    with open(summary_path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_obstacles(scan_id):
    """world_map_summary.json에 저장된 clusters를 그대로 반환 (WorldMapObstacle과 동일 스키마)."""
    summary = load_summary(scan_id)
    return summary.get("clusters", [])


def load_points(scan_id, voxel_size_m=POINTS_DOWNSAMPLE_VOXEL_M, max_points=MAX_POINTS_RETURNED):
    """merged_base_roi.npy(base_link 기준 merge된 point cloud)를 읽어서 voxel
    downsample + 개수 상한으로 브라우저 전송에 적합하게 줄인다."""
    scan_dir = _scan_dir(scan_id)
    npy_path = os.path.join(scan_dir, "merged_base_roi.npy")
    if not os.path.exists(npy_path):
        raise FileNotFoundError(f"merged_base_roi.npy가 없습니다: {scan_dir}")

    points = np.load(npy_path)
    if voxel_size_m and voxel_size_m > 0 and points.shape[0] > 0:
        points = voxel_downsample(points, voxel_size_m)

    if points.shape[0] > max_points:
        rng = np.random.default_rng(0)
        idx = rng.choice(points.shape[0], size=max_points, replace=False)
        points = points[idx]

    return points

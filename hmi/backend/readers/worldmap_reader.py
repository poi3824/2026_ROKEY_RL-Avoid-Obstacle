# world_map_node가 저장한 data/world_maps/world_map_update_*/ 스캔 결과를 읽는 모듈.
#
# src/hmi_bridge/hmi_bridge/worldmap_loader.py(list_scan_ids/load_summary/load_obstacles/
# load_points - 가장 완성도 높은 구현)와 src/hmi_interface/hmi_interface/
# worldmap_catalog_reader.py(fetch_recent_scans - scan_id 목록 + cluster_count만 가벼운
# 버전)가 같은 RECORD_DIR/scan_id 정규식 스캔 로직을 각자 재구현하고 있었다 - 여기서
# 하나로 합친다. fetch_recent_scans()는 list_scan_ids()/load_summary()를 그대로
# 재사용해서 만든다(중복 재구현 금지).
#
# world_map_algo는 rclpy를 import하지 않는(코드 상단 주석에 명시된) 순수 알고리즘
# 모듈이라 ROS 워크스페이스를 소스하지 않고도 단독 import 가능하다 - 이 저장소의
# offline_icp_experiment.py 등이 이미 쓰는 것과 동일한 sys.path 패턴을 그대로 따른다
# (Flask 백엔드가 ROS를 직접 붙잡지 않는다는 원칙과도 맞다 - rclpy는 여전히 안 쓴다).
import json
import os
import re
import sys

_PKG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
    "src", "pointcloud_perception", "pointcloud_perception",
)
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

import world_map_algo as algo  # noqa: E402  (sys.path 조정 이후에 import해야 함)
import numpy as np  # noqa: E402

RECORD_DIR = algo.RECORD_DIR

POINTS_DOWNSAMPLE_VOXEL_M = 0.01
MAX_POINTS_RETURNED = 50000
MAX_SCANS = 100

_SCAN_ID_RE = re.compile(r"^world_map_update_([0-9]{8})_([0-9]{6})$")


def _parse_scan_id(scan_id):
    m = _SCAN_ID_RE.match(scan_id)
    if not m:
        return None
    date_part, time_part = m.groups()
    return f"{date_part[:4]}-{date_part[4:6]}-{date_part[6:]} {time_part[:2]}:{time_part[2:4]}:{time_part[4:]}"


def list_scan_ids():
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
    summary = load_summary(scan_id)
    return summary.get("clusters", [])


def load_points(scan_id, voxel_size_m=POINTS_DOWNSAMPLE_VOXEL_M, max_points=MAX_POINTS_RETURNED):
    scan_dir = _scan_dir(scan_id)
    npy_path = os.path.join(scan_dir, "merged_base_roi.npy")
    if not os.path.exists(npy_path):
        raise FileNotFoundError(f"merged_base_roi.npy가 없습니다: {scan_dir}")

    points = np.load(npy_path)
    if voxel_size_m and voxel_size_m > 0 and points.shape[0] > 0:
        points = algo.voxel_downsample(points, voxel_size_m)

    if points.shape[0] > max_points:
        rng = np.random.default_rng(0)
        idx = rng.choice(points.shape[0], size=max_points, replace=False)
        points = points[idx]

    return points


def fetch_recent_scans(limit=MAX_SCANS):
    """DB 탭 '월드' 서브탭용 - list_scan_ids()에 cluster_count/timestamp를 붙인다."""
    scan_ids = list_scan_ids()[:limit]
    scans = []
    for scan_id in scan_ids:
        cluster_count = None
        has_summary = False
        try:
            summary = load_summary(scan_id)
            has_summary = True
            cluster_count = len(summary.get("clusters", []))
        except FileNotFoundError:
            pass
        except (OSError, json.JSONDecodeError):
            pass

        scans.append({
            "scan_id": scan_id,
            "timestamp": _parse_scan_id(scan_id),
            "cluster_count": cluster_count,
            "has_summary": has_summary,
        })
    return scans

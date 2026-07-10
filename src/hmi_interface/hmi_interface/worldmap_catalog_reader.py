"""world_map_node가 저장한 data/world_maps/world_map_update_*/ 스캔 결과를
목록으로 읽어주는 순수 파일 I/O 모듈 (DB 아님).

point cloud/마스크 같은 큰 바이너리 산출물(.npy/.ply)은 SQL 행에 넣을 데이터가
아니라서 그대로 파일로 둔다 - 여기서는 "어떤 스캔이 언제 있었고 장애물이 몇 개
잡혔는지"만 가볍게 나열한다(실제 3D 뷰어는 hmi_bridge가 별도로 담당 - 이 모듈은
hmi_bridge를 건드리지도, 의존하지도 않는다).

RECORD_DIR은 pointcloud_perception.world_map_algo에서 그대로 가져다 쓴다 -
따로 하드코딩하면 두 값이 나중에 어긋날 수 있다(실제로 이 워크스페이스에서
한 번 겪은 문제: 옛 워크스페이스 경로가 하드코딩돼 있던 버그).
"""
import json
import os
import re

from pointcloud_perception.world_map_algo import RECORD_DIR

_SCAN_ID_RE = re.compile(r"^world_map_update_([0-9]{8})_([0-9]{6})$")
MAX_SCANS = 100


def _parse_scan_id(scan_id):
    m = _SCAN_ID_RE.match(scan_id)
    if not m:
        return None
    date_part, time_part = m.groups()
    return f"{date_part[:4]}-{date_part[4:6]}-{date_part[6:]} {time_part[:2]}:{time_part[2:4]}:{time_part[4:]}"


def fetch_recent_scans(limit=MAX_SCANS):
    """RECORD_DIR 밑의 스캔들을 최신순으로 나열한다. 디렉토리가 아직 없으면
    (한 번도 스캔 안 했으면) 빈 리스트를 반환한다."""
    if not os.path.isdir(RECORD_DIR):
        return []

    scan_ids = sorted(
        (name for name in os.listdir(RECORD_DIR) if _SCAN_ID_RE.match(name)
         and os.path.isdir(os.path.join(RECORD_DIR, name))),
        reverse=True,
    )[:limit]

    scans = []
    for scan_id in scan_ids:
        scan_dir = os.path.join(RECORD_DIR, scan_id)
        summary_path = os.path.join(scan_dir, "world_map_summary.json")
        cluster_count = None
        if os.path.exists(summary_path):
            try:
                with open(summary_path, "r", encoding="utf-8") as f:
                    summary = json.load(f)
                cluster_count = len(summary.get("clusters", []))
            except (OSError, json.JSONDecodeError):
                pass  # 요약 파일이 깨져 있어도 스캔 자체는 목록에 보여준다(클러스터 수만 비움).

        scans.append({
            "scan_id": scan_id,
            "timestamp": _parse_scan_id(scan_id),
            "cluster_count": cluster_count,
            "has_summary": os.path.exists(summary_path),
        })

    return scans

"""my_robot_pkg.pick_logger가 쓰는 SQLite DB를 읽기 전용으로 여는 모듈.

pick_logger.py가 WAL 모드로 여는 이유 자체가 "쓰는 쪽(motion_node)과 동시에
다른 프로세스(여기, HMI)가 읽을 수 있게" 하기 위함이라, 여기서는 그 계약을
그대로 이용한다 - 별도 락/재시도 없이 매 요청마다 짧게 열고 닫는다.
"""
import os
import sqlite3

DEFAULT_DB_PATH = os.path.join(os.path.expanduser("~"), ".ros", "my_robot_pkg", "pick_log.db")

MAX_ROWS = 200


def _connect(db_path=DEFAULT_DB_PATH):
    # uri=True + mode=ro: 실수로라도 이 쪽에서 쓰기 쿼리를 못 하게 원천 차단한다.
    uri = f"file:{db_path}?mode=ro"
    conn = sqlite3.connect(uri, uri=True)
    conn.row_factory = sqlite3.Row
    return conn


def db_exists(db_path=DEFAULT_DB_PATH):
    return os.path.exists(db_path)


def fetch_recent_attempts(limit=MAX_ROWS, db_path=DEFAULT_DB_PATH):
    """pick_attempts 테이블의 최근 기록을 최신순으로 반환한다.

    DB 파일이 아직 없으면(motion_node를 한 번도 안 띄웠으면) 빈 리스트를
    반환한다 - HMI는 로봇이 안 떠 있어도 뜰 수 있어야 하므로 예외를 던지지 않는다.
    """
    if not db_exists(db_path):
        return []

    conn = _connect(db_path)
    try:
        rows = conn.execute(
            "SELECT id, ts, obj_label, attempt_no, surface_z_mm, gripper_width_mm, "
            "grip_detected, motion_done, success "
            "FROM pick_attempts ORDER BY id DESC LIMIT ?",
            (limit,),
        ).fetchall()
    finally:
        conn.close()

    return [dict(r) for r in rows]


def fetch_summary(db_path=DEFAULT_DB_PATH):
    """오늘 하루 기준 총 시도/성공/성공률을 계산한다 (대시보드 요약 카드용)."""
    if not db_exists(db_path):
        return {"total": 0, "success": 0, "success_rate": None}

    conn = _connect(db_path)
    try:
        row = conn.execute(
            "SELECT COUNT(*) AS total, SUM(success) AS success FROM pick_attempts "
            "WHERE date(ts, 'unixepoch', 'localtime') = date('now', 'localtime')"
        ).fetchone()
    finally:
        conn.close()

    total = row["total"] or 0
    success = row["success"] or 0
    rate = round(100 * success / total, 1) if total else None
    return {"total": total, "success": success, "success_rate": rate}

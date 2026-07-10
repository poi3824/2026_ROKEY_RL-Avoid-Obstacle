# my_robot_pkg.pick_logger가 쓰는 SQLite DB를 읽기 전용으로 여는 모듈.
# src/hmi_interface/hmi_interface/pick_log_reader.py 그대로 이관 - 로직 무변경
# (그쪽 원본은 hmi_interface가 deprecated 처리될 때까지 그대로 둔다).
#
# pick_logger.py가 WAL 모드로 여는 이유 자체가 "쓰는 쪽(motion_node)과 동시에
# 다른 프로세스(여기, HMI)가 읽을 수 있게" 하기 위함이라, 여기서는 그 계약을
# 그대로 이용한다 - 별도 락/재시도 없이 매 요청마다 짧게 열고 닫는다.
import os
import sqlite3

DEFAULT_DB_PATH = os.path.join(os.path.expanduser("~"), ".ros", "my_robot_pkg", "pick_log.db")

MAX_ROWS = 200


def _connect(db_path=DEFAULT_DB_PATH):
    uri = f"file:{db_path}?mode=ro"
    conn = sqlite3.connect(uri, uri=True)
    conn.row_factory = sqlite3.Row
    return conn


def db_exists(db_path=DEFAULT_DB_PATH):
    return os.path.exists(db_path)


def fetch_recent_attempts(limit=MAX_ROWS, db_path=DEFAULT_DB_PATH):
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

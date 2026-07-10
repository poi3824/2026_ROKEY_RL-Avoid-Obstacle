# voice_interface.voice_logger가 쓰는 SQLite DB(voice_events)를 읽기 전용으로 여는 모듈.
# src/hmi_interface/hmi_interface/voice_log_reader.py 그대로 이관 - 로직 무변경.
import os
import sqlite3

DEFAULT_DB_PATH = os.path.join(os.path.expanduser("~"), ".ros", "voice_interface", "voice_log.db")

MAX_ROWS = 200


def _connect(db_path=DEFAULT_DB_PATH):
    uri = f"file:{db_path}?mode=ro"
    conn = sqlite3.connect(uri, uri=True)
    conn.row_factory = sqlite3.Row
    return conn


def db_exists(db_path=DEFAULT_DB_PATH):
    return os.path.exists(db_path)


def fetch_recent_events(limit=MAX_ROWS, db_path=DEFAULT_DB_PATH):
    if not db_exists(db_path):
        return []

    conn = _connect(db_path)
    try:
        rows = conn.execute(
            "SELECT id, ts, kind, text FROM voice_events ORDER BY id DESC LIMIT ?",
            (limit,),
        ).fetchall()
    finally:
        conn.close()

    return [dict(r) for r in rows]

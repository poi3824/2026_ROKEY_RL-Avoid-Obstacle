"""voice_interface.voice_logger가 쓰는 SQLite DB(voice_events)를 읽기 전용으로 여는 모듈.

pick_log_reader.py와 완전히 같은 패턴 - WAL 모드로 쓰는 쪽(get_keyword_node)과
동시에 이 쪽(HMI)이 읽어도 안전하다.
"""
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
    """voice_events 테이블의 최근 기록을 최신순으로 반환한다.

    DB 파일이 아직 없으면(get_keyword_node를 한 번도 안 띄웠으면) 빈 리스트를
    반환한다 - HMI는 음성 노드가 안 떠 있어도 뜰 수 있어야 한다.
    """
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

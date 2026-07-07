"""pick 시도 기록을 SQLite에 남기는 로거.

motion_executor.pick()이 시도할 때마다(성공/실패 모두) 한 행씩 남긴다.
나중에 HMI(Flask 등)가 같은 DB 파일을 읽기 전용으로 열어 조회/디버깅에 쓸 수 있도록,
쓰기 쪽(여기)에서 WAL 모드를 켜서 robot_action_node가 쓰는 동안에도 다른 프로세스가
동시에 읽을 수 있게 한다.
"""
import os
import sqlite3
import time

DEFAULT_DB_PATH = os.path.join(os.path.expanduser("~"), ".ros", "my_robot_pkg", "pick_log.db")

_SCHEMA = """
CREATE TABLE IF NOT EXISTS pick_attempts (
    id             INTEGER PRIMARY KEY AUTOINCREMENT,
    ts             REAL NOT NULL,
    obj_label      TEXT,
    attempt_no     INTEGER NOT NULL,
    surface_z_mm   REAL,
    gripper_width_mm REAL,
    grip_detected  INTEGER,
    motion_done    INTEGER,
    success        INTEGER NOT NULL
)
"""


class PickLogger:
    def __init__(self, db_path=DEFAULT_DB_PATH):
        os.makedirs(os.path.dirname(db_path), exist_ok=True)
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute(_SCHEMA)
        self._conn.commit()

    def log_attempt(
        self, obj_label, attempt_no, surface_z, gripper_width, grip_detected, motion_done, success,
    ):
        """pick() 한 번의 시도(attempt) 결과를 한 행으로 남긴다. 실패해도 예외를 올리지 않는다."""
        try:
            self._conn.execute(
                "INSERT INTO pick_attempts "
                "(ts, obj_label, attempt_no, surface_z_mm, gripper_width_mm, "
                " grip_detected, motion_done, success) VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                (
                    time.time(), obj_label, attempt_no, surface_z, gripper_width,
                    None if grip_detected is None else int(grip_detected),
                    None if motion_done is None else int(motion_done),
                    int(success),
                ),
            )
            self._conn.commit()
        except sqlite3.Error as e:
            print(f"[PickLogger] insert 실패, 기록 없이 계속 진행: {e}")

    def close(self):
        self._conn.close()

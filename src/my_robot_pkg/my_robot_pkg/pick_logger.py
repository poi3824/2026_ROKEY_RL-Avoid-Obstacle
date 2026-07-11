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
        self._migrate_add_angle_delta_column()
        self._conn.commit()

    def _migrate_add_angle_delta_column(self):
        # 2026-07-11: HMI Performance 탭의 그립 각도 정렬 게이지용으로 추가.
        # CREATE TABLE IF NOT EXISTS는 이미 존재하는(옛 스키마) 테이블엔 컬럼을
        # 안 얹어주므로, 기존에 pick_log.db를 갖고 있던 배포에서도 깨지지 않게
        # 컬럼 존재 여부를 직접 확인하고 없으면 ALTER TABLE로 추가한다.
        cols = [row[1] for row in self._conn.execute("PRAGMA table_info(pick_attempts)")]
        if "angle_delta_deg" not in cols:
            self._conn.execute("ALTER TABLE pick_attempts ADD COLUMN angle_delta_deg REAL")

    def log_attempt(
        self, obj_label, attempt_no, surface_z, gripper_width, grip_detected, motion_done, success,
        angle_delta_deg=None,
    ):
        """pick() 한 번의 시도(attempt) 결과를 한 행으로 남긴다. 실패해도 예외를 올리지 않는다.

        angle_delta_deg: motion_node.compute_grasp_c()가 계산한 grasp 정렬 오차(deg,
        [-90,90)) - 0에 가까울수록 그리퍼 축이 물체의 짧은 변에 정렬됨. get_grasp_delta
        콜백이 없거나(구형 호출부) 아직 탐지가 없었으면 None.
        """
        try:
            self._conn.execute(
                "INSERT INTO pick_attempts "
                "(ts, obj_label, attempt_no, surface_z_mm, gripper_width_mm, "
                " grip_detected, motion_done, success, angle_delta_deg) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (
                    time.time(), obj_label, attempt_no, surface_z, gripper_width,
                    None if grip_detected is None else int(grip_detected),
                    None if motion_done is None else int(motion_done),
                    int(success), angle_delta_deg,
                ),
            )
            self._conn.commit()
        except sqlite3.Error as e:
            print(f"[PickLogger] insert 실패, 기록 없이 계속 진행: {e}")

    def close(self):
        self._conn.close()

"""음성 상호작용(웨이크워드->STT->LLM->TTS) 이벤트를 SQLite에 남기는 로거.

my_robot_pkg.pick_logger와 같은 패턴 - HMI(hmi_interface)가 같은 DB 파일을
읽기 전용(WAL)으로 열어 STT-TTS 탭의 "STT" 서브탭에서 조회한다.

한 행 = 의미 있는 이벤트 하나(STT 인식 결과, 완성된 명령, 되묻기, STOP/RESUME/
IGNORE_HAND, TTS 발화). 여러 세부 테이블로 나누지 않고 kind로 구분되는 단일
이벤트 로그로 두는 이유는, "이 세션에 무슨 일이 있었는지"를 시간순으로 훑어보는
용도가 우선이기 때문이다 - 슬롯필링(멀티턴)으로 한 명령이 여러 라운드에 걸쳐
완성되는 흐름도 시간순으로 그대로 보인다.
"""
import os
import sqlite3
import time

DEFAULT_DB_PATH = os.path.join(os.path.expanduser("~"), ".ros", "voice_interface", "voice_log.db")

_SCHEMA = """
CREATE TABLE IF NOT EXISTS voice_events (
    id   INTEGER PRIMARY KEY AUTOINCREMENT,
    ts   REAL NOT NULL,
    kind TEXT NOT NULL,
    text TEXT
)
"""


class VoiceLogger:
    def __init__(self, db_path=DEFAULT_DB_PATH):
        os.makedirs(os.path.dirname(db_path), exist_ok=True)
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute(_SCHEMA)
        self._conn.commit()

    def log_event(self, kind, text):
        """이벤트 하나를 기록한다. 실패해도 예외를 올리지 않는다(로깅은 부가 기능이라
        _listen_loop/TTS 흐름을 막으면 안 된다)."""
        try:
            self._conn.execute(
                "INSERT INTO voice_events (ts, kind, text) VALUES (?, ?, ?)",
                (time.time(), kind, text),
            )
            self._conn.commit()
        except sqlite3.Error as e:
            print(f"[VoiceLogger] insert 실패, 기록 없이 계속 진행: {e}")

    def close(self):
        self._conn.close()

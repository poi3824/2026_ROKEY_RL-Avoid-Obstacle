import os

from dotenv import load_dotenv

load_dotenv()


class Config:
    HOST = os.environ.get("HMI_BACKEND_HOST", "0.0.0.0")
    PORT = int(os.environ.get("HMI_BACKEND_PORT", "5100"))
    BRIDGE_TOKEN = os.environ.get("HMI_BRIDGE_TOKEN", "")
    DEBUG = os.environ.get("HMI_BACKEND_DEBUG", "1") == "1"

    # 명령 중복 방지 / pending 응답 대기 TTL (초)
    COMMAND_DEDUP_TTL_SEC = float(os.environ.get("HMI_COMMAND_DEDUP_TTL_SEC", "60"))
    PENDING_CONTROL_TTL_SEC = float(os.environ.get("HMI_PENDING_CONTROL_TTL_SEC", "30"))

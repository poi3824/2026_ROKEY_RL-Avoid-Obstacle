# 실기 버그 회귀 테스트: `ros2 run hmi_ros_bridge hmi_ros_bridge_server`가 Ctrl+C로
# 안 죽던 문제. 원인은 socketio.Client(handle_sigint=True, 기본값)가 SIGINT 핸들러를
# 통째로 가로채서 rclpy.init()의 SIGINT 처리와 충돌하는 것이었다(socketio_worker.py
# 주석 참고) - 코드 리뷰만으로는 못 잡는 종류의 버그라(신호 처리는 프로세스 레벨
# 동작이라 단위 테스트로 안 보임), 실제 서브프로세스를 띄우고 진짜 SIGINT를 보내서
# 확인한다.
import os
import signal
import subprocess
import sys
import time

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_TEST_DIR)

_SCRIPT = """
import sys
sys.path.insert(0, {pkg_dir!r})
import rclpy
from hmi_ros_bridge.bridge_node import BridgeNode
from hmi_ros_bridge.emit_channel import EmitChannel
from hmi_ros_bridge.socketio_worker import SocketioWorker

rclpy.init()
emit_channel = EmitChannel()
node = BridgeNode(emit_channel)
worker = SocketioWorker(
    url="http://localhost:1", token="x",  # 연결 대상 없음 - 재시도 스레드만 돌면 충분
    emit_channel=emit_channel, command_handler=node.handle_command,
)
worker.start()
print("READY", flush=True)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    worker.stop()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print("SHUTDOWN_OK", flush=True)
"""


def test_sigint_actually_terminates_the_process():
    proc = subprocess.Popen(
        [sys.executable, "-u", "-c", _SCRIPT.format(pkg_dir=_PKG_DIR)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        deadline = time.time() + 10.0
        ready = False
        while time.time() < deadline:
            line = proc.stdout.readline()
            if "READY" in line:
                ready = True
                break
        assert ready, "프로세스가 rclpy.spin() 시작 전에 준비 안 됨"

        proc.send_signal(signal.SIGINT)

        try:
            remaining_output = proc.communicate(timeout=5.0)[0]
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.communicate()
            raise AssertionError(
                "SIGINT를 보냈는데 5초 안에 종료되지 않음 - "
                "socketio.Client(handle_sigint=False) 설정이 빠졌을 가능성"
            )

        assert "SHUTDOWN_OK" in remaining_output, f"정상 종료 로그 없음: {remaining_output}"
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()

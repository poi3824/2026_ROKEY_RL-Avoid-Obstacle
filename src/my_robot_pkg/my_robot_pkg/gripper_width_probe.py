"""디버깅용: 그리퍼 실제 너비(레지스터 267)와 status를 터미널에 계속 찍어주는 독립 스크립트.

gripper.py는 ROS가 아니라 raw socket으로 그리퍼와 직접 통신하므로 rclpy 없이 단독 실행된다.
손으로 그리퍼를 열고/비어 있는 채로 닫고/실제 통을 쥐게 하면서 get_width()가
기대한 mm 값을 돌려주는지 확인하는 용도. pick() 재시도 로직의 임계값을 정하기 전에
먼저 이 스크립트로 실측해본다.
"""
import time

from my_robot_pkg.gripper import RG2Gripper

TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
LOG_INTERVAL_SEC = 0.3


def main():
    gripper = RG2Gripper(TOOLCHARGER_IP, TOOLCHARGER_PORT)
    print("Ctrl+C로 종료. 그리퍼를 손으로 열고/닫고/통을 쥐어보면서 값을 확인하세요.")
    try:
        while True:
            width = gripper.get_width()
            busy, grip_detected = gripper.get_status()
            width_str = f"{width:.1f}mm" if width is not None else "read-fail"
            print(f"width={width_str}  busy={busy}  grip_detected={grip_detected}")
            time.sleep(LOG_INTERVAL_SEC)
    except KeyboardInterrupt:
        pass
    finally:
        gripper.close_connection()


if __name__ == "__main__":
    main()

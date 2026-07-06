# movel/movej/gripper 등 실제 로봇 구동을 전담하는 모듈.
#
# robot_action_node가 import해서 같은 프로세스 안에서 사용한다 (별도 ROS 노드 아님,
# 토픽/서비스가 아니라 그냥 함수 호출로 연결됨).
#
# 나중에 장애물 회피(rl_avoidance_node)가 붙으면, 이동(transit) 구간의
# move_linear를 /avoidance_cmd 반영 버전으로 바꾸는 지점이 이 클래스가 된다.
import time

LIFT_HEIGHT = 50.0
GRIPPER_SETTLE_SEC = 1  # 그리퍼가 닫히고/열리는 시간 대기


class MotionExecutor:
    def __init__(self, movel, movej, mwait, gripper, velocity, acc, stop):
        self._movel = movel
        self._movej = movej
        self._mwait = mwait
        self.gripper = gripper
        self.velocity = velocity
        self.acc = acc
        # 2026-07: DSR_ROBOT2에는 모션을 즉시 멈추는 stop() 함수가 없다
        # (drl_stop은 DRL 스크립트 실행 정지용이라 다른 것). motion/move_stop
        # [dsr_msgs2/MoveStop] 서비스를 직접 호출하는 함수를 robot_action_node가
        # 만들어서 movel/movej/mwait와 같은 방식으로 주입한다.
        self._stop = stop

    def move_linear(self, posx):
        self._movel(posx, self.velocity, self.acc)

    def wait(self, seconds=None):
        if seconds is None:
            self._mwait()
        else:
            self._mwait(seconds)

    def go_home(self, home_pos):
        """초기화 시 대기 위치로 이동하고 그리퍼를 연다 (robot_init 대응)."""
        self.move_linear(home_pos)
        self.wait(1.0)
        self.gripper.open_gripper()
        self.wait(1.0)

    def return_home(self, home_pos):
        """작업 완료 후 대기 위치로 복귀한다 (그리퍼 조작 없음)."""
        self.move_linear(home_pos)
        self.wait()

    def move_to_scan(self, scan_pos):
        """물체를 내려다보는 스캔 위치로 이동한다."""
        self.move_linear(scan_pos)
        self.wait()

    def pick(self, source_pos):
        """source_pos로 이동해 집고 LIFT_HEIGHT만큼 들어올린다."""
        self.move_linear(source_pos)
        self.wait()
        self.gripper.close_gripper()
        time.sleep(GRIPPER_SETTLE_SEC)

        lift_pos = source_pos[:2] + [source_pos[2] + LIFT_HEIGHT] + source_pos[3:]
        self.move_linear(lift_pos)
        self.wait()

    def place(self, target_pos):
        """target_pos 위로 이동한 뒤 LIFT_HEIGHT만큼 내려놓고 그리퍼를 연다."""
        self.move_linear(target_pos)
        self.wait()

        down_pos = target_pos[:2] + [target_pos[2] - LIFT_HEIGHT] + target_pos[3:]
        self.move_linear(down_pos)
        self.gripper.open_gripper()
        time.sleep(GRIPPER_SETTLE_SEC)

    def apply_avoidance_cmd(self, cmd):
        """rl_avoidance_node가 보내는 /avoidance_cmd(topic)를 받아 실시간으로
        이동 명령에 반영하는 자리. pointcloud_node/rl_avoidance_node가 아직
        없어서 미구현.
        """
        raise NotImplementedError("rl_avoidance_node 연동 전까지 미구현")
    
    def stop(self, stop_mode=1):
       
        # (0=QSTOP_STO, 1=QSTOP, 2=SSTOP, 3=HOLD)
        
        return self._stop(stop_mode)

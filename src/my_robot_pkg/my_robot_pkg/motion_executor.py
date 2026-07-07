# movel/movej/gripper 등 실제 로봇 구동을 전담하는 모듈.
#
# robot_action_node가 import해서 같은 프로세스 안에서 사용한다 (별도 ROS 노드 아님,
# 토픽/서비스가 아니라 그냥 함수 호출로 연결됨).
#
# 나중에 장애물 회피(rl_avoidance_node)가 붙으면, 이동(transit) 구간의
# move_linear를 /avoidance_cmd 반영 버전으로 바꾸는 지점이 이 클래스가 된다.

GRIPPER_TIMEOUT_SEC = 3.0  # 그리퍼 모션(busy bit 해제) 대기 타임아웃
PLACE_CLEARANCE = 10.0  # 표면 위 여유 간격(mm), 그리퍼가 바닥/테이블에 닿기 직전까지만 내려가게
PICK_HOVER_HEIGHT = 100.0  # depth를 읽을 안전 hover 높이(mm). source_pos 바로 옆에서 읽으면
                            # 그리퍼 손가락이 화면에 걸리거나 카메라-그리퍼 오프셋 때문에
                            # 엉뚱한 지점을 읽어서, place처럼 좀 떨어진 높이에서 먼저 읽는다.
PICK_CLEARANCE = 10.0  # 물체 표면 위 여유 간격(mm)


class MotionExecutor:
    def __init__(self, movel, movej, mwait, gripper, velocity, acc, stop, get_surface_z=None):
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
        self.get_surface_z = get_surface_z  # object_detection/detection.py의 get_surface_z()를 주입받음

    def move_linear(self, posx):
        self._movel(posx, self.velocity, self.acc)
        self._mwait()

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
        """source_pos 위 안전 hover 높이에서 depth를 확인하고 내려가 집는다.

        place와 동일한 패턴: 물체 바로 옆(source_pos)이 아니라 PICK_HOVER_HEIGHT만큼
        떨어진 위치에서 먼저 depth를 읽어야, 그리퍼 손가락이 화면에 걸리거나
        카메라-그리퍼 오프셋으로 엉뚱한 지점을 읽는 문제를 피할 수 있다.
        surface_z를 못 구하면(카메라 문제 등) source_pos 그대로 잡는다.

        Returns:
            bool: 그리퍼 status register의 grip_detected 비트로 확인한
            실제 파지 성공 여부. False면 그리퍼를 도로 열고 리턴한다
            (빈 그리퍼로 옮기는 것을 막기 위함).
        """
        hover_pos = source_pos[:2] + [source_pos[2] + PICK_HOVER_HEIGHT] + source_pos[3:]
        self.move_linear(hover_pos)
        self.wait()

        surface_z = self.get_surface_z() if self.get_surface_z else None
        if surface_z is not None:
            down_pos = source_pos[:2] + [surface_z + PICK_CLEARANCE] + source_pos[3:]
        else:
            down_pos = source_pos

        self.move_linear(down_pos)
        self.gripper.close_gripper()
        motion_done, grip_detected = self.gripper.wait_grip_done(GRIPPER_TIMEOUT_SEC)

        if not motion_done or not grip_detected:
            self.gripper.open_gripper()
            self.gripper.wait_grip_done(GRIPPER_TIMEOUT_SEC)
            return False

        self.move_linear(hover_pos)
        return True

    def place(self, target_pos):
        """target_pos 위로 이동한 뒤, 카메라 depth로 표면 위치를 확인해 내려놓고 그리퍼를 연다.

        get_surface_z 콜백이 없거나 depth를 못 읽으면(카메라 문제 등)
        target_pos 그대로 내려가는 것으로 fallback한다.
        """
        self.move_linear(target_pos)
        self.wait()

        surface_z = self.get_surface_z() if self.get_surface_z else None
        if surface_z is not None:
            down_pos = target_pos[:2] + [surface_z + PLACE_CLEARANCE] + target_pos[3:]
        else:
            down_pos = target_pos
            self.get_logger().warn("Depth를 못 읽어서 target_pos 그대로 내려감")

        self.move_linear(down_pos)
        self.gripper.open_gripper()
        self.gripper.wait_grip_done(GRIPPER_TIMEOUT_SEC)
        self.move_linear(target_pos)

    def apply_avoidance_cmd(self, cmd):
        """rl_avoidance_node가 보내는 /avoidance_cmd(topic)를 받아 실시간으로
        이동 명령에 반영하는 자리. pointcloud_node/rl_avoidance_node가 아직
        없어서 미구현.
        """
        raise NotImplementedError("rl_avoidance_node 연동 전까지 미구현")
    
    def stop(self, stop_mode=1):
       
        # (0=QSTOP_STO, 1=QSTOP, 2=SSTOP, 3=HOLD)
        
        return self._stop(stop_mode)

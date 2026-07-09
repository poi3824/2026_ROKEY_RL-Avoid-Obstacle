# movel/movej/gripper 등 실제 로봇 구동을 전담하는 모듈.
#
# robot_action_node가 import해서 같은 프로세스 안에서 사용한다 (별도 ROS 노드 아님,
# 토픽/서비스가 아니라 그냥 함수 호출로 연결됨).
#
# 나중에 장애물 회피(rl_avoidance_node)가 붙으면, 이동(transit) 구간의
# move_linear를 /avoidance_cmd 반영 버전으로 바꾸는 지점이 이 클래스가 된다.
#
# 2026-07: move_linear는 movel(동기)이 아니라 amovel(비동기)을 주입받아 쓴다.
# DSR_ROBOT2 소스 확인 결과 movel/amovel은 컨트롤러로 보내는 sync_type 값만 다르고,
# movel(sync_type=0)은 로봇이 물리적으로 멈출 때까지 서비스 응답 자체가 안 돌아와서
# 그동안 dsr_node가 spin되지 않는다 — 그 사이 들어온 /emergency_stop 요청이
# 로봇이 멈추고 나서야 처리된다는 뜻. amovel(sync_type=1)은 명령을 큐잉만 하고
# 바로 리턴하므로, 그 뒤 check_motion()을 짧은 간격으로 폴링하면서 dsr_node에
# spin 기회를 자주 줘서 응급 정지 요청이 이동 도중에도 즉시 처리되게 한다.
import contextlib
import time
import copy

_DR_STATE_IDLE = 0  # DSR_ROBOT2.DR_STATE_IDLE과 동일 (check_motion()이 이 값이면 정지 상태)
MOTION_POLL_INTERVAL = 0.05  # check_motion() 폴링 간격(초)

GRIPPER_TIMEOUT_SEC = 3.0  # 그리퍼 모션(busy bit 해제) 대기 타임아웃
PLACE_CLEARANCE = 10.0  # 표면 위 여유 간격(mm), 그리퍼가 바닥/테이블에 닿기 직전까지만 내려가게
PICK_HOVER_HEIGHT = 100.0  # depth를 읽을 안전 hover 높이(mm). source_pos 바로 옆에서 읽으면
                            # 그리퍼 손가락이 화면에 걸리거나 카메라-그리퍼 오프셋 때문에
                            # 엉뚱한 지점을 읽어서, place처럼 좀 떨어진 높이에서 먼저 읽는다.
# 2026-07-08: 파지 성공 후 place로 이동하기 전에 들르는 안전 높이. source_pos
# 상대값(hover_pos+오프셋)이 아니라 베이스 기준 절대 Z로 고정한다 — 물체마다
# depth 측정값이 들쭉날쭉해도 이동 중 안전 높이 자체는 항상 일정하게 유지된다.
PICK_RETRACT_Z = 460.0
PICK_CLEARANCE = 10.0  # 물체 표면 위 여유 간격(mm)

DEFAULT_GRIP_MIN_WIDTH = 30.0  # mm. 통이 바뀌면 이 fallback이 아니라 robot_action_node의
                                # ~grip_min_width_mm 파라미터를 바꿔서 대응한다 (재빌드 불필요).
PICK_MAX_ATTEMPTS = 3  # 최초 시도 + 재시도 2회

# 2026-07-07: 손 감지 일시정지는 stop_mode=1(QSTOP, Category 2)을 쓴다.
# emergency_stop의 stop_mode=0(QSTOP_STO)과 달리 서보 토크를 유지하므로,
# 손이 치워지면 같은 목적지로 movel을 다시 issue하는 것만으로 재개할 수 있다.
HAND_PAUSE_STOP_MODE = 1

MOTION_START_SETTLE_SEC = 0.15

# 2026-07-09: 스캔 스윕(sweep_to_detect) 전용 속도. ACTION_RESULT_TIMEOUT_SEC(130s,
# brain_node)에 여유가 충분해서(스윕 거리 100~300mm 기준 7~20초) 굳이 느리게 갈
# 필요는 없어 일반 이동과 같은 속도로 맞춘다.
SWEEP_VELOCITY = 30.0
SWEEP_ACC = 30.0
# hand 감지 타이머와 같은 주기 — object_detection_node에 과도한 부하를 주지 않으면서도
# 스윕 도중 물체가 보이면 충분히 빠르게 반응한다.
SWEEP_VISIBILITY_CHECK_INTERVAL_SEC = 0.3


class EmergencyStop(Exception):
    """move_linear가 폴링 도중 응급 정지 요청을 감지하면 발생시킨다."""


class MotionExecutor:
    def __init__(
        self, movel, movej, mwait, gripper, velocity, acc, stop,
        get_surface_z=None, redetect=None, grip_min_width=None, logger=None,
        check_motion=None, estop_event=None, hand_pause_event=None,
        dsr_lock=None,
    ):
        self._movel = movel  # amovel을 주입받는다 (위 모듈 주석 참고)
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
        self.redetect = redetect  # robot_action_node.get_target_pos(label)을 주입받음 (pick 재시도용)
        # 통 크기가 바뀌면 코드를 고치지 않고 ROS 파라미터로 조정할 수 있도록 주입받는다.
        self.grip_min_width = grip_min_width if grip_min_width is not None else DEFAULT_GRIP_MIN_WIDTH
        self.logger = logger  # pick_logger.PickLogger를 주입받음 (없으면 기록 생략)
        self._check_motion = check_motion  # DSR_ROBOT2.check_motion을 주입받음
        self._estop_event = estop_event  # robot_action_node의 threading.Event를 주입받음
        # 2026-07-07: object_detection_node가 /hand_detected 토픽으로 발행하는 걸
        # robot_action_node가 구독해서 세팅/해제하는 이벤트. move_linear는 그냥
        # 이 값만 폴링한다.
        self._hand_pause_event = hand_pause_event
        # amovel/check_motion/stop이 전부 dsr_node를 spin하므로, 동시 호출을
        # 막기 위해 이 락으로 감싼다.
        self._dsr_lock = dsr_lock if dsr_lock is not None else contextlib.nullcontext()

    def _handle_interrupts(self, posx, velocity, acc):
        """이동 폴링 도중 응급정지/손 감지 일시정지를 처리한다.

        move_linear와 sweep_to_detect가 공유하는 헬퍼다(2026-07-09 리팩터링 —
        원래 move_linear 안에 있던 로직을 그대로 뽑아낸 것, 동작 변화 없음).

        응급정지면 stop() 호출 후 EmergencyStop을 발생시켜 상위(pick/place/스윕)의
        나머지 단계를 중단시킨다. 손 일시정지면 stop_mode=1로 멈췄다가, 이벤트가
        풀리면 같은 posx로 movel을 다시 issue해서 이동을 재개한다 — 응급정지와
        달리 여기서 함수가 끝나지 않고 계속 진행된다.
        """
        if self._estop_event is not None and self._estop_event.is_set():
            self._stop()
            raise EmergencyStop("이동 중 응급 정지 감지")

        if self._hand_pause_event is not None and self._hand_pause_event.is_set():
            self._stop(HAND_PAUSE_STOP_MODE)
            while self._hand_pause_event.is_set():
                if self._estop_event is not None and self._estop_event.is_set():
                    raise EmergencyStop("일시정지 중 응급 정지 감지")
                time.sleep(MOTION_POLL_INTERVAL)
            with self._dsr_lock:
                self._movel(posx, velocity, acc)  # 같은 목적지로 재개
            # 재개도 amovel 재발행이라 처음과 같은 레이스가 재현될 수 있어 같은 지연을 준다.
            time.sleep(MOTION_START_SETTLE_SEC)

    def move_linear(self, posx):
        """amovel로 이동 명령을 큐잉하고, check_motion()을 폴링하며 완료를 기다린다.

        폴링 간격마다 dsr_node가 spin될 기회를 주기 때문에, 그 사이 들어온
        /emergency_stop 요청이 이 함수가 리턴하기 전에 처리될 수 있다. 응급 정지/
        손 감지 일시정지 처리는 _handle_interrupts()가 담당한다.

        2026-07-09: amovel 직후 check_motion()이 아직 컨트롤러에 새 명령이
        등록되기 전 상태(방금 끝난 이전 명령의 stale idle)를 읽고 너무 일찍
        리턴하는 레이스가 실기에서 확인됐다(예: pick 성공 후 후퇴 이동을 건너뛰고
        바로 다음 place 목적지로 가버림). 예전엔 호출부가 self.wait()(DSR_ROBOT2의
        raw mwait())로 한 번 더 확인했는데, mwait()은 hand_pause_event/estop_event를
        전혀 보지 않는 SDK 함수라(_handle_interrupts를 안 거침) 그 대기 도중 손이
        들어와도 정지/재개가 안 되는 구멍이 새로 생겼다. 그래서 raw mwait() 대신
        같은 인터럽트 인지 폴링(_poll_until_idle)을 두 번 돌려서 stale idle을
        잡는다 — 두 번째 폴링 동안에도 손 감지/응급정지를 계속 체크한다.
        """
        # self._stop은 (robot_action_node.stop) 자체적으로 dsr_lock을 잡으므로
        # 여기서는 self._movel/self._check_motion 호출만 감싼다 — self._stop
        # 호출을 dsr_lock 안에서 또 감싸면 non-reentrant lock이라 데드락난다.
        with self._dsr_lock:
            self._movel(posx, self.velocity, self.acc)  # amovel 큐잉, 바로 리턴

        if self._check_motion is None:
            # check_motion이 주입되지 않은 경우(테스트 등) 기존처럼 mwait로 대기.
            self._mwait()
            return

        self._poll_until_idle(posx)
        self._poll_until_idle(posx)  # stale-idle 재확인, 위 docstring 참고

        if self._estop_event is not None and self._estop_event.is_set():
            raise EmergencyStop("이동 중 응급 정지 감지")

    def _poll_until_idle(self, posx):
        """check_motion()이 IDLE을 보고할 때까지 폴링하며, 매 주기 _handle_interrupts()로
        손 감지 일시정지/응급 정지를 처리한다. move_linear()가 이 함수를 연달아 두 번
        호출해서 stale-idle 오탐을 잡는다(move_linear() docstring 참고).
        """
        time.sleep(MOTION_START_SETTLE_SEC)
        while True:
            with self._dsr_lock:
                motion_state = self._check_motion()
            if motion_state == _DR_STATE_IDLE:
                return

            self._handle_interrupts(posx, self.velocity, self.acc)
            time.sleep(MOTION_POLL_INTERVAL)

    def sweep_to_detect(self, pose_a, pose_b, is_visible_cb,
                        visibility_check_interval=SWEEP_VISIBILITY_CHECK_INTERVAL_SEC):
        """pose_a -> pose_b로 느리게(SWEEP_VELOCITY) 이동하며 물체가 온전히 보이는지
        주기적으로 확인하다가, 보이는 즉시 멈추고 True를 반환한다.

        2026-07-09: 고정 스캔 위치 하나만 보면 물체가 프레임 가장자리에 걸려
        반만 보일 수 있고, 그러면 세그멘테이션 마스크가 잘려 grasp 각도가
        틀리게 나온다(여러 프레임을 모아도 못 고치는 편향 — 실기로 확인됨).
        그래서 스캔 범위를 두 지점으로 넓히되, 무거운 8프레임 융합 탐지
        (get_best_detection)는 로봇이 완전히 멈춘 뒤에만 하고, 움직이는 동안은
        is_visible_cb(가벼운 단일 프레임 체크)로 "잘리지 않고 온전히 보이는지"만
        확인한다.

        pose_a에서 이미 보이면 스윕 자체를 생략한다. pose_b까지 다 갔는데도
        못 찾으면(왕복 없이) False를 반환한다 — 호출부(motion_node)가 기존
        pick 실패와 동일하게 처리한다.
        """
        self.move_linear(pose_a)
        if is_visible_cb():
            return True

        with self._dsr_lock:
            self._movel(pose_b, SWEEP_VELOCITY, SWEEP_ACC)

        if self._check_motion is None:
            self._mwait()
            return is_visible_cb()

        time.sleep(MOTION_START_SETTLE_SEC)
        last_check = time.time()
        while True:
            with self._dsr_lock:
                motion_state = self._check_motion()
            if motion_state == _DR_STATE_IDLE:
                break

            self._handle_interrupts(pose_b, SWEEP_VELOCITY, SWEEP_ACC)

            if time.time() - last_check >= visibility_check_interval:
                last_check = time.time()
                if is_visible_cb():
                    self._stop()
                    return True

            time.sleep(MOTION_POLL_INTERVAL)

        if self._estop_event is not None and self._estop_event.is_set():
            raise EmergencyStop("스윕 중 응급 정지 감지")

        # pose_b에 막 도착한 순간은 마지막 주기적 체크를 놓쳤을 수 있으니 한 번 더 확인.
        return is_visible_cb()

    def wait(self, seconds=None):
        if seconds is None:
            self._mwait()
        else:
            self._mwait(seconds)

    def go_home(self, home_pos):
        """초기화 시 대기 위치로 이동하고 그리퍼를 연다 (robot_init 대응)."""
        self.move_linear(home_pos)
        self.gripper.open_gripper()
        self.wait(1.0)

    def return_home(self, home_pos):
        """작업 완료 후 대기 위치로 복귀한다 (그리퍼 조작 없음)."""
        self.move_linear(home_pos)

    def move_to_scan(self, scan_pos):
        """물체를 내려다보는 스캔 위치로 이동한다."""
        self.move_linear(scan_pos)

    def pick(self, source_pos, obj_label=None, feedback_cb=None):
        """source_pos 위 안전 hover 높이에서 depth를 확인하고 내려가 집는다.

        place와 동일한 패턴: 물체 바로 옆(source_pos)이 아니라 PICK_HOVER_HEIGHT만큼
        떨어진 위치에서 먼저 depth를 읽어야, 그리퍼 손가락이 화면에 걸리거나
        카메라-그리퍼 오프셋으로 엉뚱한 지점을 읽는 문제를 피할 수 있다.
        surface_z를 못 구하면(카메라 문제 등) source_pos 그대로 잡는다.

        grip_detected 비트만으로는 빈 채로 오검출될 수 있어, 그리퍼가 닫힌 뒤
        실제 너비(레지스터 267)도 같이 확인한다. self.grip_min_width(통 크기에 맞춰
        robot_action_node의 ~grip_min_width_mm 파라미터로 조정) 밑으로 닫혔으면
        아무것도 안 잡힌 것으로 보고 재시도한다 — 다시 hover로 올라가 obj_label로
        카메라 재탐지 후 그 좌표로 다시 내려간다. redetect가 없거나 재탐지에
        실패하면 같은 source_pos로 재시도한다.

        Returns:
            bool: 실제 파지 성공 여부(모션 완료 + grip_detected + 너비 확인).
            PICK_MAX_ATTEMPTS번 다 실패하면 그리퍼를 열어둔 채 False를 반환한다
            (빈 그리퍼로 옮기는 것을 막기 위함).
        """
        for attempt in range(PICK_MAX_ATTEMPTS):
            if feedback_cb:
                feedback_cb("descending", attempt + 1)

            hover_pos = source_pos[:2] + [source_pos[2] + PICK_HOVER_HEIGHT] + source_pos[3:]
            temp = copy.deepcopy(hover_pos)
            temp[2] = PICK_RETRACT_Z
            moving_pos = temp

            self.move_linear(hover_pos)

            surface_z = self.get_surface_z() if self.get_surface_z else None
            if surface_z is not None:
                down_pos = source_pos[:2] + [surface_z + PICK_CLEARANCE] + source_pos[3:]
            else:
                down_pos = source_pos

            self.move_linear(down_pos)

            if feedback_cb:
                feedback_cb("gripping", attempt + 1)
            self.gripper.close_gripper()
            motion_done, grip_detected = self.gripper.wait_grip_done(GRIPPER_TIMEOUT_SEC)
            width = self.gripper.get_width()
            width_ok = width is None or width >= self.grip_min_width
            success = bool(motion_done and grip_detected and width_ok)
            print(f"motion_done:{motion_done}, grip_detected: {grip_detected}, width_ok: {width_ok}, success: {success}")

            if self.logger:
                self.logger.log_attempt(
                    obj_label, attempt + 1, surface_z, width, grip_detected, motion_done, success,
                )

            if success:
                # 파지 성공 후 PICK_RETRACT_Z로 후퇴. 여기서 다음 move_linear(place
                # 등)를 곧장 이어붙여도 안전한 이유는 move_linear() 자체의 stale-idle
                # 재확인 로직(move_linear() docstring, 2026-07-09 참고)이 처리한다 —
                # 예전엔 여기서 self.wait()를 따로 호출해서 막았었다.
                self.move_linear(moving_pos)
                return True

            else:
                print(
                    f"[MotionExecutor] pick 실패 (attempt {attempt + 1}/{PICK_MAX_ATTEMPTS}): "
                    f"motion_done={motion_done} grip_detected={grip_detected} width={width}"
                )
                self.gripper.open_gripper()
                self.gripper.wait_grip_done(GRIPPER_TIMEOUT_SEC)
                self.move_linear(hover_pos)

                if attempt == PICK_MAX_ATTEMPTS - 1:
                    return False

                if feedback_cb:
                    feedback_cb("retry", attempt + 2)

                if self.redetect and obj_label:
                    redetected_pos = self.redetect(obj_label)
                    if redetected_pos is not None:
                        source_pos = redetected_pos

        return False

    def place(self, target_pos, feedback_cb=None):
        """target_pos 위로 이동한 뒤, 카메라 depth로 표면 위치를 확인해 내려놓고 그리퍼를 연다.

        get_surface_z 콜백이 없거나 depth를 못 읽으면(카메라 문제 등)
        target_pos 그대로 내려가는 것으로 fallback한다.
        """
        self.move_linear(target_pos)

        if feedback_cb:
            feedback_cb("descending")

        surface_z = self.get_surface_z() if self.get_surface_z else None
        if surface_z is not None:
            down_pos = target_pos[:2] + [surface_z + PLACE_CLEARANCE] + target_pos[3:]
        else:
            down_pos = target_pos
            print("[MotionExecutor] Depth를 못 읽어서 target_pos 그대로 내려감")

        self.move_linear(down_pos)

        if feedback_cb:
            feedback_cb("releasing")
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

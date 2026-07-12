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
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

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

# 2026-07-10: move_via_rl()이 접근할 target 상단 안전 높이(mm) — PICK_HOVER_HEIGHT/
# PICK_RETRACT_Z와 대칭. target_pos 그대로가 아니라 이만큼 띄운 지점을 RL의 목표로 준다.
PLACE_HOVER_HEIGHT = 100.0

DEFAULT_GRIP_MIN_WIDTH = 30.0  # mm. 통이 바뀌면 이 fallback이 아니라 robot_action_node의
                                # ~grip_min_width_mm 파라미터를 바꿔서 대응한다 (재빌드 불필요).
PICK_MAX_ATTEMPTS = 3  # 최초 시도 + 재시도 2회

# 2026-07-07: 손 감지 일시정지는 stop_mode=1(QSTOP, Category 2)을 쓴다.
# emergency_stop의 stop_mode=0(QSTOP_STO)과 달리 서보 토크를 유지하므로,
# 손이 치워지면 같은 목적지로 movel을 다시 issue하는 것만으로 재개할 수 있다.
HAND_PAUSE_STOP_MODE = 1
# 2026-07-09: safety_monitor.ESTOP_STOP_MODE와 동일한 값(HOLD) — 서보 토크를
# 유지한 채 멈추므로 RESUME 시 같은 posx로 movel을 다시 issue해 이어갈 수 있다.
ESTOP_HOLD_STOP_MODE = 3

MOTION_START_SETTLE_SEC = 0.15

# 2026-07-09: 스캔 스윕(sweep_to_detect) 전용 속도. ACTION_RESULT_TIMEOUT_SEC(130s,
# brain_node)에 여유가 충분해서(스윕 거리 100~300mm 기준 7~20초) 굳이 느리게 갈
# 필요는 없어 일반 이동과 같은 속도로 맞춘다.
SWEEP_VELOCITY = 30.0
SWEEP_ACC = 30.0
# hand 감지 타이머와 같은 주기 — object_detection_node에 과도한 부하를 주지 않으면서도
# 스윕 도중 물체가 보이면 충분히 빠르게 반응한다.
SWEEP_VISIBILITY_CHECK_INTERVAL_SEC = 0.3
# motion_node.CHECK_VISIBILITY_TIMEOUT과 같은 값 — 논블로킹 가시성 체크(아래
# sweep_to_detect 참고)가 이 시간 넘게 응답이 없으면 그 요청은 포기하고 다음
# 주기에 새로 시작한다. 값 자체는 두 파일에 독립적으로 존재하지만(모듈 결합을
# 피하려고), 같은 서비스 호출의 타임아웃이므로 한쪽만 바꾸면 다른 쪽도 맞출 것.
CHECK_VISIBILITY_STALL_TIMEOUT_SEC = 2.0


class EmergencyStop(Exception):
    """move_linear가 폴링 도중 응급 정지 요청을 감지하면 발생시킨다."""


class GoalCancelled(Exception):
    """move_linear/sweep_to_detect가 폴링 도중 goal 취소 요청을 감지하면 발생시킨다.

    estop_event/hand_pause_event(안전 상태 기반, RUN이 되면 자동으로 clear됨)와
    별개의 cancel_event를 쓴다 — 취소는 "멈췄다 재개"가 아니라 "멈추면 그걸로 끝"이라
    _handle_interrupts의 hold-and-resume 루프에 태우면 안 되고, safety 하트비트가
    estop_event를 지워도 취소 자체는 무효화되면 안 되기 때문이다.
    """


class MotionExecutor:
    def __init__(
        self, movel, movej, mwait, gripper, velocity, acc, stop,
        get_surface_z=None, redetect=None, grip_min_width=None, logger=None,
        check_motion=None, estop_event=None, hand_pause_event=None,
        dsr_lock=None, amovej=None, get_current_posj=None, get_current_posx=None,
        cancel_event=None, get_grasp_delta=None, on_rl_step=None, on_grasp_progress=None,
    ):
        self._movel = movel  # amovel을 주입받는다 (위 모듈 주석 참고)
        self._movej = movej
        self._mwait = mwait
        # 2026-07-10: move_via_rl()/move_joint() 전용 — dsr_policy_path 통합을 위해 추가.
        # amovej는 amovel의 조인트 스페이스 버전(vendor SDK에 존재 확인), get_current_posj/x는
        # 매 스텝 실제 상태를 읽어 정책에 넣거나 수렴 여부를 판단하는 데 쓴다.
        self._amovej = amovej
        self._get_current_posj = get_current_posj
        self._get_current_posx = get_current_posx
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
        self.get_grasp_delta = get_grasp_delta  # motion_node.get_last_grasp_delta를 주입받음 (없으면 None 기록)
        # 2026-07-12 (v2): attempt 완료 후 딱 1번 쏘던 정적 스냅샷(on_grasp_logged)을
        # 없애고, move_linear(hover_pos, ...)가 회전+접근하는 동안 매 폴링 틱마다
        # "지금 wrist yaw가 grasp_c에서 얼마나 남았는지"를 실시간으로 쏘는 방식으로
        # 바꿨다 - 탐지 시점의 오차에서 시작해 실제로 정렬되는 과정 자체가 게이지에
        # 보이도록. DB 로깅(log_attempt)은 그대로 유지 - 이건 화면 표시 경로만 바뀜.
        self.on_grasp_progress = on_grasp_progress
        # 2026-07-12: move_via_rl()이 매 스텝 호출 - motion_node가 hmi/rl_reach_progress로
        # 발행하도록 주입받음(없으면 print()만 하고 넘어감, 기존 동작 그대로).
        self.on_rl_step = on_rl_step
        # 2026-07-12 버그 수정: _align_tcp_vertical()이 move_via_rl()과 같은
        # episode_id/goal_threshold_m/max_steps를 재사용해 hmi/rl_reach_progress를
        # 이어서 발행할 수 있도록 캐싱해둔다(아래 move_via_rl() 참고) - 새
        # episode_id를 쓰면 RL 오차 차트가 "새 에피소드"로 오인해 리셋돼버린다.
        self._last_rl_episode_id = None
        self._last_rl_goal_threshold_m = None
        self._last_rl_max_steps = None
        self._check_motion = check_motion  # DSR_ROBOT2.check_motion을 주입받음
        self._estop_event = estop_event  # robot_action_node의 threading.Event를 주입받음
        # 2026-07-07: object_detection_node가 /hand_detected 토픽으로 발행하는 걸
        # robot_action_node가 구독해서 세팅/해제하는 이벤트. move_linear는 그냥
        # 이 값만 폴링한다.
        self._hand_pause_event = hand_pause_event
        # 2026-07-09 버그 수정: goal 취소 전용 이벤트. estop_event에 얹어 쓰면
        # safety_monitor의 RUN 하트비트(0.5초 주기)가 estop_event를 지울 때
        # 취소 자체도 같이 무효화돼버리는 레이스가 있었다 — 완전히 분리한다.
        self._cancel_event = cancel_event
        # amovel/check_motion/stop이 전부 dsr_node를 spin하므로, 동시 호출을
        # 막기 위해 이 락으로 감싼다.
        self._dsr_lock = dsr_lock if dsr_lock is not None else contextlib.nullcontext()

    def _check_cancelled(self):
        """cancel_event가 세팅돼 있으면 로봇을 멈추고 GoalCancelled를 던진다.

        estop/hand_pause와 달리 "멈췄다 재개"가 없다 — 취소는 멈추면 그걸로 끝이라
        _handle_interrupts의 hold-and-resume 루프를 타면 안 된다.
        """
        if self._cancel_event is not None and self._cancel_event.is_set():
            self._stop(HAND_PAUSE_STOP_MODE)
            raise GoalCancelled("goal 취소 요청 감지")

    def _handle_interrupts(self, pos, velocity, acc, reissue=None):
        """이동 폴링 도중 응급정지/손 감지 일시정지를 처리한다.

        move_linear/move_joint/sweep_to_detect가 공유하는 헬퍼다. reissue는 재개 시
        큐잉할 함수(기본 self._movel) — move_joint()는 self._amovej를 넘겨서 조인트
        스페이스 이동도 같은 방식으로 정지/재개되게 한다.

        2026-07-09: 응급정지도 손 감지 일시정지와 똑같이 "그 자리에서 홀드 후
        재개" 방식으로 바뀌었다(이전엔 액션 자체를 중단시켰는데, 사용자가 "정지"
        후 "다시 시작해"라고 하면 하던 동작을 그대로 이어가길 원해서 바꿈).
        safety_monitor의 ESTOP_STOP_MODE가 HOLD(서보 토크 유지)로 바뀐 덕에,
        정지 뒤에도 같은 목적지로 movel/movej를 다시 issue하는 것만으로 이동을
        이어갈 수 있다. 응급정지/손 일시정지 둘 다 같은 방식이라 하나로 합쳤다 —
        둘 중 하나라도 활성 상태면 멈추고, 둘 다 풀릴 때까지 기다렸다가 재개한다.
        """
        self._check_cancelled()
        reissue = self._movel if reissue is None else reissue

        estop_active = self._estop_event is not None and self._estop_event.is_set()
        hand_active = self._hand_pause_event is not None and self._hand_pause_event.is_set()
        if not (estop_active or hand_active):
            return

        self._stop(ESTOP_HOLD_STOP_MODE if estop_active else HAND_PAUSE_STOP_MODE)
        while True:
            self._check_cancelled()
            estop_active = self._estop_event is not None and self._estop_event.is_set()
            hand_active = self._hand_pause_event is not None and self._hand_pause_event.is_set()
            if not (estop_active or hand_active):
                break
            time.sleep(MOTION_POLL_INTERVAL)

        with self._dsr_lock:
            reissue(pos, velocity, acc)  # 같은 목적지로 재개
        # 재개도 amovel/amovej 재발행이라 처음과 같은 레이스가 재현될 수 있어 같은 지연을 준다.
        time.sleep(MOTION_START_SETTLE_SEC)

    def move_linear(self, posx, on_angle_progress=None, on_tick=None):
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

        on_angle_progress: 2026-07-12 추가. 주어지면 폴링 틱마다 현재 wrist yaw가
        이 이동의 목표 orientation(posx[5])에서 얼마나 남았는지(deg, mod-180 정규화)를
        실어 호출한다 - pick()이 grasp_c로 회전+접근하는 hover_pos 이동에만 넘긴다
        (다른 이동은 그냥 안 넘기면 됨, 기본값 None이라 동작 변화 없음).

        on_tick: 2026-07-12 추가. on_angle_progress와 달리 계산 없이 매 폴링 틱마다
        현재 flange posx(get_current_posx()[0])를 그대로 실어 호출만 한다 - 호출부가
        원하는 값(예: _align_tcp_vertical()의 tilt_x/tilt_y)을 알아서 계산하게 하는
        범용 훅이다. 기본값 None이면 아무 일도 안 함.
        """
        self._check_cancelled()

        # self._stop은 (robot_action_node.stop) 자체적으로 dsr_lock을 잡으므로
        # 여기서는 self._movel/self._check_motion 호출만 감싼다 — self._stop
        # 호출을 dsr_lock 안에서 또 감싸면 non-reentrant lock이라 데드락난다.
        with self._dsr_lock:
            self._movel(posx, self.velocity, self.acc)  # amovel 큐잉, 바로 리턴

        if self._check_motion is None:
            # check_motion이 주입되지 않은 경우(테스트 등) 기존처럼 mwait로 대기.
            self._mwait()
            return

        self._poll_until_idle(posx, on_angle_progress=on_angle_progress, on_tick=on_tick)
        self._poll_until_idle(posx, on_angle_progress=on_angle_progress, on_tick=on_tick)  # stale-idle 재확인, 위 docstring 참고

        self._check_cancelled()
        if self._estop_event is not None and self._estop_event.is_set():
            raise EmergencyStop("이동 중 응급 정지 감지")

    def move_joint(self, posj):
        """amovej로 이동 명령을 큐잉하고, check_motion()을 폴링하며 완료를 기다린다.

        move_linear()의 조인트 스페이스 버전 — 이중 폴링(stale-idle 방어)과
        손 감지/응급정지 처리(_handle_interrupts, reissue=amovej)까지 완전히 동일한
        구조다. move_via_rl()이 매 스텝 이걸 호출한다(2026-07-10).
        2026-07-10 병합: move_linear()와 마찬가지로 _check_cancelled()를 시작/끝
        양쪽에서 호출한다 — 안 그러면 place_via_rl()의 RL 스텝 도중 goal이
        취소돼도 이 함수 진입 시점/폴링 종료 직후의 좁은 창에서는 못 잡는다
        (폴링 루프 내부는 _poll_until_idle -> _handle_interrupts를 통해 이미 매
        주기 확인된다).
        """
        self._check_cancelled()

        with self._dsr_lock:
            self._amovej(posj, self.velocity, self.acc)

        if self._check_motion is None:
            self._mwait()
            return

        self._poll_until_idle(posj, reissue=self._amovej)
        self._poll_until_idle(posj, reissue=self._amovej)

        self._check_cancelled()
        if self._estop_event is not None and self._estop_event.is_set():
            raise EmergencyStop("이동 중 응급 정지 감지")

    def _poll_until_idle(self, pos, reissue=None, on_angle_progress=None, on_tick=None):
        """check_motion()이 IDLE을 보고할 때까지 폴링하며, 매 주기 _handle_interrupts()로
        손 감지 일시정지/응급 정지를 처리한다. move_linear()/move_joint()가 이 함수를
        연달아 두 번 호출해서 stale-idle 오탐을 잡는다(move_linear() docstring 참고).

        on_angle_progress/on_tick: move_linear()의 동명 파라미터 참고.
        """
        time.sleep(MOTION_START_SETTLE_SEC)
        while True:
            self._emit_angle_progress(pos, on_angle_progress)
            if on_tick is not None:
                with self._dsr_lock:
                    current_posx = self._get_current_posx()
                if current_posx is not None:
                    on_tick(current_posx[0])
            with self._dsr_lock:
                motion_state = self._check_motion()
            if motion_state == _DR_STATE_IDLE:
                return

            self._handle_interrupts(pos, self.velocity, self.acc, reissue=reissue)
            time.sleep(MOTION_POLL_INTERVAL)

    def _emit_angle_progress(self, target_pos, on_angle_progress):
        """target_pos[5](목표 wrist yaw, deg)와 현재 실제 wrist yaw의 차이를
        mod-180 정규화(그리퍼가 대칭이라 [-90,90) - motion_node.compute_grasp_c()와
        동일 규칙)해서 on_angle_progress에 실어 호출한다. 콜백/get_current_posx
        둘 다 없으면 조용히 넘어간다(기존 move_linear() 호출부는 영향 없음)."""
        if on_angle_progress is None or self._get_current_posx is None:
            return
        with self._dsr_lock:
            current_yaw = self._get_current_posx()[0][5]
        target_yaw = target_pos[5]
        delta_deg = ((target_yaw - current_yaw + 90.0) % 180.0) - 90.0
        on_angle_progress(delta_deg)

    def sweep_to_detect(self, pose_a, pose_b, is_visible_cb,
                        start_visibility_cb=None, poll_visibility_cb=None,
                        visibility_check_interval=SWEEP_VISIBILITY_CHECK_INTERVAL_SEC,
                        visibility_stall_timeout=CHECK_VISIBILITY_STALL_TIMEOUT_SEC):
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

        2026-07-09 버그 수정: 이동 중 반복 체크는 start_visibility_cb(논블로킹,
        요청만 큐잉)/poll_visibility_cb(논블로킹, 완료 여부만 확인)로 한다.
        이전엔 is_visible_cb()를 이동 폴링 루프 안에서 동기 호출했는데, 이
        호출이 끝날 때까지(최대 수 초) 같은 루프의 _handle_interrupts()가 전혀
        안 돌아 손 감지/응급정지 반응이 지연되는 문제가 있었다. is_visible_cb는
        로봇이 이미 멈춰 있는 pose_a/최종 확인 시점에만 쓴다 — 그때는 모션
        폴링 루프 자체가 없어 블로킹이어도 인터럽트 반응성에 영향이 없다.
        start/poll 콜백이 없으면(하위 호환) 이전처럼 is_visible_cb를 그대로 쓴다.
        """
        self.move_linear(pose_a)
        if is_visible_cb():
            return True

        with self._dsr_lock:
            self._movel(pose_b, SWEEP_VELOCITY, SWEEP_ACC)

        if self._check_motion is None:
            self._mwait()
            return is_visible_cb()

        use_async_check = start_visibility_cb is not None and poll_visibility_cb is not None

        time.sleep(MOTION_START_SETTLE_SEC)
        last_check = time.time()
        pending_since = None
        while True:
            self._check_cancelled()
            with self._dsr_lock:
                motion_state = self._check_motion()
            if motion_state == _DR_STATE_IDLE:
                break

            self._handle_interrupts(pose_b, SWEEP_VELOCITY, SWEEP_ACC)

            if not use_async_check:
                if time.time() - last_check >= visibility_check_interval:
                    last_check = time.time()
                    if is_visible_cb():
                        self._stop()
                        return True
            elif pending_since is not None:
                visible = poll_visibility_cb()
                if visible is not None:
                    pending_since = None
                    if visible:
                        self._stop()
                        return True
                elif time.time() - pending_since > visibility_stall_timeout:
                    pending_since = None  # 응답 없는 요청은 포기하고 다음 주기에 새로 시작
            elif time.time() - last_check >= visibility_check_interval:
                last_check = time.time()
                start_visibility_cb()
                pending_since = time.time()

            time.sleep(MOTION_POLL_INTERVAL)

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

            # on_angle_progress: 이 이동의 orientation이 곧 grasp_c(짧은 변 정렬 목표)라
            # HMI 그립 각도 게이지가 여기서만 실시간으로 갱신된다(아래 다른 move_linear
            # 호출들은 정렬과 무관한 이동이라 안 넘김).
            self.move_linear(hover_pos, on_angle_progress=self.on_grasp_progress)

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
            # 2026-07-09 버그 수정: get_width()는 소켓 레벨에서 이미 2회 재시도한
            # 뒤에도 실패해야 None을 반환한다 — 그런데도 None을 "통과"로 두면
            # 이 검사를 추가한 목적(grip_detected 오탐 방지)이 정확히 그 순간에
            # 무력화된다(Modbus 통신 문제 + 빈 그립이 겹치면 놓침). None도 실패로
            # 처리해 기존 재시도/재탐지 경로를 타게 한다.
            width_ok = width is not None and width >= self.grip_min_width
            success = bool(motion_done and grip_detected and width_ok)
            print(f"motion_done:{motion_done}, grip_detected: {grip_detected}, width_ok: {width_ok}, success: {success}")

            if self.logger:
                grasp_delta_deg = self.get_grasp_delta() if self.get_grasp_delta else None
                self.logger.log_attempt(
                    obj_label, attempt + 1, surface_z, width, grip_detected, motion_done, success,
                    grasp_delta_deg,
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

    def move_via_rl(self, target_pos, max_steps=None, goal_threshold_m=None):
        """dsr_policy_path의 학습된 reach 정책으로 현재 자세에서 target_pos(x,y,z, mm,
        base frame)까지 movej 스텝을 반복해 이동한다(2026-07-10).

        원본 dsr_policy_path.run_policy_live()와 달리 DEFAULT_JOINT_POS로 리셋하는
        첫 스텝이 없다 — pick 직후 hover/후퇴 자세에서 바로 시작한다(이 체크포인트는
        reset_joints_from_ik_table로 다양한 시작 자세를 학습했다는 근거로 생략했지만,
        실기에서 직접 검증된 가정은 아니다 — 첫 테스트에서 눈여겨볼 지점).

        매 스텝이 move_joint()를 거치므로 손 감지/응급정지가 스텝 단위로 반응한다 —
        스텝 하나의 관절 이동폭만큼은 못 끊는다. 이 체크포인트는 장애물 회피를 학습
        하지 않았다(dsr_policy_path 모듈 docstring 캐비어트 1) — 지금은 RL 통합 자체의
        신뢰성부터 검증하는 단계로 진행한다.

        target_pos의 orientation(rx,ry,rz)은 정책이 무시한다(캐비어트 4) — x,y,z만 쓴다.

        Returns:
            bool: goal_threshold_m 이내로 수렴하면 True, max_steps 안에 못 하면(또는
            관절 한계 세이프티로 중단되면) False.
        """
        from my_robot_pkg import dsr_policy_path as rl  # 첫 호출까지 torch/체크포인트 로드를 늦춘다

        max_steps = rl.MAX_STEPS if max_steps is None else max_steps
        goal_threshold_m = rl.GOAL_POS_THRESHOLD_M if goal_threshold_m is None else goal_threshold_m
        # 2026-07-10 버그 수정: target_pos는 이 코드베이스의 다른 모든 곳(move_linear 등)과
        # 동일하게 FLANGE 기준 pose다 — RG2 TCP는 195mm 오프셋만큼 떨어져 있는데(모듈
        # 상단 주석 참고), 그걸 안 거치고 target_pos[:3]를 그대로 정책의 목표 TCP
        # 위치로 썼었다. 그러면 policy_step()의 pos_err 계산(TCP 위치 vs target_pos_m)이
        # 애초에 도달 불가능한 목표를 기준으로 계산돼 절대 수렴하지 않는다(실기에서
        # pos_err가 25.4cm 근처에 멈춰 안 줄어드는 것으로 확인됨 — flange_posx_to_tcp_pos_m
        # 없이 direct offset을 썼을 때의 오차와 방향이 일치). target_pos의 orientation을
        # 그대로 써서(_align_tcp_vertical과 동일한 함수) 실제 TCP가 도달해야 할 지점을 구한다.
        target_pos_m = rl.flange_posx_to_tcp_pos_m(target_pos)

        # 2026-07-12: HMI Z축 정렬 레이더 게이지용 - "목표 접근축을 따라 내려다본 뷰"의
        # 기준 평면(목표 자세 자신의 로컬 X/Y축)을 미리 구해둔다. target_pos의 orientation은
        # 정책이 무시하지만(위 캐비어트 4), _align_tcp_vertical이 수렴 후 피벗할 최종
        # 목표 자세이기도 해서 "정렬돼야 할 방향"의 기준으로 쓰기에 정확히 맞는 값이다.
        target_rotation = Rotation.from_euler("ZYZ", target_pos[3:6], degrees=True)
        target_x_axis = target_rotation.apply([1.0, 0.0, 0.0])
        target_y_axis = target_rotation.apply([0.0, 1.0, 0.0])

        # 2026-07-12: HMI Performance 탭의 RL 스텝별 오차 차트용 - 호출 1번(= 에피소드
        # 1개)마다 새 id를 발급해서 on_rl_step 구독 쪽(프론트)이 "새 에피소드 시작"을
        # step==0 여부가 아니라 이 id 변화로 판단하게 한다(이벤트 유실에도 안전).
        episode_id = str(uuid.uuid4())
        # 버그 수정: _align_tcp_vertical()이 이 값들을 재사용해 같은 episode_id로
        # hmi/rl_reach_progress를 이어서 발행할 수 있게 캐싱(위 __init__ 주석 참고).
        self._last_rl_episode_id = episode_id
        self._last_rl_goal_threshold_m = goal_threshold_m
        self._last_rl_max_steps = max_steps

        prev_action = np.zeros(6, dtype=np.float32)
        pos_err_m = None  # max_steps=0(호출 안 됨)이면 아래 max_steps 도달 이벤트가 None을 그대로 보냄
        tilt_x = tilt_y = None
        for step in range(max_steps):
            with self._dsr_lock:
                current_posj_deg = self._get_current_posj()

            target_joint_pos_deg, prev_action, diag = rl.policy_step(
                current_posj_deg, target_pos_m, prev_action
            )

            ok, worst = rl.check_joint_limit_safety(target_joint_pos_deg)
            if not ok:
                print(
                    f"[MotionExecutor] RL reach 중단: joint_{worst + 1} 목표가 물리 한계 "
                    f"안전마진 안으로 들어옴 ({step}스텝째)"
                )
                self._emit_rl_step(
                    episode_id, step, None, goal_threshold_m, max_steps,
                    done=True, reason="joint_limit_abort",
                )
                return False

            self.move_joint(target_joint_pos_deg)

            with self._dsr_lock:
                flange_posx = self._get_current_posx()[0]
            tcp_pos_m = rl.flange_posx_to_tcp_pos_m(flange_posx)
            pos_err_m = float(np.linalg.norm(tcp_pos_m - target_pos_m))

            # 2026-07-12: 목표 접근축을 따라 내려다봤을 때 현재 TCP Z축이 찍히는 점의
            # 좌표(목표 자세의 로컬 X/Y축에 대한 내적 = sin(기울기각)의 성분). ZAxisAlignGauge의
            # 레이더 뷰가 그대로 쓴다 - 벡터가 단위벡터라 추가 정규화가 필요 없다.
            current_z_axis = Rotation.from_euler(
                "ZYZ", flange_posx[3:6], degrees=True
            ).apply([0.0, 0.0, 1.0])
            tilt_x = float(np.dot(current_z_axis, target_x_axis))
            tilt_y = float(np.dot(current_z_axis, target_y_axis))

            print(
                f"[MotionExecutor] RL reach step {step}: pos_err={pos_err_m * 100:.1f}cm "
                f"(raw_action_norm={diag['raw_action_norm']:.3f})"
            )

            if pos_err_m < goal_threshold_m:
                print(f"[MotionExecutor] RL reach 목표 도달 ({step + 1}스텝, {pos_err_m * 100:.1f}cm)")
                self._emit_rl_step(
                    episode_id, step, pos_err_m, goal_threshold_m, max_steps,
                    done=True, reason="goal_reached", tilt_x=tilt_x, tilt_y=tilt_y,
                )
                return True

            self._emit_rl_step(
                episode_id, step, pos_err_m, goal_threshold_m, max_steps, done=False,
                tilt_x=tilt_x, tilt_y=tilt_y,
            )

        print(f"[MotionExecutor] RL reach: max_steps({max_steps}) 도달, 목표 미도달")
        self._emit_rl_step(
            episode_id, max_steps - 1, pos_err_m, goal_threshold_m, max_steps,
            done=True, reason="max_steps", tilt_x=tilt_x, tilt_y=tilt_y,
        )
        return False

    def _emit_rl_step(
        self, episode_id, step, pos_err_m, goal_threshold_m, max_steps, done, reason=None,
        tilt_x=None, tilt_y=None,
    ):
        """on_rl_step 콜백이 있으면 HMI용 페이로드로 감싸 호출한다(없으면 조용히 무시 -
        기존 print() 기반 동작과 100% 동일하게 유지). pos_err_m은 joint_limit_abort처럼
        아직 한 번도 못 구했을 수 있어 None을 허용한다. tilt_x/tilt_y도 마찬가지 이유로
        None을 허용한다(joint_limit_abort 시점엔 이번 스텝의 flange_posx를 아직 안 읽음)."""
        if self.on_rl_step is None:
            return
        self.on_rl_step({
            "episode_id": episode_id,
            "step": step,
            "pos_err_mm": None if pos_err_m is None else pos_err_m * 1000.0,
            "goal_threshold_mm": goal_threshold_m * 1000.0,
            "max_steps": max_steps,
            "done": done,
            "reason": reason,
            "tilt_x": tilt_x,
            "tilt_y": tilt_y,
        })

    def _align_tcp_vertical(self, target_orientation):
        """move_via_rl()이 멈춘 자리에서 TCP 위치는 고정한 채 orientation만
        target_orientation(ZYZ euler, deg)으로 맞춘다(2026-07-10).

        dsr_policy_path의 정책은 orientation을 학습 보상(tcp_axis_alignment)으로만
        느슨하게 맞추므로(캐비어트 4), move_via_rl() 도착 시 TCP가 살짝 기울어 있을 수
        있다. 이후 하강/배치 단계가 정확한 자세에서 시작하도록 여기서 한 번 정렬한다 —
        TCP 위치(제자리)는 그대로 두고 orientation만 바꾸는 피벗이라, move_linear() 하나로
        처리할 수 있다.

        2026-07-12 버그 수정: move_via_rl()의 스텝 루프 안에서만 tilt_x/tilt_y(HMI Z축
        정렬 게이지용)를 발행했고, 정작 그 기울기를 실제로 교정하는 이 이동 중에는
        발행이 전혀 없었다 - 그래서 게이지가 이 정렬이 끝날 때까지 move_via_rl()의
        마지막 스텝 값에 멈춰 있는 것처럼 보였다. move_via_rl()과 완전히 동일한 공식
        (target_x_axis/target_y_axis = target_orientation의 로컬 X/Y축)으로 매 폴링
        틱마다 계산해 같은 hmi/rl_reach_progress로 이어서 발행한다 - 새 episode_id를
        발급하면 RL 오차 차트가 "새 에피소드"로 오인해 방금 그린 차트를 리셋해버리므로,
        move_via_rl()이 캐싱해둔 episode_id/goal_threshold_m/max_steps를 그대로 재사용한다
        (move_via_rl()이 한 번도 안 불렸으면 - 있을 수 없는 경로지만 - 새로 발급).
        """
        from my_robot_pkg import dsr_policy_path as rl

        with self._dsr_lock:
            current_flange_posx = self._get_current_posx()[0]
        tcp_pos_m = rl.flange_posx_to_tcp_pos_m(current_flange_posx)

        r_new = Rotation.from_euler("ZYZ", target_orientation, degrees=True)
        flange_pos_m = tcp_pos_m - r_new.apply(rl.FLANGE_TO_TCP_OFFSET_M)
        new_posx = list(flange_pos_m * 1000.0) + list(target_orientation)

        target_x_axis = r_new.apply([1.0, 0.0, 0.0])
        target_y_axis = r_new.apply([0.0, 1.0, 0.0])
        episode_id = self._last_rl_episode_id or str(uuid.uuid4())
        goal_threshold_m = self._last_rl_goal_threshold_m or 0.0
        max_steps = self._last_rl_max_steps or 0

        def _on_tick(flange_posx):
            current_z_axis = Rotation.from_euler(
                "ZYZ", flange_posx[3:6], degrees=True
            ).apply([0.0, 0.0, 1.0])
            tilt_x = float(np.dot(current_z_axis, target_x_axis))
            tilt_y = float(np.dot(current_z_axis, target_y_axis))
            self._emit_rl_step(
                episode_id, max_steps, None, goal_threshold_m, max_steps,
                done=False, tilt_x=tilt_x, tilt_y=tilt_y,
            )

        self.move_linear(new_posx, on_tick=_on_tick)

        # 정렬 완료 시점의 최종 tilt도 한 번 더 쏴서(폴링 마지막 틱과 실제 정지 사이
        # 미세한 차이까지 반영) 게이지가 확실히 최종값에 멈추게 한다.
        with self._dsr_lock:
            final_flange_posx = self._get_current_posx()[0]
        _on_tick(final_flange_posx)

    def place_via_rl(self, target_pos, feedback_cb=None):
        """pick 직후 hover 자세에서 RL 정책으로 target_pos 상단까지 옮기고, 도착
        지점에서 orientation을 재정렬한 뒤, depth 기반으로 실제 내려놓기까지
        마친다(2026-07-10, move_to_place_hover에서 확장).

        RL 이동(move_via_rl) + 정렬(_align_tcp_vertical)까지는 TCP 위치/자세만
        맞추는 접근 단계이고, 그 뒤 하강/그리퍼 개방/후퇴는 기존 place()의 depth
        기반 로직을 그대로 재사용한다 — target_pos에 박힌 z값이 아니라
        get_surface_z()로 그 순간 실측한 표면 높이를 기준으로 내려가므로, hover
        접근 높이(PLACE_HOVER_HEIGHT)를 얼마로 잡든 실제 배치 높이 자체는
        안전하다.

        Returns:
            bool: RL이 목표에 수렴 못 하면 False(정렬/하강/배치 전부 생략).
            그 외에는 True(하강 depth를 못 읽어도 target_pos로 fallback해서
            끝까지는 진행한다 — 기존 place()와 동일한 정책).
        """
        hover_pos = target_pos[:2] + [target_pos[2] + PLACE_HOVER_HEIGHT] + target_pos[3:]

        if feedback_cb:
            feedback_cb("rl_transit")
        if not self.move_via_rl(hover_pos):
            return False

        if feedback_cb:
            feedback_cb("aligning")
        self._align_tcp_vertical(target_pos[3:6])

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
        # 2026-07-12: 놓은 자리(target_pos)로 그대로 복귀하는 대신 hover_pos(위에서
        # 이미 계산해둔 접근 높이)로 떠서 끝낸다 - pick()의 성공 후 PICK_RETRACT_Z
        # 후퇴와 대칭되는 자리. brain_node.execute_command()가 이 직후 매 물체마다
        # home으로 보내므로, 여기서 물체 바로 위에 떠 있어야 그 home 이동이 방금
        # 놓은 물체를 스치지 않고 안전하게 시작된다.
        self.move_linear(hover_pos)
        return True

    def apply_avoidance_cmd(self, cmd):
        """rl_avoidance_node가 보내는 /avoidance_cmd(topic)를 받아 실시간으로
        이동 명령에 반영하는 자리. pointcloud_node/rl_avoidance_node가 아직
        없어서 미구현.
        """
        raise NotImplementedError("rl_avoidance_node 연동 전까지 미구현")
    
    def stop(self, stop_mode=1):
       
        # (0=QSTOP_STO, 1=QSTOP, 2=SSTOP, 3=HOLD)
        
        return self._stop(stop_mode)

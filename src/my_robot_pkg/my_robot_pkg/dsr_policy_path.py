"""Real-robot deployment wrapper for the trained LongRun PureReach policy
(2026-07-07_21-10-00/model_49999.pt, 50000-iteration NoIK run), see conversation 2026-07-08.

Usage (on the robot control PC, inside a script using DSR_ROBOT2 -- DR_init must already be configured,
see main() below for the setup this needs):

    from dsr_policy_path import run_policy_live
    executed_path = run_policy_live(target_posx=[x_mm, y_mm, z_mm, rx, ry, rz])

WHAT THIS DOES
--------------
The trained policy is a per-step REACTIVE controller (observation -> action every control step). This
runs it LIVE and CLOSED-LOOP: at every step it reads the robot's ACTUAL current joint position (via
DSR_ROBOT2's get_current_posj()), computes the next action from THAT real state, commands the robot
there (movej + mwait), and only then reads the real position again for the next step.

This replaces an earlier OFFLINE/quasi-static version that assumed each commanded target was reached
exactly before computing the next observation, with no real feedback in the loop. That assumption was
verified (see conversation 2026-07-06) to accumulate large position drift over many steps (300+ mm by
the end of a rollout) -- tracking error that doesn't show up in a single step compounds badly when every
subsequent observation is computed from an already-wrong assumed state. Reading the REAL position back
every step avoids this entirely, at the cost of needing a live connection to the robot (no more
precompute-a-path-then-execute-later; see conversation for why this trade was made).

Per-step action formula (CORRECTED 2026-07-08, see conversation -- a prior version of this module had
this backwards): checked directly against this run's saved ``params/env.yaml``, which shows
``use_default_offset: true`` / ``use_current_pos_offset: false`` for ``arm_action`` -- i.e. the policy's
own target is an offset from the FIXED default pose every step, NOT an incremental delta from wherever
the arm currently is:

    policy_target = DEFAULT_joint_pos + 0.1 * raw_action(obs)

This matters because the applied joint delta each step is ``policy_target - CURRENT_joint_pos`` (mirrors
``mdp/actions.py``'s ``apply_actions()`` exactly), NOT ``scale*raw_action`` alone. The two coincide only
when the arm happens to be exactly at the default pose (e.g. this module's own step 0, right after homing
there); once the arm has moved away from default at all, they diverge, growing worse each further step
away. This is suspected to have been the actual root cause of repeated real-robot divergence seen while
debugging this (present from the first deployment of this module onward): every Isaac Sim reproduction of
the same scenario necessarily used the correct training-code formula and converged fine, while the real
robot -- running this module's WRONG formula -- diverged from step 1 onward in a growing, monotonic way,
exactly consistent with a progressively-wrong offset compounding every step rather than a one-off error.

(IK guidance is disabled for this checkpoint -- ``rate_ramp_steps=1`` -- so ``rate`` is 1.0 from
essentially the first control step onward and the IK term's own contribution to the blend is
negligible; not replicated here). The OBSERVATION'S ``joint_pos`` term is relative to the same fixed
default pose (``mdp.joint_pos_rel`` against the asset's ``default_joint_pos``) -- obs layout is otherwise
unchanged from the previous checkpoint:

    obs = concat(
        joint_pos - DEFAULT_JOINT_POS,      # 6
        joint_vel (assumed 0 -- see caveat 6),  # 6
        [x,y,z,qw,qx,qy,qz] of target in base_link frame,           # 7
        5 copies of [0,0,-5, 0,0,0, 0] (5 obstacle slots x [rel_pos(3),half_extent(3),presence(1)] --
                  this checkpoint was trained with 0 obstacles always, all slots parked at (0,0,-5) far
                  below the floor; see conversation 2026-07-06 -- an earlier version fed all-zeros here,
                  which is NOT what an inactive slot looked like during training),  # 35
        previous raw_action (0 on the very first step),             # 6
    )                                                                  # = 60 total

CAVEATS (read before trusting this on real hardware)
-----------------------------------------------------
1. This checkpoint (PureReach) was trained with ZERO obstacles -- it has NEVER learned to avoid
   anything. Do not run it near obstacles/people without independent safety monitoring.
2. Low-X targets (target_x roughly < 0.25m in the base frame, i.e. reaching mostly sideways rather than
   forward) are a confirmed weak region -- see conversation 2026-07-08: across the FULL 50000-iteration
   training run, 404/1554 table points NEVER once reached the 0.02m goal threshold, and 96% of those
   have x<0.25m. Separately verified these ARE kinematically reachable (real IK solutions exist, no
   virtual-object/floor collision, valid motion paths found) -- this is a genuine policy competence gap,
   not a physical limitation, so a low-X target may simply fail to converge in the field. Treat any
   low-x target result with extra caution/verification, especially very-near-base targets (xy<0.1m from
   the base axis), which are the worst subset.
3. Each step is a real, executed robot motion (movej + mwait) -- there is no "preview the path first"
   step anymore. Start with a short max_steps / slow vel,acc and a human ready to e-stop.
4. target_posx's rx/ry/rz are IGNORED (see conversation 2026-07-06). This checkpoint's downward-facing
   gripper behavior comes from a SEPARATE reward (tcp_axis_alignment) tied to a fixed WORLD axis, not
   from the target orientation observation -- during training, that observation's roll/pitch were never
   randomized at all (roll=0, pitch=-pi/2 always, only yaw varied; see
   config/m0609/joint_pos_env_cfg.py's `ranges.pitch = (-1.5708, -1.5708)`). Feeding an arbitrary
   user-supplied orientation there was verified to be WORSE than ignoring it, so this module always
   feeds the fixed, training-matching value below and never looks at target_posx[3:6].
5. target_posx position (x,y,z, the only part that's actually used) is in DSR's native MILLIMETERS
   (converted to meters internally).
6. joint_vel is still always fed as 0 (no real per-step velocity estimate) -- a simpler simplification
   than the old quasi-static position one, and lower-risk: velocity noise was part of training anyway
   (see joint_vel's Unoise in reach_obstacle_avoidance_env_cfg.py's PolicyCfg), so 0 is a plausible noisy
   sample rather than a structurally-wrong assumption like the old "instantly reached target" one was.
7. Early stopping now uses the robot's REAL current_posx (via get_current_posx()) compared against the
   target, since real feedback is available every step -- unlike the old offline version, this is a
   genuine, meaningful check, not just a documented no-op.
8. Checkpoint choice: this uses iter 50000 (the final checkpoint) per explicit request. Note (see
   conversation 2026-07-08) that this run's OWN iter-3000 checkpoint scored higher on both the training
   distribution and a held-out generalization test (goal_reached 82.4% vs 79.1%) -- iter 50000 was kept
   here only because it was explicitly asked for, not because it's this run's best-performing checkpoint.
9. No reactive floor-avoidance correction is applied here (REMOVED 2026-07-08, see conversation): the
   training lineage this deployment path now tracks (NoFloorJac onward) no longer includes the reactive
   Jacobian-based floor-avoidance action correction at all -- the ablation confirmed the policy can learn
   real floor avoidance for the virtual held-object from the dense ``grasped_object_floor_proximity_
   penalty`` reward gradient alone (that penalty stayed small, -0.0002 to -0.0007, throughout the ablation
   run with no action-level correction). Deploying a checkpoint trained WITHOUT this correction while
   still applying it here would be a genuine train/deploy mismatch (the exact class of bug this whole
   debugging effort has been about), so it was deleted rather than left in as a no-op. If a FUTURE
   checkpoint is ever trained WITH the Jacobian correction again, this module would need it re-added --
   check the training config's ``floor_avoidance_object_length_m`` before assuming otherwise.
10. A real deployment run shows a persistent period-2 far/close oscillation in ``pos_err`` (see
    conversation 2026-07-08) for the SAME checkpoint/target that Isaac Sim converges smoothly on --
    traced to movej+mwait reaching each computed target EXACTLY every step (tracking error ~0.01-0.03deg),
    unlike sim's actual PD-actuator/decimation physics (measured settling ratio ~0.20-0.25 mean, but not a
    stable constant -- varies a lot per joint, so there's no simple fixed ratio to replicate). EMA-
    smoothing the movej TARGET across steps was TRIED as a fix and made things WORSE -- it converged to a
    stable but WRONG fixed point (best err 9.2cm, never under 5cm) instead, because smoothing the target
    changes the actual joint trajectory the arm follows, which changes what the network observes next
    step -- an out-of-distribution trajectory relative to training (where the arm reaches the RAW target
    every step), i.e. a NEW mismatch introduced by the fix. Reverted that approach.

    UPDATED (2026-07-09/10): instead of smoothing the TARGET across steps, ``SETTLING_RATIO`` (below)
    scales EACH step's OWN computed delta independently before executing it (``executed_delta_rad =
    SETTLING_RATIO * joint_delta_rad``) -- the policy's per-step decision cadence is unchanged, and the
    REAL resulting position (not a fictional smoothed one) is what the next observation reads. This is
    a genuinely different mechanism from the reverted EMA-target-smoothing above, not a re-application of
    it -- see ``policy_step()``/its call sites for the current, single source of truth for this formula.
"""

from __future__ import annotations

import os

import numpy as np
import torch
from scipy.spatial.transform import Rotation

try:
    from ament_index_python.packages import get_package_share_directory

    CHECKPOINT_PATH = os.path.join(
        get_package_share_directory("my_robot_pkg"), "resource", "model_10000_obs_avoid.pt"
    )
except (ImportError, LookupError):
    # fallback for running this file directly (not as an installed ROS2 package), e.g. plain `python3
    # dsr_policy_path.py` from the source tree rather than `ros2 run`/an installed console_script.
    CHECKPOINT_PATH = os.path.join(
        os.path.dirname(__file__), "..", "resource", "model_10000_obs_avoid.pt"
    )
# 2026-07-13: PureReach(model_14000_smooth.pt, 장애물 0개로 학습)에서 실제 장애물
# 커리큘럼으로 학습된 iter 10000 체크포인트로 교체. actor 구조(60->128->128->6, ELU)는
# 동일해서 로딩 코드는 변경 없음 -- curriculum_state(success_ema 0.992, collision_ema
# 0.0003)로 실제 장애물 학습된 체크포인트임을 확인했다. obstacle 슬롯 규약(좌표계/
# half_extent 의미)은 이 머신에 training config가 없어 100% 검증은 못했고, 실기
# 관찰(raw_action이 장애물 유무에 따라 실제로 달라짐)로 1차 확인만 된 상태 --
# 이상 동작 시 아래 obstacles_to_obs_block()의 half_extent/rel_pos 가정부터 의심할 것.
DEFAULT_JOINT_POS_DEG = np.array([0.0, 0.0, 90.0, 0.0, 90.0, 180.0])
DEFAULT_JOINT_POS_RAD = np.radians(DEFAULT_JOINT_POS_DEG)
ACTION_SCALE = 0.1
ACTION_CLAMP = 5.0
# Fraction of each step's computed joint delta actually commanded (see conversation 2026-07-09, caveat
# 10 above for the full reasoning) -- matches the measured ~0.20-0.25 mean settling ratio of training's
# sim physics (PD-actuator dynamics under decimation=2 completing only part of a step's intended delta,
# not an instant teleport-to-target the way movej+mwait's exact tracking does). Midpoint of that measured
# range; not a stable per-joint constant, so this is a single approximation, not an exact match.
SETTLING_RATIO = 0.225
# 2026-07-10: 0.03 -> 0.04로 상향. 실기에서 pos_err가 3.5cm(구 threshold 3cm 바로
# 위)에 멈춰 수렴 판정을 못 받고 계속 도는 경우가 확인됨(dsr_policy_path 모듈
# docstring 캐비어트 10의 "genuine but wrong equilibrium"과 일치) - 아직 충분히
# 학습되지 않은 체크포인트라는 걸 감안해 도달 판정 범위를 넓힌다.
GOAL_POS_THRESHOLD_M = 0.04
MAX_STEPS = 360  # matches this task's time_out horizon
# Interactive prompt's default (see conversation 2026-07-08) -- deliberately much lower than MAX_STEPS
# (the task's true horizon, still used as run_policy_live's own function-level default): while this
# checkpoint/deployment path is still being validated on real hardware, a short default keeps each
# interactive test conservative (a few steps to check behavior, not the full horizon) unless the user
# explicitly types a larger number.
DEFAULT_INTERACTIVE_MAX_STEPS = 50

# Hard joint limits (deg), from the URDF (joint_3 is the binding constraint at +-150deg, the rest
# +-360deg). Single safety abort, run_policy_live's per-step check below: aborts before commanding a
# target_joint_pos_deg (this step's CURRENT joint angle + the policy's computed delta) that falls within
# this margin of the real m0609 limit on any joint (see conversation 2026-07-08, caught from a real-robot
# runaway: joint_4 wound up to -329deg, joint_6 past -360deg, one step at a time, never reversing, for a
# target the policy was clearly struggling with -- previously caught by a separate fixed
# drift-from-default-pose check; merged here since checking the actual physical limit against the real
# current position is the more direct, physically-meaningful bound, and joint_3's tight +-150deg range
# means a runaway is caught on that joint well before 90deg of drift regardless).
JOINT_LIMITS_DEG = np.array([360.0, 360.0, 150.0, 360.0, 360.0, 360.0])
JOINT_LIMIT_SAFETY_MARGIN_DEG = 10.0  # abort before actually reaching the hard limit, not at it
# fixed target orientation matching training's roll=0, pitch=+pi/2, yaw=0 (quat_from_euler_xyz(0,
# +pi/2, 0) in isaaclab.utils.math's w,x,y,z convention) -- see caveat 4 above for why this is a fixed
# constant rather than derived from caller input. NOTE (2026-07-06): the sign of pitch was previously
# wrong (-pi/2, giving local +X -> world +Z, i.e. UP, verified by direct quaternion-rotation math) --
# had no effect on trained behavior since this observation slot isn't reward-tied (tcp_axis_alignment
# uses a separate, fixed-world-axis check, not this command value), but was still a real error, caught
# while regenerating the training's IK-based start-pose table with the same constant.
TARGET_QUAT_WXYZ = np.array([0.70710678, 0.0, 0.70710678, 0.0], dtype=np.float32)

# Loads the raw .pt checkpoint directly (see conversation 2026-07-08) instead of the exported ONNX model
# -- a diagnostic control to test whether the real-robot divergence seen with the ONNX path also occurs
# with a completely independent inference implementation (rules out an onnxruntime-specific execution
# bug, as opposed to the export itself: already cross-checked in Isaac Sim to match the true rsl_rl
# policy to ~1e-6, so this isn't expected to change behavior, but the real robot is the actual test that
# matters here). Reconstructs just the actor's mean-action MLP from actor_state_dict, mirroring
# export_onnx.py's ActorWrapper exactly (128,128 hidden, ELU -- see rsl_rl_ppo_cfg.py).
_checkpoint = torch.load(CHECKPOINT_PATH, map_location="cpu", weights_only=False)
_actor_keys = _checkpoint["actor_state_dict"]
_linear_keys = sorted({k.split(".")[1] for k in _actor_keys if k.startswith("mlp.") and "weight" in k}, key=int)
_layers = []
for _i, _idx in enumerate(_linear_keys):
    _w = _actor_keys[f"mlp.{_idx}.weight"]
    _layers.append(torch.nn.Linear(_w.shape[1], _w.shape[0]))
    if _i < len(_linear_keys) - 1:
        _layers.append(torch.nn.ELU())
_policy_mlp = torch.nn.Sequential(*_layers)
_sd = {}
_li = 0
for _idx in _linear_keys:
    _sd[f"{_li}.weight"] = _actor_keys[f"mlp.{_idx}.weight"]
    _sd[f"{_li}.bias"] = _actor_keys[f"mlp.{_idx}.bias"]
    _li += 2
_policy_mlp.load_state_dict(_sd)
_policy_mlp.eval()

# inactive-obstacle-slot pattern actually seen during training (see conversation 2026-07-06):
# mdp/events.py's reset_obstacles parks inactive slots at park_pos=(0,0,-5) (env-local, far below the
# floor), NOT (0,0,0) -- each slot's observation is [rel_pos(3), half_extent(3), presence(1)], so an
# inactive slot is [0,0,-5, 0,0,0, 0]. Feeding all-zeros instead (as an earlier version of this module
# did) put a value the network never saw during training into 15 of the 35 obstacle-block entries.
_INACTIVE_SLOT = np.array([0.0, 0.0, -5.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
_OBSTACLES_ALL_INACTIVE = np.tile(_INACTIVE_SLOT, 5)  # 5 slots x 7 = 35
MAX_OBSTACLE_SLOTS = 5

# world_map_algo.RECORD_DIR과 동일한 경로 (pointcloud_perception 패키지 의존성을
# 새로 만들지 않기 위해 여기서는 그냥 같은 절대경로를 하드코딩 -- HMI backend .env
# fallback도 같은 방식으로 이 경로를 하드코딩하고 있어 이 레포에서 이미 쓰는 패턴).
WORLD_MAPS_DIR = os.path.expanduser("~/RL-Avoid-Obstacle/data/world_maps")


def load_latest_world_map_obstacles(world_maps_dir=WORLD_MAPS_DIR):
    """가장 최근 world_map_update_* 스캔의 clusters를 반환한다. 스캔이 하나도 없으면
    빈 리스트 -- move_via_rl()이 "월드맵 업데이트해줘"를 아직 한 번도 안 받은 상태에서도
    죽지 않고 장애물 인식 없이(전부 비활성 슬롯) 그대로 동작해야 하기 때문에 예외를
    던지지 않는다(2026-07-13 테스트 스크립트의 load_latest_world_map_obstacles()는
    없으면 RuntimeError -- 대화형 테스트용이라 그게 맞았지만, 프로덕션 경로는 그래서는
    안 된다는 차이가 있다)."""
    import glob
    import json

    scan_dirs = sorted(glob.glob(os.path.join(world_maps_dir, "world_map_update_*")))
    if not scan_dirs:
        return []
    summary_path = os.path.join(scan_dirs[-1], "world_map_summary.json")
    try:
        with open(summary_path) as f:
            return json.load(f)["clusters"]
    except (OSError, KeyError, ValueError):
        return []


def obstacles_to_obs_block(clusters):
    """world_map cluster 리스트(centroid/safety_radius/safety_height) -> obstacle
    observation 5-slot(35차원) 배열. rel_pos는 base_link 기준 centroid(m) 그대로
    (target_pos와 같은 프레임이라는 가정), half_extent는 [safety_radius,
    safety_radius, safety_height/2] (WorldMapObstacle.msg 주석: "RL/motion
    planning에서 그대로 쓸 안전 여유 포함 값" -- 이 용도로 이미 계산돼 있는 필드).
    5개 넘으면 앞 5개만 사용, 정렬/필터링은 하지 않는다."""
    slots = []
    for c in clusters[:MAX_OBSTACLE_SLOTS]:
        cx, cy, cz = c["centroid"]
        half_extent = [c["safety_radius"], c["safety_radius"], c["safety_height"] / 2.0]
        slots.append(np.array([cx, cy, cz, *half_extent, 1.0], dtype=np.float32))
    while len(slots) < MAX_OBSTACLE_SLOTS:
        slots.append(_INACTIVE_SLOT.copy())
    obs_block = np.concatenate(slots)
    assert obs_block.shape[0] == 35
    return obs_block


def get_live_obstacles_obs():
    """move_via_rl()이 스텝 루프 시작 전 한 번 호출 -- 최신 world_map 스캔을 읽어
    obstacle observation으로 변환한다. 스캔이 없거나 읽기 실패하면 조용히
    _OBSTACLES_ALL_INACTIVE로 폴백한다(장애물 인식 없이 기존 PureReach와 동일하게
    동작 -- 스캔 파일 문제로 pick-and-place 전체가 죽으면 안 되므로)."""
    clusters = load_latest_world_map_obstacles()
    if not clusters:
        return _OBSTACLES_ALL_INACTIVE
    return obstacles_to_obs_block(clusters)

# get_current_posx() reports the FLANGE (tool0) pose, not the actual RG2 gripper tip -- the controller's
# own TCP/tool setting (set_tcp/get_tcp) doesn't apply while under external/PC control (see conversation
# 2026-07-06), so this offset has to be applied here instead. Derived from the URDF's tool0->rg2_body
# (rpy 0,-pi/2,0) -> rg2_tcp (xyz 0.195,0,0) chain: works out to exactly 195mm along the flange's own
# local Z axis (the usual "TCP along tool Z" convention).
# 2026-07-10: made public (was _FLANGE_TO_TCP_OFFSET_M) -- motion_executor.py's TCP-fixed-point
# realignment step (after move_via_rl()) needs this same offset.
FLANGE_TO_TCP_OFFSET_M = np.array([0.0, 0.0, 0.195])


def flange_posx_to_tcp_pos_m(flange_posx):
    """flange_posx: [x_mm,y_mm,z_mm,rx,ry,rz] (ZYZ euler, deg) as returned by get_current_posx() ->
    actual rg2_tcp position in the base frame, meters."""
    flange_pos_m = np.asarray(flange_posx[:3], dtype=np.float64) / 1000.0
    R_flange_to_base = Rotation.from_euler("ZYZ", flange_posx[3:6], degrees=True)
    return flange_pos_m + R_flange_to_base.apply(FLANGE_TO_TCP_OFFSET_M)


def _build_obs(joint_pos_rad, joint_vel_rad_s, target_pos_m, target_quat_wxyz, prev_action, obstacles_obs=None):
    obstacles_obs = _OBSTACLES_ALL_INACTIVE if obstacles_obs is None else obstacles_obs
    joint_pos_rel = joint_pos_rad - DEFAULT_JOINT_POS_RAD
    obs = np.concatenate(
        [joint_pos_rel, joint_vel_rad_s, target_pos_m, target_quat_wxyz, obstacles_obs, prev_action]
    ).astype(np.float32)
    assert obs.shape[0] == 60, f"expected 60-dim obs, got {obs.shape[0]}"
    return obs


def policy_step(current_posj_deg, target_pos_m, prev_action, target_quat_wxyz=None, obstacles_obs=None):
    """한 스텝 분의 정책 추론 + SETTLING_RATIO 적용까지 마친 다음 관절 목표를 계산한다.

    motion_executor.MotionExecutor.move_via_rl()과 아래 run_policy_live() 둘 다 이 함수 하나를
    쓴다 -- 실행 델타 계산식(특히 SETTLING_RATIO)이 두 군데서 따로 구현되면 한쪽만 고치고
    다른 쪽을 깜빡하는 식으로 조용히 어긋날 위험이 있어 한 곳으로 모았다.

    Args:
        current_posj_deg: 현재 관절각(deg), get_current_posj() 그대로.
        target_pos_m: 목표 TCP 위치(x,y,z), base frame, meters. orientation은 정책이
            무시한다(모듈 docstring 캐비어트 4) -- 여기 넘길 필요 없음.
        prev_action: 이전 스텝의 raw_action(np.ndarray, 6). 첫 스텝은 zeros(6).
        obstacles_obs: 35차원 obstacle observation(get_live_obstacles_obs()/
            obstacles_to_obs_block() 결과). None이면 전부 비활성 슬롯(기존 PureReach와
            동일하게 장애물 인식 없이 동작) -- 2026-07-13 추가.

    Returns:
        (target_joint_pos_deg, raw_action, diag)
        target_joint_pos_deg: list[float], 이번 스텝에 실제로 커맨드할 관절각(deg).
        raw_action: np.ndarray, 다음 스텝의 prev_action으로 넘길 값.
        diag: dict, 로깅용 중간값(raw_action_norm/full_delta_deg_norm/executed_delta_deg_norm).
    """
    target_quat_wxyz = TARGET_QUAT_WXYZ if target_quat_wxyz is None else target_quat_wxyz
    joint_vel_rad_s = np.zeros(6, dtype=np.float32)  # 캐비어트 6: 실측 대신 항상 0

    joint_pos_rad = np.radians(np.asarray(current_posj_deg, dtype=np.float64))
    obs = _build_obs(joint_pos_rad, joint_vel_rad_s, target_pos_m, target_quat_wxyz, prev_action, obstacles_obs)
    with torch.no_grad():
        raw_action = _policy_mlp(torch.from_numpy(obs[None, :])).numpy()[0]
    raw_action = np.clip(np.nan_to_num(raw_action, nan=0.0), -ACTION_CLAMP, ACTION_CLAMP)

    # CORRECTED (모듈 docstring 참고): use_default_offset=True -- 정책의 목표는 DEFAULT_JOINT_POS_RAD
    # + scale*raw_action이지, current_joint_pos + scale*raw_action이 아니다.
    policy_target_rad = DEFAULT_JOINT_POS_RAD + ACTION_SCALE * raw_action
    joint_delta_rad = policy_target_rad - joint_pos_rad
    # SETTLING_RATIO: 캐비어트 10 참고 -- sim의 PD-actuator/decimation 물리는 한 스텝에 델타의
    # 일부만 실제로 도달하는데, movej+mwait은 매 스텝 델타 전체에 정확히 도달해버려서 발생하는
    # 진동을 이 스케일링으로 흉내낸다.
    executed_delta_rad = SETTLING_RATIO * joint_delta_rad
    target_joint_pos_deg = np.degrees(joint_pos_rad + executed_delta_rad).tolist()

    diag = {
        "raw_action_norm": float(np.linalg.norm(raw_action)),
        "full_delta_deg_norm": float(np.linalg.norm(np.degrees(joint_delta_rad))),
        "executed_delta_deg_norm": float(np.linalg.norm(np.degrees(executed_delta_rad))),
    }
    return target_joint_pos_deg, raw_action, diag


def check_joint_limit_safety(target_joint_pos_deg):
    """target_joint_pos_deg(이번 스텝에 커맨드하려는 관절각, deg)가 JOINT_LIMITS_DEG -
    JOINT_LIMIT_SAFETY_MARGIN_DEG 안전마진 안이면 (True, None), 아니면
    (False, 한계에 가장 가까운 관절 인덱스)를 반환한다. run_policy_live()의 기존 어보트
    체크를 그대로 함수로 뽑은 것 -- 동작 변화 없음."""
    near_limit = np.abs(target_joint_pos_deg) > (JOINT_LIMITS_DEG - JOINT_LIMIT_SAFETY_MARGIN_DEG)
    if np.any(near_limit):
        return False, int(np.argmax(near_limit))
    return True, None


def run_policy_live(
    target_posx,
    max_steps: int = MAX_STEPS,
    goal_threshold_m: float = GOAL_POS_THRESHOLD_M,
    vel: float = 30,
    acc: float = 30,
):
    """Run the trained policy live/closed-loop on the real robot: read actual posj, compute one action,
    move there, repeat -- see module docstring for why (replaces an earlier offline version that
    accumulated significant drift).

    Args:
        target_posx: length-6 target pose ``[x, y, z, rx, ry, rz]`` -- only x,y,z (millimeters) used,
            rx,ry,rz ignored (see caveat 4).
        max_steps: safety cap (matches this task's 360-step training horizon).
        goal_threshold_m: stop early once the robot's real TCP position is within this of the target.
        vel, acc: passed straight to movej -- start conservative (see caveat 3).

    Returns:
        List of length-6 posj waypoints (degrees) actually commanded, in order (for logging/review).
    """
    from DSR_ROBOT2 import get_current_posj, get_current_posx, movej, mwait, DR_BASE

    # Start from a known, exact-downward pose for safety/reproducibility (see conversation 2026-07-08):
    # unlike the earlier seed123_v2 checkpoint, THIS one was trained with reset_joints_from_ik_table --
    # a diverse table of real-IK-solved start configs, not one fixed default -- so it is no longer
    # strictly true that this default is the ONLY start distribution it has seen. Kept as the starting
    # pose anyway for simplicity/conservatism (it's a known-good, exactly-downward configuration), not
    # because straying from it is known to be unsafe the way it was for the old checkpoint.
    print(f"  moving to default pose {DEFAULT_JOINT_POS_DEG.tolist()} before starting...")
    movej(DEFAULT_JOINT_POS_DEG.tolist(), vel=vel, acc=acc)
    mwait()

    # calibration check (see conversation 2026-07-08, investigating a real-robot-only steady-state error
    # where the closed loop consistently converges to a FIXED joint config ~11.5cm off target despite
    # raw_action staying large/nonzero -- i.e. the policy reaches exactly the joint target it intends,
    # just not the one that solves the real task): print the REAL flange pose at the exact default joint
    # config, to compare against what URDF-based forward kinematics predicts for the SAME joint angles.
    # A mismatch here would mean the real robot's joint-zero calibration doesn't exactly match the
    # URDF/sim model the policy was trained against -- an error type the commanded-vs-read tracking check
    # can't catch (that only verifies RELATIVE consistency, not absolute zero-calibration).
    default_flange_posx = get_current_posx(ref=DR_BASE)[0]
    print(f"  [calibration check] real flange posx at default pose {DEFAULT_JOINT_POS_DEG.tolist()}: "
          f"{list(default_flange_posx)}")

    target_pos_mm = np.asarray(target_posx[:3], dtype=np.float32)
    target_pos_m = target_pos_mm / 1000.0
    target_quat_wxyz = TARGET_QUAT_WXYZ  # target_posx[3:6] intentionally ignored, see caveat 4

    prev_action = np.zeros(6, dtype=np.float32)

    # TARGET SMOOTHING TRIED AND REVERTED (see conversation 2026-07-08): EMA-smoothing the movej target
    # across steps (TARGET_EMA_ALPHA=0.35) was tried to suppress a period-2 far/close oscillation seen on
    # real hardware that sim (same checkpoint/target) didn't show. Result: WORSE, not better -- the real
    # test converged to a stable but WRONG fixed point (best err 9.2cm, never under the 5cm threshold,
    # raw_action_norm staying large/confident while delta_deg_norm collapsed to ~0) -- the exact "genuine
    # but wrong equilibrium" failure mode seen earlier in this whole investigation. Root cause: smoothing
    # the target changes the ACTUAL joint trajectory the arm follows, which changes what joint_pos_rel the
    # network observes next step -- effectively feeding it an out-of-distribution trajectory relative to
    # what it saw in training (where the arm reaches the RAW target, unsmoothed, every step) -- a NEW
    # train/deploy mismatch introduced by the fix itself. Reverted: policy_target_rad is used directly
    # again, matching the version that was actually converging (just with cosmetic oscillation) before.

    # Tracks the best (closest) position error seen, for diagnostic reporting only (see conversation
    # 2026-07-08: the earlier "return to best point seen" recovery move was removed -- this just reports
    # how close the arm got, without physically moving back there).
    executed_path_deg = []
    best_err_m = float("inf")
    goal_reached = False
    # tracking-error diagnostic (see conversation 2026-07-08): compares what was COMMANDED last step
    # against what get_current_posj() actually reads back this step -- isolates whether movej+mwait is
    # genuinely reaching its commanded target before the next observation is computed (a real-hardware-
    # only failure mode: servo tracking lag/backlash/overshoot that Isaac Sim, which treats each step as
    # exactly-settled, cannot reproduce). prev_commanded_deg starts at the default pose since that's what
    # was actually commanded (movej above) before this loop begins.
    prev_commanded_deg = DEFAULT_JOINT_POS_DEG.copy()
    for step in range(max_steps):
        current_posj_deg = get_current_posj()
        tracking_err_deg = np.asarray(current_posj_deg) - prev_commanded_deg
        worst_track = int(np.argmax(np.abs(tracking_err_deg)))
        print(
            f"  [tracking] commanded_last={[round(v,2) for v in prev_commanded_deg.tolist()]} "
            f"read_now={[round(v,2) for v in current_posj_deg]} "
            f"err={[round(v,2) for v in tracking_err_deg.tolist()]} "
            f"(worst: joint_{worst_track+1}={tracking_err_deg[worst_track]:.2f}deg)"
        )
        # 2026-07-10: 실제 정책 추론 + SETTLING_RATIO 적용 로직은 policy_step()으로 뽑았다
        # (motion_executor.move_via_rl()과 공유) -- 아래는 그 결과를 그대로 쓰면서 진단
        # 로그만 기존과 동일하게 유지한다.
        target_joint_pos_deg, prev_action, diag = policy_step(
            current_posj_deg, target_pos_m, prev_action, target_quat_wxyz
        )
        print(
            f"  [delta] raw_action_norm={diag['raw_action_norm']:.3f} "
            f"full_delta_deg_norm={diag['full_delta_deg_norm']:.3f} "
            f"executed_delta_deg_norm={diag['executed_delta_deg_norm']:.3f} "
            f"(settling_ratio={SETTLING_RATIO})"
        )

        # Safety abort (see conversation 2026-07-08, caught from a real-robot runaway where joint_4/6
        # wound up past their limits one step at a time without ever reversing) -- check BEFORE issuing
        # movej, not after. Single check: target_joint_pos_deg is this step's CURRENT joint angle plus
        # the policy's computed delta, so this is already evaluated against the real current position,
        # not a fixed reference pose.
        ok, worst = check_joint_limit_safety(target_joint_pos_deg)
        if not ok:
            print(
                f"  ABORT: joint_{worst+1} target {target_joint_pos_deg[worst]:.1f}deg is within "
                f"{JOINT_LIMIT_SAFETY_MARGIN_DEG}deg of its physical limit -- stopping before "
                f"commanding this step (best err so far: {best_err_m*100:.1f}cm)"
            )
            break

        executed_path_deg.append(target_joint_pos_deg)
        prev_commanded_deg = np.asarray(target_joint_pos_deg)

        movej(target_joint_pos_deg, vel=vel, acc=acc)
        mwait()

        # explicit ref=DR_BASE (see conversation 2026-07-06): don't rely on the client library's
        # _g_coord default -- the controller's own reference-coordinate state can persist across
        # sessions/scripts, so a stale non-base setting could silently make this comparison meaningless.
        # get_current_posx() reports the FLANGE pose, not the RG2 gripper tip (TCP isn't applied under
        # external/PC control -- see conversation 2026-07-06), so convert to the real TCP position here.
        flange_posx = get_current_posx(ref=DR_BASE)[0]
        current_tcp_pos_m = flange_posx_to_tcp_pos_m(flange_posx)
        pos_err_m = float(np.linalg.norm(current_tcp_pos_m - target_pos_m))
        print(f"  step {step}: posj={[round(x,2) for x in target_joint_pos_deg]} pos_err={pos_err_m*100:.1f}cm")

        if pos_err_m < best_err_m:
            best_err_m = pos_err_m

        if pos_err_m < goal_threshold_m:
            print(f"  reached target within {goal_threshold_m*100:.0f}cm after {step+1} steps")
            goal_reached = True
            break

    if not goal_reached:
        print(f"  never reached {goal_threshold_m*100:.0f}cm; best err seen: {best_err_m*100:.1f}cm")

    return executed_path_deg


def main():
    # real robot integration (see conversation 2026-07-06) -- mirrors this package's own robot_move.py
    # for the DR_init/DSR_ROBOT2 setup convention.
    import rclpy
    import DR_init

    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "m0609"
    VELOCITY, ACC = 30, 30  # start conservative -- see caveat 3 in the module docstring

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    rclpy.init()
    dsr_node = rclpy.create_node("rl_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    print("start")
    try:
        from DSR_ROBOT2 import get_current_posj, get_current_posx, DR_BASE
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return

    x, sol = get_current_posx(ref=DR_BASE)
    print(f"current posx (flange, base frame): {x}, solution space: {sol}")
    print(f"current TCP position (base frame, mm): {(flange_posx_to_tcp_pos_m(x) * 1000).tolist()}")
    print(f"current posj: {get_current_posj()}")

    target_input = input("target posx [x_mm, y_mm, z_mm, rx, ry, rz] (space-separated): ")
    target_posx = [float(v) for v in target_input.split()]
    if len(target_posx) != 6:
        print("Expected 6 values (x y z rx ry rz), aborting.")
        return

    max_steps_input = input(f"max_steps [default {DEFAULT_INTERACTIVE_MAX_STEPS}]: ").strip()
    max_steps = int(max_steps_input) if max_steps_input else DEFAULT_INTERACTIVE_MAX_STEPS

    confirm = input(f"About to run the policy LIVE on the real robot for up to {max_steps} steps. Proceed? [y/N]: ")
    if confirm.strip().lower() != "y":
        print("Aborted.")
        return

    executed_path = run_policy_live(target_posx=target_posx, max_steps=max_steps, vel=VELOCITY, acc=ACC)
    print(f"executed {len(executed_path)} steps")
    final_x, final_sol = get_current_posx(ref=DR_BASE)
    print(f"final posx (flange, base frame): {final_x}, solution space: {final_sol}")
    print(f"final TCP position (base frame, mm): {(flange_posx_to_tcp_pos_m(final_x) * 1000).tolist()}")
    print(f"final posj: {get_current_posj()}")

    rclpy.shutdown()
    dsr_node.destroy_node()


if __name__ == "__main__":
    main()

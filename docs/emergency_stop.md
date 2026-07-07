# 응급정지 / 손 감지 일시정지 — 개발 기록

2026-07-06 ~ 2026-07-07에 걸쳐 추가된 안전 정지 시스템의 설계 배경과 시행착오를
기록한다. 코드 자체의 최신 상태는 각 파일을 보면 되고, 이 문서는 **왜 지금 이
구조가 됐는지**(특히 몇 번 갈아엎은 이유)를 남기는 데 목적이 있다.

## 최종 아키텍처

두 가지 서로 다른 "정지"가 있고, 완전히 분리된 이벤트로 동작한다.

| | 응급정지 | 손 감지 일시정지 |
|---|---|---|
| 트리거 | 음성 "정지/멈춰/스톱/중지/그만", `/emergency_stop` 서비스(버튼용) | `object_detection_node`가 `/hand_detected`로 발행 |
| stop_mode | 0 (QSTOP_STO, 서보 토크 끊김) | 1 (QSTOP, 토크 유지) |
| move_linear 반응 | `EmergencyStop` 예외 → 호출부까지 전파, 전체 중단 | 함수 안에서 대기했다가 자동 재개, 예외 없음 |
| 재개 | 사람이 다음 명령을 새로 줘야 함 | 손이 치워지면 자동으로 이어감 |

```
음성 "정지"                         /emergency_stop 서비스(버튼 등)
(get_keyword_node 백그라운드           (dsr_node에 등록)
 스레드가 로컬 키워드 체크 후                 │
 직접 호출, LLM 안 거침)                     │
        │                                    │
        └──────────► trigger_emergency_stop() ◄──────────┘
                      ├─ emergency_stop() = stop(stop_mode=0)
                      └─ estop_event.set()
                                │
                                ▼
                  motion_executor.move_linear()의 폴링 루프가 감지
                  → self._stop() 호출 + EmergencyStop 예외 발생
                  → pick/place/heard 전체 중단
                  → run_once()가 EmergencyStop만 따로 잡아 return_home 생략,
                    다음 명령 대기


object_detection_node                robot_action_node
┌──────────────────────┐            ┌───────────────────────────┐
│ 5Hz 타이머            │  /hand_    │ hand_detected_sub          │
│ 단일 프레임 YOLO      ├─detected──►│  → hand_pause_event.set/  │
│ (hand 클래스만 체크)   │  (Bool)    │    clear()                 │
└──────────────────────┘            └───────────────┬─────────────┘
                                                      ▼
                                     motion_executor.move_linear()
                                     폴링 중 감지 → stop(mode=1) →
                                     이벤트 풀릴 때까지 대기 → 같은
                                     좌표로 movel 재발행(자동 재개)
```

## 파일별 최종 변경

- **`my_robot_pkg/robot_action_node.py`**
  - `emergency_stop()`, `trigger_emergency_stop()`, `/emergency_stop` 서비스(`dsr_node`에 등록)
  - `estop_event`, `dsr_lock`(모듈 전역), `hand_pause_event`(인스턴스)
  - `/hand_detected` 구독 (`_on_hand_detected`)
  - `self` 전용 `SingleThreadedExecutor` + 상시 spin 스레드, `_wait_for()` 헬퍼
- **`my_robot_pkg/motion_executor.py`**
  - `move_linear()`: `movel`(동기) → `amovel`(비동기) + `check_motion()` 폴링
  - `EmergencyStop` 예외, `HAND_PAUSE_STOP_MODE = 1`
  - `estop_event`/`hand_pause_event`/`dsr_lock` 주입받아 폴링 루프에서 확인
- **`object_detection/detection.py`**
  - `HAND_CHECK_INTERVAL_SEC = 0.2` 타이머, `/hand_detected`(Bool) 퍼블리셔
- **`object_detection/yolo.py`**
  - `YoloModel.has_label(frame, target)`: 단일 프레임 경량 감지 (pick용 멀티프레임 감지와 분리)
- **`voice_interface/robot_get_keyword_node.py`**
  - 웨이크워드→녹음→STT→LLM 루프를 서비스 콜백에서 빼내 상시 백그라운드 스레드로 전환
  - STT 텍스트에 정지 키워드가 있으면 LLM 호출 없이 바로 `/emergency_stop` 호출

## 시행착오 (왜 이렇게 됐는가)

### 1. "응급정지 함수 하나 만들어줘" → `movel`이 동기라 무용지물이었음
처음엔 `stop()`/`emergency_stop()`/`/emergency_stop` 서비스만 만들었는데,
`robot_action_node`가 `movel`(동기 함수)로 움직이는 동안은 그 호출 자체가
로봇이 물리적으로 멈출 때까지 리턴하지 않아서, 그동안 `/emergency_stop`
요청을 처리할 수가 없었다. 사용자가 "amovel로 비동기 처리하면 되지 않냐"고
제안 → DSR_ROBOT2 벤더 소스를 직접 읽어 확인:
`movel`/`amovel`은 컨트롤러로 보내는 `sync_type` 값만 다르고(0=완료까지 응답
안 옴, 1=큐잉만 하고 바로 응답), `amovel` + `check_motion()` 폴링 조합으로
바꾸면 폴링 간격마다 `dsr_node`가 spin되어 그 사이 들어온 요청이 처리될 수
있음을 확인하고 반영.

### 2. 손 감지 도입 → `dsr_node` 동시 spin으로 "손 넣어도 안 멈춤"
YOLO 클래스에 이미 `hand`/`obstacle`이 학습돼 있었던 걸 발견하고 재사용.
`is_hand_detected()`(→ `get_target_pos("hand")` → `get_current_posx()`)를
백그라운드 스레드에서 돌렸는데, `get_current_posx()`도 `dsr_node`를 spin하는
함수라 `move_linear`의 `check_motion()` 폴링과 **동시에 같은 노드를 spin**하게
되어 조용히 실패 → 데몬 스레드가 죽어서 이후 손을 넣어도 무반응이었음.
`dsr_lock`으로 `dsr_node`를 건드리는 모든 지점을 직렬화해서 해결.

### 3. "wait set index too big" → `self`(RobotActionNode) 쪽도 문제
손 감지 체크 빈도가 늘면서 `rclpy.spin_until_future_complete(self, ...)`를
executor 없이 반복 호출하는 기존 패턴(호출마다 임시 executor 생성)이 rclpy
내부 wait set 관리를 깨뜨림. `self` 전용 `SingleThreadedExecutor`를 만들어
재사용하는 방식으로 전환.

### 4. "generator already executing" → global executor 충돌
위 수정을 `rclpy.spin(self)`(executor 인자 없이)로 처음 구현했는데, rclpy
소스 확인 결과 `spin()`/`spin_until_future_complete()`는 executor를 안 넘기면
**프로세스 전체가 공유하는 global executor** 하나를 쓴다는 걸 발견. DSR_ROBOT2
벤더 코드가 `dsr_node`에 대해 이미 이 global executor를 쓰고 있어서, 내
상시 spin 스레드와 충돌. `self` 전용 `SingleThreadedExecutor`를 명시적으로
만들어 그것만 spin하도록 수정(global executor는 전혀 안 건드림)하여 해결.

### 5. "되긴 하는데 많이 느림" → 손 감지가 pick 카메라 자원을 잠식
손 감지가 pick과 똑같은 `get_target_pos()`(~1초 멀티프레임 YOLO)를 재사용하고
있어서, `get_surface_z()`/실제 물체 재탐지 같은 pick 본연의 카메라 호출과
계속 락을 두고 경합 → 전체 파이프라인이 눈에 띄게 느려짐.
**"체크 빈도를 낮추자"는 대안은 안전 반응성이 떨어져서 기각**하고, 대신
`object_detection_node`에 완전히 독립된 5Hz 단일 프레임 체크 + `/hand_detected`
토픽 발행을 추가해 pick과 아예 자원을 안 나눠 쓰도록 재설계. 결과적으로
반응 속도(5Hz)도 이전보다 빨라지고 pick 속도 저하도 사라짐.

### 6. 음성 STOP도 로봇이 바쁠 때는 안 들리던 문제
`get_keyword_node`가 서비스로 호출될 때만 웨이크워드/STT를 도는 구조라,
로봇이 pick/place로 바쁜 동안은 음성 자체를 안 듣고 있었음. 웨이크워드→녹음→
STT→LLM 루프를 서비스 콜백 밖으로 빼서 항상 도는 백그라운드 스레드로 전환하고,
STT 텍스트에서 정지 키워드를 로컬로 먼저 체크해 LLM 왕복 없이 바로
`/emergency_stop`을 부르도록 해서 로봇 상태와 무관하게 음성 정지가 동작하게
만듦.

## 알려진 제약

- 손 감지 반응 지연은 최대 ~0.2초(5Hz 타이머 주기) + YOLO 추론 시간.
- `/emergency_stop` 서비스는 로봇이 완전히 정지해 다음 명령을 기다리는 동안에는
  (그 사이 `dsr_node`를 spin하는 호출이 없어서) 처리가 약간 지연될 수 있음 —
  다만 그 시점엔 로봇이 이미 안 움직이므로 안전상 문제는 아님.
- `motion_executor.place()`의 depth 실패 분기에 `self.get_logger()` 호출이
  남아있는데, `MotionExecutor`는 `Node`를 상속하지 않아 그 경로를 타면
  `AttributeError`가 날 수 있음 (아직 미수정, 별도 이슈).

# 2026_ROKEY_RL-Avoid-Obstacle

Doosan 협동로봇(m0609) 기반 음성 pick & place 워크스페이스. YOLO(세그멘테이션)로 물체를 인식하고, 음성 명령(웨이크워드 + STT + LLM)으로 지정한 통을 집어 목표 위치에 놓는다. 손 감지/음성 정지 안전 기능 포함.

## 구성 패키지

- `my_robot_pkg` — `brain_node`(오케스트레이터) + `motion_node`(로봇/그리퍼 제어, DSR_ROBOT2 연동)
- `object_detection` — RealSense + YOLO seg로 물체 3D 위치/각도, 손 감지
- `voice_interface` — 웨이크워드(openWakeWord) + STT(Whisper) + LLM(GPT-4o) 명령 파싱
- `safety_monitor` — 손 감지/음성 정지 통합 판단 + 하드 정지
- `robot_interfaces`, `od_msg`, `obstacle_avoidance_msgs` — 커스텀 액션/서비스/메시지 인터페이스
- `pointcloud_perception` — 월드맵 스캔(`world_map_node`, "월드맵 업데이트" 음성 명령으로 트리거)
- `obstacle_avoidance` — `rl_avoidance_node`, 월드맵/실시간 장애물 정보를 구독해 회피 명령을 내는 노드(policy 추론은 아직 stub)

---

## 1. 사전 준비 (외부 의존성)

이 워크스페이스 자체에는 로봇 드라이버가 들어있지 않다. 아래를 **별도 워크스페이스**에 미리 설치/빌드해두고, 이 워크스페이스보다 먼저 소스(source)해야 한다.

- **ROS2 Humble**
- **Doosan 로봇 드라이버**: [doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2) (`dsr_common2`, `dsr_controller2`, `dsr_hardware2`, `dsr_msgs2`, `dsr_bringup2`) — `DSR_ROBOT2`/`DR_init` 모듈과 `dsr_msgs2` 서비스/메시지를 `motion_node`, `safety_monitor_node`가 직접 import한다.
- **Intel RealSense SDK + ROS2 래퍼**: `librealsense2`, `realsense2_camera` — D435/D435i 기준.
- 로봇 컨트롤러(`dsr_bringup2`)와 RealSense 카메라 노드는 **이 워크스페이스의 launch에 안 들어있다** — 사용자가 직접 먼저 띄워야 한다.

## 2. Python 패키지 설치

`package.xml`/`setup.py`에 rosdep으로 안 잡히는 pip 전용 의존성들이다:

```bash
pip install --user ultralytics opencv-python numpy scipy \
    openai langchain langchain-openai python-dotenv \
    pyaudio sounddevice openwakeword websockets
```

- `ultralytics`가 `torch`를 자동 설치한다(CPU 전용이면 기본 설치로 충분, GPU 가속하려면 CUDA 빌드 별도 설치).
- `pyaudio`는 시스템에 `portaudio19-dev`가 필요할 수 있다: `sudo apt install portaudio19-dev`.
- `websockets`는 `hmi_interface`의 `hmi_voice_bridge`(STT-TTS 탭 오브 애니메이션용 브릿지)가 사용한다.

## 3. 내 컴퓨터/로봇 셋업에 맞게 반드시 바꿔야 하는 값들

**여기가 이 README의 핵심이다.** 아래 값들은 다른 로봇 셀/컴퓨터에서 그대로 쓰면 안 되고, 각자 환경에서 다시 측정/설정해야 한다.

### 3.1 로봇 ID / 모델

- `src/my_robot_pkg/my_robot_pkg/motion_node.py` — `ROBOT_ID = "dsr01"`, `ROBOT_MODEL = "m0609"`
- `src/safety_monitor/safety_monitor/safety_monitor_node.py` — `robot_id` 파라미터(기본 `"dsr01"`, `motion_node`의 `ROBOT_ID`와 반드시 일치해야 함)

`dsr_bringup2`를 띄울 때 지정한 네임스페이스/모델과 같아야 한다.

### 3.2 그리퍼 (OnRobot RG2, Modbus TCP)

- `src/my_robot_pkg/my_robot_pkg/motion_node.py` — `TOOLCHARGER_IP = "192.168.1.1"`, `TOOLCHARGER_PORT = "502"`
- 그리퍼 최대 폭/힘이 다르면 `src/my_robot_pkg/my_robot_pkg/gripper.py`의 `RG2Gripper.__init__` 기본값(`max_width=1100`, `max_force=400`) 확인
- 통 크기가 다르면 재빌드 없이 런치 인자로 조정: `grip_min_width_mm` (launch 파일 인자, 기본 30.0)

### 3.3 로봇 자세 좌표 (POSITION_COORDS)

- `src/my_robot_pkg/my_robot_pkg/brain_node.py`의 `POSITION_COORDS` — `home`/`scan`/`target1~3`을 `[x, y, z, rx, ry, rz]`(mm, deg, ZYZ)로 정의.
- **이건 로봇 셀(테이블/물체 배치)마다 완전히 다르다.** 티칭 펜던트나 `get_current_posx()`로 실제 원하는 위치를 하나씩 찍어서 다시 채워야 한다.

### 3.4 카메라-그리퍼 캘리브레이션

- `src/my_robot_pkg/resource/T_gripper2camera.npy` — 그리퍼 TCP 기준 카메라 위치/자세 4x4 변환행렬(hand-eye calibration 결과물).
- 카메라를 다른 위치/각도로 마운트했다면 **반드시 새로 캘리브레이션**해야 한다. 이 파일이 틀리면 3D 위치 계산 전체가 어긋난다(예: 물체 대신 옆 배경의 depth를 읽는 등).

### 3.5 grasp yaw 캘리브레이션 (미완료 상태로 커밋됨)

- `src/my_robot_pkg/my_robot_pkg/motion_node.py` — `GRASP_AXIS_IMG_ANGLE_DEG = 0.0`, `GRASP_ANGLE_SIGN = 1.0`
- 세그멘테이션으로 구한 물체 회전각을 그리퍼 wrist yaw(C)로 변환할 때 쓰는 상수인데, **아직 실제 하드웨어로 캘리브레이션 안 된 placeholder 값**이다. 실사용 전에 `object_detection/object_detection/angle_probe.py`로 실측해서 채워야 한다.

### 3.6 마이크 장치 인덱스

- `src/voice_interface/voice_interface/robot_get_keyword_node.py`의 `MicConfig(..., device_index=10, ...)`
- 마이크 인덱스는 USB 포트/OS마다 다르다. 확인 방법:
  ```python
  import pyaudio
  p = pyaudio.PyAudio()
  for i in range(p.get_device_count()):
      print(i, p.get_device_info_by_index(i)['name'])
  ```

### 3.7 OpenAI API 키

- `src/voice_interface/resource/.env` 파일에 아래 한 줄로 넣는다(이 파일은 `.gitignore`에 이미 걸려있어 커밋되지 않음 — 새로 만들어야 함):
  ```
  OPENAI_API_KEY=sk-...
  ```
- Whisper(STT)와 GPT-4o(명령 파싱) 둘 다 이 키를 쓴다.

### 3.8 웨이크워드 / YOLO 모델

- 웨이크워드는 `"헬로우 로키"` 전용으로 학습된 `src/voice_interface/resource/hello_rokey_8332_32.tflite`를 쓴다. 다른 웨이크워드를 쓰려면 [openWakeWord](https://github.com/dscripka/openWakeWord)로 새로 학습해서 교체.
- YOLO 모델은 `src/object_detection/object_detection/yolo.py`의 `YOLO_MODEL_FILENAME = "yolov8s_seg_250.pt"` — 자기 물체로 새로 학습한 세그멘테이션 모델을 `resource/`에 넣고 파일명을 맞춘다. **모델을 바꾸면 클래스 순서가 다를 수 있으니 `class_name_tool.json`의 라벨 매핑을 모델의 실제 `model.names`와 대조해서 반드시 다시 맞출 것** (안 맞으면 엉뚱한 물체를 잡으러 감).

## 4. 빌드

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source ~/<doosan-robot2-workspace>/install/setup.bash   # 로봇 드라이버 워크스페이스
colcon build
source install/setup.bash
```

## 5. 실행

**순서 상관없이** 아래를 다 띄운 뒤(각 노드가 서로 필요한 서비스/액션서버를 기다리게 되어 있음):

```bash
# 1) 로봇 드라이버 (별도 워크스페이스)
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=<로봇IP> model:=m0609

# 2) RealSense 카메라
ros2 launch realsense2_camera rs_launch.py

# 3) 이 워크스페이스의 애플리케이션 노드들
ros2 launch my_robot_pkg pnp_bringup.launch.py
```

디버깅할 땐 3번 대신 아래처럼 노드를 하나씩 따로 띄우면 로그 확인이 훨씬 편하다:

```bash
ros2 run object_detection object_detection_node
ros2 run voice_interface get_keyword_node
ros2 run safety_monitor safety_monitor_node
ros2 run my_robot_pkg motion_node
ros2 run my_robot_pkg brain_node
ros2 run pointcloud_perception world_map_node
ros2 run obstacle_avoidance rl_avoidance_node
```

`world_map_node`/`rl_avoidance_node`는 서로/다른 노드를 기다리지 않고 독립적으로
뜬다. `world_map_node`는 `update_world_map` 서비스가 실제로 호출될 때만 MoveLine을
움직인다 — 로봇 드라이버(`dsr_bringup2`)와 RealSense 카메라가 먼저 떠 있어야
스캔이 성공한다.

## 6. 음성 명령 사용법

1. "헬로우 로키"로 웨이크업
2. 명령: `"빨간색 통을 1번 위치로 옮겨"` (obj_A/B/C = 빨강/파랑/초록, target1/2/3)
3. 웨이크워드 한 번으로 세션이 열려 있는 동안(`MAX_SESSION_SEC`)은 재웨이크 없이 연달아 명령 가능
4. "정지"/"멈춰" 등으로 즉시 정지, "다시 시작해" 등으로 재개(안전 래치만 해제, 자동 재시도는 안 함)
5. "월드맵 업데이트 해줘"/"월드맵 스캔 시작" 등으로 전체 스캔 트리거 — brain_node가
   `update_world_map` 서비스를 호출하고, 완료/실패/시간초과를 음성으로 안내한다
   (스캔은 수 분 걸릴 수 있음). 결과 장애물 목록은 `/world_map/obstacles`로
   publish되고 `rl_avoidance_node`가 구독해 캐시한다(정책 반영은 아직 미구현).

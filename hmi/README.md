# hmi/ - React + Flask-SocketIO 통합 HMI

이 디렉터리는 콜콘(colcon) 패키지가 아닌 일반 웹 프로젝트다. ROS와의 유일한
접점은 `src/hmi_ros_bridge`가 `/ros` Socket.IO 네임스페이스에 클라이언트로
붙는 것뿐이며, 이 안의 Flask 프로세스는 rclpy를 직접 import하지 않는다.

**Phase 0~6 전부 구현/검증 완료** (React Mock Dashboard → DB API 이관 →
Voice Socket.IO → Vision/Detection → Safety/Task 상태 → R3F 3D Viewer).

**기존 `src/hmi_bridge`, `src/hmi_interface`는 이 작업과 무관하게 그대로
동작한다** - 의도적으로 삭제/종료하지 않고 병행 운영 중이다. Phase 6까지
끝나 기능적으로는 대체 가능하지만, deprecated 처리 및 최종 삭제는 사용자가
직접 검토한 뒤 결정할 일이라 이 세션에서 건드리지 않았다.

## 구조

```
hmi/
  schemas/    JSON Schema 공통 계약 (Python jsonschema 런타임 검증 + TS 타입의 근거)
  backend/    Flask REST API + Flask-SocketIO ("/" 브라우저용, "/ros" 브릿지용)
  frontend/   React + Vite
```

## 개발 환경 실행

### 1. Node.js
이 머신엔 기본적으로 Node가 없어서 nvm으로 설치했다:
```bash
export NVM_DIR="$HOME/.nvm"; [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
nvm use --lts
```
새 터미널을 열 때마다 위 두 줄이 필요하다(설치 스크립트가 `~/.bashrc`에도
추가해뒀지만 비대화형 셸에선 자동 로드되지 않을 수 있음).

### 2. Backend
```bash
cd hmi/backend
python3 -m venv .venv          # 반드시 PYTHONPATH가 비어 있는 셸에서 실행할 것 -
unset PYTHONPATH                # ROS 워크스페이스들의 setup.bash가 PYTHONPATH에
.venv/bin/pip install -r requirements.txt   # ROS 패키지 경로를 잔뜩 얹어두면
                                              # venv 격리가 깨진다(실제로 겪음).
cp .env.example .env            # 필요시 값 수정 (HMI_BRIDGE_TOKEN 등)
.venv/bin/python run.py         # http://localhost:5100
```
`GET /api/health`로 확인 가능. 테스트: `unset PYTHONPATH && .venv/bin/python -m pytest tests/ -v`
(실제 hmi_ros_bridge/React 없이 Flask-SocketIO test_client 2개로 전체
command→ack→terminal task_status→command_result 흐름과 토큰 인증을 검증한다).

### 3. Frontend
```bash
cd hmi/frontend
cp .env.example .env
npm install
npm run dev                     # http://localhost:5173, Vite dev server
```

### 4. hmi_ros_bridge (ROS 2 패키지)
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hmi_ros_bridge   # 저장소 루트에서
source install/setup.bash
ros2 launch hmi_ros_bridge hmi_bringup.launch.py   # hmi_ros_bridge_server + hmi_vision_stream 한 번에
```
개별로 띄우려면 `ros2 run hmi_ros_bridge hmi_ros_bridge_server` / `ros2 run hmi_ros_bridge
hmi_vision_stream`(디버깅 중 로그를 노드별로 분리해서 보고 싶을 때 편함).

`hmi/backend/.env`를 자동으로 읽어 `HMI_BRIDGE_TOKEN`/`HMI_BACKEND_*`를 공유한다
(다른 위치를 쓰려면 `HMI_ROS_BRIDGE_ENV_FILE` 환경변수로 지정). `python-socketio[client]`,
`python-dotenv`는 시스템/사용자 Python에 `pip install --user`로 설치되어 있어야 한다
(이 워크스페이스가 이미 `websockets`를 그렇게 설치해 쓰는 것과 동일한 관행).

**Ctrl+C가 안 먹는 것처럼 보이면**: `python-socketio`의 `socketio.Client(handle_sigint=True,
기본값)`가 프로세스의 SIGINT 핸들러를 가로채서 `rclpy.init()`의 SIGINT 처리와 충돌하는
실기 버그가 있었다(`socketio_worker.py`에서 `handle_sigint=False`로 수정됨,
`test_sigint_shutdown.py`가 회귀 테스트) - 최신 빌드인지(`colcon build`) 확인할 것.

테스트: `python3 -m pytest src/hmi_ros_bridge/test/test_emit_channel.py -p no:anyio`
(순수 로직, ROS 불필요), `source /opt/ros/humble/setup.bash && python3 -m pytest
src/hmi_ros_bridge/test/test_bridge_node.py -p no:anyio` (실제 rclpy 그래프로 검증 -
**주의**: 실제 로봇 스택이 이미 떠 있는 상태에서 이 테스트를 돌리면 `/safety/state`,
`hmi/task_status/*` 같은 운영 토픽에 테스트용 가짜 메시지가 실제로 발행돼 대시보드에
잠깐 섞여 보일 수 있다 - 로봇 스택이 안 떠 있을 때만 돌릴 것).
`-p no:anyio`가 필요한 이유: `python-socketio[client]`가 끌어온 `anyio`가 이 환경의
구버전 시스템 pytest(6.2.5)와 안 맞는 pytest 플러그인을 등록해서 죽는 문제가 있음.

### 5. World / Robot 3D Viewer (Phase 6)
`hmi/backend`가 떠 있으면(Phase 2의 `/api/worldmap/*` 그대로 재사용) 별도
설정 없이 동작한다 - React가 `@react-three/fiber`로 `data/world_maps/`의
저장된 스캔을 직접 렌더링한다(기존 hmi_bridge의 iframe 뷰어를 대체, 그쪽은
계속 무수정 병행 운영).

## 전체 실행 순서 (로봇 실기 기준, 터미널 6개)

```bash
# 1) 로봇 드라이버 (직접)
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=<로봇IP> model:=m0609

# 2) RealSense 카메라 (직접)
ros2 launch realsense2_camera rs_align_depth_launch.py ...

# 3) pick-and-place 애플리케이션 노드 전부 (기존 my_robot_pkg, 무수정)
ros2 launch my_robot_pkg pnp_bringup.launch.py

# 4) HMI ROS 노드 2개 (신규)
ros2 launch hmi_ros_bridge hmi_bringup.launch.py

# 5) hmi/backend
cd hmi/backend && unset PYTHONPATH && .venv/bin/python run.py

# 6) hmi/frontend
cd hmi/frontend && npm run dev
```
1)/2)가 먼저 떠 있어야 3)의 pick/place 동작이 실제로 성공한다(3)/4)는 순서 상관없이
서로 기다리며 재시도한다). 5)/6)은 ROS 노드가 아니라서 launch에 안 넣었다(venv/npm
활성화 필요, 로그/재시작 관리가 launch보다 직접 터미널이 편함).

## 프로덕션 실행 구조 (아직 미확정, 후속 작업)

- `npm run build` → `hmi/frontend/dist/`
- Flask가 `dist/`를 정적 파일로 서빙 + `/api/*`, Socket.IO만 담당 (단일 origin, 단일 포트)
- `socketio.run()`의 내장 Werkzeug dev 서버 대신 gunicorn + eventlet(또는 동급) 뒤에서 실행
- 영상(MJPEG, `hmi_vision_stream` 8767)은 현재 별도 포트(`VITE_VISION_STREAM_URL`)로
  유지 중, 이후 `/stream/vision` same-origin proxy로 통합 검토(아직 미구현)

## Socket.IO 네임스페이스

- `/` : React(socket.io-client)
- `/ros` : hmi_ros_bridge(python-socketio Client), `auth={"token": HMI_BRIDGE_TOKEN}` 필수

이벤트 계약은 `hmi/schemas/*.schema.json` 참고. 자세한 이벤트 카탈로그와
설계 배경은 프로젝트 대화 기록(HMI 재구축 논의) 참고 - 요약:

| 이벤트 | 방향 | 스키마 |
|---|---|---|
| `voice_status`, `voice_log` | Bridge→Flask→Browser | ad-hoc |
| `safety_status` | Bridge→Flask→Browser | safety_status.schema.json (`/safety/state` 그대로, 노드 무수정) |
| `task_status` | Bridge→Flask→Browser | task_status_event.schema.json (source: manipulation\|world_map 태깅) |
| `bridge_status` | Flask→Browser | `{connected: bool}` |
| `command` | Browser→Flask→Bridge | command.schema.json |
| `command_ack` | Bridge→Flask→요청자 | command_ack.schema.json |
| `command_result` | Flask→요청자 | command_result.schema.json (Flask가 terminal task_status에서만 파생, Bridge는 만들지 않음) |

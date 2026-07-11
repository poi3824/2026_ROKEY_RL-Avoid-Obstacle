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

## 전체 실행 순서 (로봇 실기 기준, 터미널 7개)

```bash
# 1) 로봇 드라이버 (직접)
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=<로봇IP> model:=m0609

# 2) RealSense 카메라 (직접)
ros2 launch realsense2_camera rs_align_depth_launch.py ...

# 2.5) 카메라-로봇 캘리브레이션 TF (직접, alias: camera_attach) - 빠뜨리기 쉬움 주의!
#      link_6 -> camera_link 정적 TF. 이게 없으면 world_map_node가 point cloud를
#      base_link 기준으로 변환하지 못해 스캔이 실패한다(TF 트리가 로봇 쪽과
#      카메라 쪽으로 분리됨). 2026-07-11에 실제로 이걸 빼먹고 실행해서 겪은 문제.
ros2 run tf2_ros static_transform_publisher --x 0.00 --y 0.05 --z 0.03 --roll -1.5708 --pitch -1.5708 --yaw 0.0 --frame-id link_6 --child-frame-id camera_link

# 3) pick-and-place 애플리케이션 노드 전부 (기존 my_robot_pkg, 무수정)
ros2 launch my_robot_pkg pnp_bringup.launch.py

# 4) HMI ROS 노드 2개 (신규)
ros2 launch hmi_ros_bridge hmi_bringup.launch.py

# 5) hmi/backend
cd hmi/backend && unset PYTHONPATH && .venv/bin/python run.py

# 6) hmi/frontend
cd hmi/frontend && npm run dev
```
1)/2)/2.5)가 먼저 떠 있어야 3)의 pick/place와 월드맵 스캔이 실제로 성공한다(3)/4)는
순서 상관없이 서로 기다리며 재시도한다). 5)/6)은 ROS 노드가 아니라서 launch에 안
넣었다(venv/npm 활성화 필요, 로그/재시작 관리가 launch보다 직접 터미널이 편함).

**실행 전에 아래 "로컬 머신 사전 설정"이 이미 적용돼 있어야 한다** - 안 그러면 노드
수가 많은 이 전체 구성에서 point cloud가 유실되거나 노드가 아예 안 뜬다.

## 로컬 머신 사전 설정 (git 대상 아님, 이 노트북에 1회성으로 적용됨)

2026-07-11: 로봇드라이버+카메라만 켜고 `ros2 service call /update_world_map ...`을
직접 호출하면 스캔이 잘 되는데, 위 6~7단계 전체 구성(pnp_bringup + hmi 전부)으로
실행하면 `world_map_node`가 point cloud를 아예 못 받아 스캔이 실패하는 문제가 있었다.
`~/.bashrc`와 커널 sysctl 설정 문제로 확인/수정 완료 - **git으로 추적되지 않는
시스템 설정**이라 이 노트북이 초기화되거나 새 개발 머신으로 옮기면 다시 적용해야 한다.

원인 1) `CYCLONEDDS_URI`가 루프백이 아니라 WiFi 인터페이스(`wlp4s0`)로 고정돼 있어서,
같은 기계 안에서만 오가는 ROS2 트래픽(카메라 -> world_map_node 등)까지 WiFi 스택을
거치며 point cloud처럼 큰 메시지의 패킷이 유실됨.

원인 2) 루프백(`lo`)은 멀티캐스트를 지원하지 않아(`ip link show lo`에 `MULTICAST`
플래그 없음) CycloneDDS가 멀티캐스트 없이 고정 포트 슬롯(기본 0~9, 10개)만 스캔하는
폴백 모드로 들어가는데, pnp_bringup(7개) + hmi_ros_bridge(2개) + 로봇드라이버 내부
노드들을 다 합치면 13개+가 되어 슬롯이 고갈되고 `Failed to find a free participant
index for domain 90` 에러로 일부 노드가 아예 못 뜸.

`~/.bashrc`에 적용된 최종 설정 (`ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` 아래):
```bash
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="lo"/>
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>'
```
(다른 PC에서 이 ROS2 그래프를 봐야 하는 상황이 생기면 `name="lo"`를 원래 값인
`name="wlp4s0"`로 되돌리면 되는데, 그러면 원인 1)이 재발하니 대신 `lo`/`wlp4s0`
둘 다 리스트에 넣는 걸 먼저 검토할 것.)

추가로 커널 UDP 소켓 버퍼도 기본값(208KB)이 point cloud 같은 대용량 메시지엔 작아서
`/etc/sysctl.d/60-cyclonedds.conf`로 확장:
```
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
net.core.wmem_max=2147483647
net.core.wmem_default=2147483647
```
새 머신에서 처음 설정할 때: 위 두 블록을 각각 `~/.bashrc`와 `/etc/sysctl.d/60-cyclonedds.conf`에
넣고 `sudo sysctl --system`으로 즉시 적용. `.bashrc`는 새 터미널을 열어야 반영된다
(이미 떠 있는 터미널에는 적용 안 됨 - 전체 스택을 새 터미널들로 재시작해야 함).

**증상으로 다시 이 문제인지 확인하는 법**: `world_map_node` 로그에 `settle 동안 point
cloud를 아직 한 번도 받지 못함` / `last_seen_stamp=None`이 뜨는데 `ros2 topic hz
/camera/camera/depth/color/points`로는 정상 30Hz가 나온다면(퍼블리셔는 살아있는데
구독자만 못 받는 상황), 십중팔구 이 문제다. `ros2 node list`에 같은 이름 노드가
중복으로 뜨는 건 대부분 `ros2 daemon` 캐시 오염이라 무관하니 `ros2 daemon stop &&
ros2 daemon start`로만 정리하면 된다.

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

# hmi/ - React + Flask-SocketIO 통합 HMI (재구축 중)

이 디렉터리는 콜콘(colcon) 패키지가 아닌 일반 웹 프로젝트다. ROS와의 유일한
접점은 `src/hmi_ros_bridge`(Phase 3에서 생성 예정)가 `/ros` Socket.IO
네임스페이스에 클라이언트로 붙는 것뿐이며, 이 안의 Flask 프로세스는 rclpy를
직접 import하지 않는다.

**기존 `src/hmi_bridge`, `src/hmi_interface`는 이 작업과 무관하게 그대로
동작한다** - Phase 6까지 병행 운영하고, 새 구조가 완전히 그 기능을 대체한
뒤에만 deprecated 처리한다. 지금 이 디렉터리를 추가한다고 기존 서비스가
바뀌거나 중단되지 않는다.

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

## 프로덕션 실행 구조 (목표, Phase 6에서 확정)

- `npm run build` → `hmi/frontend/dist/`
- Flask가 `dist/`를 정적 파일로 서빙 + `/api/*`, Socket.IO만 담당 (단일 origin, 단일 포트)
- `socketio.run()`의 내장 Werkzeug dev 서버 대신 gunicorn + eventlet(또는 동급) 뒤에서 실행
- 영상(MJPEG)은 Phase 4까지 별도 포트(`VITE_VISION_STREAM_URL`)로 유지, 이후 `/stream/vision`
  same-origin proxy로 통합 검토

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

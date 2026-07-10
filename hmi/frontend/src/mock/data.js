// Phase 1 전용 mock 데이터. 형태는 hmi/schemas/*.schema.json과 최대한 동일하게
// 맞춰서, Phase 2/3/5가 실제 API/Socket.IO로 교체할 때 컴포넌트 쪽 코드를
// 거의 안 건드리고 훅 내부 구현만 바꾸면 되게 한다.

export const MOCK_SAFETY_STATUS = {
  state: "RUN",
  reason: "",
  timestamp: Date.now() / 1000,
};

export const MOCK_TASK_STATUS = {
  manipulation: {
    task_id: "mock-task-1",
    mode: "pick_place",
    phase: "descending",
    title: "빨간 통을 1번 위치로 옮기기",
    detail: "그리퍼 하강 중",
    step_index: 2,
    step_total: 4,
    progress: 0.5,
    status: "RUNNING",
    timestamp: Date.now() / 1000,
  },
  world_map: {
    task_id: "mock-task-2",
    mode: "world_map_scan",
    phase: "idle",
    title: "",
    detail: "",
    step_index: null,
    step_total: null,
    progress: null,
    status: "IDLE",
    timestamp: Date.now() / 1000,
  },
};

export const MOCK_VOICE_STATUS = { state: "idle", level: 0 };

export const MOCK_VOICE_LOGS = [
  { level: "INFO", text: "웨이크워드 감지: 헬로우 로키", stamp: Date.now() / 1000 - 30 },
  { level: "INFO", text: "STT 인식 결과: 빨간색 통을 1번 위치로 옮겨", stamp: Date.now() / 1000 - 20 },
  { level: "INFO", text: "명령 파싱 완료: obj_A -> target1", stamp: Date.now() / 1000 - 10 },
];

export const MOCK_BRIDGE_CONNECTED = false;

// pick_attempts/voice_events/worldmap_scans/performance summary는 Phase 2부터
// hmi/frontend/src/hooks/useDbData.js가 실제 hmi/backend REST API에서 가져온다 -
// 더 이상 mock이 필요 없어 여기서 제거했다.

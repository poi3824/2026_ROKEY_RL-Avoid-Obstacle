import { useState } from "react";
import AppShell from "./components/layout/AppShell";
import NodeStatus from "./components/panels/NodeStatus";
import TaskProgress from "./components/panels/TaskProgress";
import KpiStrip from "./components/panels/KpiStrip";
import VoiceConsole from "./components/panels/VoiceConsole";
import VisionPanel from "./components/panels/VisionPanel";
import RobotViewer from "./components/panels/RobotViewer";
import DatabasePage from "./components/pages/DatabasePage";
import PerformancePage from "./components/pages/PerformancePage";
import SettingsPage from "./components/pages/SettingsPage";
import { useDbData } from "./hooks/useDbData";
import { useBridgeStatus } from "./hooks/useBridgeStatus";
import { useVoiceStatus } from "./hooks/useVoiceStatus";
import { useVisionStream } from "./hooks/useVisionStream";
import { useSafetyStatus } from "./hooks/useSafetyStatus";
import { useTaskStatus } from "./hooks/useTaskStatus";

// Phase 2: DatabasePage/PerformancePage는 useDbData()로 실제 REST API(hmi/backend)에 연결됨.
// Phase 3: VoiceConsole/bridge_status는 useVoiceStatus()/useBridgeStatus()로 실제
// Socket.IO(hmi_ros_bridge -> hmi/backend -> 브라우저)에 연결됨.
// Phase 4: VisionPanel은 useVisionStream()으로 hmi_vision_stream(MJPEG, 8767)에
// 연결 - 자체 YOLO 로드 없이 object_detection_node의 hmi/vision_detections를 재사용.
// Phase 5: TaskProgress는 useSafetyStatus()/useTaskStatus()로 실제 /safety/state,
// hmi/task_status/{manipulation,world_map}에 연결됨 - 두 모델을 절대 섞지 않는다
// (safety는 TaskProgress 헤더에 배지로만 같이 보여줄 뿐, 데이터는 분리 유지).
// 기존 hmi_interface(:5050)/hmi_bridge(:5000)는 계속 병행 운영 중.
export default function App() {
  const [activeId, setActiveId] = useState("dashboard");
  const [selectedScanId, setSelectedScanId] = useState(null);
  const db = useDbData();
  const bridgeConnected = useBridgeStatus();
  const { voice, logs: voiceLogs, toggleRecord } = useVoiceStatus();
  const vision = useVisionStream();
  const safetyStatus = useSafetyStatus();
  const taskStatus = useTaskStatus();

  return (
    <AppShell activeId={activeId} onSelect={setActiveId} bridgeConnected={bridgeConnected}>
      {activeId === "dashboard" && (
        <div>
          <NodeStatus />
          <div className="card">
            <div className="split-row" style={{ marginBottom: 0 }}>
              <VisionPanel vision={vision} bare />
              <RobotViewer bare />
            </div>
          </div>
          <TaskProgress tasks={taskStatus} safety={safetyStatus} />
          <KpiStrip summary={db.summary} error={db.error} />
        </div>
      )}

      {activeId === "voice" && (
        <VoiceConsole voice={voice} logs={voiceLogs} onToggleRecord={toggleRecord} />
      )}

      {activeId === "vision" && <VisionPanel vision={vision} />}

      {activeId === "viewer3d" && <RobotViewer scanId={selectedScanId} />}

      {activeId === "database" && (
        <div>
          {db.error ? <div className="note">API 연결 실패: {db.error} (hmi/backend가 떠 있는지, VITE_API_BASE가 맞는지 확인)</div> : null}
          <DatabasePage
            pickAttempts={db.pickAttempts}
            voiceEvents={db.voiceEvents}
            worldmapScans={db.worldmapScans}
            onSelectScan={(id) => { setSelectedScanId(id); setActiveId("viewer3d"); }}
          />
        </div>
      )}

      {activeId === "performance" && <PerformancePage summary={db.summary} />}

      {activeId === "settings" && <SettingsPage />}
    </AppShell>
  );
}

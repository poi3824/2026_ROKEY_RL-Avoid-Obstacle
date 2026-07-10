import { useState } from "react";
import AppShell from "./components/layout/AppShell";
import SafetyStatus from "./components/panels/SafetyStatus";
import TaskProgress from "./components/panels/TaskProgress";
import VoiceConsole from "./components/panels/VoiceConsole";
import VisionPanel from "./components/panels/VisionPanel";
import RobotViewer from "./components/panels/RobotViewer";
import DatabasePage from "./components/pages/DatabasePage";
import PerformancePage from "./components/pages/PerformancePage";
import SettingsPage from "./components/pages/SettingsPage";
import {
  MOCK_SAFETY_STATUS, MOCK_TASK_STATUS, MOCK_VOICE_STATUS, MOCK_VOICE_LOGS,
  MOCK_BRIDGE_CONNECTED,
} from "./mock/data";
import { useDbData } from "./hooks/useDbData";

// Phase 1: SafetyStatus/TaskProgress/VoiceConsole은 아직 mock (Phase 3/5에서 Socket.IO로 교체).
// Phase 2: DatabasePage/PerformancePage는 useDbData()로 실제 REST API(hmi/backend)에 연결됨 -
// 기존 hmi_interface(:5050)/hmi_bridge(:5000)는 계속 별도로 병행 운영 중.
export default function App() {
  const [activeId, setActiveId] = useState("dashboard");
  const [selectedScanId, setSelectedScanId] = useState(null);
  const [yoloEnabled, setYoloEnabled] = useState(false);
  const db = useDbData();

  return (
    <AppShell activeId={activeId} onSelect={setActiveId} bridgeConnected={MOCK_BRIDGE_CONNECTED}>
      {activeId === "dashboard" && (
        <div>
          <SafetyStatus status={MOCK_SAFETY_STATUS} />
          <TaskProgress tasks={MOCK_TASK_STATUS} />
          <div className="split-row">
            <VisionPanel connected={false} />
            <RobotViewer />
          </div>
        </div>
      )}

      {activeId === "voice" && (
        <VoiceConsole voice={MOCK_VOICE_STATUS} logs={MOCK_VOICE_LOGS} onToggleRecord={() => {}} />
      )}

      {activeId === "vision" && (
        <VisionPanel connected={false} yoloEnabled={yoloEnabled} onToggleYolo={() => setYoloEnabled((v) => !v)} />
      )}

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

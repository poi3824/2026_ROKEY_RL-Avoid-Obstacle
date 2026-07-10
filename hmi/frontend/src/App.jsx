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
  MOCK_BRIDGE_CONNECTED, MOCK_PICK_ATTEMPTS, MOCK_VOICE_EVENTS, MOCK_WORLDMAP_SCANS,
  MOCK_PERFORMANCE_SUMMARY,
} from "./mock/data";

// Phase 1: 전부 mock 데이터 - 실제 REST(Phase 2)/Socket.IO(Phase 3/5) 연결은 다음
// Phase에서 이 컴포넌트들의 props를 채우는 훅으로 교체한다(컴포넌트 자체는 안 바뀜).
export default function App() {
  const [activeId, setActiveId] = useState("dashboard");
  const [selectedScanId, setSelectedScanId] = useState(null);
  const [yoloEnabled, setYoloEnabled] = useState(false);

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
        <DatabasePage
          pickAttempts={MOCK_PICK_ATTEMPTS}
          voiceEvents={MOCK_VOICE_EVENTS}
          worldmapScans={MOCK_WORLDMAP_SCANS}
          onSelectScan={(id) => { setSelectedScanId(id); setActiveId("viewer3d"); }}
        />
      )}

      {activeId === "performance" && <PerformancePage summary={MOCK_PERFORMANCE_SUMMARY} />}

      {activeId === "settings" && <SettingsPage />}
    </AppShell>
  );
}

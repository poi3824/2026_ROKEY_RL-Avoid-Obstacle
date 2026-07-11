import { useCallback, useEffect, useRef, useState } from "react";
import { socket, sendCommand } from "../api/socket";

const MAX_LOG_LINES = 200;

// Phase 3: VoiceConsole이 쓰는 실데이터 훅 - voice_status/voice_log Socket.IO
// 이벤트를 구독하고, 녹음 시작/중지는 sendCommand()로 command_ack까지 기다린다.
export function useVoiceStatus() {
  const [voice, setVoice] = useState({ state: "idle", level: 0 });
  const [logs, setLogs] = useState([]);
  const [busy, setBusy] = useState(false);
  const busyRef = useRef(false);

  useEffect(() => {
    function onVoiceStatus(data) {
      setVoice({ state: data.state ?? "idle", level: data.level ?? 0 });
    }
    function onVoiceLog(entry) {
      setLogs((prev) => [...prev, entry].slice(-MAX_LOG_LINES));
    }
    socket.on("voice_status", onVoiceStatus);
    socket.on("voice_log", onVoiceLog);
    return () => {
      socket.off("voice_status", onVoiceStatus);
      socket.off("voice_log", onVoiceLog);
    };
  }, []);

  const toggleRecord = useCallback(async () => {
    if (busyRef.current) return;
    busyRef.current = true;
    setBusy(true);
    const action = voice.state === "recording" ? "voice.stop_record" : "voice.start_record";
    try {
      const ack = await sendCommand(action);
      if (!ack.ok) {
        setLogs((prev) => [...prev, { level: "ERROR", text: `명령 실패: ${ack.error}`, stamp: Date.now() / 1000 }]);
      }
    } catch (e) {
      setLogs((prev) => [...prev, { level: "ERROR", text: `명령 타임아웃: ${e.message}`, stamp: Date.now() / 1000 }]);
    } finally {
      busyRef.current = false;
      setBusy(false);
    }
  }, [voice.state]);

  return { voice, logs, busy, toggleRecord };
}

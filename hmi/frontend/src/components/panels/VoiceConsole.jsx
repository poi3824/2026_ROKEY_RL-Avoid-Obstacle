import LogPanel from "./LogPanel";
import "./VoiceConsole.css";

const STATE_META = {
  idle: { label: "대기 중 (웨이크워드 대기)", color: "#97a0b3" },
  recording: { label: "녹음 중", color: "#d1373f" },
  processing: { label: "처리 중 (LLM 분석)", color: "#c07a10" },
  speaking: { label: "응답 중 (TTS)", color: "#12946b" },
};

export default function VoiceConsole({ voice, logs, onToggleRecord }) {
  const meta = STATE_META[voice?.state] || STATE_META.idle;
  const scale = 1 + (voice?.level ?? 0) * 0.9;

  return (
    <div className="voice-page">
      <div className="card orb-card">
        <h3>실시간 음성 상태</h3>
        <div className="orb-wrap">
          <div className="orb-rings">
            <div className="orb-ring r3" style={{ background: meta.color, transform: `scale(${1 + (voice?.level ?? 0) * 1.4})` }} />
            <div className="orb-ring r2" style={{ background: meta.color, transform: `scale(${1 + (voice?.level ?? 0) * 1.1})` }} />
            <div className="orb-ring r1" style={{ background: meta.color, transform: `scale(${1 + (voice?.level ?? 0) * 0.9})` }} />
            <div className="orb-core" style={{ background: meta.color, transform: `scale(${scale})` }} />
          </div>
          <div className="orb-state"><span className="dot" style={{ background: meta.color }} />{meta.label}</div>
          <button
            type="button"
            className={"btn-hmi record-btn" + (voice?.state === "recording" ? " is-recording" : "")}
            onClick={onToggleRecord}
          >
            {voice?.state === "recording" ? "⏹️ 녹음 중지" : "\u{1F3A4} 녹음 시작"}
          </button>
        </div>
      </div>
      <LogPanel
        className="fill" title="get_keyword_node 로그" entries={logs}
        emptyText="아직 로그 없음 - get_keyword_node에서 뭔가 로그를 찍으면 여기 나타납니다."
      />
    </div>
  );
}

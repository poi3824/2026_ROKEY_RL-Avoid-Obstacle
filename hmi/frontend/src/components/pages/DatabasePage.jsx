import { useState } from "react";

const SUBTABS = [
  { id: "robot", label: "로봇로그" },
  { id: "stt", label: "STT" },
  { id: "world", label: "월드" },
];

function fmtTs(ts) {
  return ts ? new Date(ts * 1000).toLocaleString("ko-KR") : "-";
}

export default function DatabasePage({ pickAttempts = [], voiceEvents = [], worldmapScans = [], onSelectScan }) {
  const [sub, setSub] = useState("robot");

  return (
    <div>
      <div className="subtabs">
        {SUBTABS.map((t) => (
          <button
            key={t.id}
            type="button"
            className={"subtab-btn" + (sub === t.id ? " active" : "")}
            onClick={() => setSub(t.id)}
          >
            {t.label}
          </button>
        ))}
      </div>

      {sub === "robot" && (
        <div className="card">
          <table className="hmi-table">
            <thead><tr><th>ID</th><th>시각</th><th>물체</th><th>시도</th><th>그리퍼폭(mm)</th><th>결과</th></tr></thead>
            <tbody>
              {pickAttempts.length === 0 ? (
                <tr><td colSpan={6} style={{ color: "var(--muted)" }}>기록 없음</td></tr>
              ) : (
                pickAttempts.map((r) => (
                  <tr key={r.id}>
                    <td>{r.id}</td><td>{fmtTs(r.ts)}</td><td>{r.obj_label}</td>
                    <td>{r.attempt_no}</td><td>{r.gripper_width_mm}</td>
                    <td><span className={"badge " + (r.success ? "good" : "critical")}>{r.success ? "성공" : "실패"}</span></td>
                  </tr>
                ))
              )}
            </tbody>
          </table>
        </div>
      )}

      {sub === "stt" && (
        <div className="card">
          <table className="hmi-table">
            <thead><tr><th>ID</th><th>시각</th><th>종류</th><th>내용</th></tr></thead>
            <tbody>
              {voiceEvents.length === 0 ? (
                <tr><td colSpan={4} style={{ color: "var(--muted)" }}>기록 없음</td></tr>
              ) : (
                voiceEvents.map((r) => (
                  <tr key={r.id}><td>{r.id}</td><td>{fmtTs(r.ts)}</td><td>{r.kind}</td><td>{r.text}</td></tr>
                ))
              )}
            </tbody>
          </table>
        </div>
      )}

      {sub === "world" && (
        <div className="card">
          <table className="hmi-table">
            <thead><tr><th>scan_id</th><th>시각</th><th>장애물 클러스터</th></tr></thead>
            <tbody>
              {worldmapScans.length === 0 ? (
                <tr><td colSpan={3} style={{ color: "var(--muted)" }}>기록 없음</td></tr>
              ) : (
                worldmapScans.map((r) => (
                  <tr key={r.scan_id}>
                    <td><a href="#" onClick={(e) => { e.preventDefault(); onSelectScan?.(r.scan_id); }}>{r.scan_id}</a></td>
                    <td>{r.timestamp}</td><td>{r.cluster_count}</td>
                  </tr>
                ))
              )}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
}

// 재사용 가능한 로그 스트림 표시 컴포넌트. VoiceConsole이 지금 쓰지만, /rosout
// 필터링 로그 외의 다른 출처(예: bridge 자체 로그)에도 그대로 재사용 가능하게 범용으로 둔다.
//
// 2026-07-12 버그 수정: 이 파일이 LogPanel.css를 import한 적이 없어서 .log-box의
// 고정 높이/overflow-y:auto/배경 스타일이 처음부터 전혀 적용되지 않고 있었다 -
// 로그가 쌓일수록 페이지 자체가 계속 길어지던 문제의 진짜 원인이었다.
import "./LogPanel.css";

const LEVEL_COLOR = {
  DEBUG: "var(--muted)",
  INFO: "#5b9dd9",
  WARN: "var(--warn)",
  ERROR: "var(--critical)",
  FATAL: "var(--critical)",
};

export default function LogPanel({ title = "로그", entries = [], emptyText = "아직 로그 없음", className = "" }) {
  // 2026-07-12: entries는 들어온 순서(과거->최신)로 누적되므로, 최신이 위로
  // 오도록 표시 직전에만 뒤집는다(누적/트리밍 로직 자체는 안 건드림).
  const displayEntries = [...entries].reverse();
  return (
    <div className={"card" + (className ? " " + className : "")}>
      <h3>{title}</h3>
      <div className="log-box">
        {displayEntries.length === 0 ? (
          <div className="log-empty">{emptyText}</div>
        ) : (
          displayEntries.map((entry, i) => {
            const level = (entry.level || "INFO").toUpperCase();
            const d = new Date((entry.stamp || Date.now() / 1000) * 1000);
            const pad = (n) => String(n).padStart(2, "0");
            return (
              <div className="log-line" key={i}>
                <span className="log-time">{pad(d.getHours())}:{pad(d.getMinutes())}:{pad(d.getSeconds())}</span>
                <span className="log-level" style={{ color: LEVEL_COLOR[level] || LEVEL_COLOR.INFO }}>{level}</span>
                <span className="log-text">{entry.text}</span>
              </div>
            );
          })
        )}
      </div>
    </div>
  );
}

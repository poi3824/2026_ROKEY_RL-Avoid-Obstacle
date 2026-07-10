// 재사용 가능한 로그 스트림 표시 컴포넌트. VoiceConsole이 지금 쓰지만, /rosout
// 필터링 로그 외의 다른 출처(예: bridge 자체 로그)에도 그대로 재사용 가능하게 범용으로 둔다.
const LEVEL_COLOR = {
  DEBUG: "var(--muted)",
  INFO: "#5b9dd9",
  WARN: "var(--warn)",
  ERROR: "var(--critical)",
  FATAL: "var(--critical)",
};

export default function LogPanel({ title = "로그", entries = [], emptyText = "아직 로그 없음" }) {
  return (
    <div className="card">
      <h3>{title}</h3>
      <div className="log-box">
        {entries.length === 0 ? (
          <div className="log-empty">{emptyText}</div>
        ) : (
          entries.map((entry, i) => {
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

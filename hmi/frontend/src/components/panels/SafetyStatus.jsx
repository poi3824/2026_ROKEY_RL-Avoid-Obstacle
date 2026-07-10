// safety_status.schema.json 그대로 렌더링. Task 상태와는 절대 같은 컴포넌트/모델로
// 섞지 않는다(TaskProgress가 별도) - RUN/PAUSE/ESTOP만 다룬다.
const STATE_META = {
  RUN: { label: "정상 진행", cls: "good" },
  PAUSE: { label: "일시정지", cls: "warn" },
  ESTOP: { label: "비상정지", cls: "critical" },
};

export default function SafetyStatus({ status }) {
  const meta = STATE_META[status?.state] || { label: "알 수 없음", cls: "muted" };
  return (
    <div className="card">
      <h3>Safety 상태</h3>
      <div style={{ display: "flex", alignItems: "center", gap: 12 }}>
        <span className={"badge " + meta.cls} style={{ fontSize: 13, padding: "6px 14px" }}>
          {status?.state ?? "-"} · {meta.label}
        </span>
        {status?.reason ? <span style={{ fontSize: 12.5, color: "var(--ink-soft)" }}>{status.reason}</span> : null}
      </div>
    </div>
  );
}

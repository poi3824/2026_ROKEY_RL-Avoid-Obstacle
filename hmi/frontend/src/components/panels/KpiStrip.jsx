// db.summary는 useDbData()가 이미 /api/db/summary에서 받아오는 값이라 이 컴포넌트는
// 별도 fetch 없이 그대로 재사용한다 (PerformancePage.jsx와 같은 필드: total/success/success_rate).
function dbStatus(summary, error) {
  if (error) return { text: "연결 실패", cls: "critical" };
  if (summary == null) return { text: "확인 중…", cls: "muted" };
  if (summary.total === 0 && summary.success === 0) return { text: "DB 비어있음/없음", cls: "warn" };
  return { text: "연결됨", cls: "good" };
}

export default function KpiStrip({ summary, error }) {
  const status = dbStatus(summary, error);
  return (
    <div className="kpi-row">
      <div className="kpi">
        <div className="label">오늘 Pick 시도</div>
        <div className="value">{summary?.total ?? "–"}</div>
      </div>
      <div className="kpi">
        <div className="label">오늘 Pick 성공</div>
        <div className="value">{summary?.success ?? "–"}</div>
      </div>
      <div className="kpi">
        <div className="label">오늘 성공률</div>
        <div className="value">{summary?.success_rate != null ? summary.success_rate + "%" : "–"}</div>
      </div>
      <div className="kpi">
        <div className="label">DB 연결</div>
        <div className="value" style={{ fontSize: 17 }}>
          <span className={"badge " + status.cls}>{status.text}</span>
        </div>
      </div>
    </div>
  );
}

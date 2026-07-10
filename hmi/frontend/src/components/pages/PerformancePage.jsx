export default function PerformancePage({ summary }) {
  return (
    <div>
      <p className="lede">오늘 pick 성능 요약 - Phase 2에서 /api/db/summary에 실데이터 연결.</p>
      <div className="kpi-row">
        <div className="kpi"><div className="label">오늘 Pick 시도</div><div className="value">{summary?.total ?? "–"}</div></div>
        <div className="kpi"><div className="label">오늘 Pick 성공</div><div className="value">{summary?.success ?? "–"}</div></div>
        <div className="kpi"><div className="label">오늘 성공률</div><div className="value">{summary?.success_rate != null ? summary.success_rate + "%" : "–"}</div></div>
      </div>
    </div>
  );
}

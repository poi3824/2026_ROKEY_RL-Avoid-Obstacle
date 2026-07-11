import RlErrorTrend from "../panels/RlErrorTrend";
import GraspAngleGauge from "../panels/GraspAngleGauge";
import { useRlReachProgress } from "../../hooks/useRlReachProgress";

// pickAttempts는 useDbData()가 id DESC로 이미 정렬해서 준다 - 맨 앞부터 훑어서
// angle_delta_deg가 채워진(마이그레이션 이후 기록된) 첫 행을 최신값으로 쓴다.
// 옛 행(컬럼 추가 전)은 null이라 건너뛴다.
function latestGraspDelta(pickAttempts) {
  const row = pickAttempts?.find((r) => r.angle_delta_deg != null);
  return row ? row.angle_delta_deg : null;
}

export default function PerformancePage({ summary, pickAttempts }) {
  const graspDelta = latestGraspDelta(pickAttempts);
  const rl = useRlReachProgress();

  return (
    <div>
      <p className="lede">오늘 pick 성능 요약 - Phase 2에서 /api/db/summary에 실데이터 연결.</p>
      <div className="kpi-row">
        <div className="kpi"><div className="label">오늘 Pick 시도</div><div className="value">{summary?.total ?? "–"}</div></div>
        <div className="kpi"><div className="label">오늘 Pick 성공</div><div className="value">{summary?.success ?? "–"}</div></div>
        <div className="kpi"><div className="label">오늘 성공률</div><div className="value">{summary?.success_rate != null ? summary.success_rate + "%" : "–"}</div></div>
      </div>
      <div className="split-row">
        {rl.steps.length > 0 ? (
          <RlErrorTrend steps={rl.steps} goalThresholdMm={rl.goalThresholdMm} isMock={false} />
        ) : (
          <RlErrorTrend />
        )}
        {graspDelta != null ? (
          <GraspAngleGauge deltaDeg={graspDelta} isMock={false} />
        ) : (
          <GraspAngleGauge />
        )}
      </div>
    </div>
  );
}

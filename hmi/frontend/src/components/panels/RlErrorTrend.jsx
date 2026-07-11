// TODO(mock): rl_avoidance_node.infer()가 아직 NotImplementedError라(obstacle_avoidance/
// rl_avoidance_node.py) 스텝별 목표 거리 오차를 만들어내는 소스가 없다. policy 추론이
// 붙으면 MOCK_STEP_ERRORS 자리를 실제 에피소드 로그로 교체한다.
const MOCK_STEP_ERRORS = [
  182, 150, 132, 108, 96, 82, 71, 63, 58, 50,
  44, 38, 33, 29, 26, 23, 19, 21, 17, 15, 13,
];

// dsr_policy_path.py 학습 리포트 주석의 "0.02m(=20mm) goal threshold"를 그대로 참고.
const GOAL_THRESHOLD_MM = 20;

const W = 640, H = 220, PAD_L = 44, PAD_R = 16, PAD_T = 14, PAD_B = 30;

export default function RlErrorTrend({ steps = MOCK_STEP_ERRORS, isMock = true }) {
  const domainMax = Math.ceil(Math.max(...steps) / 20) * 20;
  const x = (i) => PAD_L + (i / (steps.length - 1)) * (W - PAD_L - PAD_R);
  const y = (v) => PAD_T + (1 - v / domainMax) * (H - PAD_T - PAD_B);

  const reachedStep = steps.findIndex((v) => v < GOAL_THRESHOLD_MM);
  const points = steps.map((v, i) => `${x(i)},${y(v)}`).join(" ");
  const yTicks = [0, 0.25, 0.5, 0.75, 1].map((f) => Math.round(domainMax * (1 - f)));

  return (
    <div className="card">
      <h3>
        RL 시나리오 스텝별 목표 거리 오차
        {isMock ? <span className="badge warn" style={{ marginLeft: 8 }}>MOCK DATA</span> : null}
      </h3>
      <svg viewBox={`0 0 ${W} ${H}`} width="100%" height={H} role="img" aria-label="한 시나리오 내 스텝별 목표 거리 오차 추이">
        <g stroke="var(--line)" strokeWidth="1">
          <line x1={PAD_L} y1={PAD_T} x2={PAD_L} y2={H - PAD_B} />
          <line x1={PAD_L} y1={H - PAD_B} x2={W - PAD_R} y2={H - PAD_B} />
          {yTicks.map((v) => (
            <line key={v} x1={PAD_L} y1={y(v)} x2={W - PAD_R} y2={y(v)} strokeDasharray="2 4" />
          ))}
        </g>
        <g fill="var(--muted)" fontSize="10" textAnchor="end">
          {yTicks.map((v) => <text key={v} x={PAD_L - 6} y={y(v) + 3}>{v}</text>)}
        </g>
        <line
          x1={PAD_L} y1={y(GOAL_THRESHOLD_MM)} x2={W - PAD_R} y2={y(GOAL_THRESHOLD_MM)}
          stroke="var(--warn)" strokeWidth="1.4" strokeDasharray="5 4"
        />
        <text x={W - PAD_R} y={y(GOAL_THRESHOLD_MM) - 4} fill="var(--warn)" fontSize="10.5" fontWeight="700" textAnchor="end">
          goal {GOAL_THRESHOLD_MM}mm
        </text>
        {reachedStep >= 0 ? (
          <line
            x1={x(reachedStep)} y1={PAD_T} x2={x(reachedStep)} y2={H - PAD_B}
            stroke="var(--good)" strokeWidth="1.2" strokeDasharray="3 3"
          />
        ) : null}
        <polyline fill="none" stroke="var(--accent)" strokeWidth="2" strokeLinejoin="round" strokeLinecap="round" points={points} />
        {steps.map((v, i) => {
          const isLast = i === steps.length - 1;
          return (
            <circle
              key={i} cx={x(i)} cy={y(v)} r={isLast ? 6 : 3}
              fill={isLast ? "var(--surface)" : v < GOAL_THRESHOLD_MM ? "var(--good)" : "var(--accent)"}
              stroke={isLast ? "var(--good)" : "none"} strokeWidth={isLast ? 2.5 : 0}
            />
          );
        })}
        <text x={W / 2} y={H - 6} fill="var(--muted)" fontSize="10.5" textAnchor="middle">
          스텝 (한 시나리오/에피소드, 0~{steps.length - 1})
        </text>
      </svg>
    </div>
  );
}

// TODO(mock): robot_action_node.py의 compute_grasp_c()가 매 pick마다 계산하는 델타
// (비전이 감지한 물체 짧은 변의 이미지 각도 vs GRASP_AXIS_IMG_ANGLE_DEG 기준각)가
// pick_logger.py의 pick_attempts 테이블엔 안 남는다(obj_label/attempt_no/surface_z_mm/
// gripper_width_mm/grip_detected/motion_done/success만 기록). DB에 필드가 추가되면
// MOCK_DELTA_DEG 자리를 최근 pick의 실제 델타로 교체한다.
const MOCK_DELTA_DEG = 6;

const W = 320, H = 190;
const CX = W / 2, CY = 150, R = 120, ARC_WIDTH = 18, NEEDLE_R = R - 26;

// compute_grasp_c()가 delta를 [-90, 90)로 정규화 - 0에 가까울수록 물체의 짧은 변에
// 그리퍼 축이 맞고, ±90에 가까워지면 반대쪽(긴 변)을 잡게 된다.
const ZONES = [
  { from: -90, to: -60, color: "var(--critical)" },
  { from: -60, to: -20, color: "var(--warn)" },
  { from: -20, to: 20, color: "var(--good)" },
  { from: 20, to: 60, color: "var(--warn)" },
  { from: 60, to: 90, color: "var(--critical)" },
];

function polar(deltaDeg, radius) {
  const theta = ((90 - deltaDeg) * Math.PI) / 180;
  return { x: CX + radius * Math.cos(theta), y: CY - radius * Math.sin(theta) };
}

function statusFor(delta) {
  const abs = Math.abs(delta);
  if (abs < 20) return { label: "짧은 면 정렬", cls: "good" };
  if (abs < 60) return { label: "정렬 벗어남", cls: "warn" };
  return { label: "긴 면 근접", cls: "critical" };
}

export default function GraspAngleGauge({ deltaDeg = MOCK_DELTA_DEG, isMock = true }) {
  const status = statusFor(deltaDeg);
  const needleTip = polar(deltaDeg, NEEDLE_R);
  const deltaLabel = Math.round(deltaDeg * 10) / 10;

  return (
    <div className="card">
      <h3>
        비전 그립 각도 정렬 (짧은 면 기준)
        {isMock ? <span className="badge warn" style={{ marginLeft: 8 }}>MOCK DATA</span> : null}
      </h3>
      <svg viewBox={`0 0 ${W} ${H}`} width="100%" height={H} role="img" aria-label="비전이 감지한 물체 각도와 그립 기준축 사이의 정렬 오차 게이지">
        {ZONES.map((z) => {
          const p1 = polar(z.from, R);
          const p2 = polar(z.to, R);
          return (
            <path
              key={z.from}
              d={`M ${p1.x} ${p1.y} A ${R} ${R} 0 0 1 ${p2.x} ${p2.y}`}
              fill="none" stroke={z.color} strokeWidth={ARC_WIDTH} strokeLinecap="butt"
            />
          );
        })}
        {[-90, -45, 0, 45, 90].map((d) => {
          const p = polar(d, R + 18);
          return (
            <text key={d} x={p.x} y={p.y + 3} fill="var(--muted)" fontSize="10.5" textAnchor="middle">
              {d}°
            </text>
          );
        })}
        <line x1={CX} y1={CY} x2={needleTip.x} y2={needleTip.y} stroke="var(--ink)" strokeWidth="3" strokeLinecap="round" />
        <circle cx={CX} cy={CY} r="6" fill="var(--ink)" />
        <text x={CX} y={CY + 34} fill="var(--ink)" fontSize="22" fontWeight="800" textAnchor="middle">
          {deltaLabel > 0 ? "+" : ""}{deltaLabel}°
        </text>
      </svg>
      <div style={{ textAlign: "center", marginTop: -6 }}>
        <span className={"badge " + status.cls}>{status.label}</span>
      </div>
    </div>
  );
}

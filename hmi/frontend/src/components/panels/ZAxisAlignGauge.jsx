// TODO(mock): motion_executor.py의 move_via_rl()이 스텝마다 이동한 뒤 TCP 자세는
// 알지만(_get_current_posx()), 목표 접근축(target_pos의 orientation) 대비 현재 TCP
// Z축이 옆으로 얼마나 벗어났는지(tiltX, tiltY)를 계산해 _emit_rl_step() 페이로드에
// 실어 보내는 부분은 아직 없다(pos_err_mm만 있음). RL의 orientation 보상
// (tcp_axis_alignment, _align_tcp_vertical() 주석 참고)이 느슨해서 스텝 중 TCP Z축이
// 목표 접근축에서 자주 벗어나는데, 그 방향/크기를 실시간으로 보여주기 위한 카드.
// 실데이터가 들어오면 MOCK_TILT_X/Y 자리를 최근 스텝의 실제 값으로 교체한다.
//
// tiltX/tiltY는 "목표 접근축을 따라 내려다봤을 때" 현재 Z축이 찍히는 점의 좌표다 -
// 목표 자세의 로컬 X축/Y축에 현재 Z축 단위벡터를 내적한 값(= sin(기울기각) 성분).
// 임의의 기준(월드 좌우 등)을 잡지 않고 목표 자세 자신의 축을 기준으로 삼기 때문에,
// "왼쪽/오른쪽"이 실제로 물리적 의미를 가진다(그리퍼가 어느 방향으로 넘어갔는지).
const MOCK_TILT_X = 0.12;
const MOCK_TILT_Y = -0.08;

// 링 경계(deg). GOOD은 tolerance 이내, WARN은 2x 이내, 그 밖은 CRITICAL - 다이얼
// 바깥 경계(MAX_DEG)에서 클램프해 큰 기울기도 항상 원 안에 표시한다.
const MOCK_TOLERANCE_DEG = 10;
const MAX_DEG = 30;

const W = 320, H = 250;
const CX = W / 2, CY = 116, R = 92;

// deg -> 화면 반지름. sin() 투영이라 작은 각도에서는 거의 선형이고, MAX_DEG
// 근처에서는 자연스럽게 압축된다(과도한 기울기끼리의 정밀 구분보다 "많이
// 틀어졌다"는 신호 자체가 중요한 구간이라 오히려 적절하다).
const domainSin = Math.sin((MAX_DEG * Math.PI) / 180);
function radiusForDeg(deg) {
  return (Math.sin((deg * Math.PI) / 180) / domainSin) * R;
}

function statusFor(magDeg, toleranceDeg) {
  if (magDeg <= toleranceDeg) return { label: "정렬 양호", cls: "good" };
  if (magDeg <= toleranceDeg * 2) return { label: "정렬 벗어남", cls: "warn" };
  return { label: "정렬 위험", cls: "critical" };
}

export default function ZAxisAlignGauge({
  tiltX = MOCK_TILT_X, tiltY = MOCK_TILT_Y, toleranceDeg = MOCK_TOLERANCE_DEG, isMock = true,
}) {
  const magSin = Math.min(1, Math.hypot(tiltX, tiltY));
  const magDeg = (Math.asin(magSin) * 180) / Math.PI;
  const status = statusFor(magDeg, toleranceDeg);
  const color = status.cls === "good" ? "var(--good)" : status.cls === "warn" ? "var(--warn)" : "var(--critical)";

  // 다이얼 밖으로 나가면(magDeg > MAX_DEG) 방향은 유지한 채 테두리에 클램프.
  const screenR = Math.min(radiusForDeg(magDeg), R);
  const dotX = magSin > 0 ? CX + (tiltX / magSin) * screenR : CX;
  const dotY = magSin > 0 ? CY - (tiltY / magSin) * screenR : CY;

  const goodR = radiusForDeg(toleranceDeg);
  const warnR = radiusForDeg(Math.min(toleranceDeg * 2, MAX_DEG));

  const magLabel = Math.round(magDeg * 10) / 10;

  return (
    <div className="card">
      <h3>
        TCP Z축 정렬 — 위에서 본 접근축
        {isMock ? <span className="badge warn" style={{ marginLeft: 8 }}>MOCK DATA</span> : null}
      </h3>
      <svg
        viewBox={`0 0 ${W} ${H}`}
        style={{ display: "block", width: "100%", maxWidth: W, height: "auto", margin: "0 auto" }}
        role="img"
        aria-label="목표 접근축을 따라 내려다본 TCP Z축 편차 — 중심에 가까울수록 정렬이 잘 맞음"
      >
        {/* 존 배경: critical(전체) -> warn -> good 순으로 덮어 그림 */}
        <circle cx={CX} cy={CY} r={R} fill="var(--critical)" opacity="0.10" />
        <circle cx={CX} cy={CY} r={warnR} fill="var(--warn)" opacity="0.14" />
        <circle cx={CX} cy={CY} r={goodR} fill="var(--good)" opacity="0.18" />

        {/* 링 눈금 */}
        {[goodR, warnR, R].map((r, i) => (
          <circle key={i} cx={CX} cy={CY} r={r} fill="none" stroke="var(--line)" strokeWidth="1" />
        ))}
        <text x={CX} y={CY - goodR - 4} fill="var(--good)" fontSize="9.5" fontWeight="700" textAnchor="middle">{toleranceDeg}&#176;</text>
        <text x={CX} y={CY - warnR - 4} fill="var(--warn)" fontSize="9.5" fontWeight="700" textAnchor="middle">{Math.min(toleranceDeg * 2, MAX_DEG)}&#176;</text>
        <text x={CX} y={CY - R - 4} fill="var(--muted)" fontSize="9.5" textAnchor="middle">{MAX_DEG}&#176;+</text>

        {/* 목표축 자신의 X/Y 기준선 (희미하게) */}
        <line x1={CX - R} y1={CY} x2={CX + R} y2={CY} stroke="var(--line)" strokeWidth="1" strokeDasharray="2 4" />
        <line x1={CX} y1={CY - R} x2={CX} y2={CY + R} stroke="var(--line)" strokeWidth="1" strokeDasharray="2 4" />
        <text x={CX + R + 6} y={CY + 3} fill="var(--muted)" fontSize="9.5">X</text>
        <text x={CX} y={CY - R - 14} fill="var(--muted)" fontSize="9.5" textAnchor="middle">Y</text>

        {/* 중심: 완전 정렬 지점 */}
        <circle cx={CX} cy={CY} r="3.5" fill="var(--ink)" />

        {/* 현재 편차 벡터 + 점 */}
        <line x1={CX} y1={CY} x2={dotX} y2={dotY} stroke={color} strokeWidth="2" strokeLinecap="round" opacity="0.7" />
        <circle cx={dotX} cy={dotY} r="7" fill="var(--surface)" stroke={color} strokeWidth="3" />
      </svg>
      <div style={{ textAlign: "center", marginTop: -6 }}>
        <span style={{ fontSize: 22, fontWeight: 800, marginRight: 10, color: "var(--ink)" }}>
          {magLabel}&#176;
        </span>
        <span className={"badge " + status.cls}>{status.label}</span>
      </div>
    </div>
  );
}

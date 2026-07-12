// TODO(mock): motion_executor.py의 move_via_rl()이 스텝마다 이동한 뒤 TCP 자세는
// 알지만(_get_current_posx()), 목표 접근축 대비 벌어진 각도(z_align_deg)를 계산해
// _emit_rl_step() 페이로드에 실어 보내는 부분은 아직 없다(pos_err_mm만 있음).
// RL의 orientation 보상(tcp_axis_alignment, _align_tcp_vertical() 주석 참고)이
// 느슨해서 스텝 중 TCP Z축이 목표 접근축에서 자주 벗어나는데, 그 크기를
// 실시간으로 보여주기 위한 카드. 실데이터가 들어오면 MOCK_TILT_DEG 자리를
// 최근 스텝의 실제 값으로 교체한다.
const MOCK_TILT_DEG = 15.2;

// _align_tcp_vertical()이 재정렬하기 전까지 허용 가능한 것으로 볼 기울기(deg).
// 실제 임계값은 RL 쪽 담당자가 정책 안전마진에 맞춰 조정할 자리.
const MOCK_TOLERANCE_DEG = 10;

const W = 320, H = 190;
const PX = 132, PY = 20, L = 128; // pivot(TCP)과 목표 접근축 길이

function tipFor(deg, radius) {
  const theta = (deg * Math.PI) / 180;
  return { x: PX + radius * Math.sin(theta), y: PY + radius * Math.cos(theta) };
}

function statusFor(tiltDeg, toleranceDeg) {
  const abs = Math.abs(tiltDeg);
  if (abs <= toleranceDeg) return { label: "정렬 양호", cls: "good" };
  if (abs <= toleranceDeg * 2) return { label: "정렬 벗어남", cls: "warn" };
  return { label: "정렬 위험", cls: "critical" };
}

export default function ZAxisAlignGauge({
  tiltDeg = MOCK_TILT_DEG, toleranceDeg = MOCK_TOLERANCE_DEG, isMock = true,
}) {
  const status = statusFor(tiltDeg, toleranceDeg);
  const overTolerance = Math.abs(tiltDeg) > toleranceDeg;
  const mainColor = !overTolerance ? "var(--good)" : status.cls === "critical" ? "var(--critical)" : "var(--warn)";

  const tip = tipFor(tiltDeg, L);

  // 허용 영역을 정확한 각도 쐐기 대신 피벗 아래 반원(돔) 형태로 표시 - 숫자 라벨 없이
  // "이 근방이면 안전"이라는 느낌만 전달한다.
  const domeR = L * 0.42;

  // 피벗 옆 작은 각도 호(0도 = 목표축 방향에서 실제 기울기까지)
  const ARC_R = 44;
  const arcFrom = tipFor(0, ARC_R);
  const arcTo = tipFor(tiltDeg, ARC_R);
  const sweep = tiltDeg >= 0 ? 1 : 0;

  // 화살촉(삼각형)
  const theta = (tiltDeg * Math.PI) / 180;
  const dir = { x: Math.sin(theta), y: Math.cos(theta) };
  const back = { x: tip.x - dir.x * 13, y: tip.y - dir.y * 13 };
  const perp = { x: dir.y * 5.5, y: -dir.x * 5.5 };

  const tiltLabel = Math.round(tiltDeg * 10) / 10;

  return (
    <div className="card">
      <h3>
        TCP Z축 정렬 (목표 접근축 기준)
        {isMock ? <span className="badge warn" style={{ marginLeft: 8 }}>MOCK DATA</span> : null}
      </h3>
      <svg
        viewBox={`0 0 ${W} ${H}`}
        style={{ display: "block", width: "100%", maxWidth: W, height: "auto", margin: "0 auto" }}
        role="img"
        aria-label="목표 접근축 대비 TCP Z축 기울기와 허용 범위"
      >
        <path
          d={`M ${PX - domeR} ${PY} A ${domeR} ${domeR} 0 0 1 ${PX + domeR} ${PY} Z`}
          fill="var(--good)" opacity="0.16"
        />

        <line x1={PX} y1={PY} x2={PX} y2={PY + L} stroke="var(--ink-soft)" strokeWidth="1.5" strokeDasharray="6 4" />
        <text x={PX} y={PY + L + 16} fill="var(--ink-soft)" fontSize="10" textAnchor="middle">목표 접근축</text>

        <path d={`M ${arcFrom.x} ${arcFrom.y} A ${ARC_R} ${ARC_R} 0 0 ${sweep} ${arcTo.x} ${arcTo.y}`} fill="none" stroke={mainColor} strokeWidth="1.6" />

        <line x1={PX} y1={PY} x2={tip.x} y2={tip.y} stroke={mainColor} strokeWidth="3.5" strokeLinecap="round" />
        <path d={`M ${tip.x} ${tip.y} L ${back.x + perp.x} ${back.y + perp.y} L ${back.x - perp.x} ${back.y - perp.y} Z`} fill={mainColor} />
        <text x={tip.x + 10} y={tip.y} fill={mainColor} fontSize="11" fontWeight="700">Z축</text>

        <circle cx={PX} cy={PY} r="6" fill="var(--ink)" />
        <text x={PX} y={PY - 12} fill="var(--ink-soft)" fontSize="10" fontWeight="700" textAnchor="middle">TCP</text>
      </svg>
      <div style={{ textAlign: "center", marginTop: -6 }}>
        <span style={{ fontSize: 22, fontWeight: 800, marginRight: 10, color: "var(--ink)" }}>
          {tiltLabel > 0 ? "+" : ""}{tiltLabel}&#176;
        </span>
        <span className={"badge " + status.cls}>{status.label}</span>
      </div>
    </div>
  );
}

import { SAFETY_META } from "../../constants/safety";

// PAUSE/ESTOP일 때만 탭에 상관없이 항상 보이는 전역 경고 배너 - 지금까지는
// TaskProgress 카드(대시보드 탭)까지 스크롤해야만 안전 상태를 알 수 있었다.
// RUN이거나 아직 상태를 못 받았으면(safety == null) 조용히 숨는다.
export default function SafetyBanner({ safety }) {
  if (!safety || safety.state === "RUN") return null;
  const meta = SAFETY_META[safety.state] || { label: "알 수 없음", cls: "warn" };

  return (
    <div className={"safety-banner " + meta.cls}>
      <span className="safety-banner-badge">{safety.state}</span>
      <span className="safety-banner-label">{meta.label}</span>
      {safety.reason ? <span className="safety-banner-reason">{safety.reason}</span> : null}
    </div>
  );
}

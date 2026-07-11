import { useSystemHealth } from "../../hooks/useSystemHealth";

const th = { textAlign: "left", color: "var(--ink-soft)" };

// 설정 페이지용 "시스템 정보" 카드 예시.
// - hmi/backend, hmi_ros_bridge, 마지막 확인 행: /api/health를 실제로 호출해서 얻은
//   값(fetchHealth() 최초 소비). bridge_connected는 헤더 배지(useBridgeStatus)와
//   같은 값이지만 여기선 REST 폴링이라 소켓보다 갱신이 느릴 수 있다.
// - API Base/Vision Stream/프론트엔드 모드: 빌드 타임 Vite env라 백엔드 호출 없이 바로 읽음.
// - ROS_DISTRO/빌드 버전: health.py가 아직 안 주는 값이라 TBD로 자리만 잡아둠 -
//   필요해지면 health.py에 필드 추가 후 여기 연결.
export default function SystemInfo() {
  const { health, error } = useSystemHealth();

  const backendStatus = error ? "연결 실패" : health ? "정상" : "확인 중…";
  const backendCls = error ? "critical" : health ? "good" : "muted";
  const bridgeStatus = health?.bridge_connected ? "연결됨" : "연결 안 됨";
  const bridgeCls = health?.bridge_connected ? "good" : "muted";
  const lastChecked = health?.timestamp
    ? new Date(health.timestamp * 1000).toLocaleTimeString("ko-KR")
    : "-";

  return (
    <div className="card" style={{ maxWidth: 480 }}>
      <h3>시스템 정보</h3>
      <table className="hmi-table">
        <tbody>
          <tr><th style={th}>hmi/backend</th><td><span className={"badge " + backendCls}>{backendStatus}</span></td></tr>
          <tr><th style={th}>hmi_ros_bridge</th><td><span className={"badge " + bridgeCls}>{bridgeStatus}</span></td></tr>
          <tr><th style={th}>마지막 확인</th><td>{lastChecked}</td></tr>
          <tr><th style={th}>API Base</th><td>{import.meta.env.VITE_API_BASE || "-"}</td></tr>
          <tr><th style={th}>Vision Stream</th><td>{import.meta.env.VITE_VISION_STREAM_URL || "-"}</td></tr>
          <tr><th style={th}>프론트엔드 모드</th><td>{import.meta.env.MODE}</td></tr>
          <tr><th style={th}>ROS_DISTRO / 빌드 버전</th><td style={{ color: "var(--muted)" }}>TBD - health.py 확장 필요</td></tr>
        </tbody>
      </table>
    </div>
  );
}

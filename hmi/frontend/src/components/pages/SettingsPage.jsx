import SystemInfo from "../panels/SystemInfo";

export default function SettingsPage() {
  return (
    <div>
      <div className="note">
        읽기 전용 설정 자리 - 실시간 반영(ROS 파라미터 get/set)은 hmi_ros_bridge가
        필요하다. 로봇 제어 관련 값은 motion_node.py에 하드코딩된 값을 그대로 보여준다
        (수정 기능 없음, Phase 3 이후 검토).
      </div>
      <SystemInfo />
      <div className="card" style={{ maxWidth: 480 }}>
        <table className="hmi-table">
          <tbody>
            <tr><th style={{ textAlign: "left", color: "var(--ink-soft)" }}>VELOCITY / ACC</th><td>70 / 70</td></tr>
            <tr><th style={{ textAlign: "left", color: "var(--ink-soft)" }}>grip_min_width_mm</th><td>30.0</td></tr>
            <tr><th style={{ textAlign: "left", color: "var(--ink-soft)" }}>SWEEP_VELOCITY</th><td>30</td></tr>
          </tbody>
        </table>
      </div>
    </div>
  );
}

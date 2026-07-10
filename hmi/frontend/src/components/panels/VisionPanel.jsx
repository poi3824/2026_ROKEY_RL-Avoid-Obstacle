// MJPEG는 Phase 4까지 별도 포트를 VITE_VISION_STREAM_URL로만 참조한다(하드코딩 금지).
// 최종적으로는 Flask가 /stream/vision 같은 origin으로 프록시하는 걸 목표로 한다(README 참고).
const VISION_STREAM_URL = import.meta.env.VITE_VISION_STREAM_URL;

export default function VisionPanel({ connected = false, yoloEnabled = false, onToggleYolo }) {
  return (
    <div className="card" style={{ padding: 0, overflow: "hidden" }}>
      {connected ? (
        <img
          src={`${VISION_STREAM_URL}/stream`}
          alt="카메라 스트림"
          style={{ display: "block", width: "100%", maxHeight: 520, objectFit: "contain", background: "#10141b" }}
        />
      ) : (
        <div className="placeholder-box">
          카메라 피드 연결 안 됨 (Phase 4에서 hmi_ros_bridge 연동 예정 - {VISION_STREAM_URL || "VITE_VISION_STREAM_URL 미설정"})
        </div>
      )}
      <div style={{ padding: 12, display: "flex", alignItems: "center", gap: 10 }}>
        <button type="button" className="btn-hmi" onClick={onToggleYolo} disabled={!connected}>
          {yoloEnabled ? "\u{1F9E0} YOLO 추론 끄기" : "\u{1F9E0} YOLO 추론 켜기"}
        </button>
        <span style={{ fontSize: 11.5, color: "var(--muted)" }}>
          {connected ? (yoloEnabled ? "켜짐" : "꺼짐 - 원본 프레임 표시 중") : "스트림 연결 후 사용 가능"}
        </span>
      </div>
    </div>
  );
}

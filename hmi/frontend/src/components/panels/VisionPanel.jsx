// MJPEG는 Phase 4까지 별도 포트를 VITE_VISION_STREAM_URL로만 참조한다(하드코딩 금지).
// 최종적으로는 Flask가 /stream/vision 같은 origin으로 프록시하는 걸 목표로 한다(README 참고).
export default function VisionPanel({ vision }) {
  const { connected, streamSrc, overlayEnabled, onLoad, onError, toggleOverlay, streamUrl } = vision;

  return (
    <div className="card" style={{ padding: 0, overflow: "hidden" }}>
      {streamSrc ? (
        <img
          key={streamSrc}
          src={streamSrc}
          alt="카메라 스트림"
          onLoad={onLoad}
          onError={onError}
          style={{
            display: connected ? "block" : "none",
            width: "100%", maxHeight: 520, objectFit: "contain", background: "#10141b",
          }}
        />
      ) : null}
      {!connected && (
        <div className="placeholder-box">
          카메라 스트림 연결 안 됨 - hmi_vision_stream이 떠 있는지 확인하세요
          ({streamUrl || "VITE_VISION_STREAM_URL 미설정"}, 3초 후 재시도)
        </div>
      )}
      <div style={{ padding: 12, display: "flex", alignItems: "center", gap: 10 }}>
        <button type="button" className="btn-hmi" onClick={toggleOverlay} disabled={!connected}>
          {overlayEnabled ? "\u{1F9E0} Detection 오버레이 끄기" : "\u{1F9E0} Detection 오버레이 켜기"}
        </button>
        <span style={{ fontSize: 11.5, color: "var(--muted)" }}>
          {connected
            ? (overlayEnabled
              ? "켜짐 - object_detection_node의 hmi/vision_detections 재사용 중 (자체 YOLO 로드 없음)"
              : "꺼짐 - 원본 프레임 표시 중")
            : "스트림 연결 후 사용 가능"}
        </span>
      </div>
    </div>
  );
}

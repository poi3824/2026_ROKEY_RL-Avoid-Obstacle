import { useCallback, useEffect, useRef, useState } from "react";

// Phase 4: hmi_ros_bridge의 hmi_vision_stream(MJPEG, VITE_VISION_STREAM_URL)에
// 연결. Socket.IO와는 완전히 별개 채널 - <img> 태그가 multipart/x-mixed-replace
// 스트림을 그대로 재생한다(합의된 설계: 영상은 Phase 4까지 이 방식 유지).
const VISION_STREAM_URL = import.meta.env.VITE_VISION_STREAM_URL;
const RETRY_DELAY_MS = 3000;

export function useVisionStream() {
  const [connected, setConnected] = useState(false);
  const [streamSrc, setStreamSrc] = useState(null);
  const [overlayEnabled, setOverlayEnabled] = useState(false);
  const retryTimerRef = useRef(null);

  useEffect(() => {
    if (!VISION_STREAM_URL) return undefined;
    setStreamSrc(`${VISION_STREAM_URL}/stream`);
    return () => clearTimeout(retryTimerRef.current);
  }, []);

  const onLoad = useCallback(() => setConnected(true), []);

  const onError = useCallback(() => {
    setConnected(false);
    retryTimerRef.current = setTimeout(() => {
      setStreamSrc(`${VISION_STREAM_URL}/stream?_=${Date.now()}`);
    }, RETRY_DELAY_MS);
  }, []);

  const toggleOverlay = useCallback(async () => {
    if (!VISION_STREAM_URL) return;
    const next = !overlayEnabled;
    try {
      const res = await fetch(`${VISION_STREAM_URL}/overlay?enabled=${next}`);
      const data = await res.json();
      setOverlayEnabled(!!data.overlay_enabled);
    } catch {
      // 스트림이 연결 안 된 상태에서 토글해도 무시 - 재연결 후 다시 누르면 됨
    }
  }, [overlayEnabled]);

  return { connected, streamSrc, overlayEnabled, onLoad, onError, toggleOverlay, streamUrl: VISION_STREAM_URL };
}

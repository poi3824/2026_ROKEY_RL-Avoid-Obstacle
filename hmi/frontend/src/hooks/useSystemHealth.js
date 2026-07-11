import { useEffect, useState } from "react";
import { fetchHealth } from "../api/client";

// SettingsPage "시스템 정보" 카드가 쓰는 훅. /api/health는 backend/api/health.py에
// 이미 있었지만(status/timestamp/bridge_connected) 지금까지 프론트엔드 어디서도
// 소비하지 않던 엔드포인트다.
export function useSystemHealth() {
  const [health, setHealth] = useState(null);
  const [error, setError] = useState(null);

  useEffect(() => {
    let cancelled = false;

    async function load() {
      try {
        const data = await fetchHealth();
        if (cancelled) return;
        setHealth(data);
        setError(null);
      } catch (e) {
        if (!cancelled) setError(e.message);
      }
    }

    load();
    const intervalId = setInterval(load, 10000);
    return () => {
      cancelled = true;
      clearInterval(intervalId);
    };
  }, []);

  return { health, error };
}

import { useEffect, useState } from "react";
import { fetchDbSummary, fetchPickAttempts, fetchVoiceEvents, fetchWorldmapScans } from "../api/client";

// Phase 2: DatabasePage/PerformancePage가 쓰는 실데이터 훅. mock/data.js와 같은
// shape을 유지해서 컴포넌트 쪽은 mock -> real 전환에 코드 변경이 필요 없다.
export function useDbData() {
  const [summary, setSummary] = useState(null);
  const [pickAttempts, setPickAttempts] = useState([]);
  const [voiceEvents, setVoiceEvents] = useState([]);
  const [worldmapScans, setWorldmapScans] = useState([]);
  const [error, setError] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    let cancelled = false;

    async function load() {
      try {
        const [s, p, v, w] = await Promise.all([
          fetchDbSummary(),
          fetchPickAttempts(50),
          fetchVoiceEvents(50),
          fetchWorldmapScans(30),
        ]);
        if (cancelled) return;
        setSummary(s);
        setPickAttempts(p.rows);
        setVoiceEvents(v.rows);
        setWorldmapScans(w.rows);
        setError(null);
      } catch (e) {
        if (!cancelled) setError(e.message);
      } finally {
        if (!cancelled) setLoading(false);
      }
    }

    load();
    const intervalId = setInterval(load, 5000);
    return () => {
      cancelled = true;
      clearInterval(intervalId);
    };
  }, []);

  return { summary, pickAttempts, voiceEvents, worldmapScans, error, loading };
}

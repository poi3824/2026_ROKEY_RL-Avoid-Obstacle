import { useEffect, useState } from "react";
import { fetchWorldmapList, fetchWorldmapLatest, fetchWorldmapObstacles, fetchWorldmapPoints } from "../api/client";

// Phase 6: RobotViewer(@react-three/fiber)가 쓰는 훅 - scanId가 없으면 최신
// 스캔을 자동으로 고른다(기존 hmi_bridge의 world_map_viewer.js와 동일 동작).
export function useWorldMapScan(requestedScanId) {
  const [scanIds, setScanIds] = useState([]);
  const [scanId, setScanId] = useState(requestedScanId || null);
  const [points, setPoints] = useState([]);
  const [obstacles, setObstacles] = useState([]);
  const [error, setError] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    let cancelled = false;
    async function loadList() {
      try {
        const { scan_ids } = await fetchWorldmapList();
        if (cancelled) return;
        setScanIds(scan_ids);
        if (requestedScanId && scan_ids.includes(requestedScanId)) {
          setScanId(requestedScanId);
        } else if (scan_ids.length > 0) {
          const latest = await fetchWorldmapLatest();
          if (!cancelled) setScanId(latest.scan_id);
        } else {
          setLoading(false);
        }
      } catch (e) {
        if (!cancelled) {
          setError(e.message);
          setLoading(false);
        }
      }
    }
    loadList();
    return () => { cancelled = true; };
  }, [requestedScanId]);

  useEffect(() => {
    if (!scanId) return undefined;
    let cancelled = false;
    setLoading(true);
    async function loadScan() {
      try {
        const [pointsResp, obstaclesResp] = await Promise.all([
          fetchWorldmapPoints(scanId),
          fetchWorldmapObstacles(scanId),
        ]);
        if (cancelled) return;
        setPoints(pointsResp.points);
        setObstacles(obstaclesResp.obstacles);
        setError(null);
      } catch (e) {
        if (!cancelled) setError(e.message);
      } finally {
        if (!cancelled) setLoading(false);
      }
    }
    loadScan();
    return () => { cancelled = true; };
  }, [scanId]);

  return { scanIds, scanId, setScanId, points, obstacles, error, loading };
}

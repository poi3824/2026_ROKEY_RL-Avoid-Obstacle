import { useEffect, useState } from "react";
import {
  fetchWorldmapList,
  fetchWorldmapLatest,
  fetchWorldmapObstacles,
  fetchWorldmapPoints,
  fetchWorldmapVariants,
} from "../api/client";

// Phase 6: RobotViewer(@react-three/fiber)가 쓰는 훅 - scanId가 없으면 최신
// 스캔을 자동으로 고른다(기존 hmi_bridge의 world_map_viewer.js와 동일 동작).
//
// 2026-07-12: World Map 실측 검증(TF-only vs TF+ICP, DBSCAN만 vs Hough 2차분리)을
// 3D로 비교하기 위해 pointVariant/obstacleVariant 토글을 추가. ICP variant는
// offline_icp_experiment.py로 미리 계산해둔 스캔에서만 쓸 수 있어서 scanId가
// 바뀔 때마다 icpAvailable도 같이 갱신한다(없는데 클릭하면 404가 나는 걸 막기 위함).
export function useWorldMapScan(requestedScanId) {
  const [scanIds, setScanIds] = useState([]);
  const [scanId, setScanId] = useState(requestedScanId || null);
  const [pointVariant, setPointVariant] = useState("raw"); // "raw" | "icp"
  const [obstacleVariant, setObstacleVariant] = useState("hough"); // "hough" | "dbscan_only"
  const [icpAvailable, setIcpAvailable] = useState(false);
  const [dbscanOnlyAvailable, setDbscanOnlyAvailable] = useState(false);
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
    async function loadVariants() {
      try {
        const { icp_available, dbscan_only_available } = await fetchWorldmapVariants(scanId);
        if (cancelled) return;
        setIcpAvailable(icp_available);
        setDbscanOnlyAvailable(dbscan_only_available);
        if (!icp_available) setPointVariant("raw"); // 이 스캔엔 ICP 결과가 없음 - 강제로 raw로
        if (!dbscan_only_available) setObstacleVariant("hough"); // DBSCAN-only 미리 계산본이 없음
      } catch {
        if (!cancelled) {
          setIcpAvailable(false);
          setDbscanOnlyAvailable(false);
        }
      }
    }
    loadVariants();
    return () => { cancelled = true; };
  }, [scanId]);

  useEffect(() => {
    if (!scanId) return undefined;
    let cancelled = false;
    setLoading(true);
    async function loadScan() {
      try {
        const [pointsResp, obstaclesResp] = await Promise.all([
          fetchWorldmapPoints(scanId, pointVariant),
          fetchWorldmapObstacles(scanId, obstacleVariant),
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
  }, [scanId, pointVariant, obstacleVariant]);

  return {
    scanIds, scanId, setScanId,
    points, obstacles, error, loading,
    pointVariant, setPointVariant,
    obstacleVariant, setObstacleVariant,
    icpAvailable,
    dbscanOnlyAvailable,
  };
}

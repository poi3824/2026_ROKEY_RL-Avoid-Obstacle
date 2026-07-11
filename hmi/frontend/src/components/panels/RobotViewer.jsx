import Scene from "../viewer3d/Scene";
import { useWorldMapScan } from "../../hooks/useWorldMapScan";
import "./RobotViewer.css";

// Phase 6: hmi_bridge/static/js/world_map_viewer.js(vanilla three.js, iframe으로
// 끼워넣던 것)를 @react-three/fiber 컴포넌트로 포팅 - iframe 없이 React 안에서
// 직접 렌더링한다. hmi_bridge 패키지는 계속 무수정/병행 운영 중이며, 실제
// 대체가 검증된 뒤에만 별도로 deprecated 처리한다.
export default function RobotViewer({ scanId: requestedScanId, bare = false }) {
  const { scanIds, scanId, setScanId, points, obstacles, error, loading } = useWorldMapScan(requestedScanId);

  return (
    <div className={bare ? "" : "card"}>
      <div className="viewer3d-head">
        <h3 style={{ margin: 0 }}>World / Robot 3D Viewer</h3>
        <select
          value={scanId || ""}
          onChange={(e) => setScanId(e.target.value)}
          disabled={scanIds.length === 0}
        >
          {scanIds.length === 0 ? <option value="">스캔 없음</option> : null}
          {scanIds.map((id) => (
            <option key={id} value={id}>{id}</option>
          ))}
        </select>
      </div>

      {error ? (
        <div className="note">World Map API 오류: {error} (hmi/backend가 떠 있는지 확인)</div>
      ) : null}

      <div className="viewer3d-canvas-wrap">
        {scanId ? (
          <Scene points={points} obstacles={obstacles} />
        ) : (
          <div className="placeholder-box" style={{ height: "100%" }}>
            {loading ? "불러오는 중..." : "저장된 월드맵 스캔이 없습니다"}
          </div>
        )}
      </div>

      {scanId ? (
        <div className="viewer3d-status">
          {scanId} - {points.length} points, {obstacles.length} obstacles
        </div>
      ) : null}
    </div>
  );
}

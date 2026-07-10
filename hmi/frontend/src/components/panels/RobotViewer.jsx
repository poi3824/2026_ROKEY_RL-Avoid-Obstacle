// Phase 6에서 @react-three/fiber + drei로 교체될 자리 - 지금은 순수 placeholder.
// world_map_viewer.js(hmi_bridge, vanilla three.js)의 OrbitControls/Grid/Axes/
// PointCloud/ObstacleMap 로직이 여기로 포팅될 예정.
export default function RobotViewer({ scanId }) {
  return (
    <div className="card">
      <h3>World / Robot 3D Viewer</h3>
      <div className="placeholder-box">
        {scanId ? `3D 뷰 - ${scanId} (Phase 6: @react-three/fiber 포팅 예정)` : "3D 뷰 - Phase 6에서 연동 예정"}
      </div>
    </div>
  );
}

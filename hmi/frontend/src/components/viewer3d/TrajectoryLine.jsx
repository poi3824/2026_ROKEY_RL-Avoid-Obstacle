import { useMemo } from "react";
import * as THREE from "three";
import { Line } from "@react-three/drei";

// 범용 궤적/경로 라인 - world_map_node의 스캔 pose 시퀀스나 향후 로봇 TCP 궤적
// 표시에 재사용 가능하게 별도 컴포넌트로 둔다(Phase 6 컴포넌트 목록 항목).
export default function TrajectoryLine({ points, color = 0x2f5fe0 }) {
  const vecPoints = useMemo(
    () => (points || []).map(([x, y, z]) => new THREE.Vector3(x, y, z)),
    [points],
  );
  if (vecPoints.length < 2) return null;
  return <Line points={vecPoints} color={color} lineWidth={1.5} />;
}

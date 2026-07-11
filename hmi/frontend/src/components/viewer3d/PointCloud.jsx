import { useMemo } from "react";
import * as THREE from "three";

// hmi_bridge/static/js/world_map_viewer.js의 renderPoints() 포팅 - 높이(z)로
// 색을 입혀 테이블(낮음, 파랑)과 장애물 위쪽(높음, 노랑)을 구분한다.
export default function PointCloud({ points }) {
  const { positions, colors } = useMemo(() => {
    if (!points || points.length === 0) {
      return { positions: new Float32Array(0), colors: new Float32Array(0) };
    }
    const positions = new Float32Array(points.length * 3);
    let zMin = Infinity;
    let zMax = -Infinity;
    for (let i = 0; i < points.length; i++) {
      positions[i * 3] = points[i][0];
      positions[i * 3 + 1] = points[i][1];
      positions[i * 3 + 2] = points[i][2];
      zMin = Math.min(zMin, points[i][2]);
      zMax = Math.max(zMax, points[i][2]);
    }

    const colors = new Float32Array(points.length * 3);
    const zSpan = Math.max(zMax - zMin, 1e-6);
    const lowColor = new THREE.Color(0x2255aa);
    const highColor = new THREE.Color(0xffdd33);
    const tmpColor = new THREE.Color();
    for (let i = 0; i < points.length; i++) {
      const t = (points[i][2] - zMin) / zSpan;
      tmpColor.copy(lowColor).lerp(highColor, t);
      colors[i * 3] = tmpColor.r;
      colors[i * 3 + 1] = tmpColor.g;
      colors[i * 3 + 2] = tmpColor.b;
    }
    return { positions, colors };
  }, [points]);

  if (positions.length === 0) return null;

  return (
    <points>
      <bufferGeometry>
        <bufferAttribute attach="attributes-position" args={[positions, 3]} />
        <bufferAttribute attach="attributes-color" args={[colors, 3]} />
      </bufferGeometry>
      <pointsMaterial size={0.004} vertexColors />
    </points>
  );
}

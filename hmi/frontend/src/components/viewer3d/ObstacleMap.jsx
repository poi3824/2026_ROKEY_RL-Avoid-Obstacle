// hmi_bridge/static/js/world_map_viewer.js의 renderObstacles() 포팅 - 장애물
// 원기둥(body) + safety_radius 와이어프레임을 그린다. CylinderGeometry는
// 로컬 Y축이 높이 방향이라 90도 돌려서 ROS Z가 높이가 되게 맞춘다(원본과 동일).
function Obstacle({ obstacle }) {
  const [cx, cy, cz] = obstacle.centroid;
  return (
    <group>
      <mesh rotation={[Math.PI / 2, 0, 0]} position={[cx, cy, cz]}>
        <cylinderGeometry args={[obstacle.radius, obstacle.radius, obstacle.height, 24, 1, true]} />
        <meshBasicMaterial color={0xff9933} transparent opacity={0.35} side={2} />
      </mesh>
      <mesh
        rotation={[Math.PI / 2, 0, 0]}
        position={[cx, cy, obstacle.z_min + obstacle.safety_height / 2]}
      >
        <cylinderGeometry args={[obstacle.safety_radius, obstacle.safety_radius, obstacle.safety_height, 24, 1, true]} />
        <meshBasicMaterial color={0xff3333} wireframe transparent opacity={0.5} />
      </mesh>
    </group>
  );
}

export default function ObstacleMap({ obstacles }) {
  if (!obstacles || obstacles.length === 0) return null;
  return (
    <group>
      {obstacles.map((obs) => (
        <Obstacle key={obs.id} obstacle={obs} />
      ))}
    </group>
  );
}

// 범용 위치 마커(목표 지점, place target 등) - 지금은 world map 데이터에 직접
// 연결돼 있지 않지만, Phase 6 컴포넌트 목록에 맞춰 재사용 가능한 형태로 둔다.
export default function TargetMarker({ position, color = 0x2f5fe0, label }) {
  if (!position) return null;
  const [x, y, z] = position;
  return (
    <group position={[x, y, z]}>
      <mesh rotation={[Math.PI, 0, 0]}>
        <coneGeometry args={[0.02, 0.05, 16]} />
        <meshBasicMaterial color={color} />
      </mesh>
      {label ? (
        <mesh position={[0, 0, 0.03]}>
          <sphereGeometry args={[0.003, 8, 8]} />
          <meshBasicMaterial color={color} />
        </mesh>
      ) : null}
    </group>
  );
}

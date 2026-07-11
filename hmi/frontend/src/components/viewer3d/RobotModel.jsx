// 실제 URDF/메시 로딩 파이프라인은 아직 없다 - base_link 원점을 표시하는
// 간단한 placeholder geometry만 둔다(Phase 6 컴포넌트 목록 항목, 실제 로봇
// 팔 모델 연동은 후속 작업).
export default function RobotModel() {
  return (
    <mesh position={[0, 0, 0.02]}>
      <boxGeometry args={[0.08, 0.08, 0.04]} />
      <meshStandardMaterial color={0x5a6478} />
    </mesh>
  );
}

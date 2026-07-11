import { Canvas } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import PointCloud from "./PointCloud";
import ObstacleMap from "./ObstacleMap";
import RobotModel from "./RobotModel";

// hmi_bridge/static/js/world_map_viewer.js(vanilla three.js)의 좌표계 처리를
// 그대로 포팅: three.js는 Y-up이 기본이지만 base_link(ROS)는 Z-up이다.
// rosRoot 그룹을 X축으로 -90도 돌려두면 자식들은 (x,y,z)를 ROS 좌표 그대로
// 넣어도 화면에서 Z가 위로 보인다. Grid는 원본과 동일하게 이 그룹 밖(회전 안
// 됨)에 둔다 - GridHelper 기본 방향(Y-up XZ평면)이 이미 base_link의 XY 평면과
// 맞기 때문(원본 world_map_viewer.js 주석과 동일한 이유).
export default function Scene({ points, obstacles }) {
  return (
    <Canvas
      camera={{ position: [0.8, -0.8, 0.8], up: [0, 0, 1], fov: 60, near: 0.01, far: 100 }}
      style={{ background: "#111111" }}
    >
      <ambientLight intensity={0.6} />
      <directionalLight position={[1, 1, 2]} intensity={0.6} />

      <gridHelper args={[2, 20, 0x444444, 0x222222]} />

      <group rotation={[-Math.PI / 2, 0, 0]}>
        <axesHelper args={[0.2]} />
        <RobotModel />
        <PointCloud points={points} />
        <ObstacleMap obstacles={obstacles} />
      </group>

      <OrbitControls target={[0.4, 0, 0.1]} />
    </Canvas>
  );
}

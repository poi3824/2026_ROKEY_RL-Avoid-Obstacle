// 노드 상태는 아직 실데이터 연동 전이다 - Flask/브릿지가 각 노드의 생사를 직접
// 알 방법이 없고(rclpy 그래프에서 별도 헬스체크 토픽/서비스가 필요), 실제 노드
// 이름만 미리 배치해둔 자리다(기존 hmi_interface/templates/index.html과 동일 원칙).
// Safety(RUN/PAUSE/ESTOP) 배지는 여기 없다 - TaskProgress 헤더 쪽에서 보여준다.
const NODES = [
  { name: "brain_node", desc: "오케스트레이터", state: "good" },
  { name: "motion_node", desc: "로봇/그리퍼 제어", state: "good" },
  { name: "safety_monitor_node", desc: "안전 판단/하드정지", state: "good" },
  { name: "object_detection_node", desc: "YOLO + hand 감지", state: "good" },
  { name: "get_keyword_node", desc: "웨이크워드/STT/LLM", state: "good" },
  { name: "world_map_node", desc: "월드맵 스캔", state: "good" },
  { name: "rl_avoidance_node", desc: "회피 정책 추론 (미구현)", state: "warn" },
];

export default function NodeStatus() {
  return (
    <div className="card">
      <h3>노드 상태</h3>
      <div className="node-tiles">
        {NODES.map((node) => (
          <div key={node.name} className={"node-tile " + node.state}>
            <div className="node-tile-top">
              <span className={"led-lg " + node.state} />
              <span className="node-tile-state">{node.state === "warn" ? "stub" : "실행중"}</span>
            </div>
            <div className="node-tile-name">{node.name}</div>
            <div className="node-tile-desc">{node.desc}</div>
          </div>
        ))}
      </div>
    </div>
  );
}

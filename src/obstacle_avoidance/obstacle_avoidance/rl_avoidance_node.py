# 학습된 회피 정책(policy)을 로드해서 추론만 수행하는 노드 (학습 자체는
# MuJoCo(dsr_mujoco)에서 오프라인으로 진행하고, 여기서는 결과 policy만 불러온다).
#
# 별도 프로세스로 띄운다 (이유는 pointcloud_node 주석 참고 - robot_action_node의
# blocking 서비스 호출과 분리하기 위함).
#
# 여기서 나가는 /avoidance_cmd만 robot_action_node와 주고받는 진짜 ROS 통신이고,
# robot_action_node 안의 motion_executor 호출은 ROS가 아니라 같은 프로세스 안의
# 함수 호출이다 (architecture_draft.drawio 1페이지 참고).
#
# TODO: policy(.zip/.pt) 로드 및 obstacle_state -> action 추론 로직 미구현.
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray

from obstacle_avoidance_msgs.msg import AvoidanceCmd, WorldMapUpdate


class RLAvoidanceNode(Node):
    def __init__(self):
        super().__init__('rl_avoidance_node')

        self.policy = self.load_policy()

        self.obstacle_state_sub = self.create_subscription(
            Float32MultiArray, '/obstacle_state', self.obstacle_state_callback, 10
        )

        # world_map_node가 TRANSIENT_LOCAL로 publish한다(world_map_node.py의
        # "월드맵은 상태에 가깝다" 주석 참고) - 구독도 같은 QoS로 맞춰야 늦게
        # 구독을 시작해도 마지막 스캔 결과를 즉시 받는다. QoS가 안 맞으면
        # 연결 자체가 조용히 안 된다.
        world_map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.world_map_obstacles = []
        self.world_map_sub = self.create_subscription(
            WorldMapUpdate, '/world_map/obstacles', self.world_map_callback, world_map_qos
        )

        self.avoidance_cmd_pub = self.create_publisher(
            AvoidanceCmd, '/avoidance_cmd', 10
        )

    def load_policy(self):
        # TODO: 실제 학습된 policy(.zip/.pt) 로드는 policy가 준비되면 구현한다.
        # 지금은 world_map 구독 flow를 테스트하려면 노드가 기동은 돼야 하므로
        # 더미로 채워둔다 - infer()는 여전히 미구현이라 /obstacle_state가
        # 들어와도 그쪽은 그대로 죽는다.
        self.get_logger().warn("load_policy(): 더미 - 실제 policy 로드 미구현")
        return None

    def obstacle_state_callback(self, msg):
        dx, dy, dz, active = self.infer(msg)

        cmd = AvoidanceCmd()
        cmd.dx = dx
        cmd.dy = dy
        cmd.dz = dz
        cmd.active = active
        self.avoidance_cmd_pub.publish(cmd)

    def world_map_callback(self, msg):
        # TODO: infer()가 이 정적 장애물 맵을 관측(observation)에 반영하는 로직은
        # policy가 실제로 구현된 뒤에 붙인다. 지금은 최신 월드맵을 저장하고 내용을
        # 그대로 로그로 찍어서, STT -> world_map_node -> 여기까지 flow가 끝까지
        # 도달하고 값이 맞는지 확인하는 용도로만 쓴다.
        self.world_map_obstacles = list(msg.obstacles)
        self.get_logger().info(
            f"world map updated: {len(msg.obstacles)} obstacles (scan_dir={msg.scan_dir})"
        )
        for obs in msg.obstacles:
            self.get_logger().info(
                f"  obstacle id={obs.id} "
                f"centroid=({obs.centroid.x:.3f}, {obs.centroid.y:.3f}, {obs.centroid.z:.3f}) "
                f"radius={obs.radius:.3f} safety_radius={obs.safety_radius:.3f} "
                f"height={obs.height:.3f} safety_height={obs.safety_height:.3f} "
                f"shape_type={obs.shape_type} num_points={obs.num_points} "
                f"confidence={obs.confidence:.2f}"
            )

    def infer(self, obstacle_state_msg):
        raise NotImplementedError('policy 추론 로직 미구현')


def main(args=None):
    rclpy.init(args=args)
    node = RLAvoidanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

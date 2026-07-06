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
from std_msgs.msg import Float32MultiArray

from obstacle_avoidance_msgs.msg import AvoidanceCmd


class RLAvoidanceNode(Node):
    def __init__(self):
        super().__init__('rl_avoidance_node')

        self.policy = self.load_policy()

        self.obstacle_state_sub = self.create_subscription(
            Float32MultiArray, '/obstacle_state', self.obstacle_state_callback, 10
        )
        self.avoidance_cmd_pub = self.create_publisher(
            AvoidanceCmd, '/avoidance_cmd', 10
        )

    def load_policy(self):
        raise NotImplementedError('학습된 policy 로드 미구현')

    def obstacle_state_callback(self, msg):
        dx, dy, dz, active = self.infer(msg)

        cmd = AvoidanceCmd()
        cmd.dx = dx
        cmd.dy = dy
        cmd.dz = dz
        cmd.active = active
        self.avoidance_cmd_pub.publish(cmd)

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

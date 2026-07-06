# 이동(transit) 구간의 장애물을 카메라로 인식해 rl_avoidance_node(obstacle_avoidance
# 패키지)에 전달하는 노드. point cloud 처리 의존성(open3d 등)이 RL 추론 의존성
# (torch 등)과 달라서 별도 패키지(pointcloud_perception)로 분리했다.
#
# 별도 프로세스로도 띄운다: robot_action_node는 get_keyword 응답을
# spin_until_future_complete로 blocking 대기하는 구간이 있어서, 같은 프로세스에
# 있으면 그동안 이 회피 루프도 같이 멈추기 때문.
#
# 구독 토픽은 실제 `ros2 topic list`/`topic info`/`topic echo`로 확인함:
#   /camera/camera/depth/color/points  [sensor_msgs/msg/PointCloud2]
#   frame_id: camera_depth_optical_frame
#
# 주의: frame_id가 camera_depth_optical_frame이다. object_detection이 쓰는
# T_gripper2camera.npy는 color 프레임 기준(픽셀 + color intrinsics)으로 만들어진
# 것으로 보이는데, depth와 color 광학 프레임 원점이 다를 수 있어서 로봇 베이스
# 좌표계로 변환하기 전에 이 캘리브레이션을 그대로 재사용해도 되는지 확인 필요.
#
# TODO: point cloud 다운샘플(voxel), self-filter(로봇팔/그리퍼 자기 자신 제거),
# 로봇 베이스 좌표계 변환, obstacle_state 계산 로직 미구현.
# /obstacle_state 메시지 타입도 RL observation 설계가 끝나면 커스텀 메시지로
# 바꿔야 한다 (지금은 Float32MultiArray로 자리만 잡아둠).
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray

POINTS_TOPIC = '/camera/camera/depth/color/points'


class PointCloudNode(Node):
    def __init__(self):
        super().__init__('pointcloud_node')

        self.points_sub = self.create_subscription(
            PointCloud2, POINTS_TOPIC, self.points_callback, 10
        )
        self.obstacle_state_pub = self.create_publisher(
            Float32MultiArray, '/obstacle_state', 10
        )

    def points_callback(self, msg):
        obstacle_state = self.compute_obstacle_state(msg)
        self.obstacle_state_pub.publish(obstacle_state)

    def compute_obstacle_state(self, points_msg):
        raise NotImplementedError('obstacle_state 계산 로직 미구현')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

# pick-and-place 애플리케이션 노드들을 한 번에 띄우는 launch (개편 v2).
#
# 로봇 드라이버(dsr_bringup2)와 RealSense 카메라는 여기서 안 띄운다 — 사용자가 직접
# 켠 뒤 이 launch를 실행한다. (드라이버가 제공하는 /dsr01/motion/* 서비스와 카메라
# /camera/camera/* 토픽이 이미 떠 있어야 함.)
#
# 띄우는 노드:
#   object_detection_node : YOLO 검출(get_3d_position) + hand 감지(/hand_detected)
#   get_keyword_node      : STT (웨이크워드 → STT → LLM), 음성정지 → /voice/estop
#   safety_monitor_node   : /hand_detected + /voice/estop 감시 → /safety/state, 하드정지
#   motion_node           : dsr_node + MotionExecutor + Action server(MoveTo/Pick/Place)
#   brain_node            : 오케스트레이터. get_keyword 수신 → Pick/Place goal,
#                            WORLD_MAP 명령 수신 시 update_world_map 서비스 호출
#   world_map_node        : MoveLine으로 스캔 경로를 훑고 DBSCAN 클러스터링해서
#                            /world_map/obstacles publish (update_world_map 서비스 제공)
#   rl_avoidance_node     : /world_map/obstacles 구독(장애물 맵 캐시) + /obstacle_state
#                            구독 → /avoidance_cmd publish. policy 로드/추론은 아직 stub.
#
# 사용:
#   ros2 launch my_robot_pkg pnp_bringup.launch.py
#   ros2 launch my_robot_pkg pnp_bringup.launch.py grip_min_width_mm:=25.0
#
# 기동 순서는 신경 쓰지 않아도 된다 — 각 노드가 wait_for_service/wait_for_server로
# 상대가 뜰 때까지 기다린다(motion은 get_3d_position을, brain은 get_keyword와 motion
# 액션 서버를 기다림). world_map_node/rl_avoidance_node는 서로/brain_node를 기다리지
# 않고 각자 독립적으로 뜬다 — world_map_node는 update_world_map 서비스가 호출될 때만
# 실제로 MoveLine을 사용한다.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    grip_min_width = LaunchConfiguration("grip_min_width_mm")

    args = [
        DeclareLaunchArgument(
            "grip_min_width_mm", default_value="30.0",
            description="파지 성공으로 인정할 최소 그리퍼 너비(mm). 통 크기에 맞춰 조정.",
        ),
    ]

    object_detection_node = Node(
        package="object_detection", executable="object_detection_node",
        name="object_detection_node", output="screen",
    )
    get_keyword_node = Node(
        package="voice_interface", executable="get_keyword_node",
        name="get_keyword_node", output="screen",
    )
    safety_monitor_node = Node(
        package="safety_monitor", executable="safety_monitor_node",
        name="safety_monitor_node", output="screen",
    )
    motion_node = Node(
        package="my_robot_pkg", executable="motion_node",
        name="motion_node", output="screen",
        parameters=[{"grip_min_width_mm": grip_min_width}],
    )
    brain_node = Node(
        package="my_robot_pkg", executable="brain_node",
        name="brain_node", output="screen",
    )
    world_map_node = Node(
        package="pointcloud_perception", executable="world_map_node",
        name="world_map_node", output="screen",
    )
    rl_avoidance_node = Node(
        package="obstacle_avoidance", executable="rl_avoidance_node",
        name="rl_avoidance_node", output="screen",
    )

    return LaunchDescription(args + [
        object_detection_node,
        get_keyword_node,
        safety_monitor_node,
        motion_node,
        brain_node,
        world_map_node,
        rl_avoidance_node,
    ])

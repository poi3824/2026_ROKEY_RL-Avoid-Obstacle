# HMI 쪽 ROS 노드 2개를 한 번에 띄우는 launch.
#
# 로봇 드라이버(dsr_bringup2)와 RealSense 카메라는 여기서 안 띄운다 - 사용자가
# 직접 켠다(my_robot_pkg/launch/pnp_bringup.launch.py와 동일 원칙). hmi/backend
# (Flask)와 hmi/frontend(Vite)도 ROS 노드가 아니라 여기 안 넣는다 - venv/npm
# 활성화가 필요해서 launch의 ExecuteProcess로 묶으면 오히려 로그/재시작 관리가
# 불편해진다(합의: 따로 실행).
#
# 띄우는 노드:
#   hmi_ros_bridge_server : /voice, /safety/state, hmi/task_status/* 구독 +
#                            hmi/backend "/ros" 네임스페이스에 Socket.IO 연결
#   hmi_vision_stream      : 카메라 MJPEG(8767) - object_detection_node의
#                            hmi/vision_detections를 재사용(자체 YOLO 로드 없음)
#
# 사용:
#   ros2 launch hmi_ros_bridge hmi_bringup.launch.py
#
# 순서 상관없이 떠도 된다 - hmi_ros_bridge_server는 hmi/backend가 아직 없어도
# 지수 백오프로 계속 재연결을 시도하고(죽지 않음), hmi_vision_stream도
# object_detection_node가 없으면 감지 오버레이만 못 그릴 뿐 스트림 자체는 켜진다.
from launch import LaunchDescription
from launch_ros.actions import Node

UNBUFFERED_ENV = {"PYTHONUNBUFFERED": "1"}


def generate_launch_description():
    hmi_ros_bridge_server = Node(
        package="hmi_ros_bridge", executable="hmi_ros_bridge_server",
        name="hmi_ros_bridge", output="both", additional_env=UNBUFFERED_ENV,
    )
    hmi_vision_stream = Node(
        package="hmi_ros_bridge", executable="hmi_vision_stream",
        name="hmi_vision_stream", output="both", additional_env=UNBUFFERED_ENV,
    )

    return LaunchDescription([
        hmi_ros_bridge_server,
        hmi_vision_stream,
    ])

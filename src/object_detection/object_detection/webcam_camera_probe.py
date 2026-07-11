"""디버깅용: RealSense 없이 일반 웹캠으로 object_detection_node/HMI 파이프라인을
테스트하기 위한 임시 노드.

/camera/camera/color/image_raw + /camera/camera/color/camera_info를 웹캠
프레임으로 흉내낸다. ImgNode(realsense.py)가 기대하는 토픽/타입과 동일하므로
object_detection_node를 코드 변경 없이 그대로 띄울 수 있다. depth는 흉내내지
않는다 - hand 체크 타이머(detect_frame)와 hmi/vision_detections 발행 경로는
컬러 프레임만 쓰므로 이 테스트(HMI 오버레이 확인)엔 depth가 필요 없다. 단,
get_3d_position(실제 pick 좌표 계산)은 depth가 없으면 계속 대기하니 이 노드로는
확인할 수 없다.

카메라 intrinsics는 실측값이 없어(웹캠은 캘리브레이션 안 됨) 프레임 크기 기준
근사값(fx=fy=width, 프레임 중심을 principal point로)을 쓴다 - object_detection_node
초기화는 intrinsics "수신 여부"만 기다리므로 이 정도로 충분하고, 이 테스트의
목적도 3D 좌표 정밀도가 아니라 detection/mask 오버레이가 HMI까지 오는지
확인하는 것이다.
"""
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

CAMERA_DEVICE = 0
PUBLISH_FPS = 15.0


class WebcamCameraProbeNode(Node):
    def __init__(self):
        super().__init__('webcam_camera_probe_node')
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(CAMERA_DEVICE)
        if not self.cap.isOpened():
            raise RuntimeError(f"웹캠(장치 {CAMERA_DEVICE})을 열 수 없습니다.")

        self.color_pub = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera/color/camera_info', 10)
        self._camera_info = None

        self.create_timer(1.0 / PUBLISH_FPS, self._on_frame_timer)
        self.get_logger().info(
            f"웹캠(장치 {CAMERA_DEVICE}) 스트리밍 시작 -> /camera/camera/color/image_raw"
        )

    def _build_camera_info(self, width, height):
        info = CameraInfo()
        info.width = width
        info.height = height
        fx = fy = float(width)
        ppx, ppy = width / 2.0, height / 2.0
        info.k = [fx, 0.0, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0]
        return info

    def _on_frame_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("웹캠 프레임을 읽지 못했습니다.")
            return
        stamp = self.get_clock().now().to_msg()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_color_optical_frame'
        self.color_pub.publish(msg)

        if self._camera_info is None:
            h, w = frame.shape[:2]
            self._camera_info = self._build_camera_info(w, h)
        self._camera_info.header.stamp = stamp
        self.info_pub.publish(self._camera_info)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamCameraProbeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

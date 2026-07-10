"""디버깅용: YOLO 추론 결과(바운딩박스+라벨+confidence)를 실시간으로 화면에 띄워주는 임시 노드.

hand_detected 오탐(그리퍼/물체가 손으로 오인식되는지) 등을 눈으로 확인하기 위한 용도.
object_detection_node와 별개로 단독 실행 가능 — 카메라 토픽은 구독만 하므로(발행 안 함)
이미 실행 중인 object_detection_node와 동시에 띄워도 안전하다.
"""
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from object_detection.yolo import YoloModel


class YoloProbeNode(Node):
    def __init__(self):
        super().__init__('yolo_probe_node')
        self.bridge = CvBridge()
        self.yolo = YoloModel()
        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.get_logger().info("컬러 프레임 대기 중... (창에 포커스 두고 'q'로 종료)")

    def color_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.yolo._model_lock:
            results = self.yolo.model([frame], verbose=False)
        # ultralytics Results.plot()이 박스/라벨/confidence를 프레임에 직접 그려서 반환한다.
        annotated = results[0].plot()
        cv2.imshow("YOLO probe (q: quit)", annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = YoloProbeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

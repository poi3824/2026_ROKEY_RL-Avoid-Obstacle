# RealSense 컬러 카메라(/camera/camera/color/image_raw)를 구독해서 MJPEG로
# 자체 HTTP 서버(기본 8766 포트)에 스트리밍하는 작은 rclpy 노드.
#
# voice_bridge와 같은 원칙 - hmi_interface의 Flask 앱(app.py)은 ROS를 직접
# 붙잡지 않는다. 이 노드가 카메라 드라이버(realsense2_camera, 외부 실행)가
# 이미 발행 중인 토픽을 직접 구독해서 자체 포트로 서빙하고, 브라우저는
# <img src="http://<host>:8766/stream"> 하나로 바로 재생한다(MJPEG는 별도
# JS/websocket 없이 <img> 태그가 그대로 재생 가능) - object_detection_node는
# 전혀 안 건드린다.
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

CAMERA_TOPIC = "/camera/camera/color/image_raw"
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8766
JPEG_QUALITY = 70  # 0~100. 화질보다 지연시간/대역폭을 우선한다(모니터링 용도라 충분).
STREAM_FPS_CAP = 20  # 프레임 재인코딩 부하를 줄이기 위한 상한 - 카메라 실제 fps와 무관.


class VisionBridge(Node):
    def __init__(self):
        super().__init__("hmi_vision_bridge")
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._last_frame_ts = 0.0

        self.create_subscription(Image, CAMERA_TOPIC, self._on_image, 1)
        self.get_logger().info(f"'{CAMERA_TOPIC}' 구독 시작")

    def _on_image(self, msg):
        # 2026-07-10: 카메라가 실제로 30fps+ 로 발행해도 JPEG 인코딩+HTTP 전송을
        # 매 프레임 다 할 필요는 없다 - STREAM_FPS_CAP으로 인코딩 자체를 스킵해서
        # CPU 낭비를 줄인다(모니터링 용도라 화질/프레임레이트를 희생해도 된다).
        now = time.time()
        if now - self._last_frame_ts < 1.0 / STREAM_FPS_CAP:
            return
        self._last_frame_ts = now

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return
        with self._lock:
            self._latest_jpeg = buf.tobytes()

    def get_latest_jpeg(self):
        with self._lock:
            return self._latest_jpeg


def _make_handler(node):
    class MjpegHandler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # 매 프레임/요청마다 콘솔에 access log 찍히는 걸 막는다.

        def do_GET(self):
            if self.path not in ("/stream", "/"):
                self.send_response(404)
                self.end_headers()
                return

            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            try:
                while True:
                    jpeg = node.get_latest_jpeg()
                    if jpeg is None:
                        time.sleep(0.1)
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    time.sleep(1.0 / STREAM_FPS_CAP)
            except (BrokenPipeError, ConnectionResetError):
                pass  # 브라우저가 탭을 닫는 등 클라이언트가 그냥 연결을 끊은 정상적인 경우.

    return MjpegHandler


def main():
    rclpy.init()
    node = VisionBridge()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    server = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), _make_handler(node))
    node.get_logger().info(f"vision_bridge MJPEG 스트림: http://{HTTP_HOST}:{HTTP_PORT}/stream")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

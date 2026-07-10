# RealSense 컬러 카메라(/camera/camera/color/image_raw)를 구독해서 MJPEG로
# 자체 HTTP 서버(기본 8766 포트)에 스트리밍하는 작은 rclpy 노드.
#
# voice_bridge와 같은 원칙 - hmi_interface의 Flask 앱(app.py)은 ROS를 직접
# 붙잡지 않는다. 이 노드가 카메라 드라이버(realsense2_camera, 외부 실행)가
# 이미 발행 중인 토픽을 직접 구독해서 자체 포트로 서빙하고, 브라우저는
# <img src="http://<host>:8766/stream"> 하나로 바로 재생한다(MJPEG는 별도
# JS/websocket 없이 <img> 태그가 그대로 재생 가능) - object_detection_node는
# 전혀 안 건드린다.
#
# 2026-07-10: YOLO 추론 오버레이 토글 추가. object_detection_node는 프레임을
# 계속 추론해서 발행하는 게 없어서(서비스 호출 시점에만 잠깐 추론), 이 브릿지가
# object_detection.yolo.YoloModel을 별도로 하나 더 로드해 Vision 탭 전용으로
# 매 프레임 그려 넣는다 - object_detection_node의 모델 인스턴스/락과는 완전히
# 독립적이라 pick 로직과 자원을 두고 경합하지 않는다(대신 GPU/RAM을 좀 더 쓴다).
import json
import threading
import time
import urllib.parse
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
BOX_COLOR = (0, 220, 0)


class VisionBridge(Node):
    def __init__(self):
        super().__init__("hmi_vision_bridge")
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._last_frame_ts = 0.0

        # YOLO는 기본 꺼짐 - 켤 때(/yolo?enabled=1) 처음 한 번만 지연 로딩한다.
        # 모델 로딩(torch/ultralytics 초기화)이 몇 초 걸려서, 아예 안 켤 수도
        # 있는 기능을 노드 시작 시점에 미리 물고 있을 필요가 없다.
        self._yolo_enabled = False
        self._yolo_model = None
        self._yolo_load_lock = threading.Lock()

        self.create_subscription(Image, CAMERA_TOPIC, self._on_image, 1)
        self.get_logger().info(f"'{CAMERA_TOPIC}' 구독 시작")

    def set_yolo_enabled(self, enabled):
        if enabled and self._yolo_model is None:
            with self._yolo_load_lock:
                if self._yolo_model is None:  # double-checked locking - 로딩 도중 또 요청 와도 한 번만 로드
                    self.get_logger().info("YOLO 모델 로딩 중 (몇 초 걸릴 수 있음)...")
                    from object_detection.yolo import YoloModel
                    self._yolo_model = YoloModel()
                    self.get_logger().info("YOLO 모델 로딩 완료")
        self._yolo_enabled = enabled
        self.get_logger().info(f"Vision YOLO 오버레이: {'켜짐' if enabled else '꺼짐'}")

    def _on_image(self, msg):
        # 2026-07-10: 카메라가 실제로 30fps+ 로 발행해도 JPEG 인코딩+HTTP 전송을
        # 매 프레임 다 할 필요는 없다 - STREAM_FPS_CAP으로 인코딩 자체를 스킵해서
        # CPU 낭비를 줄인다(모니터링 용도라 화질/프레임레이트를 희생해도 된다).
        # YOLO가 켜져 있으면 추론 자체가 (CPU 기준) 프레임당 ~0.1초 걸려서
        # 이 콜백 처리 시간이 늘어나는 것만으로도 실질 fps가 자연스럽게 낮아진다.
        now = time.time()
        if now - self._last_frame_ts < 1.0 / STREAM_FPS_CAP:
            return
        self._last_frame_ts = now

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self._yolo_enabled and self._yolo_model is not None:
            frame = self._draw_yolo(frame)

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return
        with self._lock:
            self._latest_jpeg = buf.tobytes()

    def _draw_yolo(self, frame):
        try:
            with self._yolo_model._model_lock:
                results = self._yolo_model.model([frame], verbose=False)
        except Exception as e:
            self.get_logger().warn(f"YOLO 추론 실패(무시, 원본 프레임 표시): {e}")
            return frame

        annotated = frame.copy()
        for res in results:
            names = res.names
            boxes = res.boxes
            if boxes is None:
                continue
            for box, score, label in zip(
                boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.tolist()
            ):
                x1, y1, x2, y2 = map(int, box)
                name = names.get(int(label), str(int(label)))
                cv2.rectangle(annotated, (x1, y1), (x2, y2), BOX_COLOR, 2)
                label_text = f"{name} {score:.2f}"
                cv2.putText(
                    annotated, label_text, (x1, max(12, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, BOX_COLOR, 1, cv2.LINE_AA,
                )
        return annotated

    def get_latest_jpeg(self):
        with self._lock:
            return self._latest_jpeg


def _make_handler(node):
    class MjpegHandler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # 매 프레임/요청마다 콘솔에 access log 찍히는 걸 막는다.

        def _send_cors(self):
            # 이 서버는 Flask(app.py, 5050포트)와는 다른 포트에서 뜨므로, 브라우저의
            # fetch()로 /yolo를 토글하려면 CORS 허용 헤더가 필요하다(<img src>로
            # 스트림만 볼 때는 CORS가 필요 없지만, 토글 버튼의 fetch 호출엔 필요).
            self.send_header("Access-Control-Allow-Origin", "*")

        def do_OPTIONS(self):
            self.send_response(204)
            self._send_cors()
            self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
            self.end_headers()

        def do_GET(self):
            parsed = urllib.parse.urlparse(self.path)

            if parsed.path == "/yolo":
                qs = urllib.parse.parse_qs(parsed.query)
                enabled = qs.get("enabled", ["0"])[0].lower() in ("1", "true", "on")
                node.set_yolo_enabled(enabled)
                body = json.dumps({"yolo_enabled": enabled}).encode("utf-8")
                self.send_response(200)
                self._send_cors()
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if parsed.path not in ("/stream", "/"):
                self.send_response(404)
                self._send_cors()
                self.end_headers()
                return

            self.send_response(200)
            self._send_cors()
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

    # 2026-07-10 버그 수정: 처음엔 rclpy.spin()을 백그라운드 스레드로 돌리고
    # 메인 스레드에서 HTTP 서버(또는 signal 대기)를 했는데, Ctrl+C(SIGINT)를
    # 받으면 "terminate called without an active exception"/Aborted로 죽는
    # 걸 실기로 확인했다 - rclpy.init()이 내부적으로 등록해두는 SIGINT 처리가
    # 메인 스레드 기준으로 동작하는데, spin이 메인 스레드에 없으면 이게 깨지는
    #것으로 보인다(이 코드베이스의 motion_node 등 다른 모든 노드는 전부
    # rclpy.spin()을 메인 스레드에서 돈다 - 그쪽엔 이 문제가 없다). 그래서 반대로
    # HTTP 서버를 백그라운드 스레드로 돌리고 rclpy.spin()을 메인 스레드에 둔다.
    server = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), _make_handler(node))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(f"vision_bridge MJPEG 스트림: http://{HTTP_HOST}:{HTTP_PORT}/stream")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

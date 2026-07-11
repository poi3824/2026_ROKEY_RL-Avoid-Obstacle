# RealSense 컬러 카메라를 MJPEG로 스트리밍하는 rclpy 노드 - hmi_interface/
# vision_bridge.py의 카메라 구독/MJPEG 서버 로직을 이관하되(원본은 그대로
# 유지, 병행 운영), 오버레이 방식을 바꿨다.
#
# 원본은 Vision 탭 전용으로 YOLO 모델을 별도로 한 번 더 로드해서 매 프레임
# 직접 추론했다(GPU/RAM 이중 사용). 여기서는 대신 object_detection_node가
# Phase 4에서 새로 발행하기 시작한 hmi/vision_detections(String, JSON)를
# 구독해서 최신 감지 결과를 그대로 그려 넣는다 - 이 노드는 YOLO를 아예 로드하지
# 않는다. 감지 결과는 최대 0.3초(HAND_CHECK_INTERVAL_SEC) 정도 지연될 수
# 있지만, 모니터링 용도라 원본 vision_bridge.py의 기존 원칙("화질/프레임레이트
# 희생 가능")과 같은 트레이드오프다.
#
# MJPEG는 Socket.IO와 별개 채널로 유지한다(합의된 설계: 영상은 Phase 4까지
# 최소 이 정도, 최종적으로는 reverse proxy로 same-origin 통합 검토).
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

CAMERA_TOPIC = "/camera/camera/color/image_raw"
DETECTIONS_TOPIC = "hmi/vision_detections"
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8767  # hmi_interface/vision_bridge.py의 8766과 겹치지 않게(병행 운영 기간)
JPEG_QUALITY = 70
STREAM_FPS_CAP = 20
MASK_FILL_ALPHA = 0.35
OVERLAY_CONFIDENCE_THRESHOLD = 0.6  # vision_bridge.py의 YOLO_CONFIDENCE_THRESHOLD와 동일 기준
DETECTIONS_STALE_SEC = 2.0  # 이보다 오래된 detections는 화면에 그리지 않음(끊긴 걸 숨기지 않기 위함)

CLASS_COLORS = {
    "obj_A": (40, 40, 220),
    "obj_B": (220, 120, 30),
    "obj_C": (60, 180, 60),
    "hand": (0, 210, 255),
    "obstacle": (200, 40, 200),
}
DEFAULT_CLASS_COLOR = (170, 170, 170)


def _color_for(name):
    return CLASS_COLORS.get(name, DEFAULT_CLASS_COLOR)


class VisionStreamNode(Node):
    def __init__(self):
        super().__init__("hmi_vision_stream")
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._last_frame_ts = 0.0

        self._overlay_enabled = False
        self._latest_detections = []
        self._latest_detections_stamp = 0.0

        self.create_subscription(Image, CAMERA_TOPIC, self._on_image, 1)
        self.create_subscription(String, DETECTIONS_TOPIC, self._on_detections, 10)
        self.get_logger().info(f"'{CAMERA_TOPIC}', '{DETECTIONS_TOPIC}' 구독 시작")

    def set_overlay_enabled(self, enabled):
        self._overlay_enabled = enabled
        self.get_logger().info(f"Vision detection 오버레이: {'켜짐' if enabled else '꺼짐'}")

    def _on_detections(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self._lock:
            self._latest_detections = payload.get("detections", [])
            self._latest_detections_stamp = payload.get("stamp", time.time())

    def _on_image(self, msg):
        now = time.time()
        if now - self._last_frame_ts < 1.0 / STREAM_FPS_CAP:
            return
        self._last_frame_ts = now

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self._overlay_enabled:
            frame = self._draw_overlay(frame)

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return
        with self._lock:
            self._latest_jpeg = buf.tobytes()

    def _draw_overlay(self, frame):
        with self._lock:
            detections = list(self._latest_detections)
            stamp = self._latest_detections_stamp

        if time.time() - stamp > DETECTIONS_STALE_SEC:
            return frame  # 감지 결과가 너무 오래됨(object_detection_node 안 떠 있는 듯) - 원본만 표시

        annotated = frame.copy()
        for det in detections:
            if det.get("score", 0) < OVERLAY_CONFIDENCE_THRESHOLD:
                continue
            name = det.get("label", "?")
            color = _color_for(name)
            box = det.get("box")
            if not box or len(box) != 4:
                continue
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            label_text = f"{name} {det.get('score', 0):.2f}"
            cv2.putText(
                annotated, label_text, (x1, max(12, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA,
            )
        return annotated

    def get_latest_jpeg(self):
        with self._lock:
            return self._latest_jpeg


def _make_handler(node):
    class MjpegHandler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send_cors(self):
            self.send_header("Access-Control-Allow-Origin", "*")

        def do_OPTIONS(self):
            self.send_response(204)
            self._send_cors()
            self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
            self.end_headers()

        def do_GET(self):
            import urllib.parse
            parsed = urllib.parse.urlparse(self.path)

            if parsed.path == "/overlay":
                qs = urllib.parse.parse_qs(parsed.query)
                enabled = qs.get("enabled", ["0"])[0].lower() in ("1", "true", "on")
                node.set_overlay_enabled(enabled)
                body = json.dumps({"overlay_enabled": enabled}).encode("utf-8")
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
                pass

    return MjpegHandler


def run_server(node):
    """main.py의 rclpy.spin()과 별도 스레드에서 MJPEG 서버를 돌린다(vision_bridge.py와
    동일 원칙 - rclpy.spin()이 메인 스레드에 있어야 SIGINT가 안 깨진다)."""
    server = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), _make_handler(node))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f"hmi_vision_stream MJPEG: http://{HTTP_HOST}:{HTTP_PORT}/stream")
    return server


def main():
    rclpy.init()
    node = VisionStreamNode()
    server = run_server(node)
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

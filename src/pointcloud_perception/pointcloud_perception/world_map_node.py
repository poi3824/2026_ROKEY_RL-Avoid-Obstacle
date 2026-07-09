# "World Map Update" 음성 명령으로 트리거되는 1회성 전체 스캔 노드.
#
# pointcloud_node.py(실시간 매 프레임 obstacle_state)와는 별개로, 넓은 영역을
# MoveLine으로 훑으면서 point cloud를 base_link 기준으로 merge한 뒤 DBSCAN으로
# 군집화해서 장애물 위치를 뽑아낸다. cobot_ws의 ~/cobot_scan/world_map_scan_capture_ranged.py
# (지그재그 스캔 경로 + TF 변환)에서 이미 검증된 로직을 그대로 포팅했다.
#
# 애초에 그리드의 각 코너에서 제자리 회전(A/B만 변경)으로 중심을 바라보게
# 만들려 했었는데, 카메라가 flange 원점에서 T_gripper2camera만큼 떨어져 있어서
# 제자리 회전만으로는 원하는 옆면 뷰가 안 나온다는 걸 실제 로봇으로 확인함
# (2026-07-06). 대신 스캔 시작/끝 지점에서만 중심 반대쪽으로 더 물러나면서
# 살짝 내려간 위치로 이동해, 손으로 직접 찾은 자세로 옆면을 찍고 다시 원래
# 그리드로 돌아오는 방식으로 변경했다.
#
# service/topic 콜백(handle_update) 안에서 MoveLine 서비스 호출과 point cloud
# 대기를 위해 spin_until_future_complete/spin_once를 반복 호출해야 하는데,
# 이걸 self(서비스를 제공 중인 바로 그 노드)에 대고 호출하면 재귀적으로 꼬일 수
# 있다. detection.py(ObjectDetectionNode가 서비스 콜백 안에서 자기 자신이 아닌
# 별도 ImgNode를 spin하는 패턴)를 그대로 따라서, MoveLine/point cloud/TF는
# 별도의 ScanWorker 노드에서 처리한다.
#
# 스캔 경로 생성, TF/ROI 적용, ground z 보정, ICP 잔차보정, 클러스터링/저장 같은
# 순수 알고리즘은 world_map_algo.py로 분리되어 있다 - rclpy 등 ROS 의존성이 없어야
# offline_icp_experiment.py가 ROS 워크스페이스 없이 저장된 스캔으로 튜닝할 수 있다.
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from dsr_msgs2.srv import MoveLine

from obstacle_avoidance_msgs.msg import WorldMapObstacle, WorldMapUpdate

from pointcloud_perception.world_map_algo import (
    ENABLE_ICP_MAPPING,
    VOXEL_SIZE_M,
    align_ground_z_per_pose,
    apply_transform,
    build_map_with_icp,
    cluster_points,
    compute_ground_quality,
    crop_roi,
    generate_scan_poses,
    is_flat_pose,
    save_debug_visualizations,
    save_record,
    transform_stamped_to_matrix,
    voxel_downsample,
)

try:
    import open3d as o3d  # noqa: F401  (world_map_algo가 이미 가용성 체크를 하지만, 여긴 직접 안 씀)
    OPEN3D_AVAILABLE = True
except Exception:
    OPEN3D_AVAILABLE = False


# =========================
# 스캔 설정 (cobot_scan/world_map_scan_capture_ranged.py 와 동일한 캘리브레이션)
# =========================

POINTCLOUD_TOPIC = "/camera/camera/depth/color/points"
MOVE_LINE_SERVICE = "/dsr01/motion/move_line"
TARGET_FRAME = "base_link"

VEL = [60.0, 30.0]
ACC = [60.0, 30.0]

# Doosan MoveLine의 sync_type 의미는 설치된 dsr_msgs2/컨트롤러 버전에 따라 확인 필요.
# 기존 코드와 동일하게 0을 기본값으로 둔다. 만약 서비스가 "명령 접수 즉시 반환"이면
# 로봇이 아직 움직이는 중에 cloud를 찍을 수 있으므로 settle/discard를 길게 둔다.
MOVE_SYNC_TYPE = 0

# flat scan에서도 첫 프레임이 흔들릴 수 있어서 기존 0.8s보다 조금 여유를 둔다.
SETTLE_SEC = 1.2

# 사이드뷰/tilt 포즈는 그리퍼-카메라 offset 때문에 TCP 자세 변화가 크고,
# robot TF와 RealSense cloud가 안정되기까지 시간이 더 필요하다.
SIDE_VIEW_SETTLE_SEC = 3.0

# side -> flat으로 복귀하는 바로 다음 포즈는 특히 stale frame이 잡히기 쉬워서
# 별도의 settle 시간을 둔다. pose14 -> pose15가 거의 같은 cloud로 저장되는 현상 방지.
AFTER_SIDE_RETURN_SETTLE_SEC = 3.0


# settle 직후 첫 몇 장은 motion/auto-exposure/TF lag가 남을 수 있어 버린다.
DISCARD_FRAMES_AFTER_SETTLE = 5

FRAMES_PER_POSE = 1

# point cloud / TF 동기화 대기 시간.
# 중요한 점: point cloud stamp를 ROS 현재 시각과 직접 비교하지 않는다.
# RealSense header stamp가 ROS clock보다 약간 늦거나 앞설 수 있기 때문이다.
CLOUD_WAIT_TIMEOUT_SEC = 10.0
TF_WAIT_TIMEOUT_SEC = 5.0


# =========================
# TF / point cloud 유틸리티 (ROS 메시지 의존)
# =========================

def msg_stamp_to_sec(stamp_msg):
    return stamp_msg.sec + stamp_msg.nanosec * 1e-9


def pointcloud_msg_to_xyz(msg):
    points = [
        [p[0], p[1], p[2]]
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ]
    if not points:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(points, dtype=np.float64)


def build_world_map_update_msg(clusters, scan_dir, stamp):
    msg = WorldMapUpdate()
    msg.header.stamp = stamp
    msg.header.frame_id = TARGET_FRAME
    msg.scan_dir = scan_dir

    obstacles = []
    for c in clusters:
        obs = WorldMapObstacle()
        obs.id = int(c["id"])
        obs.centroid.x, obs.centroid.y, obs.centroid.z = (float(v) for v in c["centroid"])
        obs.radius = float(c["radius"])
        obs.height = float(c["height"])
        obs.z_min = float(c["z_min"])
        obs.z_max = float(c["z_max"])
        obs.safety_radius = float(c["safety_radius"])
        obs.safety_height = float(c["safety_height"])
        obs.shape_type = c["shape_type"]
        obs.num_points = int(c["num_points"])
        obs.confidence = float(c["confidence"])
        obstacles.append(obs)
    msg.obstacles = obstacles

    return msg


# =========================
# 스캔 워커 (MoveLine + point cloud + TF, 별도 노드)
# =========================

class ScanWorker(Node):
    def __init__(self):
        super().__init__("world_map_scan_worker")

        self.latest_cloud_msg = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # RealSense PointCloud2는 센서 스트림이므로 최신 프레임만 쓰는 쪽이 안전하다.
        # queue가 깊으면 이동/settle 중 쌓인 오래된 프레임을 나중에 잡을 수 있다.
        self.cloud_sub = self.create_subscription(
            PointCloud2, POINTCLOUD_TOPIC, self.cloud_callback, qos_profile_sensor_data
        )
        self.move_line_client = self.create_client(MoveLine, MOVE_LINE_SERVICE)

    def cloud_callback(self, msg):
        self.latest_cloud_msg = msg

    def wait_for_move_line_service(self, timeout_sec=5.0):
        return self.move_line_client.wait_for_service(timeout_sec=timeout_sec)

    def move_line(self, pose):
        req = MoveLine.Request()
        req.pos = pose
        req.vel = [float(VEL[0]), float(VEL[1])]
        req.acc = [float(ACC[0]), float(ACC[1])]
        req.time = 0.0
        req.radius = 0.0

        if hasattr(req, "ref"):
            req.ref = 0
        if hasattr(req, "mode"):
            req.mode = 0
        if hasattr(req, "blend_type"):
            req.blend_type = 0
        if hasattr(req, "sync_type"):
            req.sync_type = int(MOVE_SYNC_TYPE)

        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            return False

        result = future.result()
        if hasattr(result, "success"):
            return bool(result.success)
        return True

    def spin_sleep(self, seconds):
        """time.sleep 대신 worker node를 계속 spin하면서 대기한다.

        TF listener와 point cloud subscriber는 spin이 돌아야 최신 데이터가 들어온다.
        time.sleep만 쓰면 settle 시간 동안 callback 처리가 멈추고, 이후 오래된 큐를
        처리하면서 정착 전 프레임/TF를 잡을 수 있다.
        """
        end_time = time.time() + float(seconds)
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def latest_cloud_stamp_sec(self):
        """현재 worker가 마지막으로 받은 point cloud stamp를 초 단위로 반환."""
        if self.latest_cloud_msg is None:
            return None
        return msg_stamp_to_sec(self.latest_cloud_msg.header.stamp)

    def wait_for_cloud_newer_than(self, min_stamp_sec=None, timeout_sec=CLOUD_WAIT_TIMEOUT_SEC):
        """특정 point cloud stamp보다 새로운 프레임이 들어올 때까지 기다린다.

        주의:
        - ROS 현재 시각(self.get_clock().now())과 cloud header stamp를 비교하지 않는다.
        - RealSense/robot TF가 같은 system clock을 쓰더라도 publish latency 때문에
          cloud stamp가 현재 시각보다 늦게 보일 수 있다.
        - 그래서 "settle 종료 시각 이후"가 아니라 "마지막으로 처리한 cloud stamp 이후"
          프레임을 잡는 방식으로 바꾼다.
        """
        t0 = time.time()
        last_seen_stamp = None

        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

            msg = self.latest_cloud_msg
            if msg is None:
                continue

            stamp_sec = msg_stamp_to_sec(msg.header.stamp)
            last_seen_stamp = stamp_sec

            if min_stamp_sec is None or stamp_sec > min_stamp_sec + 1e-9:
                return msg

        if min_stamp_sec is None:
            raise TimeoutError(
                f"새 point cloud를 받지 못했습니다. "
                f"last_seen_stamp={last_seen_stamp}"
            )

        raise TimeoutError(
            f"기준 stamp 이후의 새 point cloud를 받지 못했습니다. "
            f"min_stamp={min_stamp_sec:.6f}, last_seen_stamp={last_seen_stamp}"
        )

    def discard_cloud_frames(self, count):
        """settle 직후의 오래된/흔들린 프레임을 버리고 최신 stamp까지 진행한다."""
        min_stamp = self.latest_cloud_stamp_sec()
        discarded = 0

        for _ in range(int(count)):
            msg = self.wait_for_cloud_newer_than(
                min_stamp,
                timeout_sec=CLOUD_WAIT_TIMEOUT_SEC
            )
            min_stamp = msg_stamp_to_sec(msg.header.stamp)
            discarded += 1

        return min_stamp, discarded

    def lookup_transform_strict_spin(self, target_frame, source_frame, stamp, timeout_sec=TF_WAIT_TIMEOUT_SEC):
        """정확한 stamp의 TF가 들어올 때까지 worker node를 spin하면서 기다린다.

        lookup_transform(..., timeout=...)만 쓰면 같은 노드의 TF callback이 처리되지 않는
        구간이 생길 수 있다. 특히 RealSense cloud stamp가 최신 TF보다 30~80ms 앞서면
        "extrapolation into the future"가 발생한다. 여기서는 최신 TF fallback은 하지 않고,
        요청 stamp를 커버하는 TF가 buffer에 들어올 때까지 spin_once로 기다린다.
        """
        start = time.time()
        last_error = None

        while time.time() - start < timeout_sec:
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    stamp,
                    timeout=Duration(seconds=0.0)
                )
            except Exception as e:
                last_error = e
                rclpy.spin_once(self, timeout_sec=0.02)

        requested_sec = stamp.nanoseconds * 1e-9
        raise RuntimeError(
            f"TF lookup failed after {timeout_sec:.1f}s: "
            f"{source_frame} -> {target_frame} at {requested_sec:.6f}. "
            f"last_error={last_error}"
        )

    def transform_cloud_to_base(self, msg):
        """PointCloud2를 msg.header.stamp 시점의 TF로 base_link 기준 변환한다.

        중요:
        - Time() 최신 TF fallback은 쓰지 않는다.
        - 대신 point cloud stamp를 커버하는 TF가 들어올 때까지 spin하면서 기다린다.
        - 그래도 TF가 안 들어오면 scan fail로 드러내서 잘못된 map 누적을 막는다.
        """
        source_frame = msg.header.frame_id
        stamp = Time.from_msg(msg.header.stamp)

        tf = self.lookup_transform_strict_spin(
            TARGET_FRAME,
            source_frame,
            stamp,
            timeout_sec=TF_WAIT_TIMEOUT_SEC
        )

        T = transform_stamped_to_matrix(tf)
        xyz_camera = pointcloud_msg_to_xyz(msg)
        return apply_transform(xyz_camera, T)

    def capture_cloud_at_pose(self):
        """현재 포즈에서 point cloud를 수집한다.

        settle 직후에는 아직 motion/TF/RealSense exposure가 완전히 안정되지 않은 프레임이
        들어올 수 있으므로, 먼저 DISCARD_FRAMES_AFTER_SETTLE장 버리고 그 다음 프레임을 쓴다.
        """
        collected = []

        min_stamp, discarded = self.discard_cloud_frames(DISCARD_FRAMES_AFTER_SETTLE)
        self.get_logger().debug(
            f"discarded {discarded} frames after settle, stamp={min_stamp}"
        )

        for frame_idx in range(FRAMES_PER_POSE):
            msg = self.wait_for_cloud_newer_than(
                min_stamp,
                timeout_sec=CLOUD_WAIT_TIMEOUT_SEC
            )
            min_stamp = msg_stamp_to_sec(msg.header.stamp)

            xyz_base = self.transform_cloud_to_base(msg)
            xyz_roi = crop_roi(xyz_base)
            collected.append(xyz_roi)

            self.get_logger().debug(
                f"captured frame {frame_idx}: stamp={min_stamp:.6f}, "
                f"roi_points={xyz_roi.shape[0]}"
            )

        if not collected:
            return np.empty((0, 3), dtype=np.float64)
        return np.vstack(collected)

    def run_scan(self):
        """스캔 경로를 훑으면서 point cloud를 base_link 기준으로 merge해서 반환한다."""
        if not self.wait_for_move_line_service(timeout_sec=5.0):
            raise RuntimeError(f"MoveLine 서비스({MOVE_LINE_SERVICE})를 찾을 수 없습니다.")

        poses = generate_scan_poses()
        per_pose_points = []
        prev_pose = None

        for i, pose in enumerate(poses):
            pose_type = "flat" if is_flat_pose(pose) else "side"

            self.get_logger().info(
                f"moving pose {i:03d}/{len(poses)-1:03d}: "
                f"type={pose_type}, "
                f"pose={['%.2f' % v for v in pose]}"
            )

            ok = self.move_line(pose)
            if not ok:
                raise RuntimeError(f"MoveLine 실패 (pose {i}: {pose})")

            settle_sec = SETTLE_SEC if is_flat_pose(pose) else SIDE_VIEW_SETTLE_SEC

            # side view 직후 flat 복귀 pose는 stale side frame이 잡히는 경우가 있어 추가 대기.
            if prev_pose is not None and (not is_flat_pose(prev_pose)) and is_flat_pose(pose):
                settle_sec = max(settle_sec, AFTER_SIDE_RETURN_SETTLE_SEC)

            self.get_logger().info(
                f"pose {i:03d}/{len(poses)-1:03d} reached candidate: "
                f"type={pose_type}, settle={settle_sec:.2f}s"
            )

            # time.sleep 대신 spin_sleep을 사용해서 settle 중에도 TF/cloud callback을 처리한다.
            # 이 시간 동안 latest_cloud_msg도 계속 최신 프레임으로 갱신된다.
            self.spin_sleep(settle_sec)

            before_capture_stamp = self.latest_cloud_stamp_sec()
            if before_capture_stamp is None:
                self.get_logger().warn(
                    f"pose {i:03d}: settle 동안 point cloud를 아직 한 번도 받지 못함"
                )
            else:
                self.get_logger().info(
                    f"pose {i:03d}: latest cloud before capture stamp="
                    f"{before_capture_stamp:.6f}"
                )

            pose_points = self.capture_cloud_at_pose()
            self.get_logger().info(
                f"pose {i:03d} captured {pose_points.shape[0]} ROI points"
            )
            per_pose_points.append(pose_points)
            prev_pose = pose

        ground_quality = compute_ground_quality(per_pose_points, poses)
        corrected_per_pose_points, ground_z_alignment = align_ground_z_per_pose(
            per_pose_points, poses, ground_quality
        )

        if ground_z_alignment.get("enabled"):
            ref_z = ground_z_alignment.get("reference_ground_z")
            self.get_logger().info(
                f"ground z alignment enabled: reference z={ref_z:.4f} m"
            )
            for info in ground_z_alignment.get("per_pose", []):
                if info.get("applied") and abs(info.get("z_correction_m", 0.0)) > 0.003:
                    self.get_logger().info(
                        f"pose {info['pose_index']:03d}: "
                        f"ground_z={info['ground_z_mode']:.4f}, "
                        f"z_correction={info['z_correction_m']:+.4f} m"
                    )
        else:
            self.get_logger().warn(
                f"ground z alignment disabled: {ground_z_alignment.get('reason')}"
            )

        if ENABLE_ICP_MAPPING:
            merged_points, icp_mapping_report = build_map_with_icp(
                corrected_per_pose_points, poses, ground_quality
            )
        else:
            merged_list = [p for p in corrected_per_pose_points if len(p) > 0]
            merged_points = (
                voxel_downsample(np.vstack(merged_list), VOXEL_SIZE_M)
                if merged_list else np.empty((0, 3), dtype=np.float64)
            )
            icp_mapping_report = None

        return (
            merged_points,
            poses,
            corrected_per_pose_points,
            ground_z_alignment,
            icp_mapping_report,
        )


# =========================
# 서비스 노드
# =========================

class WorldMapNode(Node):
    def __init__(self):
        super().__init__("world_map_node")

        self.worker = ScanWorker()

        # 월드맵은 "상태"에 가깝다 - RL 노드가 스캔 끝난 뒤 늦게 구독을 시작해도
        # 마지막 결과를 즉시 받도록 transient_local로 발행한다
        # (safety_monitor_node의 /safety/state와 동일한 패턴).
        world_map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.obstacle_pub = self.create_publisher(WorldMapUpdate, "/world_map/obstacles", world_map_qos)
        self.update_srv = self.create_service(Trigger, "update_world_map", self.handle_update)

        self.get_logger().info("world_map_node ready. waiting for /update_world_map calls...")

    def handle_update(self, request, response):
        self.get_logger().info("World map update requested.")

        try:
            merged_points, poses, per_pose_points, ground_z_alignment, icp_mapping_report = (
                self.worker.run_scan()
            )
        except Exception as e:
            self.get_logger().error(f"scan failed: {e}")
            response.success = False
            response.message = f"scan failed: {e}"
            return response

        if merged_points.shape[0] == 0:
            self.get_logger().warn("No points captured during scan.")
            response.success = False
            response.message = "no points captured"
            return response

        try:
            ground_z = ground_z_alignment.get("reference_ground_z")
            clusters = cluster_points(merged_points, ground_z=ground_z)
        except Exception as e:
            self.get_logger().error(f"clustering failed: {e}")
            response.success = False
            response.message = f"clustering failed: {e}"
            return response

        scan_dir = save_record(
            merged_points, clusters, poses, per_pose_points, ground_z_alignment, icp_mapping_report
        )

        # top-view 디버그 PNG + Hough 교차검증 - 실패해도 장애물 결과 응답 자체는
        # 살려야 하므로 별도로 try/except. 스캔(수 분) 대비 1~2초 추가되는 정도라
        # 무시할 만하다 (world_map_algo.save_debug_visualizations 참고).
        try:
            debug_report = save_debug_visualizations(scan_dir, merged_points, ground_z, clusters)
            for err in debug_report["errors"]:
                self.get_logger().warn(f"debug visualization: {err}")
        except Exception as e:
            self.get_logger().warn(f"debug visualization failed: {e}")

        msg = build_world_map_update_msg(clusters, scan_dir, self.get_clock().now().to_msg())
        self.obstacle_pub.publish(msg)

        self.get_logger().info(
            f"World map updated: {len(clusters)} obstacles, saved to {scan_dir}"
        )
        response.success = True
        response.message = f"{len(clusters)} obstacles detected, saved to {scan_dir}"
        return response

    def destroy_node(self):
        self.worker.destroy_node()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WorldMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

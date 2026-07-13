#!/usr/bin/env python3
"""저장된 world_map_update_* 스캔 폴더를 읽어, 포즈(스텝)별 point cloud가
그 시점 TF로 base_link에 변환된 채로 순차 누적되는 과정을 Open3D 창에서
애니메이션으로 재생한다. 데모/발표 화면 녹화용. 로봇/ROS 없이 저장된
결과 파일(scan_pose_NNN_base_roi.ply + scan_pose_NNN_meta.json)만으로 동작한다.

각 scan_pose_NNN_base_roi.ply는 world_map_node.py의
ScanWorker.transform_cloud_to_base()에서 이미 그 포즈 시점의
camera_depth_optical_frame -> base_link TF가 적용된 결과이므로,
이 스크립트는 그 파일들을 pose 순서대로 이어붙이기만 해도 실제 파이프라인의
"TF 기반 누적" 과정과 동일한 결과를 보여준다 (ICP 잔차보정/ground-z 보정 없는
raw TF-only 누적 기준).

사용법:
    python3 animate_worldmap_accumulation.py <scan_dir> \
        [--interval 0.6] [--voxel-preview 0.005] [--record-dir DIR] \
        [--fps 2] [--start-paused] [--final-voxel 0.005]

scan_dir 예:
    ~/RL-Avoid-Obstacle/data/world_maps/world_map_update_20260713_102455

키 조작:
    space       재생 / 일시정지 토글
    -> (또는 .) (일시정지 상태에서) 한 스텝 앞으로
    <- (또는 ,) (일시정지 상태에서) 한 스텝 뒤로 (누적을 그 이전 상태로 재구성)
    r           처음부터 다시 재생
    = / -       재생 속도 조절 (스텝 간 대기시간 증감)
    c           마지막 스텝 도달 후: 감지된 장애물(cluster) 표시 토글
    ESC         종료
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import time

import numpy as np
import open3d as o3d

HIGHLIGHT_COLOR = np.array([1.0, 0.30, 0.05])
PATH_COLOR = [0.85, 0.15, 0.75]
MARKER_COLOR = [0.85, 0.15, 0.75]
CLUSTER_COLOR = [1.0, 0.0, 0.0]

# world_map_algo.VOXEL_SIZE_M 과 동일 (실제 파이프라인의 최종 merge voxel 크기).
DEFAULT_FINAL_VOXEL_M = 0.005


def load_scan(scan_dir):
    """scan_pose_*_meta.json + scan_pose_*_base_roi.ply를 pose_index 순서로 복원.

    meta.json은 point가 0개인 pose에도 항상 저장되지만 ply는 point가 있을 때만
    저장되므로, meta 기준으로 순회하고 ply가 없으면 빈 배열로 채운다.
    (offline_icp_experiment.py의 load_scan()과 동일한 규칙)
    """
    meta_files = sorted(glob.glob(os.path.join(scan_dir, "scan_pose_*_meta.json")))
    if not meta_files:
        raise FileNotFoundError(f"scan_pose_*_meta.json 파일을 찾을 수 없습니다: {scan_dir}")

    poses_mm = []
    per_pose_points = []
    for meta_path in meta_files:
        idx = int(os.path.basename(meta_path).split("_")[2])
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
        poses_mm.append(meta["pose_xyz_abc"])

        ply_path = os.path.join(scan_dir, f"scan_pose_{idx:03d}_base_roi.ply")
        if os.path.exists(ply_path):
            pcd = o3d.io.read_point_cloud(ply_path)
            points = np.asarray(pcd.points, dtype=np.float64)
        else:
            points = np.empty((0, 3), dtype=np.float64)
        per_pose_points.append(points)

    return poses_mm, per_pose_points


def load_clusters(scan_dir):
    summary_path = os.path.join(scan_dir, "world_map_summary.json")
    if not os.path.exists(summary_path):
        return []
    with open(summary_path, "r", encoding="utf-8") as f:
        summary = json.load(f)
    return summary.get("clusters", [])


def height_to_colors(z, z_lo=-0.05, z_hi=0.35):
    """높이(z, m)를 파랑(낮음) -> 청록 -> 노랑(높음) 그라데이션으로 매핑."""
    stops = np.array(
        [
            [0.10, 0.25, 0.85],
            [0.10, 0.75, 0.65],
            [0.95, 0.85, 0.15],
        ]
    )
    t = np.clip((z - z_lo) / max(z_hi - z_lo, 1e-6), 0.0, 1.0)
    idx = t * (len(stops) - 1)
    lo_i = np.floor(idx).astype(int)
    hi_i = np.clip(lo_i + 1, 0, len(stops) - 1)
    frac = (idx - lo_i)[:, None]
    return stops[lo_i] * (1 - frac) + stops[hi_i] * frac


def make_cluster_geometries(clusters):
    geoms = []
    for c in clusters:
        cx, cy, cz_min = c["centroid"][0], c["centroid"][1], c["z_min"]
        radius = max(c["radius"], 0.01)
        height = max(c["height"], 0.01)
        cyl = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height, resolution=24)
        cyl.translate([cx, cy, cz_min + height / 2.0])
        cyl.paint_uniform_color(CLUSTER_COLOR)
        wire = o3d.geometry.LineSet.create_from_triangle_mesh(cyl)
        wire.paint_uniform_color(CLUSTER_COLOR)
        geoms.append(wire)
    return geoms


class AccumulationPlayer:
    def __init__(self, poses_mm, per_pose_points, clusters, args):
        self.poses_mm = poses_mm
        self.per_pose_points = per_pose_points
        self.clusters = clusters
        self.n_steps = len(poses_mm)
        self.interval = args.interval
        self.voxel_preview = args.voxel_preview
        self.record_dir = args.record_dir
        self.final_voxel = args.final_voxel

        self.step_index = -1  # 아직 아무 스텝도 표시 안 함
        self.playing = not args.start_paused
        self.settled_batches = []  # [(points, colors_settled), ...] 표시된 스텝만
        self.path_positions = []
        self.clusters_visible = False
        self.cluster_geoms = []
        self.finished_announced = False
        self._frame_counter = 0
        self._last_tick = time.time()

        if self.record_dir:
            os.makedirs(self.record_dir, exist_ok=True)

        self.pcd = o3d.geometry.PointCloud()
        self.path_line = o3d.geometry.LineSet()
        self.marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.018)
        self.marker.paint_uniform_color(MARKER_COLOR)
        self.marker_center = np.zeros(3)

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="World Map - TF 기반 point cloud 누적", width=1280, height=800)
        self.vis.add_geometry(self.pcd)
        self.vis.add_geometry(self.path_line)
        self.vis.add_geometry(self.marker)

        opt = self.vis.get_render_option()
        opt.background_color = np.array([0.05, 0.05, 0.07])
        opt.point_size = 2.5

        self._fit_camera_to_full_scan()
        self._register_keys()
        self.vis.register_animation_callback(self._on_frame)

        print(f"총 {self.n_steps}개 pose 로드 완료. space=재생/정지, ->/<-=스텝 이동, r=재시작, ESC=종료")

    def _fit_camera_to_full_scan(self):
        """빈 point cloud로 add_geometry한 직후에는 update_geometry가 카메라를
        재조정하지 않아 이후 누적되는 점들이 화면 밖에 걸릴 수 있다. 전체
        스캔 범위로 한 번 채웠다가 reset_view_point로 카메라를 맞춘 뒤 다시
        비워서, 애니메이션 내내 이 카메라 시점을 그대로 쓴다."""
        non_empty = [p for p in self.per_pose_points if p.shape[0] > 0]
        if not non_empty:
            return
        all_points = np.vstack(non_empty)
        self.pcd.points = o3d.utility.Vector3dVector(all_points)
        self.vis.update_geometry(self.pcd)
        self.vis.reset_view_point(True)
        self.pcd.points = o3d.utility.Vector3dVector(np.empty((0, 3)))
        self.pcd.colors = o3d.utility.Vector3dVector(np.empty((0, 3)))
        self.vis.update_geometry(self.pcd)

    # ---- 키 콜백 ----
    def _register_keys(self):
        self.vis.register_key_callback(32, self._cb(self.toggle_play))       # space
        self.vis.register_key_callback(262, self._cb(self.step_forward))    # right
        self.vis.register_key_callback(46, self._cb(self.step_forward))     # '.'
        self.vis.register_key_callback(263, self._cb(self.step_backward))   # left
        self.vis.register_key_callback(44, self._cb(self.step_backward))    # ','
        self.vis.register_key_callback(ord("R"), self._cb(self.restart))
        self.vis.register_key_callback(ord("C"), self._cb(self.toggle_clusters))
        self.vis.register_key_callback(61, self._cb(self.speed_up))         # '='
        self.vis.register_key_callback(45, self._cb(self.slow_down))        # '-'

    def _cb(self, fn):
        def _handler(vis):
            fn()
            return False

        return _handler

    def toggle_play(self):
        self.playing = not self.playing
        print("재생" if self.playing else "일시정지")

    def speed_up(self):
        self.interval = max(0.05, self.interval - 0.1)
        print(f"재생 간격: {self.interval:.2f}s")

    def slow_down(self):
        self.interval = min(3.0, self.interval + 0.1)
        print(f"재생 간격: {self.interval:.2f}s")

    def restart(self):
        print("처음부터 다시 재생")
        self.step_index = -1
        self.settled_batches = []
        self.path_positions = []
        self.finished_announced = False
        self.clusters_visible = False
        self._remove_cluster_geoms()
        self._refresh_pcd(highlight=None)
        self.playing = True

    def toggle_clusters(self):
        if self.step_index < self.n_steps - 1:
            print("아직 누적이 끝나지 않았습니다 (마지막 스텝 도달 후 사용 가능)")
            return
        self.clusters_visible = not self.clusters_visible
        if self.clusters_visible:
            self.cluster_geoms = make_cluster_geometries(self.clusters)
            for g in self.cluster_geoms:
                self.vis.add_geometry(g, reset_bounding_box=False)
            print(f"장애물 클러스터 {len(self.cluster_geoms)}개 표시")
        else:
            self._remove_cluster_geoms()
            print("장애물 클러스터 숨김")

    def _remove_cluster_geoms(self):
        for g in self.cluster_geoms:
            self.vis.remove_geometry(g, reset_bounding_box=False)
        self.cluster_geoms = []

    def step_forward(self):
        if self.step_index >= self.n_steps - 1:
            return
        self._advance()

    def step_backward(self):
        if self.step_index <= 0:
            self.restart()
            self.playing = False
            return
        self.step_index -= 1
        self.settled_batches = self.settled_batches[:-1]
        self.path_positions = self.path_positions[:-1]
        self._refresh_pcd(highlight=None)
        self._update_path_and_marker()
        self._capture_frame()

    # ---- 애니메이션 루프 ----
    def _on_frame(self, vis):
        if not self.playing:
            return False
        if self.step_index >= self.n_steps - 1:
            if not self.finished_announced:
                self.finished_announced = True
                self.playing = False
                self._announce_finish()
            return False
        if time.time() - self._last_tick >= self.interval:
            self._advance()
            self._last_tick = time.time()
        return False

    def _advance(self):
        self.step_index += 1
        i = self.step_index
        points = self.per_pose_points[i]
        pose_xyz_m = np.asarray(self.poses_mm[i][:3], dtype=np.float64) / 1000.0
        self.path_positions.append(pose_xyz_m)

        if points.shape[0] > 0:
            if self.voxel_preview and self.voxel_preview > 0:
                tmp = o3d.geometry.PointCloud()
                tmp.points = o3d.utility.Vector3dVector(points)
                tmp = tmp.voxel_down_sample(self.voxel_preview)
                points = np.asarray(tmp.points, dtype=np.float64)
            colors_settled = height_to_colors(points[:, 2])
            self.settled_batches.append((points, colors_settled))
            print(
                f"[step {i + 1}/{self.n_steps}] pose_xyz(mm)="
                f"{[round(v, 1) for v in self.poses_mm[i][:3]]} "
                f"이번 캡처 점 개수={points.shape[0]} "
                f"누적 총점={sum(p.shape[0] for p, _ in self.settled_batches)}"
            )
            self._refresh_pcd(highlight=(points, colors_settled))
            self._update_path_and_marker()
            self._render_once()
            self._capture_frame()
            time.sleep(min(self.interval * 0.4, 0.4))
            self._refresh_pcd(highlight=None)
        else:
            print(f"[step {i + 1}/{self.n_steps}] 이 pose는 point 없음 (건너뜀)")
            self._update_path_and_marker()

        self._render_once()
        self._capture_frame()

        if self.step_index == self.n_steps - 1:
            self.finished_announced = True
            self.playing = False
            self._announce_finish()

    def _announce_finish(self):
        total_raw = sum(p.shape[0] for p, _ in self.settled_batches)
        all_points = (
            np.vstack([p for p, _ in self.settled_batches])
            if self.settled_batches
            else np.empty((0, 3))
        )
        if all_points.shape[0] > 0 and self.final_voxel > 0:
            tmp = o3d.geometry.PointCloud()
            tmp.points = o3d.utility.Vector3dVector(all_points)
            tmp = tmp.voxel_down_sample(self.final_voxel)
            merged_n = len(tmp.points)
        else:
            merged_n = 0
        print("=" * 60)
        print(f"누적 완료: {self.n_steps}개 pose, raw 누적 점 {total_raw}개")
        print(f"실제 파이프라인 기준 voxel({self.final_voxel*1000:.0f}mm) merge 후 약 {merged_n}개")
        if self.clusters:
            print(f"저장된 world_map_summary.json 기준 감지 장애물 {len(self.clusters)}개 (c 키로 표시)")
        print("=" * 60)

    # ---- geometry 갱신 ----
    def _refresh_pcd(self, highlight):
        if not self.settled_batches:
            self.pcd.points = o3d.utility.Vector3dVector(np.empty((0, 3)))
            self.pcd.colors = o3d.utility.Vector3dVector(np.empty((0, 3)))
            self.vis.update_geometry(self.pcd)
            return

        settled_points = np.vstack([p for p, _ in self.settled_batches])
        settled_colors = np.vstack([c for _, c in self.settled_batches])

        if highlight is not None:
            hi_points, _ = highlight
            base_points = settled_points[: settled_points.shape[0] - hi_points.shape[0]]
            base_colors = settled_colors[: settled_colors.shape[0] - hi_points.shape[0]]
            hi_colors = np.tile(HIGHLIGHT_COLOR, (hi_points.shape[0], 1))
            all_points = np.vstack([base_points, hi_points]) if base_points.shape[0] else hi_points
            all_colors = np.vstack([base_colors, hi_colors]) if base_colors.shape[0] else hi_colors
        else:
            all_points, all_colors = settled_points, settled_colors

        self.pcd.points = o3d.utility.Vector3dVector(all_points)
        self.pcd.colors = o3d.utility.Vector3dVector(all_colors)
        self.vis.update_geometry(self.pcd)

    def _update_path_and_marker(self):
        if self.path_positions:
            pts = np.asarray(self.path_positions)
            self.path_line.points = o3d.utility.Vector3dVector(pts)
            if len(pts) >= 2:
                lines = [[k, k + 1] for k in range(len(pts) - 1)]
                self.path_line.lines = o3d.utility.Vector2iVector(lines)
            else:
                self.path_line.lines = o3d.utility.Vector2iVector([])
            self.path_line.colors = o3d.utility.Vector3dVector(
                np.tile(PATH_COLOR, (max(len(pts) - 1, 0), 1))
            )
            self.vis.update_geometry(self.path_line)

            new_center = pts[-1]
            self.marker.translate(new_center - self.marker_center)
            self.marker_center = new_center
            self.vis.update_geometry(self.marker)

    def _render_once(self):
        self.vis.poll_events()
        self.vis.update_renderer()

    def _capture_frame(self):
        if not self.record_dir:
            return
        path = os.path.join(self.record_dir, f"frame_{self._frame_counter:05d}.png")
        self.vis.capture_screen_image(path, do_render=True)
        self._frame_counter += 1

    def run(self):
        self.vis.run()
        self.vis.destroy_window()
        if self.record_dir and self._frame_counter > 0:
            self._export_video()

    def _export_video(self):
        out_path = os.path.join(self.record_dir, "accumulation.mp4")
        cmd = [
            "ffmpeg", "-y", "-framerate", "3",
            "-i", os.path.join(self.record_dir, "frame_%05d.png"),
            "-pix_fmt", "yuv420p", out_path,
        ]
        try:
            subprocess.run(cmd, check=True, capture_output=True)
            print(f"mp4 저장 완료: {out_path}")
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            print(f"ffmpeg 변환 실패 (프레임은 {self.record_dir}에 PNG로 남아있음): {e}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("scan_dir", help="e.g. ~/RL-Avoid-Obstacle/data/world_maps/world_map_update_20260713_102455")
    parser.add_argument("--interval", type=float, default=0.6, help="스텝 간 재생 간격(초), 기본 0.6")
    parser.add_argument(
        "--voxel-preview", type=float, default=0.005,
        help="스텝별 point cloud 미리보기용 다운샘플 voxel 크기(m). 0이면 원본 그대로 표시",
    )
    parser.add_argument("--final-voxel", type=float, default=DEFAULT_FINAL_VOXEL_M, help="누적 완료 후 최종 merge voxel 크기(m)")
    parser.add_argument("--record-dir", default=None, help="설정 시 각 스텝 스크린샷을 저장하고 종료 시 mp4로 합침 (ffmpeg 필요)")
    parser.add_argument("--start-paused", action="store_true", help="시작 시 자동재생하지 않고 대기")
    args = parser.parse_args()

    scan_dir = os.path.expanduser(args.scan_dir)
    if not os.path.isdir(scan_dir):
        print(f"scan_dir이 존재하지 않습니다: {scan_dir}", file=sys.stderr)
        sys.exit(1)

    poses_mm, per_pose_points = load_scan(scan_dir)
    clusters = load_clusters(scan_dir)

    player = AccumulationPlayer(poses_mm, per_pose_points, clusters, args)
    player.run()


if __name__ == "__main__":
    main()

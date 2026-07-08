#!/usr/bin/env python3
"""로봇 없이, 이미 저장된 world_map_update_* 폴더로 장애물(cylinder) 추출을
오프라인 튜닝하는 스크립트.

world_map_algo.cluster_points()가 하는 일(바닥 제거 -> DBSCAN -> 작은 파편 제거 ->
cylinder 파라미터 추정)을 merged_base_roi.npy + world_map_summary.json의
reference_ground_z만으로 재현한다. 원본 scan_dir은 건드리지 않고, 결과는
<scan_dir>/obstacle_extraction_offline_result/ 에 저장한다.

사용법:
    python3 offline_obstacle_experiment.py <scan_dir> \
        [--eps 0.03] [--min-points 5] [--min-cluster-points 80] \
        [--safety-radius-margin 0.04] [--safety-height-margin 0.03]
"""
import argparse
import json
import os
import sys

import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_map_algo as algo  # noqa: E402  (sys.path 조정 이후에 import해야 함)


def load_merged_points_and_ground_z(scan_dir):
    npy_path = os.path.join(scan_dir, "merged_base_roi.npy")
    if not os.path.exists(npy_path):
        raise FileNotFoundError(f"merged_base_roi.npy를 찾을 수 없습니다: {scan_dir}")
    merged_points = np.load(npy_path)

    summary_path = os.path.join(scan_dir, "world_map_summary.json")
    with open(summary_path, "r", encoding="utf-8") as f:
        summary = json.load(f)

    ground_z = (summary.get("ground_z_alignment") or {}).get("reference_ground_z")
    return merged_points, ground_z, summary


def print_obstacles(label, obstacles):
    print(f"[{label}] {len(obstacles)} obstacle(s)")
    for obs in sorted(obstacles, key=lambda o: -o["num_points"]):
        c = obs["centroid"]
        print(
            f"  id={obs['id']:>3} num_points={obs['num_points']:>6} conf={obs['confidence']:.2f} "
            f"center=({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:+.3f}) "
            f"radius={obs['radius']*1000:5.1f}mm height={obs['height']*1000:5.1f}mm "
            f"safety_r={obs['safety_radius']*1000:5.1f}mm safety_h={obs['safety_height']*1000:5.1f}mm"
        )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "scan_dir",
        help="e.g. ~/RL-Avoid-Obstacle/data/world_maps/world_map_update_20260708_111238",
    )
    parser.add_argument("--eps", type=float, default=None, help="CLUSTER_EPS_M override")
    parser.add_argument("--min-points", type=int, default=None, help="CLUSTER_MIN_POINTS override")
    parser.add_argument(
        "--min-cluster-points", type=int, default=None,
        help="MIN_CLUSTER_POINTS_FOR_OBSTACLE override (DBSCAN 통과 후 파편 제거 기준)"
    )
    parser.add_argument("--safety-radius-margin", type=float, default=None, help="SAFETY_RADIUS_MARGIN_M override")
    parser.add_argument("--safety-height-margin", type=float, default=None, help="SAFETY_HEIGHT_MARGIN_M override")
    args = parser.parse_args()

    scan_dir = os.path.expanduser(args.scan_dir)

    eps = args.eps if args.eps is not None else algo.CLUSTER_EPS_M
    min_points = args.min_points if args.min_points is not None else algo.CLUSTER_MIN_POINTS
    min_cluster_points = (
        args.min_cluster_points if args.min_cluster_points is not None
        else algo.MIN_CLUSTER_POINTS_FOR_OBSTACLE
    )
    safety_radius_margin = (
        args.safety_radius_margin if args.safety_radius_margin is not None
        else algo.SAFETY_RADIUS_MARGIN_M
    )
    safety_height_margin = (
        args.safety_height_margin if args.safety_height_margin is not None
        else algo.SAFETY_HEIGHT_MARGIN_M
    )

    merged_points, ground_z, summary = load_merged_points_and_ground_z(scan_dir)
    print(f"Loaded merged_base_roi.npy: {merged_points.shape[0]} points from {scan_dir}")
    print(f"ground_z={ground_z}")
    print(
        f"eps={eps}, min_points={min_points}, min_cluster_points={min_cluster_points}, "
        f"safety_radius_margin={safety_radius_margin}, safety_height_margin={safety_height_margin}"
    )
    print()

    original_clusters = summary.get("clusters", [])
    print(f"(참고: 원본 world_map_summary.json 저장 당시 clusters={len(original_clusters)}, "
          f"cluster_eps_m={summary.get('cluster_eps_m')}, cluster_min_points={summary.get('cluster_min_points')})")
    for c in sorted(original_clusters, key=lambda c: -c["num_points"])[:5]:
        print(f"  id={c['id']:>3} num_points={c['num_points']:>6} radius={c['radius']*1000:6.1f}mm (바닥 제거 전)")
    print()

    baseline_clusters = algo.cluster_points(merged_points, ground_z=None, eps=eps, min_points=min_points)
    print_obstacles("바닥 제거 없이 cluster_points(ground_z=None)", baseline_clusters)
    print()

    candidate_points = algo.remove_ground_band(merged_points, ground_z)
    print(f"바닥 제거: {merged_points.shape[0]} -> {candidate_points.shape[0]} points")
    print()

    obstacles = algo.cluster_points(
        merged_points, ground_z=ground_z,
        eps=eps, min_points=min_points, min_cluster_points=min_cluster_points,
        safety_radius_margin=safety_radius_margin, safety_height_margin=safety_height_margin,
    )
    print_obstacles("cluster_points(ground_z=<추정값>) (바닥 제거 + 파편 필터 후)", obstacles)

    out_dir = os.path.join(scan_dir, "obstacle_extraction_offline_result")
    os.makedirs(out_dir, exist_ok=True)

    if candidate_points.shape[0] > 0:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(candidate_points.astype(np.float64))
        o3d.io.write_point_cloud(os.path.join(out_dir, "candidate_points.ply"), pcd)

    with open(os.path.join(out_dir, "obstacles.json"), "w", encoding="utf-8") as f:
        json.dump(
            {
                "frame_id": "base_link",
                "ground_z": ground_z,
                "params": {
                    "eps": eps, "min_points": min_points,
                    "min_cluster_points": min_cluster_points,
                    "safety_radius_margin": safety_radius_margin,
                    "safety_height_margin": safety_height_margin,
                },
                "obstacles": obstacles,
            },
            f, indent=2,
        )

    print(f"\n결과 저장: {out_dir}")


if __name__ == "__main__":
    main()

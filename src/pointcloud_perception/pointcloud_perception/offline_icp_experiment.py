#!/usr/bin/env python3
"""로봇 없이, 이미 저장된 world_map_update_* 폴더로 ICP 잔차보정을 오프라인 튜닝하는 스크립트.

사용법:
    python3 offline_icp_experiment.py <scan_dir> \
        [--icp-mode {z_only,translation_only,full_se3}] \
        [--ground-z {off,side_only}] \
        [--voxel-size 0.01] [--max-corr-dist 0.03] \
        [--min-fitness 0.30] [--max-rmse 0.035] \
        [--max-xy 0.025] [--max-z 0.035] [--max-roll-pitch 3.0] [--max-yaw 2.0]

world_map_algo.py를 단독 모듈로 import하므로(sys.path에 이 스크립트 디렉토리를 추가)
colcon build/ROS source 없이 open3d + numpy만 있으면 실행 가능하다. 원본 scan_dir은
건드리지 않고, 결과는 <scan_dir>/icp_offline_result/ 에 저장한다.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_map_algo as algo  # noqa: E402  (sys.path 조정 이후에 import해야 함)


def load_scan(scan_dir):
    """scan_pose_*_meta.json + scan_pose_*_base_roi.ply를 pose_index 순서로 복원.

    meta.json은 point가 0개인 pose에도 항상 저장되지만(world_map_algo.save_record),
    ply는 point가 있을 때만 저장되므로(save_pose_cloud) meta 기준으로 순회하고
    ply가 없으면 빈 배열로 채운다.
    """
    meta_files = sorted(glob.glob(os.path.join(scan_dir, "scan_pose_*_meta.json")))
    if not meta_files:
        raise FileNotFoundError(
            f"scan_pose_*_meta.json 파일을 찾을 수 없습니다: {scan_dir}"
        )

    poses = []
    per_pose_points = []
    for meta_path in meta_files:
        idx = int(os.path.basename(meta_path).split("_")[2])
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
        poses.append(meta["pose_xyz_abc"])

        ply_path = os.path.join(scan_dir, f"scan_pose_{idx:03d}_base_roi.ply")
        if os.path.exists(ply_path):
            pcd = o3d.io.read_point_cloud(ply_path)
            points = np.asarray(pcd.points, dtype=np.float64)
        else:
            points = np.empty((0, 3), dtype=np.float64)
        per_pose_points.append(points)

    return poses, per_pose_points


def apply_cli_overrides(args):
    if args.voxel_size is not None:
        algo.ICP_VOXEL_SIZE_M = args.voxel_size
    if args.max_corr_dist is not None:
        algo.ICP_MAX_CORR_DIST_M = args.max_corr_dist
    if args.min_fitness is not None:
        algo.MIN_ICP_FITNESS = args.min_fitness
    if args.max_rmse is not None:
        algo.MAX_ICP_RMSE_M = args.max_rmse
    if args.max_xy is not None:
        algo.MAX_ICP_DELTA_XY_M = args.max_xy
    if args.max_z is not None:
        algo.MAX_ICP_DELTA_Z_M = args.max_z
    if args.max_roll_pitch is not None:
        algo.MAX_ICP_DELTA_ROLL_PITCH_DEG = args.max_roll_pitch
    if args.max_yaw is not None:
        algo.MAX_ICP_DELTA_YAW_DEG = args.max_yaw

    algo.ICP_MODE = args.icp_mode
    # 현재 align_ground_z_per_pose는 side pose만 보정하므로 "off"와 "side_only" 두
    # 상태밖에 없다 (예전 all-pose 보정 버그는 이미 제거됨).
    algo.ENABLE_GROUND_Z_ALIGNMENT = (args.ground_z != "off")


def flat_ground_z_scatter(per_pose_points, poses):
    """flat pose들의 raw ground z mode 분산(std, range)을 계산. None이면 표본 부족."""
    modes = []
    for points, pose in zip(per_pose_points, poses):
        if not algo.is_flat_pose(pose):
            continue
        est = algo.estimate_ground_z_mode(points)
        if est is not None:
            modes.append(est["z_mode"])

    if len(modes) < 2:
        return None

    modes = np.array(modes)
    return {
        "n": int(len(modes)),
        "std_m": float(modes.std()),
        "range_m": float(modes.max() - modes.min()),
    }


def z_thickness_p95_p5(points):
    if points is None or points.shape[0] == 0:
        return None
    z = points[:, 2]
    return float(np.percentile(z, 95) - np.percentile(z, 5))


def reconstruct_icp_aligned_flat_clouds(per_pose_points, poses, mapping_report):
    """mapping_report에 저장된 applied_transform을 원본 flat pose cloud에 적용해서,
    ICP 보정이 flat pose 바닥 분산을 실제로 줄였는지 재구성한다.

    build_map_with_icp()는 최종 merge된 map만 반환하고 pose별 결과는 버리므로,
    진단 목적으로만 리포트에 저장된 transform을 여기서 다시 적용한다.
    """
    aligned = []
    for i, (points, pose) in enumerate(zip(per_pose_points, poses)):
        if not algo.is_flat_pose(pose):
            aligned.append(points)
            continue

        entry = mapping_report.get(str(i))
        if not entry or not entry.get("used_in_map") or points.shape[0] == 0:
            aligned.append(np.empty((0, 3), dtype=np.float64))
            continue

        icp = entry.get("icp") or {}
        T = icp.get("applied_transform")
        if T is None:
            aligned.append(points)
        else:
            aligned.append(algo.apply_transform(points, np.asarray(T, dtype=np.float64)))

    return aligned


def summarize_icp_report(mapping_report):
    flat_total = flat_passed = flat_accept = flat_reject = 0
    side_accept = side_reject = 0
    fitness_values = []
    rmse_values = []
    axis_keys = ("dx", "dy", "dz", "roll_deg", "pitch_deg", "yaw_deg")
    raw_max = {k: 0.0 for k in axis_keys}
    applied_max = {k: 0.0 for k in axis_keys}

    for entry in mapping_report.values():
        if entry["type"] == "flat":
            flat_total += 1
            qg = entry.get("quality_gate") or {}
            if qg.get("passed"):
                flat_passed += 1

        icp = entry.get("icp")
        if not icp or "accepted" not in icp:
            continue

        if entry["type"] == "flat":
            if icp["accepted"]:
                flat_accept += 1
            else:
                flat_reject += 1
        else:
            if icp["accepted"]:
                side_accept += 1
            else:
                side_reject += 1

        if icp.get("fitness") is not None:
            fitness_values.append(icp["fitness"])
        if icp.get("inlier_rmse") is not None:
            rmse_values.append(icp["inlier_rmse"])
        for k in axis_keys:
            if "raw_delta" in icp:
                raw_max[k] = max(raw_max[k], abs(icp["raw_delta"][k]))
            if "applied_delta" in icp:
                applied_max[k] = max(applied_max[k], abs(icp["applied_delta"][k]))

    return {
        "flat_total": flat_total,
        "flat_passed_quality_gate": flat_passed,
        "flat_icp_accept": flat_accept,
        "flat_icp_reject": flat_reject,
        "side_icp_accept": side_accept,
        "side_icp_reject": side_reject,
        "mean_fitness": float(np.mean(fitness_values)) if fitness_values else None,
        "mean_rmse_m": float(np.mean(rmse_values)) if rmse_values else None,
        "raw_max_delta": raw_max,
        "applied_max_delta": applied_max,
    }


def print_scatter(label, scatter):
    if scatter is None:
        print(f"  [{label}] flat pose ground z: 유효 추정치 부족")
        return
    print(
        f"  [{label}] flat pose ground z: n={scatter['n']}, "
        f"std={scatter['std_m']*1000:.1f}mm, range={scatter['range_m']*1000:.1f}mm"
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "scan_dir",
        help="e.g. ~/RL-Avoid-Obstacle/data/world_maps/world_map_update_20260707_154351",
    )
    parser.add_argument("--icp-mode", choices=["z_only", "translation_only", "full_se3"], default="full_se3")
    parser.add_argument("--ground-z", choices=["off", "side_only"], default="side_only")
    parser.add_argument("--voxel-size", type=float, default=None, help="ICP_VOXEL_SIZE_M override")
    parser.add_argument("--max-corr-dist", type=float, default=None, help="ICP_MAX_CORR_DIST_M override")
    parser.add_argument("--min-fitness", type=float, default=None, help="MIN_ICP_FITNESS override")
    parser.add_argument("--max-rmse", type=float, default=None, help="MAX_ICP_RMSE_M override")
    parser.add_argument("--max-xy", type=float, default=None, help="MAX_ICP_DELTA_XY_M override")
    parser.add_argument("--max-z", type=float, default=None, help="MAX_ICP_DELTA_Z_M override")
    parser.add_argument(
        "--max-roll-pitch", type=float, default=None, help="MAX_ICP_DELTA_ROLL_PITCH_DEG override"
    )
    parser.add_argument("--max-yaw", type=float, default=None, help="MAX_ICP_DELTA_YAW_DEG override")
    args = parser.parse_args()

    scan_dir = os.path.expanduser(args.scan_dir)
    apply_cli_overrides(args)

    poses, per_pose_points = load_scan(scan_dir)
    print(f"Loaded {len(poses)} poses from {scan_dir}")
    print(
        f"ICP_MODE={algo.ICP_MODE}, ENABLE_GROUND_Z_ALIGNMENT={algo.ENABLE_GROUND_Z_ALIGNMENT}, "
        f"ICP_VOXEL_SIZE_M={algo.ICP_VOXEL_SIZE_M}, ICP_MAX_CORR_DIST_M={algo.ICP_MAX_CORR_DIST_M}"
    )
    print()

    print_scatter("BEFORE (raw TF)", flat_ground_z_scatter(per_pose_points, poses))

    ground_quality = algo.compute_ground_quality(per_pose_points, poses)
    corrected_points, ground_z_alignment = algo.align_ground_z_per_pose(
        per_pose_points, poses, ground_quality
    )

    # 현재 온라인 파이프라인(ENABLE_ICP_MAPPING=False)이 만들 결과 - ICP 도입 전 baseline.
    non_icp_list = [p for p in corrected_points if p.shape[0] > 0]
    non_icp_merged = (
        algo.voxel_downsample(np.vstack(non_icp_list), algo.VOXEL_SIZE_M)
        if non_icp_list else np.empty((0, 3), dtype=np.float64)
    )

    merged_points, mapping_report = algo.build_map_with_icp(corrected_points, poses, ground_quality)

    aligned_flat_clouds = reconstruct_icp_aligned_flat_clouds(per_pose_points, poses, mapping_report)
    print_scatter("AFTER ICP (flat pose, reconstructed)", flat_ground_z_scatter(aligned_flat_clouds, poses))
    print()

    report_summary = summarize_icp_report(mapping_report)
    print(
        f"flat 품질게이트 통과: "
        f"{report_summary['flat_passed_quality_gate']}/{report_summary['flat_total']}"
    )
    print(
        f"flat ICP accept/reject (raw 기준): "
        f"{report_summary['flat_icp_accept']}/{report_summary['flat_icp_reject']}"
    )
    print(
        f"side ICP accept/reject (raw 기준): "
        f"{report_summary['side_icp_accept']}/{report_summary['side_icp_reject']}"
    )
    if report_summary["mean_fitness"] is not None:
        print(
            f"평균 fitness={report_summary['mean_fitness']:.3f}, "
            f"평균 rmse={report_summary['mean_rmse_m']*1000:.1f}mm"
        )
    rm = report_summary["raw_max_delta"]
    am = report_summary["applied_max_delta"]
    axis_line = (
        "|dx|={:.1f}mm |dy|={:.1f}mm |dz|={:.1f}mm "
        "|roll|={:.2f}deg |pitch|={:.2f}deg |yaw|={:.2f}deg"
    )
    print("raw 최대     " + axis_line.format(
        rm["dx"] * 1000, rm["dy"] * 1000, rm["dz"] * 1000,
        rm["roll_deg"], rm["pitch_deg"], rm["yaw_deg"],
    ))
    print("applied 최대 " + axis_line.format(
        am["dx"] * 1000, am["dy"] * 1000, am["dz"] * 1000,
        am["roll_deg"], am["pitch_deg"], am["yaw_deg"],
    ))
    print()

    before_thickness = z_thickness_p95_p5(non_icp_merged)
    after_thickness = z_thickness_p95_p5(merged_points)
    print(
        "전체 map z thickness(P95-P5): "
        f"non-ICP={before_thickness*1000:.1f}mm -> ICP={after_thickness*1000:.1f}mm"
        if before_thickness is not None and after_thickness is not None
        else "전체 map z thickness: 계산 불가(빈 map)"
    )

    clusters_non_icp = algo.cluster_points(non_icp_merged) if non_icp_merged.shape[0] > 0 else []
    clusters_icp = algo.cluster_points(merged_points) if merged_points.shape[0] > 0 else []
    print(f"clusters: non-ICP={len(clusters_non_icp)} -> ICP={len(clusters_icp)}")

    summary_path = os.path.join(scan_dir, "world_map_summary.json")
    if os.path.exists(summary_path):
        with open(summary_path, "r", encoding="utf-8") as f:
            original_summary = json.load(f)
        print(f"(참고: 원본 world_map_summary.json 저장 당시 clusters={len(original_summary.get('clusters', []))})")

    out_dir = os.path.join(scan_dir, "icp_offline_result")
    os.makedirs(out_dir, exist_ok=True)

    np.save(os.path.join(out_dir, "merged_base_roi_icp.npy"), merged_points)
    if merged_points.shape[0] > 0:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(merged_points.astype(np.float64))
        o3d.io.write_point_cloud(os.path.join(out_dir, "merged_base_roi_icp.ply"), pcd)

    with open(os.path.join(out_dir, "icp_mapping_report.json"), "w", encoding="utf-8") as f:
        json.dump(
            {
                "icp_mode": algo.ICP_MODE,
                "ground_z_enabled": algo.ENABLE_GROUND_Z_ALIGNMENT,
                "summary": report_summary,
                "ground_quality": ground_quality,
                "ground_z_alignment": ground_z_alignment,
                "icp_mapping": mapping_report,
            },
            f,
            indent=2,
        )

    print(f"\n결과 저장: {out_dir}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""저장된 world_map_update_* 스캔의 merged point cloud에 장애물 후보 추출
파이프라인(world_map_algo.cluster_points() 내부 단계)을 실제 함수 그대로
한 단계씩 실행하고, 각 단계 결과를 top-view(XY) 산점도 PNG로 저장한다.
발표 자료(GPT 이미지 생성 참고용 실데이터 스냅샷)를 만들기 위한 오프라인
시각화 스크립트. Hough 기반 재분리는 다루지 않는다(별도 주제).

로봇/ROS 없이 저장된 스캔 폴더만으로 동작한다. world_map_algo.py를 단독
모듈로 import하므로(sys.path에 이 스크립트 디렉토리를 추가) colcon
build/ROS source 없이 open3d + numpy + matplotlib만 있으면 실행 가능하다.
원본 scan_dir은 건드리지 않고, 결과는 <scan_dir>/pipeline_debug/ 에 저장한다.

사용법:
    python3 visualize_obstacle_pipeline.py <scan_dir>

scan_dir 예: ~/RL-Avoid-Obstacle/data/world_maps/world_map_update_20260713_101023

파이프라인 단계 (world_map_algo.cluster_points() 순서와 동일):
    0. 원본 merged point cloud
    1. remove_ground_band()            ground_z + 15mm 이하 제거
    2. remove_flying_pixel_outliers()  20-이웃 통계적 이상치 제거
    3. dbscan_labels()                 eps=3cm, min_points=5 (노이즈=-1)
    4. 라벨별 그룹화 + 80점 미만 후보 제거 (MIN_CLUSTER_POINTS_FOR_OBSTACLE)
"""
import argparse
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np

_NANUM_PATH = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
if os.path.exists(_NANUM_PATH):
    fm.fontManager.addfont(_NANUM_PATH)
    matplotlib.rcParams["font.family"] = fm.FontProperties(fname=_NANUM_PATH).get_name()
matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_map_algo as algo  # noqa: E402  (sys.path 조정 이후에 import해야 함)


def load_merged_points(scan_dir):
    path = os.path.join(scan_dir, "merged_base_roi.npy")
    if not os.path.exists(path):
        raise FileNotFoundError(f"merged_base_roi.npy 없음: {path}")
    return np.load(path)


def load_ground_z(scan_dir):
    summary_path = os.path.join(scan_dir, "world_map_summary.json")
    if os.path.exists(summary_path):
        with open(summary_path, "r", encoding="utf-8") as f:
            summary = json.load(f)
        ref_z = summary.get("ground_z_alignment", {}).get("reference_ground_z")
        if ref_z is not None:
            return ref_z
    # world_map_node.py와 동일한 fallback은 없으므로, summary가 없으면 raw 추정치 사용
    est = algo.estimate_ground_z_mode(load_merged_points(scan_dir))
    return est["z_mode"] if est is not None else 0.0


def raw_groups_before_filter(candidate_points, labels):
    """cluster_points()의 1차 라벨 루프(노이즈 skip)까지만 재현 - 80점 필터는 아직 적용 안 함."""
    groups = []
    for label in sorted(set(labels.tolist())):
        if label < 0:
            continue
        groups.append(candidate_points[labels == label])
    return groups


def circle_for_group(points_xy):
    center = points_xy.mean(axis=0)
    d = np.linalg.norm(points_xy - center, axis=1)
    radius = float(np.percentile(d, algo.CYLINDER_RADIUS_PERCENTILE)) if len(d) else 0.0
    return center, max(radius, 0.005)


def setup_ax(ax, title, xlim, ylim):
    ax.set_title(title, fontsize=10, fontweight="bold")
    ax.set_xlabel("X (m)", fontsize=8)
    ax.set_ylabel("Y (m)", fontsize=8)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=7)
    ax.grid(True, alpha=0.25)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("scan_dir", help="e.g. ~/RL-Avoid-Obstacle/data/world_maps/world_map_update_20260713_101023")
    args = parser.parse_args()

    scan_dir = os.path.expanduser(args.scan_dir)
    out_dir = os.path.join(scan_dir, "pipeline_debug")
    os.makedirs(out_dir, exist_ok=True)

    raw_points = load_merged_points(scan_dir)
    ground_z = load_ground_z(scan_dir)
    print(f"scan_dir={scan_dir}")
    print(f"raw points={raw_points.shape[0]}, ground_z={ground_z:.4f}m")

    xlim = (raw_points[:, 0].min() - 0.02, raw_points[:, 0].max() + 0.02)
    ylim = (raw_points[:, 1].min() - 0.02, raw_points[:, 1].max() + 0.02)

    # ---- Stage 0: 원본 ----
    stage0 = raw_points

    # ---- Stage 1: 바닥 15mm 이하 제거 ----
    stage1 = algo.remove_ground_band(stage0, ground_z)

    # ---- Stage 2: 20-이웃 통계적 노이즈 제거 ----
    stage2 = algo.remove_flying_pixel_outliers(stage1)

    # ---- Stage 3: DBSCAN (eps=3cm, min_points=5) ----
    labels = algo.dbscan_labels(stage2, algo.CLUSTER_EPS_M, algo.CLUSTER_MIN_POINTS)
    n_noise = int(np.sum(labels < 0))
    n_clusters_raw = len(set(labels.tolist()) - {-1})

    # ---- Stage 4: 라벨별 그룹화 + 80점 미만 후보 제거 ----
    raw_groups = raw_groups_before_filter(stage2, labels)
    kept_groups = [g for g in raw_groups if g.shape[0] >= algo.MIN_CLUSTER_POINTS_FOR_OBSTACLE]
    removed_groups = [g for g in raw_groups if g.shape[0] < algo.MIN_CLUSTER_POINTS_FOR_OBSTACLE]

    stats = {
        "scan_dir": scan_dir,
        "ground_z_m": ground_z,
        "stage0_raw_points": int(stage0.shape[0]),
        "stage1_after_ground_removal": int(stage1.shape[0]),
        "stage2_after_outlier_removal": int(stage2.shape[0]),
        "stage3_dbscan_raw_clusters": n_clusters_raw,
        "stage3_noise_points": n_noise,
        "stage4_kept_candidates(>=80pts)": len(kept_groups),
        "stage4_removed_candidates(<80pts)": len(removed_groups),
        "params": {
            "ground_margin_m": algo.MIN_OBSTACLE_HEIGHT_ABOVE_GROUND_M,
            "outlier_nb_neighbors": algo.OBSTACLE_OUTLIER_NB_NEIGHBORS,
            "outlier_std_ratio": algo.OBSTACLE_OUTLIER_STD_RATIO,
            "dbscan_eps_m": algo.CLUSTER_EPS_M,
            "dbscan_min_points": algo.CLUSTER_MIN_POINTS,
            "min_cluster_points_for_obstacle": algo.MIN_CLUSTER_POINTS_FOR_OBSTACLE,
        },
    }
    print(json.dumps(stats, indent=2, ensure_ascii=False))
    with open(os.path.join(out_dir, "pipeline_stats.json"), "w", encoding="utf-8") as f:
        json.dump(stats, f, indent=2, ensure_ascii=False)

    # ==================== 시각화 ====================
    fig, axes = plt.subplots(1, 5, figsize=(26, 5.2))

    # Panel 0: 원본
    ax = axes[0]
    setup_ax(ax, f"0. 원본 Point Cloud\n{stage0.shape[0]:,}점", xlim, ylim)
    ax.scatter(stage0[:, 0], stage0[:, 1], s=1, c="steelblue", alpha=0.5)

    # Panel 1: 바닥 제거
    ax = axes[1]
    setup_ax(
        ax,
        f"1. 바닥 {algo.MIN_OBSTACLE_HEIGHT_ABOVE_GROUND_M*1000:.0f}mm 이하 제거\n"
        f"{stage0.shape[0]:,} → {stage1.shape[0]:,}점",
        xlim, ylim,
    )
    removed_mask = stage0[:, 2] <= (ground_z + algo.MIN_OBSTACLE_HEIGHT_ABOVE_GROUND_M)
    ax.scatter(stage0[removed_mask, 0], stage0[removed_mask, 1], s=1, c="lightgray", alpha=0.4, label="제거(바닥)")
    ax.scatter(stage1[:, 0], stage1[:, 1], s=1.5, c="steelblue", alpha=0.7, label="유지")
    ax.legend(fontsize=6, loc="upper right")

    # Panel 2: outlier 제거
    ax = axes[2]
    setup_ax(
        ax,
        f"2. 20-이웃 통계적 노이즈 제거\n{stage1.shape[0]:,} → {stage2.shape[0]:,}점",
        xlim, ylim,
    )
    kept_set = {tuple(p) for p in map(tuple, stage2)}
    outlier_mask = np.array([tuple(p) not in kept_set for p in stage1])
    ax.scatter(stage1[outlier_mask, 0], stage1[outlier_mask, 1], s=4, c="red", alpha=0.6, label="제거(outlier)")
    ax.scatter(stage2[:, 0], stage2[:, 1], s=1.5, c="steelblue", alpha=0.7, label="유지")
    ax.legend(fontsize=6, loc="upper right")

    # Panel 3: DBSCAN
    ax = axes[3]
    setup_ax(
        ax,
        f"3. DBSCAN (eps=3cm, min_pts=5)\n클러스터 {n_clusters_raw}개, 노이즈 {n_noise}점",
        xlim, ylim,
    )
    noise_mask = labels < 0
    ax.scatter(stage2[noise_mask, 0], stage2[noise_mask, 1], s=1, c="lightgray", alpha=0.5, label="노이즈(-1)")
    cmap = plt.get_cmap("tab20")
    unique_labels = sorted(set(labels.tolist()) - {-1})
    for i, lbl in enumerate(unique_labels):
        m = labels == lbl
        ax.scatter(stage2[m, 0], stage2[m, 1], s=2, color=cmap(i % 20), alpha=0.8)
    ax.legend(fontsize=6, loc="upper right")

    # Panel 4: 80점 미만 제거
    ax = axes[4]
    setup_ax(
        ax,
        f"4. 80점 미만 후보 제거\n채택 {len(kept_groups)}개 / 제거 {len(removed_groups)}개",
        xlim, ylim,
    )
    ax.scatter(stage2[noise_mask, 0], stage2[noise_mask, 1], s=1, c="lightgray", alpha=0.3)
    for g in kept_groups:
        ax.scatter(g[:, 0], g[:, 1], s=2, c="seagreen", alpha=0.7)
        center, radius = circle_for_group(g[:, :2])
        circle = plt.Circle(center, radius, fill=False, edgecolor="green", linewidth=1.5)
        ax.add_patch(circle)
        ax.annotate(f"n={g.shape[0]}", center, fontsize=6, color="darkgreen", ha="center")
    for g in removed_groups:
        ax.scatter(g[:, 0], g[:, 1], s=2, c="orangered", alpha=0.7)
        center, radius = circle_for_group(g[:, :2])
        circle = plt.Circle(center, radius, fill=False, edgecolor="orangered", linewidth=1.2, linestyle="--")
        ax.add_patch(circle)
        ax.annotate(f"n={g.shape[0]} 제거", center, fontsize=6, color="orangered", ha="center")

    fig.suptitle(
        f"장애물 후보 추출 파이프라인 - {os.path.basename(scan_dir)}",
        fontsize=13, fontweight="bold",
    )
    fig.tight_layout(rect=[0, 0, 1, 0.94])

    combined_path = os.path.join(out_dir, "pipeline_overview.png")
    fig.savefig(combined_path, dpi=150)
    print(f"\n저장: {combined_path}")

    # 개별 패널도 따로 저장 (GPT에 개별로 참고시키고 싶을 때 대비)
    names = ["stage0_raw", "stage1_ground_removed", "stage2_outlier_removed", "stage3_dbscan", "stage4_filtered"]
    for ax, name in zip(axes, names):
        extent = ax.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
        indiv_path = os.path.join(out_dir, f"{name}.png")
        fig.savefig(indiv_path, bbox_inches=extent.expanded(1.25, 1.3), dpi=150)
        print(f"저장: {indiv_path}")

    plt.close(fig)


if __name__ == "__main__":
    main()

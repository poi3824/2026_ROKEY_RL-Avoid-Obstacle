#!/usr/bin/env python3
"""world_map_algo.cluster_points()의 원 피팅(circle-fit) 결과를, 완전히 다른
방법(OpenCV Canny/Hough Circle Transform)으로 교차검증하는 오프라인 스크립트.

배경: cluster_points()는 점 자체에 대수적 원 피팅(Kåsa method)을 해서 중심/반지름을
구한다. 이 스크립트는 같은 스캔을 점이 아니라 "이미지"로 바꿔서 검증한다 -
merged_base_roi.npy를 top-down 방향으로 격자화해 높이(z) 이미지를 만들고
(픽셀값 = 그 칸의 최대 z), morphological closing + blur로 점 사이 빈틈을 메운 뒤
cv2.HoughCircles(내부적으로 Canny 사용)로 원을 검출한다.

중요: 교수님 가이드라인대로 바닥 제거/flying pixel 제거 같은 점 단위 필터링을
적용하기 전의 raw merged_points를 그대로 이미지화한다 - cluster_points()가
쓰는 필터링된 점과는 별도 경로다. 그래서 두 결과가 일치하면 "필터링 로직이
우연히 맞았다"가 아니라 서로 독립적인 두 방법이 같은 답에 도달했다는
교차검증이 된다.

결과는 <scan_dir>/hough_circle_validation/ 에 저장한다 (height map, canny
edge, hough 검출 오버레이 PNG + cluster_points() 대비 비교 JSON).

필요 패키지: numpy, opencv-python(cv2), open3d(merged_base_roi.npy 로드용은
np.load만 쓰므로 실제로는 불필요 - world_map_algo import 때문에 있으면 좋음),
matplotlib.

사용법:
    python3 offline_hough_circle_validation.py <scan_dir> \
        [--resolution 0.005] [--close-iterations 2] \
        [--canny1 90] [--canny2 -1(=자동, hough param1로만 사용)] \
        [--hough-param2 18] [--min-radius-m 0.02] [--max-radius-m 0.09] \
        [--match-max-dist-m 0.05]
"""
import argparse
import json
import os
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

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
    return merged_points, ground_z


def build_height_image(points_xyz, ground_z, resolution_m, close_iterations):
    """raw point cloud -> top-down 높이(z) 이미지. 픽셀값 = 그 칸의 최대 z를
    [ground_z, z_max] 범위로 0~255 정규화한 값. 점 사이 빈틈은 morphological
    closing으로 메운다.

    반환: (img_u8, blurred, x_min, y_min) - x_min/y_min은 픽셀(0,0)의 world 좌표.
    """
    x_min, x_max = points_xyz[:, 0].min(), points_xyz[:, 0].max()
    y_min, y_max = points_xyz[:, 1].min(), points_xyz[:, 1].max()
    w = int(np.ceil((x_max - x_min) / resolution_m)) + 1
    h = int(np.ceil((y_max - y_min) / resolution_m)) + 1

    height_img = np.zeros((h, w), dtype=np.float32)
    px = ((points_xyz[:, 0] - x_min) / resolution_m).astype(int)
    py = ((points_xyz[:, 1] - y_min) / resolution_m).astype(int)
    np.maximum.at(height_img, (py, px), points_xyz[:, 2])

    z_lo = ground_z if ground_z is not None else float(points_xyz[:, 2].min())
    z_hi = float(points_xyz[:, 2].max())
    span = max(z_hi - z_lo, 1e-6)
    img_u8 = np.clip((height_img - z_lo) / span * 255, 0, 255).astype(np.uint8)

    kernel = np.ones((3, 3), np.uint8)
    closed = cv2.morphologyEx(img_u8, cv2.MORPH_CLOSE, kernel, iterations=close_iterations)
    blurred = cv2.GaussianBlur(closed, (5, 5), 0)

    return img_u8, closed, blurred, x_min, y_min


def detect_circles_hough(
    blurred_img, resolution_m, x_min, y_min,
    canny1, hough_param2, min_radius_m, max_radius_m,
):
    """blurred height image에서 cv2.HoughCircles로 원을 검출해 world 좌표로 변환.

    반환: [{"center": [x,y], "radius": r}, ...] (world 단위, m)
    """
    min_r_px = max(1, int(round(min_radius_m / resolution_m)))
    max_r_px = int(round(max_radius_m / resolution_m))

    circles = cv2.HoughCircles(
        blurred_img, cv2.HOUGH_GRADIENT, dp=1, minDist=min_r_px,
        param1=canny1, param2=hough_param2,
        minRadius=min_r_px, maxRadius=max_r_px,
    )
    if circles is None:
        return []

    results = []
    for cx_px, cy_px, r_px in circles[0]:
        wx = x_min + cx_px * resolution_m
        wy = y_min + cy_px * resolution_m
        wr = r_px * resolution_m
        results.append({"center": [float(wx), float(wy)], "radius": float(wr)})
    return results


def match_hough_to_clusters(hough_circles, clusters, max_dist_m):
    """각 cluster_points() 결과에 대해 가장 가까운 hough 원을 찾아 매칭한다."""
    matches = []
    used = set()
    for c in clusters:
        ccx, ccy = c["centroid"][0], c["centroid"][1]
        best_idx, best_dist = None, None
        for i, h in enumerate(hough_circles):
            if i in used:
                continue
            d = ((h["center"][0] - ccx) ** 2 + (h["center"][1] - ccy) ** 2) ** 0.5
            if best_dist is None or d < best_dist:
                best_idx, best_dist = i, d
        if best_idx is not None and best_dist <= max_dist_m:
            used.add(best_idx)
            matches.append({"cluster": c, "hough": hough_circles[best_idx], "center_dist_m": best_dist})
        else:
            matches.append({"cluster": c, "hough": None, "center_dist_m": None})
    unmatched_hough = [h for i, h in enumerate(hough_circles) if i not in used]
    return matches, unmatched_hough


def print_comparison(matches, unmatched_hough):
    print(f"{'id':>4} {'method':<12} {'center_x':>9} {'center_y':>9} {'radius_mm':>10} {'center_dist_mm':>15}")
    for m in matches:
        c = m["cluster"]
        print(f"{c['id']:>4} {'circle-fit':<12} {c['centroid'][0]:>9.3f} {c['centroid'][1]:>9.3f} {c['radius']*1000:>10.1f} {'':>15}")
        if m["hough"] is not None:
            h = m["hough"]
            print(f"{'':>4} {'hough':<12} {h['center'][0]:>9.3f} {h['center'][1]:>9.3f} {h['radius']*1000:>10.1f} {m['center_dist_m']*1000:>15.1f}")
        else:
            print(f"{'':>4} {'hough':<12} {'(매칭 안 됨)':>36}")
    for h in unmatched_hough:
        print(f"{'?':>4} {'hough only':<12} {h['center'][0]:>9.3f} {h['center'][1]:>9.3f} {h['radius']*1000:>10.1f} {'':>15}")


def save_overlay_png(out_path, blurred_img, resolution_m, x_min, y_min, clusters, hough_circles):
    """circle-fit(파란 실선) vs hough(빨간 점선)를 height image 위에 겹쳐 그린다."""
    fig, ax = plt.subplots(figsize=(8, 10))
    ax.imshow(blurred_img, cmap="gray", origin="lower",
              extent=[x_min, x_min + blurred_img.shape[1] * resolution_m,
                      y_min, y_min + blurred_img.shape[0] * resolution_m])

    for c in clusters:
        cx, cy = c["centroid"][0], c["centroid"][1]
        ax.add_patch(plt.Circle((cx, cy), c["radius"], fill=False, color="cyan", linewidth=2, label="circle-fit"))
        ax.plot(cx, cy, marker="+", color="cyan", markersize=8)

    for h in hough_circles:
        cx, cy = h["center"]
        ax.add_patch(
            plt.Circle((cx, cy), h["radius"], fill=False, color="red", linewidth=1.5, linestyle="--", label="hough")
        )
        ax.plot(cx, cy, marker="x", color="red", markersize=6)

    handles = [
        plt.Line2D([0], [0], color="cyan", lw=2, label="circle-fit (point-based)"),
        plt.Line2D([0], [0], color="red", lw=1.5, linestyle="--", label="Hough (image-based)"),
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=8)
    ax.set_xlabel("x in base_link (m)")
    ax.set_ylabel("y in base_link (m)")
    ax.set_title("circle-fit vs Hough circle cross-validation")
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "scan_dir",
        help="e.g. ~/RL-Avoid-Obstacle/data/world_maps/world_map_update_20260708_162630",
    )
    parser.add_argument("--resolution", type=float, default=0.005, help="height image 픽셀 해상도(m/px)")
    parser.add_argument("--close-iterations", type=int, default=2, help="morphological closing 반복 횟수")
    parser.add_argument("--canny1", type=float, default=90.0, help="HoughCircles param1 (내부 Canny 상단 임계값)")
    parser.add_argument("--hough-param2", type=float, default=18.0, help="HoughCircles param2 (accumulator 임계값 - 낮을수록 더 많이 검출)")
    parser.add_argument("--min-radius-m", type=float, default=0.02, help="검출할 최소 반지름(m)")
    parser.add_argument("--max-radius-m", type=float, default=0.09, help="검출할 최대 반지름(m)")
    parser.add_argument("--match-max-dist-m", type=float, default=0.05, help="circle-fit과 hough 원을 같은 물체로 볼 최대 중심 거리(m)")
    args = parser.parse_args()

    scan_dir = os.path.expanduser(args.scan_dir)
    merged_points, ground_z = load_merged_points_and_ground_z(scan_dir)
    print(f"Loaded merged_base_roi.npy: {merged_points.shape[0]} points (필터링 전) from {scan_dir}")
    print(f"ground_z={ground_z}")

    # 기준값: world_map_algo의 실제 파이프라인(바닥/flying pixel 제거 + circle-fit)
    clusters = algo.cluster_points(merged_points, ground_z=ground_z)
    print(f"\ncluster_points() 결과 (점 기반, 필터링 적용): {len(clusters)}개")

    # 검증 대상: raw merged_points를 이미지화 -> Hough (필터링 없음)
    img_u8, closed, blurred, x_min, y_min = build_height_image(
        merged_points, ground_z, args.resolution, args.close_iterations
    )
    hough_circles = detect_circles_hough(
        blurred, args.resolution, x_min, y_min,
        args.canny1, args.hough_param2, args.min_radius_m, args.max_radius_m,
    )
    print(f"Hough 검출 결과 (이미지 기반, 필터링 없음): {len(hough_circles)}개\n")

    matches, unmatched_hough = match_hough_to_clusters(hough_circles, clusters, args.match_max_dist_m)
    print_comparison(matches, unmatched_hough)

    out_dir = os.path.join(scan_dir, "hough_circle_validation")
    os.makedirs(out_dir, exist_ok=True)

    cv2.imwrite(os.path.join(out_dir, "height_image_raw.png"), img_u8)
    cv2.imwrite(os.path.join(out_dir, "height_image_closed.png"), closed)
    edges = cv2.Canny(blurred, args.canny1 / 2, args.canny1)
    cv2.imwrite(os.path.join(out_dir, "canny_edges.png"), edges)

    overlay_path = os.path.join(out_dir, "circle_fit_vs_hough_overlay.png")
    save_overlay_png(overlay_path, blurred, args.resolution, x_min, y_min, clusters, hough_circles)

    with open(os.path.join(out_dir, "comparison.json"), "w", encoding="utf-8") as f:
        json.dump(
            {
                "params": vars(args),
                "ground_z": ground_z,
                "cluster_points_result": clusters,
                "hough_circles_raw": hough_circles,
                "matches": [
                    {
                        "cluster_id": m["cluster"]["id"],
                        "circle_fit_center": m["cluster"]["centroid"][:2],
                        "circle_fit_radius_m": m["cluster"]["radius"],
                        "hough_center": m["hough"]["center"] if m["hough"] else None,
                        "hough_radius_m": m["hough"]["radius"] if m["hough"] else None,
                        "center_dist_m": m["center_dist_m"],
                    }
                    for m in matches
                ],
            },
            f, indent=2,
        )

    print(f"\n결과 저장: {out_dir}")
    print(f"  - {overlay_path} (circle-fit vs hough 오버레이 - 육안 비교용)")


if __name__ == "__main__":
    main()

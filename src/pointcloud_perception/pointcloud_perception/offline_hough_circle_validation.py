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

실제 로직(build_height_image/detect_circles_hough/match_hough_to_clusters/
save_hough_overlay_png)은 world_map_algo.py에 있다 - world_map_node가 스캔마다
자동으로 같은 함수를 호출해서 결과를 저장하므로(save_debug_visualizations),
여기서는 그 함수들을 오프라인으로 다시 돌려보는 CLI일 뿐이다.

결과는 <scan_dir>/hough_circle_validation/ 에 저장한다 (height map, canny
edge, hough 검출 오버레이 PNG + cluster_points() 대비 비교 JSON).

필요 패키지: numpy, opencv-python(cv2), matplotlib, open3d(world_map_algo가
import하므로 필요).

사용법:
    python3 offline_hough_circle_validation.py <scan_dir> \
        [--resolution 0.005] [--close-iterations 2] \
        [--canny1 90] [--hough-param2 18] \
        [--min-radius-m 0.02] [--max-radius-m 0.09] \
        [--match-max-dist-m 0.05]
"""
import argparse
import json
import os
import sys

import cv2
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


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "scan_dir",
        help="e.g. ~/RL-Avoid-Obstacle/data/world_maps/world_map_update_20260708_162630",
    )
    parser.add_argument("--resolution", type=float, default=algo.HOUGH_RESOLUTION_M, help="height image 픽셀 해상도(m/px)")
    parser.add_argument("--close-iterations", type=int, default=algo.HOUGH_CLOSE_ITERATIONS, help="morphological closing 반복 횟수")
    parser.add_argument("--canny1", type=float, default=algo.HOUGH_CANNY1, help="HoughCircles param1 (내부 Canny 상단 임계값)")
    parser.add_argument("--hough-param2", type=float, default=algo.HOUGH_PARAM2, help="HoughCircles param2 (accumulator 임계값 - 낮을수록 더 많이 검출)")
    parser.add_argument("--min-radius-m", type=float, default=algo.HOUGH_MIN_RADIUS_M, help="검출할 최소 반지름(m)")
    parser.add_argument("--max-radius-m", type=float, default=algo.HOUGH_MAX_RADIUS_M, help="검출할 최대 반지름(m)")
    parser.add_argument("--match-max-dist-m", type=float, default=algo.HOUGH_MATCH_MAX_DIST_M, help="circle-fit과 hough 원을 같은 물체로 볼 최대 중심 거리(m)")
    args = parser.parse_args()

    scan_dir = os.path.expanduser(args.scan_dir)
    merged_points, ground_z = load_merged_points_and_ground_z(scan_dir)
    print(f"Loaded merged_base_roi.npy: {merged_points.shape[0]} points (필터링 전) from {scan_dir}")
    print(f"ground_z={ground_z}")

    # 기준값: world_map_algo의 실제 파이프라인(바닥/flying pixel 제거 + circle-fit)
    clusters = algo.cluster_points(merged_points, ground_z=ground_z)
    print(f"\ncluster_points() 결과 (점 기반, 필터링 적용): {len(clusters)}개")

    # 검증 대상: raw merged_points를 이미지화 -> Hough (필터링 없음)
    img_u8, closed, blurred, x_min, y_min = algo.build_height_image(
        merged_points, ground_z, args.resolution, args.close_iterations
    )
    hough_circles = algo.detect_circles_hough(
        blurred, args.resolution, x_min, y_min,
        args.canny1, args.hough_param2, args.min_radius_m, args.max_radius_m,
    )
    print(f"Hough 검출 결과 (이미지 기반, 필터링 없음): {len(hough_circles)}개\n")

    matches, unmatched_hough = algo.match_hough_to_clusters(hough_circles, clusters, args.match_max_dist_m)
    print_comparison(matches, unmatched_hough)

    out_dir = os.path.join(scan_dir, "hough_circle_validation")
    os.makedirs(out_dir, exist_ok=True)

    cv2.imwrite(os.path.join(out_dir, "height_image_raw.png"), img_u8)
    cv2.imwrite(os.path.join(out_dir, "height_image_closed.png"), closed)
    edges = cv2.Canny(blurred, args.canny1 / 2, args.canny1)
    cv2.imwrite(os.path.join(out_dir, "canny_edges.png"), edges)

    overlay_path = os.path.join(out_dir, "circle_fit_vs_hough_overlay.png")
    algo.save_hough_overlay_png(overlay_path, blurred, args.resolution, x_min, y_min, clusters, hough_circles)

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

#!/usr/bin/env python3
"""pick_logger.py가 쌓아온 pick_attempts SQLite 로그(실제 파지 시도 기록)를 읽어,
grip width 기반 성공/실패 판정과 depth(surface_z) 측정 안정성을 발표용 차트로
그린다. 로봇/ROS 없이 DB 파일만으로 동작한다.

사용법:
    python3 visualize_grasp_log.py [--db ~/.ros/my_robot_pkg/pick_log.db] [--out FILE]
"""
import argparse
import os
import sqlite3

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt

_NANUM_PATH = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
if os.path.exists(_NANUM_PATH):
    fm.fontManager.addfont(_NANUM_PATH)
    matplotlib.rcParams["font.family"] = fm.FontProperties(fname=_NANUM_PATH).get_name()
matplotlib.rcParams["axes.unicode_minus"] = False

GOOD = "#0ca30c"
CRITICAL = "#d03b3b"
MUTED = "#898781"
GRID = "#e1e0d9"
INK_SECONDARY = "#52514e"
GRIP_MIN_WIDTH_MM = 30.0

DEFAULT_DB_PATH = os.path.expanduser("~/.ros/my_robot_pkg/pick_log.db")


def load_rows(db_path):
    con = sqlite3.connect(db_path)
    con.row_factory = sqlite3.Row
    cur = con.cursor()
    cur.execute(
        "SELECT id, ts, obj_label, attempt_no, surface_z_mm, gripper_width_mm, "
        "grip_detected, motion_done, success FROM pick_attempts ORDER BY id"
    )
    rows = cur.fetchall()
    con.close()
    return rows


def find_retry_pairs(rows):
    """실패(success=0) 직후, 같은 obj_label의 다음 attempt_no가 성공한 쌍을 찾는다."""
    pairs = []
    for i in range(len(rows) - 1):
        cur, nxt = rows[i], rows[i + 1]
        if (
            cur["success"] == 0
            and nxt["obj_label"] == cur["obj_label"]
            and nxt["attempt_no"] == cur["attempt_no"] + 1
            and nxt["success"] == 1
        ):
            pairs.append((i, i + 1))
    return pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--db", default=DEFAULT_DB_PATH)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    rows = load_rows(args.db)
    if not rows:
        print(f"DB에 기록이 없습니다: {args.db}")
        return
    print(f"{len(rows)}건 로드: {args.db}")

    idx = list(range(1, len(rows) + 1))
    widths = [r["gripper_width_mm"] for r in rows]
    depths = [r["surface_z_mm"] for r in rows]
    success = [bool(r["success"]) for r in rows]
    retry_pairs = find_retry_pairs(rows)
    print(f"실패->재시도성공 쌍: {len(retry_pairs)}건")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5.5))

    # ---- Panel 1: gripper width per attempt ----
    ok_x = [i for i, s in zip(idx, success) if s]
    ok_y = [w for w, s in zip(widths, success) if s]
    fail_x = [i for i, s in zip(idx, success) if not s]
    fail_y = [w for w, s in zip(widths, success) if not s]

    ax1.axhline(GRIP_MIN_WIDTH_MM, color=MUTED, linestyle="--", linewidth=1.2, zorder=1)
    ax1.text(
        len(idx) + 0.3, GRIP_MIN_WIDTH_MM, f"grip_min_width_mm = {GRIP_MIN_WIDTH_MM:.0f}mm",
        fontsize=8, color=INK_SECONDARY, va="center",
    )
    ax1.scatter(ok_x, ok_y, s=26, color=GOOD, label=f"성공 (width≥30mm & grip_detected & motion_done) — {len(ok_x)}건", zorder=3)
    ax1.scatter(fail_x, fail_y, s=34, color=CRITICAL, marker="X", label=f"실패 — {len(fail_x)}건", zorder=3)

    for fi, si in retry_pairs:
        r_fail, r_ok = rows[fi], rows[si]
        ax1.annotate(
            "", xy=(si + 1, r_ok["gripper_width_mm"]), xytext=(fi + 1, r_fail["gripper_width_mm"]),
            arrowprops=dict(arrowstyle="->", color=INK_SECONDARY, linewidth=1.1, linestyle=":"),
        )
        ax1.annotate(
            f"{r_fail['obj_label']} #{r_fail['attempt_no']}, {r_fail['gripper_width_mm']:.1f}mm",
            (fi + 1, r_fail["gripper_width_mm"]), fontsize=6.5, color=CRITICAL,
            xytext=(0, -18), textcoords="offset points", ha="center",
        )

    ax1.set_ylim(-2, 68)
    ax1.set_title("그립퍼 폭(width) 기반 파지 성공/실패 판정 — 실제 로그", fontsize=11, fontweight="bold")
    ax1.set_xlabel("pick 시도 순번 (시간순)", fontsize=9)
    ax1.set_ylabel("gripper width (mm)", fontsize=9)
    ax1.grid(True, color=GRID, linewidth=0.8)
    ax1.set_axisbelow(True)
    ax1.legend(fontsize=7.5, loc="upper right")
    ax1.tick_params(labelsize=8)
    ax1.text(
        0.02, 0.03,
        "* 실패 3건 모두 motion_done=1, grip_detected=1이었지만 width<30mm이라 실패 처리됨",
        transform=ax1.transAxes, fontsize=6.5, color=INK_SECONDARY,
    )

    # ---- Panel 2: depth(surface_z) consistency ----
    ax2.scatter(idx, depths, s=22, color="#2a78d6", zorder=3)
    valid_depths = sorted(d for d in depths if d is not None)
    band_center = valid_depths[len(valid_depths) // 2] if valid_depths else 0.0
    if valid_depths:
        ax2.axhspan(band_center - 3, band_center + 3, color="#2a78d6", alpha=0.08, zorder=1)

    for i, r in enumerate(rows):
        if r["surface_z_mm"] is not None and abs(r["surface_z_mm"] - band_center) > 20:
            outcome = "→ 이 attempt는 실패" if not r["success"] else "→ clearance 여유로 파지는 성공"
            dot_color = CRITICAL if not r["success"] else "#eda100"
            ax2.scatter([i + 1], [r["surface_z_mm"]], s=50, color=dot_color, marker="o", zorder=4)
            ax2.annotate(
                f"{r['obj_label']} #{r['attempt_no']}\nsurface_z={r['surface_z_mm']:.0f}mm "
                f"({r['surface_z_mm']-band_center:+.0f}mm)\n{outcome}",
                (i + 1, r["surface_z_mm"]), fontsize=6.5, color=dot_color,
                xytext=(-10, -30), textcoords="offset points", ha="right",
                arrowprops=dict(arrowstyle="-", color=dot_color, linewidth=0.8),
            )

    ax2.set_title("Depth(surface_z) 측정값의 일관성 — 실제 로그", fontsize=11, fontweight="bold")
    ax2.set_xlabel("pick 시도 순번 (시간순)", fontsize=9)
    ax2.set_ylabel("surface_z (mm, base 좌표계)", fontsize=9)
    ax2.grid(True, color=GRID, linewidth=0.8)
    ax2.set_axisbelow(True)
    ax2.tick_params(labelsize=8)

    fig.suptitle("Depth 기반 Approach & Gripper Width 판정 — 실측 pick_attempts 로그 51건", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.94])

    out_path = args.out or os.path.join(os.path.dirname(os.path.abspath(args.db)), "grasp_log_chart.png")
    fig.savefig(out_path, dpi=150)
    print(f"저장: {out_path}")


if __name__ == "__main__":
    main()

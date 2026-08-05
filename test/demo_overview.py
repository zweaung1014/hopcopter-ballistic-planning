"""Contact sheet: every scenario in the deck on one figure.

Rebuilds each map, runs the ballistic planner with the suite's shared parameters,
and lays the six top-down results out in a 2x3 grid under one legend and one
parameter caption.  Intended as an opening or closing slide — the per-scenario
demos carry the detail.

Figures produced
----------------
  test/overview_contact_sheet.png

Run:
    python test/demo_overview.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import config
from demo_common import (
    TITLE_FS, LABEL_FS, ANNOT_FS, C_BALL,
    make_planner, diagnose_path, n_bad_hops,
    draw_topdown_compact, param_caption, save, out_path,
)

from maps import (
    tall_wall, tall_narrow_wall, barely_jumpable_wall,
    tall_stairs, stairs_with_curb, slope_crest,
)


# (title, subtitle, map builder, start, goal)
SCENARIOS = [
    ("Go around",
     "1.0 m wall, bypassable in y",
     tall_wall.build, (0.5, 2.4), (4.5, 2.4)),
    ("Hop over",
     "0.15 m ridge, no single-hop bypass",
     tall_narrow_wall.build, (0.5, 2.4), (4.5, 2.4)),
    ("Shift the takeoff",
     "0.22 m wall — only one takeoff cell clears",
     barely_jumpable_wall.build, (0.5, 2.4), (4.5, 2.4)),
    ("Climb stairs",
     "three 0.4 m risers to a 1.2 m platform",
     tall_stairs.build, (0.5, 2.5), (4.0, 2.5)),
    ("Stairs with curbs",
     "curbs are local maxima — clearance gate fires",
     stairs_with_curb.build, (0.5, 2.5), (4.5, 2.5)),
    ("Climb a slope",
     "graded ramp over a convex crest",
     slope_crest.build, (0.5, 2.5), (4.5, 2.5)),
]


def main() -> int:
    print(param_caption())
    fig, axes = plt.subplots(2, 3, figsize=(17, 11), squeeze=False)
    n_failed = 0

    for idx, (title, subtitle, builder, start, goal) in enumerate(SCENARIOS):
        ax = axes[idx // 3, idx % 3]
        m = builder()
        planner = make_planner(m, False, start, goal)
        path = planner.plan()

        draw_topdown_compact(m, ax, colorbar_on=fig)

        if path is None:
            n_failed += 1
            ax.text(0.5, 0.5, "NO PATH", ha="center", va="center", color="#b71c1c", transform=ax.transAxes)
            ax.set_title(f"{idx + 1}. {title}\n{subtitle}\nNO PATH FOUND", color="#b71c1c", wrap=True)
            continue

        diags = diagnose_path(planner, m, path)
        bad = n_bad_hops(diags)
        if bad:
            n_failed += 1

        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, color=C_BALL, linewidth=2.6, zorder=8)
        ax.plot(xs, ys, "o", color=C_BALL, markersize=6, zorder=9)
        ax.plot(*start, "go", markersize=10, zorder=10)
        ax.plot(*goal, "r*", markersize=14, zorder=10)

        ax.set_title(
            f"{idx + 1}. {title}\n{subtitle}\n"
            f"{len(path) - 1} hops · "
            f"goal z={m.get_elevation(*path[-1]):.2f} m · "
            f"{bad} hop(s) clipping", wrap=True
        )
        print(f"{idx + 1}. {title:<20} {len(path)-1} hops  "
              f"goal z={m.get_elevation(*path[-1]):.2f}  bad={bad}")

    handles = [
        plt.Line2D([], [], color=C_BALL, linewidth=2.6, marker="o",
                   label="Ballistic path (clearance ON)"),
        plt.Line2D([], [], color="green", marker="o", linestyle="none",
                   label="start"),
        plt.Line2D([], [], color="red", marker="*", linestyle="none",
                   markersize=12, label="goal"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 0.005))

    fig.suptitle(
        "Ballistic hopping planner — scenario overview", wrap=True
    )
    fig.tight_layout(rect=(0, 0.035, 1, 0.93))
    save(fig, out_path("overview_contact_sheet.png"))
    plt.close(fig)

    return 0 if n_failed == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())

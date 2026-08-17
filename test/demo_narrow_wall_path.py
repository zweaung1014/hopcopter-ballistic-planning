"""Narrow wall — single-planner path demo.

Simplified version of `demo_narrow_wall_showcase.py`: runs only the ballistic
planner (clearance ON) and renders the resulting path top-down.  No baseline
comparison, no arc-strip figure.

Figure produced
---------------
  results/energy_aware_planning/prof_narrow_wall_path_topdown.png

Run:
    python test/demo_narrow_wall_path.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

import config
from demo_common import (
    C_BALL,
    TOPDOWN_FIGSIZE,
    draw_topdown_base,
    make_planner,
    out_path,
    param_caption,
    save,
)
from maps.tall_narrow_wall import (
    WALL_HEIGHT,
    WALL_XMAX,
    WALL_XMIN,
    WALL_YMAX,
    WALL_YMIN,
    build as build_map,
)

START = (0.5, 2.4)
GOAL  = (4.5, 2.4)


# ---------------------------------------------------------------------------
# Figure
# ---------------------------------------------------------------------------

def draw_topdown(m, path: list, save_path: str) -> None:
    fig, ax = plt.subplots(figsize=TOPDOWN_FIGSIZE)
    draw_topdown_base(m, fig, ax)

    # Wall outline
    ax.add_patch(mpatches.Rectangle(
        (WALL_XMIN, WALL_YMIN),
        WALL_XMAX - WALL_XMIN,
        WALL_YMAX - WALL_YMIN,
        fill=False, edgecolor="k", linewidth=1.6, zorder=5,
    ))

    # Path
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    ax.plot(xs, ys, color=C_BALL, linewidth=2.2, zorder=8)
    ax.plot(xs, ys, "o", color=C_BALL, markersize=7, zorder=9)

    # Waypoint index labels
    for i, p in enumerate(path):
        ax.annotate(f"{i}", p, xytext=(6, 6), textcoords="offset points",
                    color=C_BALL, fontsize=9)

    # Start / goal markers
    ax.plot(*START, "go", markersize=11, zorder=10, label="start")
    ax.plot(*GOAL,  "r*", markersize=15, zorder=10, label="goal")
    ax.legend(loc="upper left")

    n_hops = len(path) - 1
    ax.set_title(
        f"tall_narrow_wall — ballistic planner path\n"
        f"wall x=[{WALL_XMIN}, {WALL_XMAX}] h={WALL_HEIGHT:.2f} m  |  "
        f"{n_hops} hop{'s' if n_hops != 1 else ''}"
    )

    fig.tight_layout()
    save(fig, save_path)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    print(param_caption())
    m = build_map()
    planner = make_planner(m, False, START, GOAL)
    path = planner.plan()

    if path is None:
        print("ERROR: planner found no path.")
        return 1

    print(f"{len(path)} waypoints, {len(path) - 1} hops:")
    for i, p in enumerate(path):
        print(f"  {i}: ({p[0]:.2f}, {p[1]:.2f})")

    draw_topdown(m, path, out_path("prof_narrow_wall_path_topdown.png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())

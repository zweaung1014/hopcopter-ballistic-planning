"""Professor showcase: prove ballistic clearance changes the A* plan.

Runs the SAME `HoppingAStarPlanner` twice on `maps/tall_wall.py` — once
with the ballistic arc-clearance gate DISABLED (baseline) and once ENABLED
(ballistic). The two paths diverge visibly: the baseline plans hops that
would slice through the 1 m tall wall, while the ballistic planner lays
waypoints noticeably farther from the wall so the parabolic trajectory
clears it.

The feasibility gate (Campana Eq. 4 + `v_s <= V_max`) is on for BOTH runs
— it enforces physics, not clearance. Only the arc-vs-terrain clearance
rejection and the proximity penalty are toggled.

Two figures are produced:
  * test/prof_showcase_topdown.png  — both paths overlaid on the map,
    baseline edges that would collide are highlighted in translucent red.
  * test/prof_showcase_arcs.png     — per-hop side-view strip: baseline
    row on top, ballistic row on bottom. Arcs are green when they clear
    and red when they'd collide (drawn using the same criterion for both
    planners so the comparison is honest).

Run:
    python test/demo_prof_showcase.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import matplotlib.patches as mpatches
import numpy as np

import config
from demo_common import (
    SLIDE_FIGSIZE, TOPDOWN_FIGSIZE, arcs_figsize,
    N_ANGLES, V_MAX,
    PRESENTATION_DPI, TITLE_FS, LABEL_FS, ANNOT_FS,
    make_planner, diagnose_edge, n_bad_hops, param_caption, save, out_path
)
from hopping_astar_planner import HoppingAStarPlanner
from map2d5 import Map2D5
from maps.tall_wall import build as build_tall_wall
from visualizer import Visualizer, draw_arc_side_view


# --- Scenario knobs (HOP_RADIUS / N_ANGLES / V_MAX come from demo_common so
# every figure in the deck quotes the same physics; config.py untouched) --- #
START = (0.5, 2.4)          # left of the wall, y=2.4 is inside the wall's y range
GOAL = (4.5, 2.4)           # right of the wall

# The wall lives at x in [2.0, 3.0], y in [1.8, 3.0], z = 1.0 (see maps/tall_wall.py).
WALL_XMIN, WALL_XMAX = 2.0, 3.0
WALL_YMIN, WALL_YMAX = 1.8, 3.0


def distance_to_wall(p: tuple[float, float]) -> float:
    """Min XY distance from `p` to the wall's axis-aligned bounding box.
    Returns 0.0 when `p` is inside the wall footprint."""
    dx = max(WALL_XMIN - p[0], 0.0, p[0] - WALL_XMAX)
    dy = max(WALL_YMIN - p[1], 0.0, p[1] - WALL_YMAX)
    return math.hypot(dx, dy)


# --- Figure 1: top-down A/B overlay -------------------------------------- #

def draw_topdown(m: Map2D5, path_base, path_ball, diags_base, save_path: str):
    fig, ax = plt.subplots(figsize=TOPDOWN_FIGSIZE)
    vis = Visualizer.__new__(Visualizer)
    vis.map_env = m; vis.fig = fig; vis.ax = ax
    ax.set_xlim(0, m.size_x); ax.set_ylim(0, m.size_y)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    vis.draw_map()

    # Wall footprint outline (already visible as elevation, but a thick
    # outline anchors the eye).
    ax.add_patch(mpatches.Rectangle(
        (WALL_XMIN, WALL_YMIN), WALL_XMAX - WALL_XMIN, WALL_YMAX - WALL_YMIN,
        fill=False, edgecolor="k", linewidth=1.6, zorder=5,
    ))

    # Baseline path (red-orange dashed, X markers).
    xs_b = [p[0] for p in path_base]; ys_b = [p[1] for p in path_base]
    ax.plot(xs_b, ys_b, color="#e65100", linewidth=2.0, linestyle="--",
            zorder=6, label="Baseline (clearance OFF)")
    ax.plot(xs_b, ys_b, "x", color="#e65100", markersize=10, zorder=7)

    # Highlight baseline edges that fail the ballistic clearance gate.
    for i, d in enumerate(diags_base):
        if d["feasible"] and d["mc"] < config.MIN_CLEARANCE:
            p0 = path_base[i]; p1 = path_base[i + 1]
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]],
                    color="#d50000", linewidth=9.0, alpha=0.65, zorder=6.5,
                    solid_capstyle="round")
            mx, my = 0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1])
            ax.annotate(
                f"COLLIDES\nmc={d['mc']:+.2f} m",
                (mx, my), xytext=(0, 22), textcoords="offset points",
                ha="center", color="#b71c1c", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.25", fc="white",
                          ec="#b71c1c", lw=1.2), zorder=11,
            )

    # Ballistic path (teal solid, o markers).
    xs_a = [p[0] for p in path_ball]; ys_a = [p[1] for p in path_ball]
    ax.plot(xs_a, ys_a, color="#00695c", linewidth=2.2, zorder=8,
            label="Ballistic (clearance ON)")
    ax.plot(xs_a, ys_a, "o", color="#00695c", markersize=7, zorder=9)

    # Annotate each waypoint with its distance to the wall.
    for i, p in enumerate(path_base):
        d = distance_to_wall(p)
        ax.annotate(f"B{i}\n{d:.2f}m", p, xytext=(6, -12),
                    textcoords="offset points", color="#e65100")
    for i, p in enumerate(path_ball):
        d = distance_to_wall(p)
        ax.annotate(f"A{i}\n{d:.2f}m", p, xytext=(6, 6),
                    textcoords="offset points", color="#00695c")

    ax.plot(START[0], START[1], "go", markersize=11, zorder=10, label="start")
    ax.plot(GOAL[0], GOAL[1], "r*", markersize=15, zorder=10, label="goal")

    handles, labels = ax.get_legend_handles_labels()
    handles.append(mpatches.Patch(color="#c62828", alpha=0.35,
                                  label="baseline edge that would collide"))
    labels.append("baseline edge that would collide")
    ax.legend(handles, labels, loc="upper left")

    ax.set_title(
        "tall_wall demo: baseline (clearance off) vs ballistic (clearance on)\n"
        "labels show waypoint distance to the wall's XY bounding box", wrap=True
    )
    fig.tight_layout()
    save(fig, save_path)


# --- Figure 2: per-hop side-view strip ----------------------------------- #

def draw_arc_strip(
    m: Map2D5,
    ballistic_planner: HoppingAStarPlanner,
    path_base, diags_base,
    path_ball, diags_ball,
    save_path: str,
):
    n_base = len(path_base) - 1
    n_ball = len(path_ball) - 1
    ncols = max(n_base, n_ball)
    fig, axes = plt.subplots(2, ncols, figsize=arcs_figsize(ncols),
                             squeeze=False)

    obs_fill = ballistic_planner._obstacle_fill
    ymax = max(1.0, obs_fill) + 0.35

    for row, (label, path, diags) in enumerate([
        ("Baseline", path_base, diags_base),
        ("Ballistic", path_ball, diags_ball),
    ]):
        for i in range(ncols):
            ax = axes[row, i]
            if i >= len(path) - 1:
                ax.axis("off")
                continue
            d = diags[i]
            if not d["feasible"]:
                ax.set_facecolor("#fbe9e7")
                ax.text(0.5, 0.5, "INFEASIBLE\n(no valid alpha_s)",
                        ha="center", va="center",
                        transform=ax.transAxes)
                ax.set_xticks([]); ax.set_yticks([])
            else:
                p0 = path[i]; p1 = path[i + 1]
                draw_arc_side_view(
                    ax,
                    (p0[0], p0[1], d["z0"]), (p1[0], p1[1], d["z1"]),
                    d["alpha_s"], m,
                    config.ROBOT_RADIUS, config.LEG_LENGTH, obs_fill,
                    config.ARC_SAMPLE_MAX_STEP,
                    min_clearance_gate=config.MIN_CLEARANCE,
                    n_lateral=config.ARC_LATERAL_SAMPLES,
                )
                ax.set_ylim(-0.05, ymax)
            if i == 0:
                ax.set_ylabel(f"{label}\nz (m)")
            # Terse: at 5 panels across a slide each is under 2.7 in wide.
            ax.set_xlabel(f"hop {i}   {path[i][0]:.1f}→{path[i+1][0]:.1f} m")
            ax.xaxis.set_major_locator(MaxNLocator(nbins=3))
            ax.yaxis.set_major_locator(MaxNLocator(nbins=4))

    fig.suptitle(
        "Per-hop side view — baseline (top) vs ballistic (bottom)\n"
        f"red = below the {config.MIN_CLEARANCE} m clearance gate  ·  "
        "both paths judged by the same criterion", wrap=True
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    save(fig, save_path)


# --- Main ---------------------------------------------------------------- #

def main() -> int:
    print(param_caption())
    m = build_tall_wall()

    planner_base = make_planner(m, True, START, GOAL)
    planner_ball = make_planner(m, False, START, GOAL)

    path_base = planner_base.plan()
    path_ball = planner_ball.plan()

    if path_base is None or path_ball is None:
        print(f"Baseline path: {'FOUND' if path_base else 'NONE'}")
        print(f"Ballistic path: {'FOUND' if path_ball else 'NONE'}")
        print("One of the runs found no path — cannot compare. Aborting.")
        return 1

    diags_base = [
        diagnose_edge(planner_ball, m, path_base[i], path_base[i + 1])
        for i in range(len(path_base) - 1)
    ]
    diags_ball = [
        diagnose_edge(planner_ball, m, path_ball[i], path_ball[i + 1])
        for i in range(len(path_ball) - 1)
    ]

    n_base_bad = n_bad_hops(diags_base)
    n_ball_bad = n_bad_hops(diags_ball)

    print("=" * 68)
    print(f"BASELINE   ({len(path_base)} waypoints, {len(path_base)-1} hops)")
    for i, p in enumerate(path_base):
        d_wall = distance_to_wall(p)
        marker = ""
        if i > 0 and i - 1 < len(diags_base):
            mc = diags_base[i - 1]["mc"]
            marker = f"   incoming mc={mc:+.3f} m"
        print(f"  B{i}: ({p[0]:.2f}, {p[1]:.2f})  d_wall={d_wall:.2f} m{marker}")
    print(f"  -> {n_base_bad} edge(s) would collide under ballistic criterion")
    print("-" * 68)
    print(f"BALLISTIC  ({len(path_ball)} waypoints, {len(path_ball)-1} hops)")
    for i, p in enumerate(path_ball):
        d_wall = distance_to_wall(p)
        marker = ""
        if i > 0 and i - 1 < len(diags_ball):
            mc = diags_ball[i - 1]["mc"]
            marker = f"   incoming mc={mc:+.3f} m"
        print(f"  A{i}: ({p[0]:.2f}, {p[1]:.2f})  d_wall={d_wall:.2f} m{marker}")
    print(f"  -> {n_ball_bad} edge(s) would collide under ballistic criterion")
    print("=" * 68)

    same = path_base == path_ball
    if same:
        print("\nWARNING: baseline and ballistic paths are identical — the "
              "scenario doesn't force a difference. Consider raising "
              "HOP_RADIUS or V_MAX in this script.")
    else:
        print("\nPaths differ. Ballistic planner routed hops farther from "
              "the wall to keep the parabolic trajectory in the clear.")

    draw_topdown(m, path_base, path_ball, diags_base,
                 out_path("prof_showcase_topdown.png"))
    draw_arc_strip(m, planner_ball, path_base, diags_base,
                   path_ball, diags_ball,
                   out_path("prof_showcase_arcs.png"))

    if matplotlib.get_backend().lower() != "agg":
        plt.show()

    # Non-zero exit if the demo is unconvincing (no divergence OR baseline
    # never collides). Both make the story weak.
    return 0 if (not same and n_base_bad > 0 and n_ball_bad == 0) else 2


if __name__ == "__main__":
    sys.exit(main())

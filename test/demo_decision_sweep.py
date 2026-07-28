"""Decision sweep: one cost model, one geometry, four wall heights — strategy flips.

Every other demo in the suite shows a single outcome, leaving the audience to
infer that one cost model produced all of them.  This one holds the geometry,
the start, the goal and every planner parameter fixed and varies only the wall
height, so the change in behaviour can only come from the cost model.

    x ∈ [2.3, 2.7], y ∈ [0.8, 4.2]   (a 0.4 m thick ridge, bypassable at the ends)
    START (0.5, 2.4) → GOAL (4.5, 2.4)

The wall is plain elevation, not an OBSTACLE sentinel, so landing on its crest is
allowed and costs `alpha_uphill * h` (with `alpha_downhill = 0.0` the descent is
free).  Going around instead costs extra travel.  The planner crosses while the
climb is cheaper and detours once it is not — the flip lands between h=1.2 m and
h=1.4 m with the suite's shared parameters.  Nothing here is tuned per panel.

Figures produced
----------------
  test/decision_sweep.png   — 2x4: top-down path per height, crossing hop below

Run:
    python test/demo_decision_sweep.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import config
from demo_common import (
    HOP_RADIUS, TITLE_FS, LABEL_FS, ANNOT_FS,
    C_BALL, C_COLLIDE, C_ANNOT,
    make_planner, diagnose_path,
    draw_topdown_compact, segment_crosses_region,
    param_caption, save, out_path,
)
from map2d5 import Map2D5
from visualizer import draw_arc_side_view


START = (0.5, 2.4)
GOAL  = (4.5, 2.4)

WALL_XMIN, WALL_XMAX = 2.3, 2.7
WALL_YMIN, WALL_YMAX = 0.8, 4.2

# Chosen to bracket the flip: three crossings then a detour.
HEIGHTS = [0.15, 0.80, 1.20, 1.40]


def build_wall(h: float) -> Map2D5:
    m = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )
    r0, c0 = m.world_to_grid(WALL_XMIN, WALL_YMIN)
    r1, c1 = m.world_to_grid(WALL_XMAX, WALL_YMAX)
    m.grid[r0:r1 + 1, c0:c1 + 1] = h
    return m


def classify(m: Map2D5, path: list, h: float) -> tuple[str, str]:
    """(strategy, detail) for a planned path.

    "AROUND" whenever any waypoint leaves the wall's y-span.  Otherwise the path
    crossed, and the detail records whether it landed on the crest or arced clear
    over it — both are "over", but they are different manoeuvres and the figure
    should not blur them.
    """
    ys = [p[1] for p in path]
    if min(ys) < WALL_YMIN or max(ys) > WALL_YMAX:
        return "AROUND", "detours past the wall end"
    on_crest = any(abs(m.get_elevation(*p) - h) < 1e-9 for p in path)
    return "OVER", ("lands on the crest" if on_crest else "arcs clear over")


def path_cost(planner, m: Map2D5, path: list) -> float:
    """Sum the planner's own edge costs along `path` (includes every penalty)."""
    total = 0.0
    for i in range(len(path) - 1):
        a = m.world_to_grid(*path[i])
        b = m.world_to_grid(*path[i + 1])
        c = planner._validate_and_cost(a, m.grid[a[0], a[1]], b)
        if c is not None:
            total += c
    return total


def crossing_hop(path: list) -> int | None:
    """Index of the hop that actually passes over the wall footprint, if any.

    Must test the full rectangle, not just the x-range: a detour hop can share
    the wall's x-range while passing outside it in y.
    """
    for i in range(len(path) - 1):
        if segment_crosses_region(path[i], path[i + 1],
                                  WALL_XMIN, WALL_XMAX,
                                  WALL_YMIN, WALL_YMAX):
            return i
    return None


def main() -> int:
    results = []
    for h in HEIGHTS:
        m = build_wall(h)
        planner = make_planner(m, False, START, GOAL)
        path = planner.plan()
        if path is None:
            print(f"h={h:.2f}: NO PATH")
            results.append((h, m, planner, None, "NO PATH", "", 0.0, None))
            continue
        strat, detail = classify(m, path, h)
        cost = path_cost(planner, m, path)
        diags = diagnose_path(planner, m, path)
        results.append((h, m, planner, path, strat, detail, cost, diags))
        print(f"h={h:.2f}  {strat:<7} ({detail})  "
              f"{len(path)-1} hops  cost={cost:.2f}  "
              f"climb penalty={config.ALPHA_UPHILL * h:.2f}")

    strategies = [r[4] for r in results]
    if "OVER" not in strategies or "AROUND" not in strategies:
        print("\nWARNING: the sweep did not produce both strategies — "
              "the flip is not being demonstrated.")

    # ---------------- figure ----------------
    n = len(results)
    fig, axes = plt.subplots(2, n, figsize=(4.6 * n, 8.6), squeeze=False)

    for col, (h, m, planner, path, strat, detail, cost, diags) in enumerate(results):
        ax = axes[0, col]
        draw_topdown_compact(m, ax, colorbar_on=fig)
        ax.add_patch(mpatches.Rectangle(
            (WALL_XMIN, WALL_YMIN), WALL_XMAX - WALL_XMIN,
            WALL_YMAX - WALL_YMIN,
            fill=False, edgecolor="k", linewidth=1.8, zorder=5,
        ))

        if path:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            colour = C_BALL if strat == "OVER" else "#4527a0"
            ax.plot(xs, ys, color=colour, linewidth=2.6, zorder=8)
            ax.plot(xs, ys, "o", color=colour, markersize=7, zorder=9)

        ax.plot(*START, "go", markersize=11, zorder=10)
        ax.plot(*GOAL, "r*", markersize=15, zorder=10)

        badge = "#00695c" if strat == "OVER" else "#4527a0"
        ax.set_title(
            f"h = {h:.2f} m\n{strat}  —  {detail}\n"
            f"cost = {cost:.2f}   (climb penalty α·h = {config.ALPHA_UPHILL * h:.2f})",
            fontsize=TITLE_FS - 2, color=badge, fontweight="bold",
        )

        # ---- bottom row: the crossing hop, or why there wasn't one ----
        axb = axes[1, col]
        hi = crossing_hop(path) if path else None
        if hi is not None and diags and diags[hi]["feasible"]:
            d = diags[hi]
            p0, p1 = path[hi], path[hi + 1]
            draw_arc_side_view(
                axb, (p0[0], p0[1], d["z0"]), (p1[0], p1[1], d["z1"]),
                d["alpha_s"], m, config.G_ACCEL, config.ROBOT_RADIUS,
                planner._obstacle_fill, config.ARC_ENDPOINT_EPSILON,
                config.ARC_SAMPLE_MAX_STEP,
            )
            axb.set_ylim(-0.1, max(HEIGHTS) + 0.5)
            axb.axhline(h, color="#e65100", linewidth=1.4, linestyle="--",
                        zorder=5, label=f"wall top z={h:.2f}")
            axb.legend(fontsize=ANNOT_FS - 1, loc="upper right")
            axb.set_xlabel(
                f"crossing hop {hi}: ({p0[0]:.1f},{p0[1]:.1f}) → "
                f"({p1[0]:.1f},{p1[1]:.1f})", fontsize=ANNOT_FS,
            )
        else:
            axb.set_facecolor("#ede7f6")
            axb.text(
                0.5, 0.5,
                "No crossing hop\n\nthe planner detoured past\nthe end of the wall\n\n"
                f"climbing would cost α·h = {config.ALPHA_UPHILL * h:.2f}\n"
                "on top of the travel distance",
                ha="center", va="center", fontsize=ANNOT_FS + 2,
                color="#4527a0", transform=axb.transAxes,
            )
            axb.set_xticks([]); axb.set_yticks([])
            axb.set_xlabel("detour", fontsize=ANNOT_FS)

        if col == 0:
            axb.set_ylabel("z (m)", fontsize=LABEL_FS)

    fig.suptitle(
        "One planner, one geometry — only the wall height changes\n"
        f"Wall x∈[{WALL_XMIN}, {WALL_XMAX}] m, y∈[{WALL_YMIN}, {WALL_YMAX}] m "
        "(bypassable at both ends) · plain elevation, so the crest is landable\n"
        f"{param_caption()}",
        fontsize=TITLE_FS,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    save(fig, out_path("decision_sweep.png"))
    plt.close(fig)

    got_both = "OVER" in strategies and "AROUND" in strategies
    return 0 if got_both else 2


if __name__ == "__main__":
    raise SystemExit(main())

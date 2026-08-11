"""Decision sweep: one cost model, one geometry, four wall heights — strategy flips.

Every other demo in the suite shows a single outcome, leaving the audience to
infer that one cost model produced all of them.  This one holds the geometry,
the start, the goal and every planner parameter fixed and varies only the wall
height, so the change in behaviour can only come from the cost model.

    x ∈ [2.3, 2.7], y ∈ [0.8, 4.2]   (a 0.4 m thick ridge, bypassable at the ends)
    START (0.5, 2.4) → GOAL (4.5, 2.4)

The wall is plain elevation, not an OBSTACLE sentinel, so landing on its crest is
allowed.  Crossing costs ENERGY — the clearance gate lifts the takeoff angle, a
steeper arc needs a faster launch, and the propellers pay for the difference.
Going around costs extra travel instead.  The planner crosses while the energy is
cheaper than the detour and detours once it is not; the flip lands between
h = 0.90 m and h = 1.20 m at the suite's shared parameters.  Nothing here is tuned
per panel.

What makes this demo worth keeping is that neither side of that comparison is a
tuned weight.  The edge cost is

    xy_dist + W_ENERGY * (e_inject + momentum thrown away)

and `W_ENERGY` is derived rather than chosen: it is set so one flat steady-state
hop's energy cost equals its distance cost (see `config.W_ENERGY`).  The flip
point is therefore a prediction of the robot's physics, not a knob setting.

This replaced an elevation penalty, `alpha_uphill * dz`, under which the flip sat
between h = 1.20 m and h = 1.40 m.  That penalty could not see this scenario
properly: a hop that arcs OVER the wall and lands on the flat on the far side has
`dz = 0` and was charged nothing at all, so only paths that landed on the crest
were ever priced.  The flip moved down because arcing over is now paid for too —
and every panel below now crosses by arcing clear rather than by landing on the
crest, which is the behaviour the old penalty was blind to.

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
    TITLE_FS, LABEL_FS, ANNOT_FS,
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

# Chosen to bracket the flip: three crossings then a detour. Re-picked when the
# cost model moved from `alpha_uphill * dz` to energy — under the old penalty
# these were [0.15, 0.80, 1.20, 1.40].
#
# Note these are calibrated against `demo_common.HOP_RADIUS` (1.5 m), the deck
# default `make_planner` uses, NOT `config.HOP_RADIUS` (1.0 m). The flip moves a
# long way with reach: at 1.0 m it sits between 0.45 and 0.60 m instead, because
# a shorter run-up buys less height for the same energy.
HEIGHTS = [0.30, 0.60, 0.90, 1.20]


def build_wall(h: float) -> Map2D5:
    """The ridge, painted in world metres.

    `paint_region` rather than a `world_to_grid` slice with an inclusive `+1`:
    that form overshoots by up to one cell and silently rescales the physical
    wall when `CELL_RESOLUTION` changes — which would move the flip point this
    demo exists to locate.
    """
    m = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )
    m.paint_region(
        h,
        x_min=WALL_XMIN, x_max=WALL_XMAX,
        y_min=WALL_YMIN, y_max=WALL_YMAX,
    )
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


def path_cost(planner, m: Map2D5, path: list) -> dict:
    """Sum the planner's own edge costs along `path`, split into its three terms.

    Returns `{"cost", "dist", "inject", "momentum"}` — the total, and the parts
    it decomposes into. The split is what makes the figure legible: it shows
    whether a crossing was paid for with propeller work or with speed.

    Chained, not mapped: `_validate_and_cost` needs the speed the robot arrived
    with, and it returns the speed it leaves with. Feeding each hop the
    start-of-chain energy instead would price hops the robot could not fly.
    """
    out = {"cost": 0.0, "dist": 0.0, "inject": 0.0, "momentum": 0.0}
    v_g = planner.v_g_initial
    for i in range(len(path) - 1):
        a = m.world_to_grid(*path[i])
        b = m.world_to_grid(*path[i + 1])
        edge = planner._validate_and_cost(a, m.grid[a[0], a[1]], b, v_g)
        if edge is None:
            continue
        cost, hop = edge
        v_out = planner._speed_bin(hop["v_g"]) * planner.speed_bin
        out["cost"] += cost
        out["dist"] += hop["X"]
        out["inject"] += hop["e_inject"]
        out["momentum"] += max(
            0.0,
            0.5 * planner.mass * (hop["v_g_in"] ** 2 - v_out ** 2),
        )
        v_g = hop["v_g"]
    return out


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
    print(param_caption())
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
              f"{len(path)-1} hops  cost={cost['cost']:.2f}  "
              f"= {cost['dist']:.2f} m travel + {config.W_ENERGY} × "
              f"({cost['inject']:.2f} J thrust + {cost['momentum']:.2f} J momentum)")

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
        # Kept to four short lines rather than one long one: at 4.6 in per
        # column, a single-line cost breakdown overruns the axes and collides
        # with its neighbours.
        ax.set_title(
            f"h = {h:.2f} m  —  {strat}\n{detail}\n"
            f"cost {cost['cost']:.2f} = {cost['dist']:.2f} m travel\n"
            f"+ {config.W_ENERGY}×({cost['inject']:.2f} J thrust "
            f"+ {cost['momentum']:.2f} J momentum)",
            color=badge, fontweight="bold", fontsize=9,
        )

        # ---- bottom row: the crossing hop, or why there wasn't one ----
        axb = axes[1, col]
        hi = crossing_hop(path) if path else None
        if hi is not None and diags and diags[hi]["feasible"]:
            d = diags[hi]
            p0, p1 = path[hi], path[hi + 1]
            draw_arc_side_view(
                axb, (p0[0], p0[1], d["z0"]), (p1[0], p1[1], d["z1"]),
                d["alpha_s"], m, config.ROBOT_RADIUS, config.LEG_LENGTH,
                planner._obstacle_fill, config.ARC_SAMPLE_MAX_STEP,
                min_clearance_gate=config.MIN_CLEARANCE,
                n_lateral=config.ARC_LATERAL_SAMPLES,
            )
            axb.set_ylim(-0.1, max(HEIGHTS) + 0.5)
            axb.axhline(h, color="#e65100", linewidth=1.4, linestyle="--",
                        zorder=5, label=f"wall top z={h:.2f}")
            # `draw_arc_side_view` labels four series with long names; at the
            # default font the legend is wider than a 4.6 in column and spills
            # over the panel to its left.
            axb.legend(loc="upper right", fontsize=6, framealpha=0.9)
            axb.set_xlabel(
                f"crossing hop {hi}: ({p0[0]:.1f},{p0[1]:.1f}) → "
                f"({p1[0]:.1f},{p1[1]:.1f})",
            )
        else:
            axb.set_facecolor("#ede7f6")
            straight = math.hypot(GOAL[0] - START[0], GOAL[1] - START[1])
            axb.text(
                0.5, 0.5,
                "No crossing hop\n\n"
                "the planner detoured past the wall end\n\n"
                f"clearing {h:.2f} m needs a steeper takeoff\n"
                f"than the extra {cost['dist'] - straight:.1f} m of travel costs",
                ha="center", va="center", fontsize=9,
                color="#4527a0", transform=axb.transAxes,
            )
            axb.set_xticks([]); axb.set_yticks([])
            axb.set_xlabel("detour")

        if col == 0:
            axb.set_ylabel("z (m)")

    fig.suptitle(
        "One planner, one geometry — only the wall height changes\n"
        f"Wall x∈[{WALL_XMIN}, {WALL_XMAX}] m, y∈[{WALL_YMIN}, {WALL_YMAX}] m "
        "(bypassable at both ends) · plain elevation, so the crest is landable\n"
        f"cost = travel + {config.W_ENERGY} m/J × (thrust + momentum thrown away)"
        " — no tuned height weight", wrap=True,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.88))  # 3-line suptitle + 4-line subtitles
    save(fig, out_path("decision_sweep.png"))
    plt.close(fig)

    got_both = "OVER" in strategies and "AROUND" in strategies
    return 0 if got_both else 2


if __name__ == "__main__":
    raise SystemExit(main())

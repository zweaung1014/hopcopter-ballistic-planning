"""Professor showcase — narrow wall: ballistic planner backs the takeoff off the wall.

Unlike the tall-wall demo (where the wall is too wide to arc over and the
planner goes *around* in y), the narrow wall here cannot be bypassed in y
either.  The robot must cross it in the x direction.

The baseline planner (clearance and stance OFF) picks the takeoff cell
closest to the wall.  The ballistic planner rejects that cell — usually
on stance, since the body + upper leg cylinder cannot rest that close —
and picks a cell one grid step further back.  Runtime coordinates and
`mc` values are printed to the console and drawn on the figures.

Two figures:
  test/prof_narrow_wall_topdown.png  — A/B overlay on the map
  test/prof_narrow_wall_arcs.png     — per-hop side-view strip

Run:
    python test/demo_narrow_wall_showcase.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

import config
from demo_common import (
    N_ANGLES, V_MAX,
    PRESENTATION_DPI, TITLE_FS, LABEL_FS, ANNOT_FS,
    make_planner, diagnose_edge, n_bad_hops, param_caption, save, out_path
)
from hopping_astar_planner import HoppingAStarPlanner
from map2d5 import Map2D5
from maps.tall_narrow_wall import (
    build as build_map,
    WALL_HEIGHT, WALL_XMIN, WALL_XMAX, WALL_YMIN, WALL_YMAX,
)
from visualizer import Visualizer, draw_arc_side_view


# --- Scenario knobs (HOP_RADIUS / N_ANGLES / V_MAX come from demo_common so
# every figure in the deck quotes the same physics) ----------------------- #
START         = (0.5, 2.4)
GOAL          = (4.5, 2.4)
H_CLEAR       = WALL_HEIGHT + config.ROBOT_RADIUS   # 0.25 m clearance threshold


def distance_to_wall(p: tuple[float, float]) -> float:
    dx = max(WALL_XMIN - p[0], 0.0, p[0] - WALL_XMAX)
    dy = max(WALL_YMIN - p[1], 0.0, p[1] - WALL_YMAX)
    return math.hypot(dx, dy)


def wall_crossing_hop(path: list, m: Map2D5) -> int | None:
    """Return index i of the first hop whose x-range overlaps [WALL_XMIN, WALL_XMAX]."""
    for i in range(len(path) - 1):
        x0, x1 = path[i][0], path[i + 1][0]
        x_lo, x_hi = min(x0, x1), max(x0, x1)
        if x_lo < WALL_XMAX and x_hi > WALL_XMIN:
            return i
    return None


# ---------------------------------------------------------------------------
# Figure 1: top-down A/B overlay
# ---------------------------------------------------------------------------

def draw_topdown(m: Map2D5, path_base, path_ball, diags_base, save_path: str):
    fig, ax = plt.subplots(figsize=(10, 9))
    vis = Visualizer.__new__(Visualizer)
    vis.map_env = m; vis.fig = fig; vis.ax = ax
    ax.set_xlim(0, m.size_x); ax.set_ylim(0, m.size_y)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    vis.draw_map()

    # Wall outline
    ax.add_patch(mpatches.Rectangle(
        (WALL_XMIN, WALL_YMIN), WALL_XMAX - WALL_XMIN, WALL_YMAX - WALL_YMIN,
        fill=False, edgecolor="k", linewidth=1.6, zorder=5,
    ))

    # Baseline path (red-orange dashed, X markers)
    xs_b = [p[0] for p in path_base]; ys_b = [p[1] for p in path_base]
    ax.plot(xs_b, ys_b, color="#e65100", linewidth=2.0, linestyle="--",
            zorder=6, label="Baseline (clearance OFF)")
    ax.plot(xs_b, ys_b, "x", color="#e65100", markersize=10, zorder=7)

    # Highlight colliding baseline edges
    for i, d in enumerate(diags_base):
        if d["feasible"] and d["mc"] < config.MIN_CLEARANCE:
            p0 = path_base[i]; p1 = path_base[i + 1]
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]],
                    color="#d50000", linewidth=9.0, alpha=0.65, zorder=6.5,
                    solid_capstyle="round")
            mx, my = 0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1])
            ax.annotate(
                f"COLLIDES\nmc={d['mc']:+.3f} m",
                (mx, my), xytext=(0, 22), textcoords="offset points",
                ha="center", color="#b71c1c", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.25", fc="white",
                          ec="#b71c1c", lw=1.2), zorder=11,
            )

    # Ballistic path (teal solid, o markers)
    xs_a = [p[0] for p in path_ball]; ys_a = [p[1] for p in path_ball]
    ax.plot(xs_a, ys_a, color="#00695c", linewidth=2.2, zorder=8,
            label="Ballistic (clearance ON)")
    ax.plot(xs_a, ys_a, "o", color="#00695c", markersize=7, zorder=9)

    # Annotate each waypoint with index and distance to wall
    for i, p in enumerate(path_base):
        d = distance_to_wall(p)
        ax.annotate(f"B{i}\n{d:.2f}m", p, xytext=(6, -14),
                    textcoords="offset points", color="#e65100")
    for i, p in enumerate(path_ball):
        d = distance_to_wall(p)
        ax.annotate(f"A{i}\n{d:.2f}m", p, xytext=(6, 6),
                    textcoords="offset points", color="#00695c")

    # Arrow between the two takeoff cells. No caption — the B{i} and A{i}
    # markers are already labelled with coordinates.
    base_idx = wall_crossing_hop(path_base, m)
    ball_idx = wall_crossing_hop(path_ball, m)
    if base_idx is not None and ball_idx is not None:
        bx, by = path_base[base_idx]
        ax_x, ay = path_ball[ball_idx]
        if abs(bx - ax_x) > 0.05:
            ax.annotate(
                "",
                xy=(ax_x, ay), xytext=(bx, ay),
                arrowprops=dict(arrowstyle="->", color="#1565c0", lw=1.8),
            )

    ax.plot(START[0], START[1], "go", markersize=11, zorder=10, label="start")
    ax.plot(GOAL[0], GOAL[1],   "r*", markersize=15, zorder=10, label="goal")

    handles, labels = ax.get_legend_handles_labels()
    handles.append(mpatches.Patch(color="#d50000", alpha=0.65,
                                  label="baseline edge that would collide"))
    labels.append("baseline edge that would collide")
    ax.legend(handles, labels, loc="upper left")

    ax.set_title(
        f"tall_narrow_wall demo: baseline (clearance off) vs ballistic (clearance on)\n"
        f"wall at x=[{WALL_XMIN},{WALL_XMAX}] h={WALL_HEIGHT:.2f} m  |  "
        f"clearance threshold = {H_CLEAR:.2f} m\n"
        f"ballistic rejects the closer takeoff — its body or leg would clip the wall — "
        f"and picks one grid step further back", wrap=True
    )
    fig.tight_layout()
    save(fig, save_path)


# ---------------------------------------------------------------------------
# Figure 2: per-hop side-view strip with clearance threshold line
# ---------------------------------------------------------------------------

def draw_arc_strip(
    m: Map2D5,
    ballistic_planner: HoppingAStarPlanner,
    path_base, diags_base,
    path_ball, diags_ball,
    save_path: str,
):
    n_base = len(path_base) - 1
    n_ball = len(path_ball) - 1
    ncols  = max(n_base, n_ball)
    fig, axes = plt.subplots(2, ncols, figsize=(3.6 * ncols, 5.6),
                             squeeze=False)

    ymax = WALL_HEIGHT + 0.6

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
                ax.text(0.5, 0.5, "INFEASIBLE", ha="center", va="center", transform=ax.transAxes)
                ax.set_xticks([]); ax.set_yticks([])
            else:
                p0 = path[i]; p1 = path[i + 1]
                draw_arc_side_view(
                    ax,
                    (p0[0], p0[1], d["z0"]), (p1[0], p1[1], d["z1"]),
                    d["alpha_s"], m,
                    config.ROBOT_RADIUS, config.LEG_LENGTH,
                    config.ARC_SAMPLE_MAX_STEP,
                    min_clearance_gate=config.MIN_CLEARANCE,
                )
                ax.set_ylim(-0.05, ymax)

                # Clearance-threshold horizontal dashed line (only for hops
                # that cross the wall's x range)
                x0r, x1r = min(p0[0], p1[0]), max(p0[0], p1[0])
                if x0r < WALL_XMAX and x1r > WALL_XMIN:
                    ax.axhline(H_CLEAR, color="#f57f17", linewidth=1.2,
                               linestyle=":", zorder=4)
                    ax.text(0.02, H_CLEAR + 0.01, f"threshold={H_CLEAR:.2f}m",
                            transform=ax.get_yaxis_transform(), color="#f57f17", va="bottom")
                    # Annotate the u_enter clearance value
                    u_enter = abs(WALL_XMIN - p0[0])
                    X = d["X"]
                    z_at_entry = u_enter * (1.0 - u_enter / X) if X > 1e-6 else 0.0
                    ax.annotate(
                        f"z(u={u_enter:.2f})={z_at_entry:.3f}m",
                        xy=(WALL_XMIN, z_at_entry),
                        xytext=(WALL_XMIN + 0.05, z_at_entry + 0.06),
                        arrowprops=dict(arrowstyle="->", lw=0.8,
                                        color="gray"),
                    )

            if i == 0:
                ax.set_ylabel(f"{label}\nz (m)")
            ax.set_xlabel(
                f"hop {i}: ({path[i][0]:.1f},{path[i][1]:.1f})"
                f" → ({path[i+1][0]:.1f},{path[i+1][1]:.1f})",
            )

    fig.suptitle(
        f"Per-hop side view: baseline (top) vs ballistic (bottom).\n"
        f"Orange dashed = clearance threshold {H_CLEAR:.2f} m.  "
        f"Red arc = below the {config.MIN_CLEARANCE} m clearance gate.  Green arc = clears.  "
        f"Both judged by the same criterion.", wrap=True
    )
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    save(fig, save_path)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    print(param_caption())
    m = build_map()

    planner_base = make_planner(m, True, START, GOAL)
    planner_ball = make_planner(m, False, START, GOAL)

    path_base = planner_base.plan()
    path_ball = planner_ball.plan()

    if path_base is None or path_ball is None:
        print(f"Baseline path: {'FOUND' if path_base else 'NONE'}")
        print(f"Ballistic path: {'FOUND' if path_ball else 'NONE'}")
        print("One planner found no path — cannot compare.")
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

    print("=" * 72)
    print(f"BASELINE   ({len(path_base)} waypoints, {len(path_base)-1} hops)")
    for i, p in enumerate(path_base):
        d_wall = distance_to_wall(p)
        note = ""
        if i > 0 and i - 1 < len(diags_base):
            d = diags_base[i - 1]
            note = f"   incoming mc={d['mc']:+.3f} m"
        print(f"  B{i}: ({p[0]:.2f},{p[1]:.2f})  d_wall={d_wall:.2f} m{note}")
    print(f"  → {n_base_bad} hop(s) would collide with the wall")

    print("-" * 72)
    print(f"BALLISTIC  ({len(path_ball)} waypoints, {len(path_ball)-1} hops)")
    for i, p in enumerate(path_ball):
        d_wall = distance_to_wall(p)
        note = ""
        if i > 0 and i - 1 < len(diags_ball):
            d = diags_ball[i - 1]
            note = f"   incoming mc={d['mc']:+.3f} m"
        print(f"  A{i}: ({p[0]:.2f},{p[1]:.2f})  d_wall={d_wall:.2f} m{note}")
    print(f"  → {n_ball_bad} hop(s) would collide with the wall")
    print("=" * 72)

    # Check x-shift of wall-crossing takeoff
    base_hi = wall_crossing_hop(path_base, m)
    ball_hi = wall_crossing_hop(path_ball, m)
    if base_hi is not None and ball_hi is not None:
        bx = path_base[base_hi][0]
        ax_x = path_ball[ball_hi][0]
        shift = bx - ax_x
        if shift > 0.0:
            # Say *which* gate rejected the baseline's takeoff cell rather than
            # assuming it was clearance. On this map it is usually stance: the
            # baseline parks close enough that the robot's body would overlap
            # the wall while standing, even though its arc clears comfortably.
            d_base = diagnose_edge(planner_ball, m,
                                   path_base[base_hi - 1], path_base[base_hi]) \
                if base_hi > 0 else None
            cell = m.world_to_grid(bx, path_base[base_hi][1])
            standable = bool(planner_ball._standable[cell[0], cell[1]])
            print(f"\nBallistic takeoff (x={ax_x:.2f}) is {shift:.2f} m LEFT "
                  f"of baseline takeoff (x={bx:.2f}).")
            if not standable:
                print(f"  Reason: the baseline's cell is not STANDABLE — the wall "
                      f"face is {WALL_XMIN - bx:.2f} m away and the body has "
                      f"radius {config.ROBOT_RADIUS} m, leaving "
                      f"{WALL_XMIN - bx - config.ROBOT_RADIUS:+.2f} m, under the "
                      f"{config.MIN_CLEARANCE} m gate.")
                if d_base is not None:
                    print(f"  Note its ARC was fine (mc={d_base['mc']:+.3f} m) — "
                          f"what fails is the standing pose, not the trajectory.")
            else:
                print(f"  Reason: the baseline's crossing arc fails the "
                      f"{config.MIN_CLEARANCE} m clearance gate.")
            print("  Caveat: only the nearest standable cell is forced. Among "
                  "equally-distant standable takeoffs the total cost ties "
                  "exactly, so the precise landing cell is A*'s tie-break.")
        elif shift < 0.0:
            print(f"\nWARNING: ballistic takeoff (x={ax_x:.2f}) is to the RIGHT "
                  f"of baseline (x={bx:.2f}) — unexpected geometry.")
        else:
            print("\nWARNING: baseline and ballistic use the same takeoff x-coordinate.")

    same = (path_base == path_ball)
    if same:
        print("\nWARNING: paths are identical — wall height may need adjusting.")

    draw_topdown(m, path_base, path_ball, diags_base,
                 out_path("prof_narrow_wall_topdown.png"))
    draw_arc_strip(m, planner_ball,
                   path_base, diags_base, path_ball, diags_ball,
                   out_path("prof_narrow_wall_arcs.png"))

    if matplotlib.get_backend().lower() != "agg":
        plt.show()

    return 0 if (not same and n_base_bad > 0 and n_ball_bad == 0) else 2


if __name__ == "__main__":
    sys.exit(main())

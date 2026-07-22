"""Barely-jumpable wall demo: ballistic planner finds the one valid takeoff cell.

Wall height = 0.22 m — only 0.055 m of clearance remains at the optimal
takeoff x = 1.7 m (wall at arc midpoint).  One grid cell (0.2 m) either
side of that position fails clearance entirely (mc ≈ −0.02 m).

The baseline planner (no clearance) charges straight through (takeoff at
x = 2.1 m, mc ≈ −0.145 m).  The ballistic planner backs off 0.4 m to
x = 1.7 m so the wall lands at u = 0.6 m–1.0 m — symmetric around the
arc apex at u = 0.8 m.

Figures produced
----------------
  test/barely_jumpable_topdown.png   — top-down A/B path overlay
  test/barely_jumpable_arcs.png      — per-hop side-view strip (both planners)
  test/barely_jumpable_crossing.png  — focused side view of wall-crossing hop

Run:
    python test/demo_barely_jumpable.py
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
from hopping_astar_planner import (
    HoppingAStarPlanner,
    feasible_alpha_interval,
    min_clearance,
)
from map2d5 import Map2D5
from maps.barely_jumpable_wall import (
    build as build_map,
    WALL_HEIGHT, WALL_XMIN, WALL_XMAX, WALL_YMIN, WALL_YMAX,
)
from visualizer import Visualizer, draw_arc_side_view


# ---------------------------------------------------------------------------
# Scenario knobs
# ---------------------------------------------------------------------------
START      = (0.5, 2.4)
GOAL       = (4.5, 2.4)
HOP_RADIUS = 1.5
N_ANGLES   = 16
V_MAX      = 6.0

# Height the arc centre must clear: wall top + robot body radius.
H_CLEAR = WALL_HEIGHT + config.ROBOT_RADIUS   # 0.32 m


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_planner(m: Map2D5, disable_clearance: bool) -> HoppingAStarPlanner:
    return HoppingAStarPlanner(
        map_env=m,
        start=START,
        goal=GOAL,
        hop_radius=HOP_RADIUS,
        n_angles=N_ANGLES,
        max_jump_height=config.MAX_JUMP_HEIGHT,
        alpha_uphill=config.ALPHA_UPHILL,
        alpha_downhill=config.ALPHA_DOWNHILL,
        g=config.G_ACCEL,
        V_max=V_MAX,
        robot_radius=config.ROBOT_RADIUS,
        clearance_margin=config.CLEARANCE_MARGIN,
        clearance_weight=config.CLEARANCE_WEIGHT,
        arc_max_step=config.ARC_SAMPLE_MAX_STEP,
        arc_endpoint_epsilon=config.ARC_ENDPOINT_EPSILON,
        obstacle_wall_extra=config.OBSTACLE_WALL_EXTRA,
        disable_clearance=disable_clearance,
    )


def diagnose_edge(
    ballistic_planner: HoppingAStarPlanner,
    m: Map2D5,
    p0: tuple[float, float],
    p1: tuple[float, float],
) -> dict:
    z0 = float(m.get_elevation(*p0))
    z1 = float(m.get_elevation(*p1))
    X  = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    Z  = z1 - z0
    iv = feasible_alpha_interval(X, Z, V_MAX, config.G_ACCEL)
    if iv is None:
        return {"feasible": False, "X": X, "Z": Z, "alpha_s": None,
                "mc": -math.inf, "z0": z0, "z1": z1}
    a  = 0.5 * (iv[0] + iv[1])
    mc = min_clearance(
        (p0[0], p0[1], z0), (p1[0], p1[1], z1), a,
        m, config.G_ACCEL, config.ROBOT_RADIUS,
        config.ARC_ENDPOINT_EPSILON, config.ARC_SAMPLE_MAX_STEP,
        ballistic_planner._obstacle_fill,
    )
    return {"feasible": True, "X": X, "Z": Z, "alpha_s": a, "mc": mc,
            "z0": z0, "z1": z1}


def distance_to_wall(p: tuple[float, float]) -> float:
    dx = max(WALL_XMIN - p[0], 0.0, p[0] - WALL_XMAX)
    dy = max(WALL_YMIN - p[1], 0.0, p[1] - WALL_YMAX)
    return math.hypot(dx, dy)


def wall_crossing_hop(path: list, _m: Map2D5) -> int | None:
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

def draw_topdown(
    m: Map2D5,
    path_base: list,
    path_ball: list,
    diags_base: list,
    save_path: str,
) -> None:
    fig, ax = plt.subplots(figsize=(10, 9))
    vis = Visualizer.__new__(Visualizer)
    vis.map_env = m
    vis.fig = fig
    vis.ax = ax
    ax.set_xlim(0, m.size_x)
    ax.set_ylim(0, m.size_y)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    vis.draw_map()

    # Wall outline
    ax.add_patch(mpatches.Rectangle(
        (WALL_XMIN, WALL_YMIN), WALL_XMAX - WALL_XMIN, WALL_YMAX - WALL_YMIN,
        fill=False, edgecolor="k", linewidth=1.6, zorder=5,
    ))

    # Baseline path (red-orange dashed, X markers)
    xs_b = [p[0] for p in path_base]
    ys_b = [p[1] for p in path_base]
    ax.plot(xs_b, ys_b, color="#e65100", linewidth=2.0, linestyle="--",
            zorder=6, label="Baseline (clearance OFF)")
    ax.plot(xs_b, ys_b, "x", color="#e65100", markersize=10, zorder=7)

    # Highlight baseline hops that clip the wall
    for i, d in enumerate(diags_base):
        if d["feasible"] and d["mc"] < 0.0:
            p0 = path_base[i]
            p1 = path_base[i + 1]
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]],
                    color="#d50000", linewidth=9.0, alpha=0.65, zorder=6.5,
                    solid_capstyle="round")
            mx = 0.5 * (p0[0] + p1[0])
            my = 0.5 * (p0[1] + p1[1])
            ax.annotate(
                f"COLLIDES\nmc={d['mc']:+.3f} m",
                (mx, my), xytext=(0, 24), textcoords="offset points",
                ha="center", fontsize=9, color="#b71c1c", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.25", fc="white",
                          ec="#b71c1c", lw=1.2), zorder=11,
            )

    # Ballistic path (teal solid, o markers)
    xs_a = [p[0] for p in path_ball]
    ys_a = [p[1] for p in path_ball]
    ax.plot(xs_a, ys_a, color="#00695c", linewidth=2.2, zorder=8,
            label="Ballistic (clearance ON)")
    ax.plot(xs_a, ys_a, "o", color="#00695c", markersize=7, zorder=9)

    # Waypoint labels
    for i, p in enumerate(path_base):
        ax.annotate(f"B{i}\n{distance_to_wall(p):.2f}m", p,
                    xytext=(6, -14), textcoords="offset points",
                    fontsize=7, color="#e65100")
    for i, p in enumerate(path_ball):
        ax.annotate(f"A{i}\n{distance_to_wall(p):.2f}m", p,
                    xytext=(6, 6), textcoords="offset points",
                    fontsize=7, color="#00695c")

    # Arrow showing x-shift at crossing
    base_idx = wall_crossing_hop(path_base, m)
    ball_idx = wall_crossing_hop(path_ball, m)
    if base_idx is not None and ball_idx is not None:
        bx, by = path_base[base_idx]
        ax_x, ay = path_ball[ball_idx]
        shift = bx - ax_x
        if abs(shift) > 0.05:
            ax.annotate(
                f"← {shift:.2f} m left\n(ballistic steps back\nfor arc clearance)",
                xy=(ax_x, ay), xytext=(bx + 0.15, ay + 0.50),
                arrowprops=dict(arrowstyle="->", color="#1565c0", lw=1.5),
                fontsize=9, color="#1565c0", fontweight="bold",
            )

    ax.plot(START[0], START[1], "go", markersize=11, zorder=10, label="start")
    ax.plot(GOAL[0], GOAL[1],   "r*", markersize=15, zorder=10, label="goal")

    handles, labels = ax.get_legend_handles_labels()
    handles.append(mpatches.Patch(color="#d50000", alpha=0.65))
    labels.append("baseline hop clips wall (mc<0)")
    ax.legend(handles, labels, loc="upper left", fontsize=8)
    ax.set_title(
        f"barely_jumpable_wall: baseline vs ballistic\n"
        f"wall x=[{WALL_XMIN},{WALL_XMAX}] h={WALL_HEIGHT} m  |  "
        f"clearance threshold H_clear={H_CLEAR:.2f} m\n"
        f"ballistic backs off 0.4 m so wall falls at arc midpoint (u=0.6–1.0 m)",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(save_path, dpi=110)
    print(f"Saved: {save_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 2: per-hop side-view strip (both planners)
# ---------------------------------------------------------------------------

def draw_arc_strip(
    m: Map2D5,
    ballistic_planner: HoppingAStarPlanner,
    path_base: list,
    diags_base: list,
    path_ball: list,
    diags_ball: list,
    save_path: str,
) -> None:
    n_base = len(path_base) - 1
    n_ball = len(path_ball) - 1
    ncols  = max(n_base, n_ball)
    fig, axes = plt.subplots(2, ncols, figsize=(3.8 * ncols, 5.8), squeeze=False)

    obs_fill = ballistic_planner._obstacle_fill
    ymax     = max(WALL_HEIGHT + 0.35, 0.55)

    for row, (row_label, path, diags) in enumerate([
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
                ax.text(0.5, 0.5, "INFEASIBLE", ha="center", va="center",
                        fontsize=9, transform=ax.transAxes)
                ax.set_xticks([])
                ax.set_yticks([])
            else:
                p0 = path[i]
                p1 = path[i + 1]
                draw_arc_side_view(
                    ax,
                    (p0[0], p0[1], d["z0"]), (p1[0], p1[1], d["z1"]),
                    d["alpha_s"], m,
                    config.G_ACCEL, config.ROBOT_RADIUS, obs_fill,
                    config.ARC_ENDPOINT_EPSILON, config.ARC_SAMPLE_MAX_STEP,
                )
                ax.set_ylim(-0.05, ymax)

                # Threshold line for wall-crossing hops only
                x_lo = min(p0[0], p1[0])
                x_hi = max(p0[0], p1[0])
                if x_lo < WALL_XMAX and x_hi > WALL_XMIN:
                    ax.axhline(H_CLEAR, color="#f57f17", linewidth=1.2,
                               linestyle=":", zorder=4)
                    ax.text(0.02, H_CLEAR + 0.01, f"H_clear={H_CLEAR:.2f}m",
                            transform=ax.get_yaxis_transform(),
                            fontsize=7, color="#f57f17", va="bottom")

            if i == 0:
                ax.set_ylabel(f"{row_label}\nz (m)", fontsize=10)
            ax.set_xlabel(
                f"hop {i}: ({path[i][0]:.1f},{path[i][1]:.1f})"
                f" → ({path[i+1][0]:.1f},{path[i+1][1]:.1f})",
                fontsize=8,
            )

    fig.suptitle(
        f"Per-hop side view — baseline (top) vs ballistic (bottom)\n"
        f"Orange dotted = H_clear={H_CLEAR:.2f} m.  "
        f"Red arc = clips wall (mc<0).  Green arc = clears.",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.91))
    fig.savefig(save_path, dpi=110)
    print(f"Saved: {save_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 3: focused side view of the wall-crossing hop (baseline vs ballistic)
# ---------------------------------------------------------------------------

def _add_wall_markings(
    ax,
    p0: tuple[float, float],
    hop_X: float,
    hop_Z: float,
) -> None:
    """Overlay wall-height, clearance-threshold, and wall-span markings."""
    # Horizontal reference lines
    ax.axhline(
        WALL_HEIGHT, color="#e65100", linewidth=1.6, linestyle="--", zorder=5,
        label=f"Wall top  z = {WALL_HEIGHT:.2f} m",
    )
    ax.axhline(
        H_CLEAR, color="#f57f17", linewidth=1.2, linestyle=":", zorder=5,
        label=f"H_clear = {H_CLEAR:.2f} m  (wall + robot radius)",
    )

    # Shade the wall's u-range on this hop's u-axis (east-only projection).
    # For purely east hops (cos_t = 1) this is exact; for diagonal hops it is
    # the x-projected shadow, which is the relevant constraint plane.
    dx_hop = p0[0]   # takeoff x — used to compute u offsets
    if hop_X > 1e-9:
        # Approximate horizontal-distance coordinates of wall entry / exit
        # assuming the hop direction has a positive x component (east or NE).
        # We project by X / dx_physical which equals 1 / cos_theta.
        dx_physical = abs(p0[0])  # not needed — easier:
        # u at which world-x equals WALL_XMIN / WALL_XMAX along this segment:
        # u = (target_x - p0_x) / cos_theta = (target_x - p0_x) * X / dx_hop
        # We already have p0 as the raw p0 tuple passed in below; reuse directly.
    pass  # markings added by caller using the p0 arg


def draw_crossing_comparison(
    m: Map2D5,
    planner_base: HoppingAStarPlanner,
    planner_ball: HoppingAStarPlanner,
    path_base: list,
    diags_base: list,
    path_ball: list,
    diags_ball: list,
    save_path: str,
) -> None:
    """Two-panel side view of the wall-crossing hop: baseline (top), ballistic (bottom)."""

    base_hi = wall_crossing_hop(path_base, m)
    ball_hi = wall_crossing_hop(path_ball, m)

    rows: list[tuple[str, list, list, int, HoppingAStarPlanner]] = []
    if base_hi is not None and diags_base[base_hi]["feasible"]:
        rows.append(("Baseline  (clearance OFF)", path_base, diags_base,
                     base_hi, planner_base))
    if ball_hi is not None and diags_ball[ball_hi]["feasible"]:
        rows.append(("Ballistic  (clearance ON)", path_ball, diags_ball,
                     ball_hi, planner_ball))

    if not rows:
        print("WARNING: no feasible wall-crossing hop found — skipping Figure 3.")
        return

    fig, axes = plt.subplots(len(rows), 1,
                             figsize=(10, 4.8 * len(rows)),
                             squeeze=False)

    for ax, (panel_label, path, diags, hi, planner) in zip(axes[:, 0], rows):
        d = diags[hi]
        p0 = path[hi]
        p1 = path[hi + 1]
        c_s = (p0[0], p0[1], d["z0"])
        c_g = (p1[0], p1[1], d["z1"])

        # Core arc + terrain plot
        draw_arc_side_view(
            ax, c_s, c_g, d["alpha_s"], m,
            config.G_ACCEL, config.ROBOT_RADIUS, planner._obstacle_fill,
            config.ARC_ENDPOINT_EPSILON, config.ARC_SAMPLE_MAX_STEP,
            label=f"Parabolic arc   mc = {d['mc']:+.4f} m",
        )

        # Wall-top line and clearance-threshold line
        ax.axhline(WALL_HEIGHT, color="#e65100", linewidth=1.8,
                   linestyle="--", zorder=6,
                   label=f"Wall top  z = {WALL_HEIGHT:.2f} m")
        ax.axhline(H_CLEAR, color="#f57f17", linewidth=1.3,
                   linestyle=":", zorder=6,
                   label=f"H_clear = {H_CLEAR:.2f} m  (wall + robot radius)")

        # Shade the u-range that lies over the wall cells.
        # u = horizontal distance along the hop direction from takeoff.
        # For an east hop (cos θ = 1) this equals Δx directly.
        X    = d["X"]
        dx_h = p1[0] - p0[0]   # signed x displacement of this hop
        dy_h = p1[1] - p0[1]
        if X > 1e-9 and abs(dx_h) > 1e-6:
            cos_t = dx_h / X
            u_wall_lo = (WALL_XMIN - p0[0]) / cos_t
            u_wall_hi = (WALL_XMAX - p0[0]) / cos_t
            u_lo = max(0.0, min(u_wall_lo, u_wall_hi))
            u_hi = min(X,   max(u_wall_lo, u_wall_hi))
            if u_lo < u_hi:
                ax.axvspan(u_lo, u_hi, alpha=0.15, color="#ff8f00",
                           zorder=2,
                           label=f"Wall  x=[{WALL_XMIN},{WALL_XMAX}] m")

                # Annotate the clearance gap at the tightest point (wall entry)
                u_tight = u_lo
                try:
                    import hopping_astar_planner as _hap
                    xdot = _hap._xdot(X, d["Z"], d["alpha_s"], config.G_ACCEL)
                    z_tight = (d["z0"]
                               + u_tight * math.tan(d["alpha_s"])
                               - config.G_ACCEL * u_tight ** 2
                               / (2.0 * xdot ** 2))
                    gap = z_tight - WALL_HEIGHT - config.ROBOT_RADIUS
                    y_bot = WALL_HEIGHT + config.ROBOT_RADIUS
                    y_top = z_tight
                    if abs(y_top - y_bot) > 0.002:
                        ax.annotate(
                            "",
                            xy=(u_tight, y_bot),
                            xytext=(u_tight, y_top),
                            arrowprops=dict(
                                arrowstyle="<->", color="#0277bd", lw=1.8,
                            ),
                        )
                        ax.text(
                            u_tight + 0.04,
                            0.5 * (y_bot + y_top),
                            f"Δz = {gap:+.3f} m",
                            fontsize=9, color="#0277bd", va="center",
                            fontweight="bold",
                        )
                except Exception:
                    pass

        ax.set_ylim(-0.05, max(WALL_HEIGHT + 0.30, 0.60))
        ax.set_xlabel("u  —  horizontal distance from takeoff (m)", fontsize=11)
        ax.set_ylabel("z  (m)", fontsize=11)
        ax.legend(fontsize=9, loc="upper right")
        # Override the title set internally by draw_arc_side_view
        ax.set_title(
            f"{panel_label}\n"
            f"hop  ({p0[0]:.2f}, {p0[1]:.2f}) → ({p1[0]:.2f}, {p1[1]:.2f})     "
            f"X = {d['X']:.2f} m     "
            f"α_s = {math.degrees(d['alpha_s']):.1f}°     "
            f"mc = {d['mc']:+.4f} m",
            fontsize=10,
        )

    fig.suptitle(
        f"Wall-crossing hop — focused side view\n"
        f"Wall  h = {WALL_HEIGHT:.2f} m,   robot_radius = {config.ROBOT_RADIUS:.2f} m,   "
        f"H_clear = {H_CLEAR:.2f} m,   arc apex ≈ 0.400 m\n"
        f"Baseline (top): arc clips through wall  |  "
        f"Ballistic (bottom): arc clears with Δz ≈ +0.055 m",
        fontsize=12,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    fig.savefig(save_path, dpi=130)
    print(f"Saved: {save_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    m = build_map()

    planner_base = make_planner(m, disable_clearance=True)
    planner_ball = make_planner(m, disable_clearance=False)

    path_base = planner_base.plan()
    path_ball = planner_ball.plan()

    if path_base is None or path_ball is None:
        print(f"Baseline path:  {'FOUND' if path_base else 'NONE'}")
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

    n_base_bad = sum(1 for d in diags_base if not d["feasible"] or d["mc"] < 0.0)
    n_ball_bad = sum(1 for d in diags_ball if not d["feasible"] or d["mc"] < 0.0)

    print("=" * 72)
    print(f"BASELINE   ({len(path_base)} waypoints, {len(path_base)-1} hops)")
    for i, p in enumerate(path_base):
        note = ""
        if i > 0 and i - 1 < len(diags_base):
            d = diags_base[i - 1]
            note = f"   incoming mc={d['mc']:+.3f} m"
        print(f"  B{i}: ({p[0]:.2f},{p[1]:.2f})  d_wall={distance_to_wall(p):.2f} m{note}")
    print(f"  → {n_base_bad} hop(s) would collide with the wall")

    print("-" * 72)
    print(f"BALLISTIC  ({len(path_ball)} waypoints, {len(path_ball)-1} hops)")
    for i, p in enumerate(path_ball):
        note = ""
        if i > 0 and i - 1 < len(diags_ball):
            d = diags_ball[i - 1]
            note = f"   incoming mc={d['mc']:+.3f} m"
        print(f"  A{i}: ({p[0]:.2f},{p[1]:.2f})  d_wall={distance_to_wall(p):.2f} m{note}")
    print(f"  → {n_ball_bad} hop(s) would collide with the wall")
    print("=" * 72)

    base_hi = wall_crossing_hop(path_base, m)
    ball_hi = wall_crossing_hop(path_ball, m)
    if base_hi is not None and ball_hi is not None:
        bx  = path_base[base_hi][0]
        ax_x = path_ball[ball_hi][0]
        shift = bx - ax_x
        if shift > 0.05:
            print(
                f"\nBallistic takeoff x={ax_x:.2f} m is {shift:.2f} m LEFT of "
                f"baseline takeoff x={bx:.2f} m.\n"
                f"Stepping back ensures the wall falls between u=0.6–1.0 m "
                f"(symmetric around the arc apex at u=0.8 m)."
            )
        elif abs(shift) <= 0.05:
            print("\nWARNING: both planners use the same takeoff x — "
                  "wall height may need adjusting.")

    if path_base == path_ball:
        print("\nWARNING: paths are identical — wall height may need adjusting.")

    out_dir = os.path.dirname(os.path.abspath(__file__))

    draw_topdown(
        m, path_base, path_ball, diags_base,
        os.path.join(out_dir, "barely_jumpable_topdown.png"),
    )
    draw_arc_strip(
        m, planner_ball,
        path_base, diags_base,
        path_ball, diags_ball,
        os.path.join(out_dir, "barely_jumpable_arcs.png"),
    )
    draw_crossing_comparison(
        m, planner_base, planner_ball,
        path_base, diags_base,
        path_ball, diags_ball,
        os.path.join(out_dir, "barely_jumpable_crossing.png"),
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

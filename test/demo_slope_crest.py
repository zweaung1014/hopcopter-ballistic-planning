"""Slope demo: the ballistic planner climbs a graded ramp and clears a convex crest.

The terrain rises continuously from the ramp toe to a crest at z=1.05 m, then
drops to a far shelf at z=0.20 m.  Because the crest is a local maximum standing
0.85 m above the shelf, a hop that comes in too flat catches the brow on the
descending part of its arc.

The baseline planner (clearance and stance OFF) picks a route whose hops the
ballistic gate rejects; the ballistic planner commits to a longer launch from
further back on the ramp so its arc peaks over the crest instead.  The exact
takeoff/landing cells are printed at run time rather than asserted here — they
move whenever the ramp geometry or `HOP_SCAN_STEP` changes.

This is the one map in the suite whose elevation varies continuously rather than
in constant-z blocks, so the arcs are checked against a genuinely graded surface.

Figures produced
----------------
  test/slope_crest_topdown.png   — top-down A/B overlay with profile landmarks
  test/slope_crest_arcs.png      — per-hop side-view strip (both planners)
  test/slope_crest_profile.png   — full terrain profile with every hop chained
                                   end to end (baseline above, ballistic below)

Run:
    python test/demo_slope_crest.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

import config
from demo_common import (
    HOP_RADIUS, N_ANGLES, V_MAX,
    PRESENTATION_DPI, TITLE_FS, LABEL_FS, ANNOT_FS,
    C_BASE, C_BALL, C_COLLIDE, C_ANNOT,
    make_planner, diagnose_path, n_bad_hops,
    draw_topdown_base, draw_ab_paths, add_collision_legend,
    print_ab_summary, param_caption, save, out_path,
)
from map2d5 import Map2D5
from maps.slope_crest import (
    build as build_map,
    RAMP_X0, RAMP_X1, CREST_Z, CREST_X1, TOP_X0, TOP_Z, RAMP_GRADE,
)
from visualizer import draw_arc_side_view


START = (0.5, 2.5)
GOAL  = (4.5, 2.5)   # on the summit plateau (z = 0.80 m)

# Height an arc must reach to pass over the crest with the body clear.
H_CLEAR = CREST_Z + config.ROBOT_RADIUS + config.MIN_CLEARANCE
# Lowest the CoM may pass over the crest: crest top, plus the body's radius,
# plus the clearance the gate demands.


def terrain_profile(m: Map2D5, y: float, n: int = 400):
    """Sample the map's own bilinear terrain along a constant-y line.

    Sampling the map rather than `slope_crest.profile_z` keeps the drawn surface
    identical to what `min_clearance` actually tested — the grid is quantised at
    0.2 m, so the two differ slightly near the breakpoints.
    """
    xs = np.linspace(0.0, m.size_x - 1e-6, n)
    zs = np.array([m.get_elevation_bilinear(x, y) for x in xs])
    return xs, zs


def crest_hop(path: list) -> int | None:
    """Index of the first hop whose x-range spans the crest."""
    for i in range(len(path) - 1):
        lo, hi = sorted((path[i][0], path[i + 1][0]))
        if lo < CREST_X1 and hi > RAMP_X1:
            return i
    return None


# ---------------------------------------------------------------------------
# Figure 1: top-down A/B overlay
# ---------------------------------------------------------------------------

def draw_topdown(m, path_base, path_ball, diags_base, n_base_bad, save_path):
    fig, ax = plt.subplots(figsize=(11, 9))
    draw_topdown_base(m, fig, ax)

    for x_bnd, label, col in [
        (RAMP_X0,  f"ramp toe\nz=0", "#33691e"),
        (RAMP_X1,  f"crest\nz={CREST_Z:.2f}", "#bf360c"),
        (CREST_X1, "crest end", "#bf360c"),
        (TOP_X0,   f"plateau\nz={TOP_Z:.2f}", "#4a148c"),
    ]:
        ax.axvline(x_bnd, color=col, linewidth=1.5, linestyle="--",
                   alpha=0.75, zorder=4)
        ax.text(x_bnd + 0.03, 0.12, label, color=col,
                va="bottom", zorder=5)

    draw_ab_paths(ax, path_base, path_ball, diags_base)

    for i, p in enumerate(path_base):
        ax.annotate(f"B{i}\nz={m.get_elevation(*p):.2f}", p,
                    xytext=(-20, -20), textcoords="offset points", color=C_BASE, zorder=10)
    for i, p in enumerate(path_ball):
        ax.annotate(f"A{i}\nz={m.get_elevation(*p):.2f}", p,
                    xytext=(6, 8), textcoords="offset points", color=C_BALL, zorder=10)

    ax.plot(*START, "go", markersize=11, zorder=11, label="start  z=0.00")
    ax.plot(*GOAL, "r*", markersize=15, zorder=11,
            label=f"goal  z={m.get_elevation(*GOAL):.2f} (summit)")
    add_collision_legend(ax)

    ax.set_title(
        f"slope_crest: continuous {RAMP_GRADE:.2f}-grade ramp to a convex crest "
        f"standing {CREST_Z - TOP_Z:.2f} m above the far shelf\n"
        f"Baseline route contains {n_base_bad} hop(s) the ballistic gate rejects; "
        f"ballistic launches from further back so its arc peaks over the brow", wrap=True
    )
    fig.tight_layout()
    save(fig, save_path)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 2: per-hop side-view strip
# ---------------------------------------------------------------------------

def draw_arc_strip(m, planner_ball, path_base, diags_base, path_ball, diags_ball,
                   save_path):
    ncols = max(len(path_base), len(path_ball)) - 1
    fig, axes = plt.subplots(2, ncols, figsize=(3.9 * ncols, 6.0), squeeze=False)
    obs_fill = planner_ball._obstacle_fill
    ymax = CREST_Z + 0.55

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
                ax.text(0.5, 0.5, "INFEASIBLE\n(physics gate)", ha="center",
                        va="center", transform=ax.transAxes)
                ax.set_xticks([]); ax.set_yticks([])
            else:
                p0, p1 = path[i], path[i + 1]
                draw_arc_side_view(
                    ax, (p0[0], p0[1], d["z0"]), (p1[0], p1[1], d["z1"]),
                    d["alpha_s"], m, config.ROBOT_RADIUS, config.LEG_LENGTH,
                    obs_fill, config.ARC_SAMPLE_MAX_STEP,
                    min_clearance_gate=config.MIN_CLEARANCE,
                    n_lateral=config.ARC_LATERAL_SAMPLES,
                )
                ax.set_ylim(-0.1, ymax)
                ax.axhline(CREST_Z, color="#bf360c", linewidth=1.0,
                           linestyle=":", alpha=0.8, zorder=4)
                ax.axhline(TOP_Z, color="#4a148c", linewidth=1.0,
                           linestyle=":", alpha=0.8, zorder=4)

            if i == 0:
                ax.set_ylabel(f"{row_label}\nz (m)")
            ax.set_xlabel(
                f"hop {i}: ({path[i][0]:.1f},{path[i][1]:.1f})"
                f" → ({path[i+1][0]:.1f},{path[i+1][1]:.1f})",
            )

    fig.suptitle(
        "Per-hop side view — baseline (top) vs ballistic (bottom)\n"
        f"Dotted: crest z={CREST_Z:.2f} m · plateau z={TOP_Z:.2f} m · "
        f"Red arc = below the {config.MIN_CLEARANCE} m clearance gate · "
        "Green = clears · Dashed = underside of the robot's body", wrap=True
    )
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    save(fig, save_path)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 3: whole-ascent profile, hops chained end to end
# ---------------------------------------------------------------------------

def draw_profile(m, path_base, diags_base, path_ball, diags_ball, save_path):
    """Terrain profile along y=2.5 with every hop's parabola drawn in world x."""
    fig, axes = plt.subplots(2, 1, figsize=(12, 9), squeeze=False, sharex=True)
    xs, zs = terrain_profile(m, START[1])

    for ax, (label, path, diags) in zip(axes[:, 0], [
        ("Baseline  (clearance OFF)", path_base, diags_base),
        ("Ballistic  (clearance ON)", path_ball, diags_ball),
    ]):
        ax.fill_between(xs, zs, -0.15, color="#8d6e63", alpha=0.55,
                        linewidth=0, label="Terrain (bilinear, as sampled)")
        ax.plot(xs, zs, color="#5d4037", linewidth=1.0)

        ax.axhline(CREST_Z, color="#bf360c", linewidth=1.3, linestyle="--",
                   zorder=4, label=f"Crest z = {CREST_Z:.2f} m")
        ax.axhline(H_CLEAR, color="#f57f17", linewidth=1.1, linestyle=":",
                   zorder=4, label=f"H_clear = {H_CLEAR:.2f} m (crest + radius + min_clearance)")
        ax.axvspan(RAMP_X1, CREST_X1, alpha=0.12, color="#ff8f00", zorder=1,
                   label=f"Crest span x=[{RAMP_X1}, {CREST_X1}] m")

        # Each hop's parabola, plotted against world x.
        for i, d in enumerate(diags):
            if not d["feasible"]:
                continue
            p0, p1 = path[i], path[i + 1]
            X, Z = d["X"], d["Z"]
            tan_a = math.tan(d["alpha_s"])
            k = (X * tan_a - Z) / (X * X)          # == g / (2 * xdot^2)
            t = np.linspace(0.0, 1.0, 120)
            u = t * X
            # The parabola is the CoM trajectory, which starts LEG_LENGTH above
            # the terrain — not on it. Omitting the offset draws every arc
            # 0.4 m too low and makes clearing hops look like they clip.
            z_arc = d["z0"] + config.LEG_LENGTH + u * tan_a - k * u * u
            x_arc = p0[0] + t * (p1[0] - p0[0])

            clips = d["mc"] < config.MIN_CLEARANCE
            ax.plot(x_arc, z_arc,
                    color=(C_COLLIDE if clips else "#2e7d32"),
                    linewidth=(3.0 if clips else 2.0),
                    zorder=6,
                    label=None if i else ("Hop arc" if not clips else "Hop arc"))
            # The leg: foot on the terrain, body centre LEG_LENGTH above it.
            ax.plot([p0[0], p0[0]], [d["z0"], d["z0"] + config.LEG_LENGTH],
                    color="#37474f", linewidth=1.0, alpha=0.55, zorder=6.5)
            ax.plot([p0[0]], [d["z0"] + config.LEG_LENGTH], "o",
                    color="#37474f", markersize=6, zorder=7)
            ax.plot([p0[0]], [d["z0"]], "|", color="#37474f",
                    markersize=7, zorder=7)
            ax.annotate(f"{i}", (p0[0], d["z0"]), xytext=(0, -14),
                        textcoords="offset points", ha="center", color="#37474f")

            if clips:
                # Mark how far the arc dips below the required clearance.
                ax.annotate(
                    f"CLIPS CREST\nmc = {d['mc']:+.3f} m",
                    xy=(0.5 * (RAMP_X1 + CREST_X1), CREST_Z),
                    xytext=(-140, 46), textcoords="offset points",
                    arrowprops=dict(arrowstyle="->", color="#b71c1c", lw=1.6), color="#b71c1c", fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.3", fc="white",
                              ec="#b71c1c", lw=1.3), zorder=12,
                )

        ax.plot([path[-1][0]], [m.get_elevation(*path[-1])], "r*",
                markersize=15, zorder=8)
        ax.set_ylim(-0.15, CREST_Z + 0.55)
        ax.set_ylabel("z  (m)")
        ax.set_title(
            f"{label}   —   {len(path) - 1} hops, "
            f"{n_bad_hops(diags)} clipping terrain", loc="left", wrap=True
        )
        ax.legend(loc="upper left")

    axes[1, 0].set_xlabel("x  (m)   —   direction of travel")
    fig.suptitle(
        "Climbing the slope: every hop drawn on the real terrain profile (y = 2.5 m)", wrap=True
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    save(fig, save_path)
    plt.close(fig)


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
        print(f"Baseline path:  {'FOUND' if path_base else 'NONE'}")
        print(f"Ballistic path: {'FOUND' if path_ball else 'NONE'}")
        print("One planner found no path — cannot compare.")
        return 1

    diags_base = diagnose_path(planner_ball, m, path_base)
    diags_ball = diagnose_path(planner_ball, m, path_ball)

    n_base_bad, n_ball_bad = print_ab_summary(
        m, path_base, diags_base, path_ball, diags_ball,
    )

    bi, ai = crest_hop(path_base), crest_hop(path_ball)
    if bi is not None:
        print(f"\nBaseline crest hop {bi}: x={path_base[bi][0]:.2f} → "
              f"{path_base[bi+1][0]:.2f}, mc={diags_base[bi]['mc']:+.4f} m")
    if ai is not None:
        print(f"Ballistic crest hop {ai}: x={path_ball[ai][0]:.2f} → "
              f"{path_ball[ai+1][0]:.2f}, mc={diags_ball[ai]['mc']:+.4f} m")

    print(f"\nGoal elevation reached: {m.get_elevation(*path_ball[-1]):.2f} m "
          f"(far shelf z={TOP_Z:.2f} m)")

    draw_topdown(m, path_base, path_ball, diags_base, n_base_bad,
                 out_path("slope_crest_topdown.png"))
    draw_arc_strip(m, planner_ball, path_base, diags_base, path_ball, diags_ball,
                   out_path("slope_crest_arcs.png"))
    draw_profile(m, path_base, diags_base, path_ball, diags_ball,
                 out_path("slope_crest_profile.png"))

    same = path_base == path_ball
    if same:
        print("\nWARNING: both planners produced the same path.")
    return 0 if (not same and n_base_bad > 0 and n_ball_bad == 0) else 2


if __name__ == "__main__":
    raise SystemExit(main())

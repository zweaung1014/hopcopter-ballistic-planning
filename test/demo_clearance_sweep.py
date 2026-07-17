"""Visual demo: sweep the takeoff X toward a pillar and show the ballistic
arc flip from ACCEPT (green, clears the pillar) to REJECT (red, collides).

Fix the pillar and goal in world coordinates; move the takeoff position
progressively closer to the pillar. For each takeoff, compute the feasible
takeoff-angle interval (Campana Eq. 4 + leg-energy bound), pick the
midpoint alpha (Campana's max-margin choice), and evaluate the arc-to-
terrain clearance with the exact same code path the planner uses.

Every column of the output figure shows:
  * top row  — top-down of the map with the XY segment overlaid;
  * bottom row — side view (u vs z) of the arc and terrain profile.

The script also prints PASS/FAIL per takeoff so the visuals and the numbers
agree.

Run:
    python test/demo_clearance_sweep.py
Output figure: test/clearance_sweep.png
"""

import math
import os
import sys

# Allow running the script from the repo root without installing.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

import config
from hopping_astar_planner import (
    feasible_alpha_interval,
    min_clearance,
)
from map2d5 import Map2D5
from visualizer import Visualizer, draw_arc_side_view


# --- Demo geometry (tuned to give a mix of ACCEPT and REJECT) ------------ #
MAP_X = 6.0          # m; map width
MAP_Y = 3.0          # m; map height (narrow, we only need y ~ 1.5 corridor)
RES = 0.1            # m; cell resolution
PILLAR_X = 3.0       # m; pillar center x
PILLAR_Y = 1.5       # m; pillar center y
PILLAR_HALF = 0.15   # m; pillar half-size in x and y (~3 cells wide)
PILLAR_H = 0.6       # m; pillar height
GOAL_X = 5.0         # m; landing x is fixed
Y_CORRIDOR = 1.5     # m; takeoff/landing y are on this line, z=0
TAKEOFF_XS = [1.0, 1.7, 2.4, 2.9]  # sweep values, closer -> shorter X

# Robot / physics for the demo (higher V_max so 4 m hops are feasible).
G = config.G_ACCEL
V_MAX = 7.0          # m/s; overrides config default for the demo
ROBOT_R = config.ROBOT_RADIUS
EPS = config.ARC_ENDPOINT_EPSILON
MAX_STEP = config.ARC_SAMPLE_MAX_STEP
WALL_EXTRA = config.OBSTACLE_WALL_EXTRA


def build_map() -> Map2D5:
    m = Map2D5(size_x=MAP_X, size_y=MAP_Y, resolution=RES)
    r0, c0 = m.world_to_grid(PILLAR_X - PILLAR_HALF, PILLAR_Y - PILLAR_HALF)
    r1, c1 = m.world_to_grid(PILLAR_X + PILLAR_HALF, PILLAR_Y + PILLAR_HALF)
    m.grid[r0:r1 + 1, c0:c1 + 1] = PILLAR_H
    return m


def obstacle_fill_for(m: Map2D5) -> float:
    non_obs = m.grid[m.grid != Map2D5.OBSTACLE]
    z_max = float(non_obs.max()) if non_obs.size else 0.0
    return z_max + WALL_EXTRA


def main() -> int:
    m = build_map()
    obs_fill = obstacle_fill_for(m)

    n = len(TAKEOFF_XS)
    fig = plt.figure(figsize=(4.2 * n, 8.0))
    axes_top: list = []
    axes_side: list = []
    for i in range(n):
        axes_top.append(fig.add_subplot(2, n, i + 1))
        axes_side.append(fig.add_subplot(2, n, n + i + 1))

    print(f"Pillar center=({PILLAR_X}, {PILLAR_Y}) h={PILLAR_H} m  "
          f"goal=({GOAL_X}, {Y_CORRIDOR})  V_max={V_MAX} m/s  r={ROBOT_R} m")
    print("-" * 78)

    n_accept = n_reject = 0
    for i, xs in enumerate(TAKEOFF_XS):
        c_s = (xs, Y_CORRIDOR, 0.0)
        c_g = (GOAL_X, Y_CORRIDOR, 0.0)
        X = GOAL_X - xs
        Z = 0.0

        # --- ballistic feasibility (Campana Eq. 4 + v_s <= V_max) ---
        iv = feasible_alpha_interval(X, Z, V_MAX, G)
        if iv is None:
            print(f"takeoff_x={xs:.2f}  X={X:.2f}   INFEASIBLE (no valid alpha)")
            axes_top[i].set_title(f"x_s={xs}  INFEASIBLE", fontsize=9)
            axes_side[i].set_title("INFEASIBLE", fontsize=9)
            n_reject += 1
            continue
        alpha_min, alpha_max = iv
        alpha_s = 0.5 * (alpha_min + alpha_max)

        mc = min_clearance(
            c_s, c_g, alpha_s, m, G, ROBOT_R, EPS, MAX_STEP, obs_fill,
        )
        verdict = "ACCEPT" if mc >= 0.0 else "REJECT"
        print(
            f"takeoff_x={xs:.2f}  X={X:.2f}  "
            f"alpha=[{math.degrees(alpha_min):.1f}, {math.degrees(alpha_max):.1f}] "
            f"mid={math.degrees(alpha_s):.1f}°   "
            f"mc={mc:+.3f} m   {verdict}"
        )
        if mc >= 0.0:
            n_accept += 1
        else:
            n_reject += 1

        # --- top-down panel ---
        top = axes_top[i]
        vis = Visualizer.__new__(Visualizer)  # minimal init: draw_map only
        vis.map_env = m
        vis.fig = fig
        vis.ax = top
        top.set_xlim(0, MAP_X)
        top.set_ylim(0, MAP_Y)
        top.set_aspect("equal")
        top.set_xlabel("X (m)")
        top.set_ylabel("Y (m)")
        vis.draw_map()
        # Overlay XY segment, takeoff, landing.
        top.plot([xs, GOAL_X], [Y_CORRIDOR, Y_CORRIDOR],
                 color="#1976d2", linewidth=1.8, zorder=6)
        top.plot(xs, Y_CORRIDOR, "go", markersize=9, zorder=7, label="takeoff")
        top.plot(GOAL_X, Y_CORRIDOR, "r*", markersize=13, zorder=7, label="land")
        # Pillar outline.
        top.add_patch(mpatches.Rectangle(
            (PILLAR_X - PILLAR_HALF, PILLAR_Y - PILLAR_HALF),
            2 * PILLAR_HALF, 2 * PILLAR_HALF,
            fill=False, edgecolor="k", linewidth=1.2, zorder=6,
        ))
        top.set_title(
            f"x_s={xs:.2f}   X={X:.2f} m   {verdict}", fontsize=9,
        )

        # --- side-view panel ---
        draw_arc_side_view(
            axes_side[i], c_s, c_g, alpha_s, m, G, ROBOT_R,
            obs_fill, EPS, MAX_STEP,
        )
        # Y range harmonised so panels are comparable.
        axes_side[i].set_ylim(-0.05, max(PILLAR_H + 0.4, 1.2))

    print("-" * 78)
    print(f"ACCEPT: {n_accept}   REJECT: {n_reject}")

    fig.suptitle(
        "Ballistic clearance sweep: takeoff moves toward the pillar\n"
        "(shorter hop -> lower arc peak over pillar -> collision)",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    out_path = os.path.join(os.path.dirname(__file__), "clearance_sweep.png")
    fig.savefig(out_path, dpi=110)
    print(f"Saved figure: {out_path}")

    if matplotlib.get_backend().lower() != "agg":
        plt.show()

    # Return non-zero if the sweep didn't cover both cases (so it's obvious
    # the parameters need tuning).
    return 0 if (n_accept > 0 and n_reject > 0) else 2


if __name__ == "__main__":
    sys.exit(main())

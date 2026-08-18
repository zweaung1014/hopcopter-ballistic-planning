"""Visual demo: sweep the takeoff X toward a pillar and show the ballistic
arc flip from ACCEPT (green, clears the pillar) to REJECT (red, collides).

Fix the pillar and goal in world coordinates; move the takeoff position
progressively closer to the pillar. For each takeoff, compute the feasible
takeoff-angle interval (the energy band from the previous hop, the min-apex
floor, then Campana's BEAM), pick the least-injection angle that clears, and
evaluate the arc-to-terrain clearance with the exact same code path the
planner uses.

Watch the interval FLOOR climb as the hop shortens (39.8 deg at X=2.1 up to
78.2 deg at X=0.8). That is the energy chain: the robot arrives with a takeoff
speed it cannot shed, and the shorter the hop, the more surplus it has to
dispose of — and the only way to dispose of it is to go steeper.

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
from demo_common import out_path
from hopping_astar_planner import (
    alpha_for_clearance,
    feasible_alpha_interval,
    terrain_profile,
)
from map2d5 import Map2D5
from visualizer import Visualizer, draw_arc_side_view


# --- Demo geometry (tuned to give a mix of ACCEPT and REJECT) ------------ #
# The four takeoffs give X = 2.1 / 1.8 / 1.3 / 0.8 m, well inside the flat-hop
# reach V_MAX^2 / g = 5.50 m, so the reach never fires and every verdict here is
# genuinely about clearance. Matches test/test_clearance_rejection.py, which
# asserts the numbers this demo draws — including PILLAR_H and the energy state
# the sweep is evaluated in, so keep the two in step.
MAP_X = 4.0          # m; map width
MAP_Y = 3.0          # m; map height (narrow, we only need y ~ 1.5 corridor)
RES = config.CELL_RESOLUTION
PILLAR_X = 2.7       # m; pillar center x
PILLAR_Y = 1.5       # m; pillar center y
PILLAR_HALF = 0.15   # m; pillar half-size in x and y
PILLAR_H = 1.6       # m; pillar height. Calibrated for the leg/body geometry AND
                     # for the energy chain: a robot that cannot shed the speed it
                     # arrives with is forced steep on short hops, and a
                     # near-vertical arc clears a low pillar trivially. See the
                     # PILLAR_H note in test/test_clearance_rejection.py.
GOAL_X = 3.2         # m; landing x is fixed
Y_CORRIDOR = 1.5     # m; takeoff/landing y are on this line, z=0
TAKEOFF_XS = [1.1, 1.4, 1.9, 2.4]  # sweep values, closer -> shorter X

# Robot / physics for the demo — all from config now that V_MAX is derived
# from the robot's stated jump height rather than tuned per demo.
G = config.G_ACCEL
V_MAX = config.V_MAX
ROBOT_R = config.ROBOT_RADIUS
LEG = config.LEG_LENGTH
GATE = config.MIN_CLEARANCE
MAX_STEP = config.ARC_SAMPLE_MAX_STEP
WALL_EXTRA = config.OBSTACLE_WALL_EXTRA

# The energy state the sweep is evaluated in: the chain's seed, i.e. the robot at
# the start of a plan. Without it `feasible_alpha_interval` reverts to the
# pre-chain behaviour and the demo would draw a ceiling no real hop ever gets.
V_G_IN = math.sqrt(2.0 * G * config.H_INITIAL / config.ETA_HOP)
ENERGY_KW = dict(
    v_s_min=math.sqrt(config.ETA_HOP) * V_G_IN,
    e_inject_max=config.E_INJECT_MAX,
    mass=config.ROBOT_MASS,
    min_apex=config.MIN_APEX_HEIGHT,
    V_g_max=config.V_G_MAX,
)


def build_map() -> Map2D5:
    m = Map2D5(size_x=MAP_X, size_y=MAP_Y, resolution=RES)
    m.paint_region(
        PILLAR_H,
        x_min=PILLAR_X - PILLAR_HALF, x_max=PILLAR_X + PILLAR_HALF,
        y_min=PILLAR_Y - PILLAR_HALF, y_max=PILLAR_Y + PILLAR_HALF,
    )
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
        iv = feasible_alpha_interval(X, Z, V_MAX, G, **ENERGY_KW)
        if iv is None:
            print(f"takeoff_x={xs:.2f}  X={X:.2f}   INFEASIBLE (no valid alpha)")
            axes_top[i].set_title(f"x_s={xs}  INFEASIBLE")
            axes_side[i].set_title("INFEASIBLE")
            n_reject += 1
            continue
        alpha_min, alpha_max = iv
        profile = terrain_profile(
            c_s, c_g, m, ROBOT_R, LEG, MAX_STEP, obs_fill,
            config.ARC_LATERAL_SAMPLES,
            min_clearance_gate=GATE,
        )
        # Same rule the planner uses: the least-injection angle that clears,
        # which is the shallowest one whenever the energy floor is binding.
        alpha_s, mc = alpha_for_clearance(
            profile, alpha_min, alpha_max, GATE,
        )
        verdict = "ACCEPT" if mc >= GATE else "REJECT"
        print(
            f"takeoff_x={xs:.2f}  X={X:.2f}  "
            f"alpha=[{math.degrees(alpha_min):.1f}, {math.degrees(alpha_max):.1f}] "
            f"chosen={math.degrees(alpha_s):.1f}°   "
            f"mc={mc:+.3f} m   {verdict}"
        )
        if mc >= GATE:
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
            f"x_s={xs:.2f}   X={X:.2f} m   {verdict}",
        )

        # --- side-view panel ---
        draw_arc_side_view(
            axes_side[i], c_s, c_g, alpha_s, m, ROBOT_R, LEG,
            obs_fill, MAX_STEP,
            min_clearance_gate=GATE,
            n_lateral=config.ARC_LATERAL_SAMPLES,
        )
        # Y range harmonised so panels are comparable.
        axes_side[i].set_ylim(-0.05, max(PILLAR_H + 0.4, 1.2))

    print("-" * 78)
    print(f"ACCEPT: {n_accept}   REJECT: {n_reject}")

    fig.suptitle(
        "Ballistic clearance sweep: takeoff moves toward the pillar\n"
        "(shorter hop -> lower arc peak over pillar -> collision)", wrap=True
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    fig_path = out_path("clearance_sweep.png")
    fig.savefig(fig_path, dpi=110)
    print(f"Saved figure: {fig_path}")

    if matplotlib.get_backend().lower() != "agg":
        plt.show()

    # Return non-zero if the sweep didn't cover both cases (so it's obvious
    # the parameters need tuning).
    return 0 if (n_accept > 0 and n_reject > 0) else 2


if __name__ == "__main__":
    sys.exit(main())

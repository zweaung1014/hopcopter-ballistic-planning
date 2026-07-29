"""Numerical assertions that mirror demo_clearance_sweep.py without any
matplotlib dependency. Suitable for CI later. Prints PASS/FAIL per case
and exits with 0 (all pass) or 1 (any fail).

Run:
    python test/test_clearance_rejection.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from hopping_astar_planner import (
    alpha_for_clearance,
    feasible_alpha_interval,
    terrain_profile,
)
from map2d5 import Map2D5


G = config.G_ACCEL
ROBOT_R = config.ROBOT_RADIUS
LEG = config.LEG_LENGTH
GATE = config.MIN_CLEARANCE
MAX_STEP = config.ARC_SAMPLE_MAX_STEP
N_LAT = config.ARC_LATERAL_SAMPLES
WALL_EXTRA = config.OBSTACLE_WALL_EXTRA

# Geometry is sized for the derived V_MAX (4.85 m/s), whose longest feasible
# flat hop is V_MAX^2 / g = 2.40 m. Spans here are 2.1/1.8/1.3/0.8 m so the
# physics gate never fires and every verdict below is genuinely about clearance.
GOAL_X = 3.2
PILLAR_X = 2.7
PILLAR_H = 0.9   # calibrated: see the sweep in case (1)


def _obs_fill(m: Map2D5) -> float:
    non_obs = m.grid[m.grid != Map2D5.OBSTACLE]
    return (float(non_obs.max()) if non_obs.size else 0.0) + WALL_EXTRA


def _pillar_map(pillar_h: float, x_center: float = PILLAR_X) -> Map2D5:
    m = Map2D5(size_x=4.0, size_y=3.0, resolution=0.1)
    m.paint_region(
        pillar_h,
        x_min=x_center - 0.15, x_max=x_center + 0.15,
        y_min=1.35, y_max=1.65,
    )
    return m


def _mc_for(xs: float, goal_x: float, pillar_h: float) -> float:
    """Clearance of a flat hop from `xs` to `goal_x` over the pillar.

    Uses the planner's own angle rule, escalation included, so a rejection here
    means no feasible takeoff angle clears — not merely that the default one
    failed.
    """
    m = _pillar_map(pillar_h)
    obs = _obs_fill(m)
    c_s = (xs, 1.5, 0.0)
    c_g = (goal_x, 1.5, 0.0)
    iv = feasible_alpha_interval(goal_x - xs, 0.0, config.V_MAX, G)
    if iv is None:
        return -math.inf  # infeasible counts as rejection
    profile = terrain_profile(c_s, c_g, m, ROBOT_R, LEG, MAX_STEP, obs, N_LAT)
    _alpha, mc = alpha_for_clearance(
        profile, iv[0], iv[1], GATE, config.ALPHA_MARGIN_FRAC
    )
    return mc


def _check(name: str, mc: float, expect_accept: bool) -> bool:
    ok = (mc >= GATE) == expect_accept
    tag = "PASS" if ok else "FAIL"
    verdict = "ACCEPT" if mc >= GATE else "REJECT"
    want = "ACCEPT" if expect_accept else "REJECT"
    print(f"  [{tag}] {name}: mc={mc:+.3f} m -> {verdict}  (expected {want})")
    return ok


def main() -> int:
    print("== ballistic clearance rejection tests ==")
    print(f"   V_max={config.V_MAX:.3f} m/s  leg={LEG} m  r={ROBOT_R} m  gate={GATE} m")

    # -- Sweep takeoff distance against a fixed pillar. --
    # Both ends of the range fail, for opposite reasons: from far back the
    # pillar sits under the arc's descending limb, from close in the arc is
    # too flat to get over it at all. Only the middle of the range clears.
    print(f"\n(1) Fixed goal x={GOAL_X}, sweep takeoff toward pillar "
          f"(x={PILLAR_X}, h={PILLAR_H}):")
    all_ok = True
    all_ok &= _check("xs=1.1 (X=2.1, far)",  _mc_for(1.1, GOAL_X, PILLAR_H), expect_accept=False)
    all_ok &= _check("xs=1.4 (X=1.8)",       _mc_for(1.4, GOAL_X, PILLAR_H), expect_accept=False)
    all_ok &= _check("xs=1.9 (X=1.3)",       _mc_for(1.9, GOAL_X, PILLAR_H), expect_accept=True)
    all_ok &= _check("xs=2.4 (X=0.8, near)", _mc_for(2.4, GOAL_X, PILLAR_H), expect_accept=False)

    # -- Feasibility gate: leg too weak. --
    print("\n(2) feasible_alpha_interval boundaries:")
    iv = feasible_alpha_interval(2.0, 0.0, 4.5, G)
    ok = iv is not None
    print(f"  [{'PASS' if ok else 'FAIL'}] X=2, Z=0, V_max=4.5 -> feasible: {iv is not None}")
    all_ok &= ok
    iv = feasible_alpha_interval(5.0, 0.0, 4.5, G)  # v_min=sqrt(g*5)=7.0 > 4.5
    ok = iv is None
    print(f"  [{'PASS' if ok else 'FAIL'}] X=5, Z=0, V_max=4.5 -> infeasible: {iv is None}")
    all_ok &= ok
    # The shipped V_MAX is exactly the speed for a 2.40 m flat hop, so that is
    # the boundary of what the robot can reach on level ground.
    ok = (feasible_alpha_interval(2.3, 0.0, config.V_MAX, G) is not None
          and feasible_alpha_interval(2.5, 0.0, config.V_MAX, G) is None)
    print(f"  [{'PASS' if ok else 'FAIL'}] shipped V_MAX reaches 2.3 m but not 2.5 m")
    all_ok &= ok

    # -- OBSTACLE cells in the arc's XY path should always reject. --
    # The region has to be at least two cells thick across the arc. A single
    # OBSTACLE cell sampled along its own boundary is averaged 50/50 with its
    # neighbour by the bilinear lookup, which halves the effective wall height
    # — real enough that maps should never rely on one-cell obstacles.
    print("\n(3) OBSTACLE region under the arc:")
    m = _pillar_map(0.0)  # ground zero everywhere
    m.set_obstacle_region(2.4, 1.35, 2.6, 1.65)
    iv = feasible_alpha_interval(2.0, 0.0, config.V_MAX, G)
    profile = terrain_profile(
        (1.5, 1.5, 0.0), (3.5, 1.5, 0.0), m, ROBOT_R, LEG,
        MAX_STEP, _obs_fill(m), N_LAT,
    )
    _a, mc = alpha_for_clearance(profile, iv[0], iv[1], GATE, config.ALPHA_MARGIN_FRAC)
    all_ok &= _check("arc over OBSTACLE region", mc, expect_accept=False)

    # OBSTACLE_WALL_EXTRA must be large enough that the tallest arc the robot
    # can fly still cannot clear an obstacle cell.
    reach = LEG + config.MAX_APEX_HEIGHT - ROBOT_R - GATE
    ok = WALL_EXTRA >= reach
    print(f"  [{'PASS' if ok else 'FAIL'}] OBSTACLE_WALL_EXTRA={WALL_EXTRA} m >= "
          f"max flyable height {reach:.2f} m")
    all_ok &= ok

    # -- Downhill jump widens the feasible interval vs a flat jump of same X. --
    print("\n(4) Downhill jump widens the feasible interval:")
    iv_flat = feasible_alpha_interval(2.0, 0.0, 4.5, G)
    iv_down = feasible_alpha_interval(2.0, -0.5, 4.5, G)
    flat_w = (iv_flat[1] - iv_flat[0]) if iv_flat else 0.0
    down_w = (iv_down[1] - iv_down[0]) if iv_down else 0.0
    ok = iv_down is not None and down_w > flat_w
    print(f"  [{'PASS' if ok else 'FAIL'}] flat width={math.degrees(flat_w):.1f}° "
          f"vs downhill width={math.degrees(down_w):.1f}°")
    all_ok &= ok

    # -- The leg/body/gate geometry must leave room to stand at all. --
    print("\n(5) Standing clearance invariant:")
    stand = LEG - ROBOT_R
    ok = stand > GATE
    print(f"  [{'PASS' if ok else 'FAIL'}] leg - radius = {stand:.3f} m > "
          f"gate {GATE} m (slack {stand - GATE:+.3f} m)")
    all_ok &= ok
    # A flat cell must be standable; a cell alongside a tall step must not be.
    mm = Map2D5(2.0, 2.0, 0.1)
    mm.paint_region(0.8, x_min=1.0)
    sm = mm.standable_mask(ROBOT_R, GATE, LEG)
    row = mm.rows // 2
    ok = bool(sm[row, 2]) and not bool(sm[row, 9])
    print(f"  [{'PASS' if ok else 'FAIL'}] flat ground standable, "
          f"cell abutting a 0.8 m step is not")
    all_ok &= ok

    print("\n" + ("ALL PASSED" if all_ok else "SOME FAILED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

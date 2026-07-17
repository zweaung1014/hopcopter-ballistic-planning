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
    feasible_alpha_interval,
    min_clearance,
)
from map2d5 import Map2D5


G = config.G_ACCEL
ROBOT_R = config.ROBOT_RADIUS
EPS = config.ARC_ENDPOINT_EPSILON
MAX_STEP = config.ARC_SAMPLE_MAX_STEP
WALL_EXTRA = config.OBSTACLE_WALL_EXTRA


def _obs_fill(m: Map2D5) -> float:
    non_obs = m.grid[m.grid != Map2D5.OBSTACLE]
    return (float(non_obs.max()) if non_obs.size else 0.0) + WALL_EXTRA


def _pillar_map(pillar_h: float, x_center: float = 3.0) -> Map2D5:
    m = Map2D5(size_x=6.0, size_y=3.0, resolution=0.1)
    r0, c0 = m.world_to_grid(x_center - 0.15, 1.35)
    r1, c1 = m.world_to_grid(x_center + 0.15, 1.65)
    m.grid[r0:r1 + 1, c0:c1 + 1] = pillar_h
    return m


def _mc_for(xs: float, goal_x: float, V_max: float, pillar_h: float) -> float:
    m = _pillar_map(pillar_h)
    obs = _obs_fill(m)
    c_s = (xs, 1.5, 0.0)
    c_g = (goal_x, 1.5, 0.0)
    X = goal_x - xs
    iv = feasible_alpha_interval(X, 0.0, V_max, G)
    if iv is None:
        return -math.inf  # infeasible counts as rejection
    a = 0.5 * (iv[0] + iv[1])
    return min_clearance(c_s, c_g, a, m, G, ROBOT_R, EPS, MAX_STEP, obs)


def _check(name: str, mc: float, expect_accept: bool) -> bool:
    ok = (mc >= 0.0) == expect_accept
    tag = "PASS" if ok else "FAIL"
    verdict = "ACCEPT" if mc >= 0.0 else "REJECT"
    want = "ACCEPT" if expect_accept else "REJECT"
    print(f"  [{tag}] {name}: mc={mc:+.3f} m -> {verdict}  (expected {want})")
    return ok


def main() -> int:
    print("== ballistic clearance rejection tests ==")

    # -- Sweep against a 0.6 m pillar with V_max=7 (same as demo). --
    print("\n(1) Fixed goal, sweep takeoff X toward pillar (h=0.6, V_max=7):")
    all_ok = True
    all_ok &= _check("xs=1.0 (X=4.0, far)", _mc_for(1.0, 5.0, 7.0, 0.6), expect_accept=True)
    all_ok &= _check("xs=1.7 (X=3.3)",       _mc_for(1.7, 5.0, 7.0, 0.6), expect_accept=True)
    all_ok &= _check("xs=2.4 (X=2.6)",       _mc_for(2.4, 5.0, 7.0, 0.6), expect_accept=False)
    all_ok &= _check("xs=2.9 (X=2.1, near)", _mc_for(2.9, 5.0, 7.0, 0.6), expect_accept=False)

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

    # -- OBSTACLE cell in the arc's XY path should always reject. --
    print("\n(3) OBSTACLE cell under the arc:")
    m = _pillar_map(0.0)  # ground zero everywhere
    m.set_obstacle(3.0, 1.5)
    a = 0.5 * sum(feasible_alpha_interval(2.0, 0.0, 7.0, G))
    mc = min_clearance((2.0, 1.5, 0.0), (4.0, 1.5, 0.0), a,
                       m, G, ROBOT_R, EPS, MAX_STEP, _obs_fill(m))
    all_ok &= _check("arc over single OBSTACLE cell", mc, expect_accept=False)

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

    print("\n" + ("ALL PASSED" if all_ok else "SOME FAILED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

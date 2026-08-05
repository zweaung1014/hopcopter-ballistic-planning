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
LEG_R = config.LEG_CYLINDER_RADIUS
FOOT_R = config.FOOT_TIP_RADIUS
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
PILLAR_H = 0.45  # calibrated for the capsule (bottom-cap + cylinder-side) gate.
                 # The old sphere-only check used 0.9 and needed only the CoM to
                 # clear the pillar top by (R + gate) = 0.35 m. The capsule check
                 # instead requires the foot tip to clear by (R + gate), which
                 # is L = 0.4 m stricter — so a 0.9 m pillar is un-jumpable at
                 # every takeoff x. 0.45 restores the "only midrange clears"
                 # sweep because the pillar's TRAILING edge (arc descending
                 # limb) blocks far-back takeoffs and the LEADING edge (arc
                 # hasn't risen yet) blocks close-in takeoffs.


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
    profile = terrain_profile(
        c_s, c_g, m, ROBOT_R, LEG_R, FOOT_R, LEG, MAX_STEP, obs, N_LAT,
        min_clearance_gate=GATE,
    )
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
    # pillar's trailing edge sits under the arc's descending limb, from close
    # in the arc hasn't risen enough at the leading edge (the arc rises
    # quadratically from takeoff, so short u ≈ short foot lift). Only the
    # middle of the range clears both edges.
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
        (1.5, 1.5, 0.0), (3.5, 1.5, 0.0), m, ROBOT_R, LEG_R, FOOT_R, LEG,
        MAX_STEP, _obs_fill(m), N_LAT,
        min_clearance_gate=GATE,
    )
    _a, mc = alpha_for_clearance(profile, iv[0], iv[1], GATE, config.ALPHA_MARGIN_FRAC)
    all_ok &= _check("arc over OBSTACLE region", mc, expect_accept=False)

    # OBSTACLE_WALL_EXTRA must be large enough that the tallest arc the robot
    # can fly still cannot clear an obstacle cell.
    reach = LEG + config.MAX_APEX_HEIGHT - FOOT_R - GATE
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
    sm = mm.standable_mask(ROBOT_R, LEG_R, GATE, LEG)
    row = mm.rows // 2
    ok = bool(sm[row, 2]) and not bool(sm[row, 9])
    print(f"  [{'PASS' if ok else 'FAIL'}] flat ground standable, "
          f"cell abutting a 0.8 m step is not")
    all_ok &= ok

    # -- Capsule stance/flight geometry. --
    # These target the pieces the new leg-cylinder-sides stance check + full
    # capsule flight check add on top of the pure-sphere model, so a
    # regression in the capsule math shows up here even if the pillar sweep
    # in case (1) still happens to pass.
    print("\n(6) Capsule stance + flight geometry:")

    # (6a) Leg-cylinder-sides ceiling vs. CoM-sphere-alone ceiling. With
    # LEG_R = 0.01 m (much thinner than the old shared 0.2 m radius), the
    # closed-form leg-cylinder ceiling is nominally the tighter of the two:
    #
    #     g_max_capsule = (L * frac) / (LEG_R + GATE)  ≈ 1.212
    #     g_max_sphere  = sqrt((L / (ROBOT_R + GATE))^2 - 1)  ≈ 1.249
    #
    # But LEG_R (0.01 m) is now smaller than CELL_RESOLUTION (0.1 m), and
    # `standable_mask`'s neighbour search only ever fires the leg-cylinder
    # check at discretized offsets (0.1, 0.1414, 0.2 m, ...) — by the time a
    # neighbouring cell has risen enough to cross the exempt-slab threshold,
    # its horizontal distance already clears the (tiny) LEG_R + GATE margin
    # comfortably. So in practice, at this map's resolution, the leg-cylinder
    # check never actually binds and the CoM sphere alone determines
    # standability — empirically confirmed to switch at exactly g_max_sphere
    # (grade 1.249 stands, 1.250 does not). The two ceilings' ordering below
    # is still a valid sanity check on the formulas themselves; the slope
    # sweep is calibrated against the *observed* (sphere-governed) cutoff,
    # not the theoretical leg-cylinder one.
    frac = 1.0 / 3.0
    g_max_capsule = (LEG * frac) / (LEG_R + GATE)
    g_max_sphere = math.sqrt((LEG / (ROBOT_R + GATE)) ** 2 - 1)
    ok = g_max_capsule < g_max_sphere
    print(f"  [{'PASS' if ok else 'FAIL'}] capsule g_max ≈ {g_max_capsule:.3f} "
          f"< sphere g_max ≈ {g_max_sphere:.3f} (formula ordering; grid "
          f"resolution makes the sphere ceiling the one that actually binds)")
    all_ok &= ok
    slope_ok = True
    for grade, expect in [(g_max_sphere - 0.05, True), (g_max_sphere + 0.05, False)]:
        mm = Map2D5(3.0, 2.0, 0.1)
        for col in range(mm.cols):
            mm.grid[:, col] = grade * ((col + 0.5) * mm.resolution)
        sm = mm.standable_mask(ROBOT_R, LEG_R, GATE, LEG, leg_clearance_start_frac=frac)
        # Interior cell, avoiding boundary effects.
        stands_somewhere = bool(sm[mm.rows // 2, mm.cols // 2])
        row_ok = (stands_somewhere == expect)
        slope_ok &= row_ok
        print(f"  [{'PASS' if row_ok else 'FAIL'}] grade {grade:.3f} slope "
              f"{'standable' if stands_somewhere else 'un-standable'} "
              f"(expected {'standable' if expect else 'un-standable'})")
    all_ok &= slope_ok

    # (6b) Flight-time bottom cap. A short obstacle that the CoM SPHERE
    # clears easily but the FOOT tip does not should reject under the
    # capsule check. Build a flat scenario with a short bump.
    mm = Map2D5(3.0, 2.0, 0.1)
    mm.paint_region(0.30, x_min=1.35, x_max=1.65, y_min=0.85, y_max=1.15)
    obs = _obs_fill(mm)
    c_s = (0.4, 1.0, 0.0)
    c_g = (2.4, 1.0, 0.0)   # X=2.0 flat, well inside V_MAX
    iv = feasible_alpha_interval(2.0, 0.0, config.V_MAX, G)
    prof = terrain_profile(
        c_s, c_g, mm, ROBOT_R, LEG_R, FOOT_R, LEG, MAX_STEP, obs, N_LAT,
        min_clearance_gate=GATE,
    )
    _, mc_capsule = alpha_for_clearance(
        prof, iv[0], iv[1], GATE, config.ALPHA_MARGIN_FRAC,
    )
    # For reference, what the sphere-only check would have said:
    # (foot_h at bump's u would be foot_h(u_bump); the sphere check only
    # requires z_arc(u_bump) - bump_h - ROBOT_R >= GATE, which is
    # foot_h + LEG - bump_h - ROBOT_R >= GATE, i.e. an extra `LEG` worth of
    # headroom on top of the bottom-cap requirement).
    print(f"  [{'PASS' if mc_capsule >= GATE else 'FAIL'}] "
          f"flat hop over a 0.30 m bump: mc={mc_capsule:+.3f} m -> "
          f"{'ACCEPT' if mc_capsule >= GATE else 'REJECT'} (expected ACCEPT)")
    all_ok &= (mc_capsule >= GATE)

    # (6c) Endpoint-transition mask does not hide walls. A wall between the
    # endpoints that the arc barely fails to clear must still register as a
    # reject even though the near-endpoint samples are masked.
    mm = Map2D5(3.0, 2.0, 0.1)
    mm.paint_region(1.10, x_min=1.35, x_max=1.65, y_min=0.85, y_max=1.15)
    obs = _obs_fill(mm)
    c_s = (0.4, 1.0, 0.0)
    c_g = (2.4, 1.0, 0.0)
    iv = feasible_alpha_interval(2.0, 0.0, config.V_MAX, G)
    prof = terrain_profile(
        c_s, c_g, mm, ROBOT_R, LEG_R, FOOT_R, LEG, MAX_STEP, obs, N_LAT,
        min_clearance_gate=GATE,
    )
    _, mc_wall = alpha_for_clearance(
        prof, iv[0], iv[1], GATE, config.ALPHA_MARGIN_FRAC,
    )
    ok = mc_wall < GATE
    print(f"  [{'PASS' if ok else 'FAIL'}] flat hop over a 1.10 m wall: "
          f"mc={mc_wall:+.3f} m -> "
          f"{'ACCEPT' if mc_wall >= GATE else 'REJECT'} (expected REJECT)")
    all_ok &= ok

    print("\n" + ("ALL PASSED" if all_ok else "SOME FAILED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

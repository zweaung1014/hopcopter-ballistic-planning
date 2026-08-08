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
MU = config.MU
MAX_STEP = config.ARC_SAMPLE_MAX_STEP
N_LAT = config.ARC_LATERAL_SAMPLES
WALL_EXTRA = config.OBSTACLE_WALL_EXTRA

# Spans here are 2.1/1.8/1.3/0.8 m, well inside the flat-hop reach
# V_MAX^2 / g = 5.50 m, so the reach never fires and every verdict below is
# genuinely about clearance.
GOAL_X = 3.2
PILLAR_X = 2.7
PILLAR_H = 1.6   # calibrated twice, for two successive model changes.
                 #
                 # It was 0.9 under a sphere-only clearance check, which needed
                 # only the CoM to clear the pillar top by (R + gate) = 0.35 m.
                 # The capsule check instead requires the FOOT TIP to clear by
                 # (R + gate), which is L = 0.4 m stricter, so 0.9 became
                 # un-jumpable at every takeoff x and it dropped to 0.45.
                 #
                 # The energy chain then undid that: the robot cannot shed the
                 # speed it arrives with, so a SHORT hop is forced steep (the
                 # sweep below runs 39.8 deg of takeoff-angle floor at X=2.1 up
                 # to 78.2 deg at X=0.8). A near-vertical arc clears a low
                 # pillar trivially, so 0.45 became jumpable from everywhere.
                 # 1.6 restores the "only midrange clears" sweep, still for the
                 # original two opposite reasons: the pillar's TRAILING edge
                 # (arc descending limb) blocks far-back takeoffs and the
                 # LEADING edge (arc hasn't risen yet) blocks close-in ones.

# The energy state these clearance tests are evaluated in. The chain's seed —
# the robot at the start of a plan, with sqrt(2 g H_INITIAL) of takeoff speed —
# because it is config-derived rather than magic, and because it is the HIGHEST
# energy the chain ever holds. That makes it the most permissive ceiling, so a
# REJECT here is a rejection for every state the robot can be in.
V_G_IN = math.sqrt(2.0 * G * config.H_INITIAL / config.ETA_HOP)
ENERGY_KW = dict(
    v_s_min=math.sqrt(config.ETA_HOP) * V_G_IN,
    e_inject_max=config.E_INJECT_MAX,
    mass=config.ROBOT_MASS,
    min_apex=config.MIN_APEX_HEIGHT,
    V_g_max=config.V_G_MAX,
)


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
    iv = feasible_alpha_interval(goal_x - xs, 0.0, config.V_MAX, G, **ENERGY_KW)
    if iv is None:
        return -math.inf  # infeasible counts as rejection
    profile = terrain_profile(
        c_s, c_g, m, ROBOT_R, LEG_R, FOOT_R, LEG, MAX_STEP, obs, N_LAT,
        min_clearance_gate=GATE,
    )
    _alpha, mc = alpha_for_clearance(
        profile, iv[0], iv[1], GATE
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

    # -- Feasibility gate: leg too weak, and the friction cone on top of it. --
    # `feasible_alpha_interval` is Campana's BEAM: Eq. 4 validity, the friction
    # cone at both contacts, and the leg-speed budget at takeoff AND landing.
    # These calls pass no normals, so they exercise the flat-ground cone.
    print("\n(2) feasible_alpha_interval boundaries:")
    iv = feasible_alpha_interval(2.0, 0.0, 4.5, G)
    ok = iv is not None
    print(f"  [{'PASS' if ok else 'FAIL'}] X=2, Z=0, V_max=4.5 -> feasible: {iv is not None}")
    all_ok &= ok
    iv = feasible_alpha_interval(5.0, 0.0, 4.5, G)  # v_min=sqrt(g*5)=7.0 > 4.5
    ok = iv is None
    print(f"  [{'PASS' if ok else 'FAIL'}] X=5, Z=0, V_max=4.5 -> infeasible: {iv is None}")
    all_ok &= ok
    # A flat hop of distance X needs v_s >= sqrt(g X), so V_MAX reaches exactly
    # V_MAX^2 / g on level ground. The cone raises the floor of the interval but
    # never its ceiling, so this reach is unaffected by MU. Derived rather than
    # written as a literal: this used to read "2.3 but not 2.5", which silently
    # encoded a tuned V_MAX = 4.85 m/s and broke when V_MAX became a derived
    # worst case of the energy chain (reach 2.40 m -> 5.50 m).
    reach = config.V_MAX ** 2 / G
    ok = (feasible_alpha_interval(reach * 0.96, 0.0, config.V_MAX, G) is not None
          and feasible_alpha_interval(reach * 1.04, 0.0, config.V_MAX, G) is None)
    print(f"  [{'PASS' if ok else 'FAIL'}] flat-hop reach = V_MAX^2/g = "
          f"{reach:.2f} m (reaches {reach * 0.96:.2f} m, not {reach * 1.04:.2f} m)")
    all_ok &= ok
    # On level ground both cones reduce to the textbook Coulomb bound, so the
    # shallowest producible takeoff is atan(1/MU) — a shallower push would slide
    # the foot out. Comparing against `mu=None` isolates the cone's contribution.
    iv_cone = feasible_alpha_interval(1.0, 0.0, config.V_MAX, G)
    iv_bare = feasible_alpha_interval(1.0, 0.0, config.V_MAX, G, mu=None)
    ok = (abs(iv_cone[0] - math.atan(1.0 / MU)) < 1e-5
          and abs(iv_cone[1] - iv_bare[1]) < 1e-9)
    print(f"  [{'PASS' if ok else 'FAIL'}] flat-ground cone floor = atan(1/MU) = "
          f"{math.degrees(math.atan(1.0 / MU)):.2f}° "
          f"(was {math.degrees(iv_bare[0]):.2f}°, ceiling unchanged at "
          f"{math.degrees(iv_cone[1]):.2f}°)")
    all_ok &= ok

    # -- OBSTACLE cells in the arc's XY path should always reject. --
    # The region has to be at least two cells thick across the arc. A single
    # OBSTACLE cell sampled along its own boundary is averaged 50/50 with its
    # neighbour by the bilinear lookup, which halves the effective wall height
    # — real enough that maps should never rely on one-cell obstacles.
    print("\n(3) OBSTACLE region under the arc:")
    m = _pillar_map(0.0)  # ground zero everywhere
    m.set_obstacle_region(2.4, 1.35, 2.6, 1.65)
    iv = feasible_alpha_interval(2.0, 0.0, config.V_MAX, G, **ENERGY_KW)
    profile = terrain_profile(
        (1.5, 1.5, 0.0), (3.5, 1.5, 0.0), m, ROBOT_R, LEG_R, FOOT_R, LEG,
        MAX_STEP, _obs_fill(m), N_LAT,
        min_clearance_gate=GATE,
    )
    _a, mc = alpha_for_clearance(profile, iv[0], iv[1], GATE)
    all_ok &= _check("arc over OBSTACLE region", mc, expect_accept=False)

    # OBSTACLE_WALL_EXTRA must be large enough that the tallest arc the robot
    # can fly still cannot clear an obstacle cell.
    reach = LEG + config.MAX_APEX_HEIGHT - FOOT_R - GATE
    ok = WALL_EXTRA >= reach
    print(f"  [{'PASS' if ok else 'FAIL'}] OBSTACLE_WALL_EXTRA={WALL_EXTRA} m >= "
          f"max flyable height {reach:.2f} m")
    all_ok &= ok

    # -- Downhill jumps: what the two new BEAM constraints changed. --
    # This section used to assert "downhill widens the interval". That was true
    # of the pre-BEAM physics and is no longer, for two independent reasons, one
    # per new constraint. Both are checked here.
    print("\n(4) Downhill jumps under the cone and the landing budget:")

    # (4a) The takeoff cone floors alpha_min. Dropping the target lowers the
    # geometric bound atan2(Z, X) below zero, and the old interval followed it
    # down — asking a push-only leg to thrust *below* the horizon. The cone
    # replaces that with the Coulomb floor, which does not depend on Z at all.
    iv_down = feasible_alpha_interval(1.0, -0.4, config.V_MAX, G)
    iv_down_bare = feasible_alpha_interval(1.0, -0.4, config.V_MAX, G, mu=None)
    ok = (iv_down_bare[0] < 0.0 < iv_down[0]
          and abs(iv_down[0] - math.atan(1.0 / MU)) < 1e-5)
    print(f"  [{'PASS' if ok else 'FAIL'}] X=1.0, Z=-0.4: alpha_min "
          f"{math.degrees(iv_down_bare[0]):+.1f}° -> {math.degrees(iv_down[0]):+.1f}° "
          f"(no longer aims into the ground)")
    all_ok &= ok

    # (4b) The landing-speed constraint caps how far the robot may drop: the leg
    # has to absorb the arrival, and `v_g^2 = v_s^2 - 2 g Z` grows with the fall.
    # The ceiling is V_max^2 / (2 g) — the fall that spends the entire budget on
    # vertical speed alone — and it is only attained in the limit of a
    # straight-down hop, since any horizontal travel needs speed of its own.
    max_drop = config.V_MAX ** 2 / (2.0 * G)
    ok = (feasible_alpha_interval(0.05, -(max_drop - 0.05), config.V_MAX, G) is not None
          and feasible_alpha_interval(0.05, -(max_drop + 0.05), config.V_MAX, G) is None)
    print(f"  [{'PASS' if ok else 'FAIL'}] near-vertical drop capped at "
          f"V_MAX^2/(2g) = {max_drop:.2f} m (accepts {max_drop - 0.05:.2f} m, "
          f"rejects {max_drop + 0.05:.2f} m)")
    all_ok &= ok

    # ...and the cap tightens as the hop gets longer, because horizontal speed
    # competes with the fall for the same budget.
    def _max_drop(X: float) -> float:
        lo, hi = 0.0, 3.0
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            if feasible_alpha_interval(X, -mid, config.V_MAX, G) is not None:
                lo = mid
            else:
                hi = mid
        return lo

    drops = [(X, _max_drop(X)) for X in (0.05, 0.5, 1.0, 1.5)]
    ok = all(a[1] > b[1] for a, b in zip(drops, drops[1:])) and drops[0][1] < max_drop
    print(f"  [{'PASS' if ok else 'FAIL'}] max drop shrinks with hop distance: "
          + ", ".join(f"X={X}->{d:.2f} m" for X, d in drops))
    all_ok &= ok

    # The old geometry for this section (X=2.0, Z=-0.5, V_max=4.5) is now
    # infeasible outright: it needs |v_g| = 5.01 m/s to land, over that budget.
    ok = feasible_alpha_interval(2.0, -0.5, 4.5, G) is None
    print(f"  [{'PASS' if ok else 'FAIL'}] the pre-BEAM downhill case "
          f"(X=2.0, Z=-0.5, V_max=4.5) is rejected by the landing budget")
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
    iv = feasible_alpha_interval(2.0, 0.0, config.V_MAX, G, **ENERGY_KW)
    prof = terrain_profile(
        c_s, c_g, mm, ROBOT_R, LEG_R, FOOT_R, LEG, MAX_STEP, obs, N_LAT,
        min_clearance_gate=GATE,
    )
    _, mc_capsule = alpha_for_clearance(
        prof, iv[0], iv[1], GATE,
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
    #
    # 1.9 m, up from 1.10 m: the arc that has to fail here is the one at
    # alpha_max, and alpha_max at X=2.0 rose from 61.8 to 75.0 deg with the
    # energy chain. Measured boundary at the shipped parameters: 1.7 m is the
    # tallest wall this hop still clears, 1.75 m the shortest it does not.
    mm = Map2D5(3.0, 2.0, 0.1)
    mm.paint_region(1.90, x_min=1.35, x_max=1.65, y_min=0.85, y_max=1.15)
    obs = _obs_fill(mm)
    c_s = (0.4, 1.0, 0.0)
    c_g = (2.4, 1.0, 0.0)
    iv = feasible_alpha_interval(2.0, 0.0, config.V_MAX, G, **ENERGY_KW)
    prof = terrain_profile(
        c_s, c_g, mm, ROBOT_R, LEG_R, FOOT_R, LEG, MAX_STEP, obs, N_LAT,
        min_clearance_gate=GATE,
    )
    _, mc_wall = alpha_for_clearance(
        prof, iv[0], iv[1], GATE,
    )
    ok = mc_wall < GATE
    print(f"  [{'PASS' if ok else 'FAIL'}] flat hop over a 1.90 m wall: "
          f"mc={mc_wall:+.3f} m -> "
          f"{'ACCEPT' if mc_wall >= GATE else 'REJECT'} (expected REJECT)")
    all_ok &= ok

    print("\n" + ("ALL PASSED" if all_ok else "SOME FAILED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

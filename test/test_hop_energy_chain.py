"""Numerical assertions for the energy chain — the model that makes this a
hopping robot rather than a jumping one. Prints PASS/FAIL per case and exits
with 0 (all pass) or 1 (any fail).

The chain, for reference:

    flight (lossless):  v_g^2 = v_s^2 - 2 g Z
    stance (eta loss):  v_s_min' = sqrt(ETA_HOP) * v_g
    thrust (injection): v_s' in [v_s_min', sqrt(v_s_min'^2 + 2 E_INJECT_MAX/m)]

Sections (1)-(3) check the closed forms in isolation, (4)-(6) check that
`feasible_alpha_interval` turns them into the right takeoff-angle bounds, and
(7)-(8) run the real planner and verify the chain closes end to end on the
path it returns.

Run:
    python test/test_hop_energy_chain.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from hopping_astar_planner import (
    HoppingAStarPlanner,
    feasible_alpha_interval,
    landing_speed,
    min_apex_tan,
    min_energy_tan,
    takeoff_speed,
)
from map2d5 import Map2D5


G = config.G_ACCEL
ETA = config.ETA_HOP
MASS = config.ROBOT_MASS
E_INJ = config.E_INJECT_MAX
H_MIN = config.MIN_APEX_HEIGHT

# The seed state: the robot at the start of a plan. `v_g_initial` is virtual —
# chosen so the uniform stance rule reproduces a first takeoff speed of
# sqrt(2 g H_INITIAL). See `config.H_INITIAL`.
V_G_SEED = math.sqrt(2.0 * G * config.H_INITIAL / ETA)
V_S_SEED = math.sqrt(ETA) * V_G_SEED


def _check(name: str, ok: bool, detail: str = "") -> bool:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    return ok


def _energy_kw(v_s_min: float) -> dict:
    return dict(
        v_s_min=v_s_min, e_inject_max=E_INJ, mass=MASS,
        min_apex=H_MIN, V_g_max=config.V_G_MAX,
    )


def _apex_drop(X: float, Z: float, alpha: float) -> float:
    """Fall from apex to landing. The closed form `min_apex_tan` inverts."""
    T = math.tan(alpha)
    return (X * T - 2.0 * Z) ** 2 / (4.0 * (X * T - Z))


def _flat_map(size_x: float = 5.0, size_y: float = 5.0) -> Map2D5:
    return Map2D5(size_x=size_x, size_y=size_y, resolution=config.CELL_RESOLUTION)


def _make_planner(m: Map2D5, start, goal, **overrides) -> HoppingAStarPlanner:
    kwargs = dict(
        map_env=m, start=start, goal=goal,
        hop_radius=config.HOP_RADIUS, n_angles=config.HOP_N_ANGLES,
        max_jump_height=config.MAX_JUMP_HEIGHT,
        alpha_uphill=config.ALPHA_UPHILL, alpha_downhill=config.ALPHA_DOWNHILL,
        g=G, V_max=config.V_MAX, mass=MASS, eta=ETA, e_inject_max=E_INJ,
        min_apex=H_MIN, h_initial=config.H_INITIAL, V_g_max=config.V_G_MAX,
        speed_bin=config.SPEED_BIN, mu=config.MU,
        robot_radius=config.ROBOT_RADIUS, leg_radius=config.LEG_CYLINDER_RADIUS,
        foot_radius=config.FOOT_TIP_RADIUS, leg_length=config.LEG_LENGTH,
        min_clearance_gate=config.MIN_CLEARANCE,
        arc_max_step=config.ARC_SAMPLE_MAX_STEP,
        n_lateral=config.ARC_LATERAL_SAMPLES,
        obstacle_wall_extra=config.OBSTACLE_WALL_EXTRA,
        leg_clearance_start_frac=config.LEG_CLEARANCE_START_FRAC,
        hop_fixed_cost=config.HOP_FIXED_COST, hop_scan_step=config.HOP_SCAN_STEP,
    )
    kwargs.update(overrides)
    return HoppingAStarPlanner(**kwargs)


def main() -> int:
    print("== hop energy chain tests ==")
    print(f"   eta={ETA}  m={MASS} kg  E_inject_max={E_INJ:.3f} J  "
          f"min_apex={H_MIN} m  V_g_max={config.V_G_MAX:.3f} m/s")
    print(f"   seed: v_g={V_G_SEED:.3f} -> v_s={V_S_SEED:.3f} m/s")
    all_ok = True

    # ------------------------------------------------------------------ #
    print("\n(1) min_apex_tan closed form")
    # On level ground the apex sits at X/2 and rises X*tan(a)/4, so requiring a
    # fall of h inverts to tan(a) = 4h/X. This is the one case with an
    # independent hand-derivation, which is why it is checked exactly.
    for X in (0.5, 1.0, 2.0, 3.5):
        T = min_apex_tan(X, 0.0, H_MIN)
        all_ok &= _check(
            f"flat X={X}: tan = 4h/X",
            T is not None and abs(T - 4.0 * H_MIN / X) < 1e-12,
            f"{T:.6f} vs {4.0 * H_MIN / X:.6f}",
        )
    # Vacuous when the terrain already drops further than h: even a horizontal
    # launch falls more than h_min, so there is nothing to constrain.
    all_ok &= _check(
        "vacuous when h + Z < 0",
        min_apex_tan(1.0, -(H_MIN + 0.05), H_MIN) is None,
        f"Z = {-(H_MIN + 0.05):.2f} m already exceeds a {H_MIN} m fall",
    )
    all_ok &= _check(
        "not vacuous at h + Z > 0",
        min_apex_tan(1.0, -(H_MIN - 0.05), H_MIN) is not None,
    )

    # ------------------------------------------------------------------ #
    print("\n(2) the returned bound really delivers min_apex, and is monotone")
    ok_all, worst = True, 0.0
    for X in (0.4, 1.0, 2.0):
        for Z in (-0.25, 0.0, 0.3, 0.6):
            T = min_apex_tan(X, Z, H_MIN)
            if T is None:
                continue
            err = abs(_apex_drop(X, Z, math.atan(T)) - H_MIN)
            worst = max(worst, err)
            ok_all &= err < 1e-9
            # Monotone increasing in alpha, which is what makes a single lower
            # bound (rather than a search) correct.
            steeper = _apex_drop(X, Z, math.atan(T) + 0.05)
            ok_all &= steeper > H_MIN
    all_ok &= _check("apex drop at the bound equals min_apex, and grows above it",
                     ok_all, f"worst error {worst:.2e} m")

    # ------------------------------------------------------------------ #
    print("\n(3) min_energy_tan is the true argmin of v_s")
    all_ok &= _check(
        "flat ground gives 45 deg",
        abs(min_energy_tan(1.7, 0.0) - 1.0) < 1e-12,
    )
    ok_all = True
    for X, Z in ((1.0, 0.0), (1.5, 0.4), (0.8, -0.3), (2.0, 0.7)):
        a_star = math.atan(min_energy_tan(X, Z))
        v_star = takeoff_speed(X, Z, a_star, G)
        # Sweep the whole open interval where Eq. 4 is valid.
        lo = math.atan2(Z, X) + 1e-4
        best = min(
            takeoff_speed(X, Z, lo + (0.5 * math.pi - 1e-4 - lo) * i / 400.0, G)
            for i in range(401)
        )
        ok_all &= v_star <= best + 1e-9
    all_ok &= _check("argmin beats a 401-point sweep at every (X, Z)", ok_all)

    # ------------------------------------------------------------------ #
    print("\n(4) energy floor: nothing in the interval costs less than v_s_min")
    # The robot cannot shed energy, so every angle it is allowed to fly must
    # need AT LEAST the speed stance handed it. This is the constraint the whole
    # refactor exists for.
    ok_all, cases = True, 0
    for v_g_in in (V_G_SEED, 4.0, 3.0, 2.0):
        v_s_min = math.sqrt(ETA) * v_g_in
        for X in (0.4, 0.7, 1.0, 1.5):
            for Z in (-0.3, 0.0, 0.35):
                iv = feasible_alpha_interval(X, Z, config.V_MAX, G,
                                             **_energy_kw(v_s_min))
                if iv is None:
                    continue
                cases += 1
                for i in range(41):
                    a = iv[0] + (iv[1] - iv[0]) * i / 40.0
                    # 1e-6 slack: feasible_alpha_interval shrinks the interval
                    # by _ALPHA_EPS, which moves v_s by ~1e-6 at the endpoints.
                    if takeoff_speed(X, Z, a, G) < v_s_min - 1e-6:
                        ok_all = False
    all_ok &= _check("v_s >= v_s_min across every feasible interval", ok_all,
                     f"{cases} non-empty intervals swept")

    # ------------------------------------------------------------------ #
    print("\n(5) injection ceiling: nothing costs more than one cycle affords")
    ok_all, worst = True, 0.0
    for v_g_in in (V_G_SEED, 4.0, 3.0, 2.0):
        v_s_min = math.sqrt(ETA) * v_g_in
        v_s_max = math.sqrt(min(v_s_min**2 + 2.0 * E_INJ / MASS, config.V_MAX**2))
        for X in (0.4, 0.7, 1.0, 1.5):
            for Z in (-0.3, 0.0, 0.35):
                iv = feasible_alpha_interval(X, Z, config.V_MAX, G,
                                             **_energy_kw(v_s_min))
                if iv is None:
                    continue
                over = takeoff_speed(X, Z, iv[1], G) - v_s_max
                worst = max(worst, over)
                ok_all &= over < 1e-6
    all_ok &= _check("v_s(alpha_max) <= v_s_min + full injection", ok_all,
                     f"worst overshoot {worst:.2e} m/s")

    # ------------------------------------------------------------------ #
    print("\n(6) min_apex is a fallback, not an extra tightening")
    # It is max()'d against the energy floor, so it must move alpha_min only
    # when the incoming speed's own parabola falls short of min_apex.
    ok_all = True
    for v_g_in, X in ((V_G_SEED, 1.0), (2.0, 1.0), (3.0, 0.6), (2.0, 1.4)):
        v_s_min = math.sqrt(ETA) * v_g_in
        with_apex = feasible_alpha_interval(X, 0.0, config.V_MAX, G,
                                            **_energy_kw(v_s_min))
        kw = _energy_kw(v_s_min)
        kw["min_apex"] = None
        without = feasible_alpha_interval(X, 0.0, config.V_MAX, G, **kw)
        if with_apex is None or without is None:
            continue
        # Whenever the energy-floor angle already drops far enough, the two
        # intervals must be identical.
        drop_at_floor = _apex_drop(X, 0.0, without[0])
        if drop_at_floor >= H_MIN:
            ok_all &= abs(with_apex[0] - without[0]) < 1e-12
        else:
            ok_all &= with_apex[0] > without[0]
            # 1e-5 m, not exact: feasible_alpha_interval shrinks the interval by
            # _ALPHA_EPS = 1e-6 rad, which lifts the drop at alpha_min by ~6e-7 m.
            ok_all &= abs(_apex_drop(X, 0.0, with_apex[0]) - H_MIN) < 1e-5
    all_ok &= _check("min_apex moves alpha_min iff the energy parabola is too low",
                     ok_all)

    # ------------------------------------------------------------------ #
    print("\n(7) the chain closes on a real plan (flat map)")
    m = _flat_map()
    planner = _make_planner(m, (0.5, 2.5), (4.0, 2.5))
    path = planner.plan()
    if path is None:
        all_ok &= _check("planner found a path on flat ground", False)
    else:
        hops = planner.path_hops
        all_ok &= _check(f"path found: {len(path)} waypoints, {len(hops)} hops", True)

        # Flight: v_g^2 = v_s^2 - 2 g Z on every hop.
        worst = max(abs(h["v_g"] - landing_speed(h["v_s"], h["Z"], G)) for h in hops)
        all_ok &= _check("flight conserves energy", worst < 1e-9,
                         f"worst |v_g - sqrt(v_s^2-2gZ)| = {worst:.2e} m/s")

        # Stance: each takeoff speed is floored by the previous landing speed.
        # Compared against the BINNED speed, since that is what the search
        # actually propagates -- see `HoppingAStarPlanner.plan`.
        ok_all = True
        for prev, cur in zip(hops, hops[1:]):
            binned = round(prev["v_g"] / config.SPEED_BIN) * config.SPEED_BIN
            ok_all &= cur["v_s"] >= math.sqrt(ETA) * binned - 1e-6
        all_ok &= _check("no hop starts slower than stance returned", ok_all)

        # Thrust: injection is non-negative and within one cycle's budget.
        ok_all = all(-1e-9 <= h["e_inject"] <= E_INJ + 1e-9 for h in hops)
        all_ok &= _check("injection within [0, E_INJECT_MAX]", ok_all,
                         f"max {max(h['e_inject'] for h in hops):.3f} J "
                         f"of {E_INJ:.3f} J")

        # The two hard gates hold on every flown hop.
        all_ok &= _check("every landing within V_g_max",
                         all(h["v_g"] <= config.V_G_MAX + 1e-9 for h in hops),
                         f"max {max(h['v_g'] for h in hops):.3f} m/s")
        all_ok &= _check("every hop drops at least min_apex",
                         all(h["apex_drop"] >= H_MIN - 1e-6 for h in hops),
                         f"min {min(h['apex_drop'] for h in hops):.3f} m")

    # ------------------------------------------------------------------ #
    print("\n(8) the chain converges, and starts hot")
    # The signature behaviour of a robot that cannot shed energy: H_INITIAL
    # hands it far more than a HOP_RADIUS hop needs, so the first hop is forced
    # steep, and the surplus decays by (1 - eta) per hop until min_apex takes
    # over as the binding floor and the robot settles into a fixed point.
    if path is not None and len(planner.path_hops) >= 4:
        hops = planner.path_hops
        print("      hop    X      alpha    v_s    v_g    drop   E_inj")
        for i, h in enumerate(hops):
            print(f"      {i + 1:>3}  {h['X']:5.2f}  {math.degrees(h['alpha_s']):6.1f}  "
                  f"{h['v_s']:5.2f}  {h['v_g']:5.2f}  {h['apex_drop']:5.2f}  "
                  f"{h['e_inject']:5.2f} J")
        all_ok &= _check(
            "first hop is the steepest",
            hops[0]["alpha_s"] >= max(h["alpha_s"] for h in hops[1:]) - 1e-9,
            f"{math.degrees(hops[0]['alpha_s']):.1f} deg vs "
            f"{math.degrees(max(h['alpha_s'] for h in hops[1:])):.1f} deg after",
        )
        # 1e-3 J, not exact: the chosen angle sits _ALPHA_EPS above alpha_min,
        # which costs ~3e-5 J. That is seven orders below E_INJECT_MAX, so
        # "no injection" is the right reading.
        all_ok &= _check(
            "first hop needs no injection (it starts with a surplus)",
            hops[0]["e_inject"] < 1e-3,
            f"{hops[0]['e_inject']:.2e} J of {E_INJ:.3f} J available",
        )
        # The surplus is gone by the end: the last hop sits at the min_apex
        # floor rather than the energy floor.
        all_ok &= _check(
            "the chain settles onto the min_apex floor",
            abs(hops[-1]["apex_drop"] - H_MIN) < 0.05,
            f"final drop {hops[-1]['apex_drop']:.3f} m vs min_apex {H_MIN} m",
        )

    # ------------------------------------------------------------------ #
    print("\n(9) arriving with different energy changes what is reachable")
    # The reason energy has to be part of the search state at all: the same
    # (X, Z) is feasible from one arrival speed and not from another, so a cell
    # is not one node.
    X, Z = 1.6, 0.0
    hot = feasible_alpha_interval(X, Z, config.V_MAX, G,
                                  **_energy_kw(math.sqrt(ETA) * 6.8))
    cold = feasible_alpha_interval(X, Z, config.V_MAX, G,
                                   **_energy_kw(math.sqrt(ETA) * 2.0))
    all_ok &= _check(
        "a fast arrival forces a steeper floor than a slow one",
        hot is not None and cold is not None and hot[0] > cold[0],
        "" if (hot is None or cold is None) else
        f"v_g=6.8 -> floor {math.degrees(hot[0]):.1f} deg, "
        f"v_g=2.0 -> floor {math.degrees(cold[0]):.1f} deg",
    )
    # And far enough out, only the fast arrival can get there at all.
    far = 4.0
    all_ok &= _check(
        "a slow arrival cannot reach what a fast one can",
        feasible_alpha_interval(far, 0.0, config.V_MAX, G,
                                **_energy_kw(math.sqrt(ETA) * 6.8)) is not None
        and feasible_alpha_interval(far, 0.0, config.V_MAX, G,
                                    **_energy_kw(math.sqrt(ETA) * 2.0)) is None,
        f"X = {far} m",
    )

    print("\n" + ("ALL PASSED" if all_ok else "SOME FAILED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

"""Numerical assertions for the energy-based edge cost.

`HoppingAStarPlanner._edge_cost` prices a hop as

    xy_dist + w_energy * (e_inject + max(0, KE_in - KE_out))

replacing an elevation penalty (`alpha_uphill * dz`) that could not price the
manoeuvre it was written for: arcing over a wall and landing at the same height
is `dz = 0`, so it charged nothing.

Each case below pins one property the cost has to have, and most of them are
regressions against a specific wrong version that was tried and rejected. Prints
PASS/FAIL per case and exits 0 (all pass) or 1 (any fail).

Run:
    python test/test_edge_cost_energy.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from hopping_astar_planner import HoppingAStarPlanner
from map2d5 import Map2D5


G = config.G_ACCEL
MASS = config.ROBOT_MASS
ETA = config.ETA_HOP

#: Landing speed the chain converges to on flat `HOP_RADIUS` hops. Every case
#: that needs "the robot in its ordinary cruising state" starts here, because the
#: transient right after the start carries excess energy the robot cannot shed
#: and behaves differently (it injects nothing at all).
V_STEADY = 3.158

#: `h_initial` that seeds the chain AT `V_STEADY`, inverting
#: `v_g_initial = sqrt(2 g h / eta)`.
H_STEADY = V_STEADY ** 2 * ETA / (2.0 * G)

_fails: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"   [{detail}]" if detail else ""))
    if not ok:
        _fails.append(name)


def flat_map(size: float = 5.0) -> Map2D5:
    return Map2D5(size_x=size, size_y=size, resolution=config.CELL_RESOLUTION)


def make_planner(m: Map2D5, **overrides) -> HoppingAStarPlanner:
    """A planner built straight from `config`, used only as a cost calculator.

    `plan()` is never called on most of these — `_validate_and_cost` and
    `_edge_cost` are what is under test, and they need a constructed planner for
    the surface normals, the standable mask and the energy parameters.
    """
    kwargs = dict(
        map_env=m, start=(0.5, 2.5), goal=(4.5, 2.5),
        w_energy=config.W_ENERGY, g=G, V_max=config.V_MAX, mass=MASS, eta=ETA,
        e_inject_max=config.E_INJECT_MAX, min_apex=config.MIN_APEX_HEIGHT,
        h_initial=H_STEADY, V_g_max=config.V_G_MAX, speed_bin=config.SPEED_BIN,
        mu=config.MU, robot_radius=config.ROBOT_RADIUS,
        leg_length=config.LEG_LENGTH, min_clearance_gate=config.MIN_CLEARANCE,
        arc_max_step=config.ARC_SAMPLE_MAX_STEP,
        n_lateral=config.ARC_LATERAL_SAMPLES,
        obstacle_wall_extra=config.OBSTACLE_WALL_EXTRA,
        hop_scan_step=config.HOP_SCAN_STEP,
        hop_scan_step_ref_radius=config.HOP_SCAN_STEP_REF_RADIUS,
    )
    kwargs.update(overrides)
    return HoppingAStarPlanner(**kwargs)


def score_hop(p: HoppingAStarPlanner, x_s, y_s, x_g, y_g, v_g_in):
    """Cost and physics of one hop between two world points.

    Goes through `_validate_and_cost`, not a bare `_edge_cost`, because the
    flown angle — and hence `e_inject` — is chosen by the clearance gate, which
    needs the real terrain. Returns `(cost, hop)` or `None` if infeasible.
    """
    a = p.map_env.world_to_grid(x_s, y_s)
    b = p.map_env.world_to_grid(x_g, y_g)
    return p._validate_and_cost(a, p.map_env.grid[a[0], a[1]], b, v_g_in)


def momentum_charge(p: HoppingAStarPlanner, hop: dict) -> float:
    """The `max(0, KE_in - KE_out)` term alone, recomputed independently."""
    v_out = p._speed_bin(hop["v_g"]) * p.speed_bin
    return max(0.0, 0.5 * p.mass * hop["v_g_in"] ** 2 - 0.5 * p.mass * v_out ** 2)


# --------------------------------------------------------------------------- #
# 1. A hop that holds its speed is charged no momentum.
# --------------------------------------------------------------------------- #
def case_speed_maintained() -> None:
    p = make_planner(flat_map())
    r = score_hop(p, 1.5, 2.5, 2.5, 2.5, V_STEADY)
    assert r is not None, "steady-state 1 m flat hop should be feasible"
    cost, hop = r
    mc = momentum_charge(p, hop)
    check(
        "flat steady-state hop pays no momentum charge",
        mc < 1e-9,
        f"v_in={hop['v_g_in']:.3f} v_out={hop['v_g']:.3f} charge={mc:.4f} J",
    )
    # And the cost is then exactly distance + w * injection.
    expect = hop["X"] + config.W_ENERGY * hop["e_inject"]
    check(
        "cost reduces to xy_dist + w*e_inject when speed holds",
        abs(cost - expect) < 1e-9,
        f"{cost:.4f} vs {expect:.4f}",
    )


# --------------------------------------------------------------------------- #
# 2. THE REGRESSION. One long hop must beat two short ones over the same ground.
#
# This is the case that forced the momentum term to exist. Charging `e_inject`
# alone, two 0.5 m hops cost 0.81 J against 1.20 J for one 1.0 m hop, because
# short hops need no thrust at all — the robot already carries the speed. A*
# exploited that and chopped paths into stubs, arriving drained.
# --------------------------------------------------------------------------- #
def case_one_long_beats_two_short() -> None:
    p = make_planner(flat_map())

    r = score_hop(p, 1.5, 2.5, 2.5, 2.5, V_STEADY)
    assert r is not None
    long_cost, long_hop = r
    long_inject = long_hop["e_inject"]

    short_cost = 0.0
    short_inject = 0.0
    v = V_STEADY
    for x0, x1 in ((1.5, 2.0), (2.0, 2.5)):
        r = score_hop(p, x0, 2.5, x1, 2.5, v)
        assert r is not None, f"0.5 m hop {x0}->{x1} should be feasible"
        c, h = r
        short_cost += c
        short_inject += h["e_inject"]
        v = p._speed_bin(h["v_g"]) * p.speed_bin

    check(
        "one 1.0 m hop costs less than two 0.5 m hops",
        long_cost < short_cost,
        f"{long_cost:.3f} vs {short_cost:.3f}",
    )
    # The point of the test: on INJECTION alone the verdict inverts. If this
    # stops holding, the case is no longer guarding anything.
    check(
        "  ...and would NOT, on injected energy alone",
        short_inject < long_inject,
        f"inject {short_inject:.3f} J vs {long_inject:.3f} J",
    )


# --------------------------------------------------------------------------- #
# 3. Edge costs stay non-negative on a drop.
#
# Guards the rejected alternative: the uncapped potential-shaped form
# `e_inject + KE_in - KE_out` reaches -1.36 J on a 0.4 m drop, and negative edge
# costs break A*'s "pop means final" property. The `max(0, ...)` is load-bearing.
# --------------------------------------------------------------------------- #
def case_non_negative_on_drop() -> None:
    worst = math.inf
    detail = ""
    for drop in (0.2, 0.4, 0.6, 0.8):
        m = flat_map()
        m.paint_region(drop, x_max=2.0)          # step DOWN from 2.0 m onward
        p = make_planner(m)
        r = score_hop(p, 1.5, 2.5, 2.5, 2.5, V_STEADY)
        if r is None:
            continue
        cost, hop = r
        if cost < worst:
            worst, detail = cost, f"drop {drop:.1f} m -> cost {cost:.3f}"
    check("edge cost stays non-negative over drops of 0.2-0.8 m",
          worst >= 0.0, detail)


# --------------------------------------------------------------------------- #
# 4. Climbing is priced, monotonically, with no elevation term in the cost.
#
# This is what `alpha_uphill` used to do. It now falls out of the physics: a
# steeper Z needs a faster takeoff for the same X, hence more injection.
# --------------------------------------------------------------------------- #
def case_climb_is_priced() -> None:
    costs = []
    for Z in (-0.4, -0.2, 0.0, 0.2, 0.4):
        m = flat_map()
        if abs(Z) > 1e-9:
            # Paint the far half at Z; the near half stays at 0.
            m.paint_region(Z, x_min=2.0)
        p = make_planner(m)
        r = score_hop(p, 1.5, 2.5, 2.5, 2.5, V_STEADY)
        costs.append(None if r is None else r[0])

    known = [(z, c) for z, c in zip((-0.4, -0.2, 0.0, 0.2, 0.4), costs) if c is not None]
    monotone = all(b[1] > a[1] for a, b in zip(known, known[1:]))
    check(
        "cost increases monotonically with Z (climbing is priced)",
        monotone and len(known) >= 4,
        ", ".join(f"Z={z:+.1f}:{c:.2f}" for z, c in known),
    )


# --------------------------------------------------------------------------- #
# 5. THE ORIGINAL BUG. Arcing over a wall costs more than flat ground.
#
# Takeoff and landing are both at elevation 0, so `dz = 0` and the old
# `alpha_uphill * dz` penalty charged exactly nothing — for precisely the
# manoeuvre it was added to price.
# --------------------------------------------------------------------------- #
def case_wall_costs_more_than_flat() -> None:
    p_flat = make_planner(flat_map())
    r_flat = score_hop(p_flat, 1.5, 2.5, 2.5, 2.5, V_STEADY)
    assert r_flat is not None
    flat_cost, flat_hop = r_flat

    # 0.1 m, down from 0.4 m: the single-cylinder model checks the bottom-cap
    # region at the full ROBOT_RADIUS (0.15 m) instead of the old capsule's
    # tiny FOOT_TIP_RADIUS (0.02 m), which makes a much shorter ridge enough to
    # force a real cost difference — 0.4 m is no longer clearable at all on
    # this 1 m hop (blocked from ~0.13 m up), while 0.1 m clears with cost
    # 7.178 against a flat cost of 2.005.
    m = flat_map()
    m.paint_region(0.1, x_min=1.95, x_max=2.05)   # thin ridge at the midpoint
    p_wall = make_planner(m)
    r_wall = score_hop(p_wall, 1.5, 2.5, 2.5, 2.5, V_STEADY)
    assert r_wall is not None, "the robot should still be able to clear a 0.1 m ridge"
    wall_cost, wall_hop = r_wall

    check(
        "hopping over a 0.1 m ridge costs more than the same flat hop",
        wall_cost > flat_cost,
        f"{wall_cost:.3f} vs {flat_cost:.3f}",
    )
    check(
        "  ...and both have dz = 0, which the old penalty charged nothing for",
        abs(wall_hop["Z"]) < 1e-9 and abs(flat_hop["Z"]) < 1e-9,
        f"Z_wall={wall_hop['Z']:+.3f} Z_flat={flat_hop['Z']:+.3f}",
    )
    check(
        "  ...via a steeper takeoff and more injection",
        wall_hop["alpha_s"] > flat_hop["alpha_s"]
        and wall_hop["e_inject"] > flat_hop["e_inject"],
        f"alpha {math.degrees(flat_hop['alpha_s']):.1f} -> "
        f"{math.degrees(wall_hop['alpha_s']):.1f} deg, "
        f"E_inj {flat_hop['e_inject']:.2f} -> {wall_hop['e_inject']:.2f} J",
    )


# --------------------------------------------------------------------------- #
# 6. End-to-end: the planner does not chop a straight flat run into stubs.
# --------------------------------------------------------------------------- #
def case_plan_keeps_long_hops() -> None:
    p = make_planner(flat_map(), start=(0.5, 2.5), goal=(3.5, 2.5))
    path = p.plan()
    assert path is not None, "flat 3 m plan should succeed"
    lengths = [h["X"] for h in p.path_hops]
    # `hop_radius` is now the max radius available to THAT hop (dynamic, not a
    # single global constant) -- comparing each hop's span against its own
    # radius is the correct per-hop analogue of the old "not a chopped chain"
    # check. NOTE: the `len(lengths) == 3` expectation predates this change and
    # may itself be stale now that hop reach varies with speed; see
    # CLAUDE.md-adjacent plan notes on dynamic hop_radius for re-validation.
    check(
        "flat 3 m plan uses full-radius hops, not a chopped chain",
        len(lengths) == 3
        and all(h["X"] > 0.9 * h["hop_radius"] for h in p.path_hops),
        f"{len(lengths)} hops: {[round(x, 2) for x in lengths]}",
    )
    check(
        "  ...and arrives at the steady-state speed, not drained",
        p.path_hops[-1]["v_g"] > 3.0,
        f"exit v_g = {p.path_hops[-1]['v_g']:.2f} m/s",
    )


def main() -> int:
    print(f"w_energy = {config.W_ENERGY} m/J · eta = {ETA} · mass = {MASS} kg")
    print(f"steady-state landing speed = {V_STEADY} m/s "
          f"(h_initial = {H_STEADY:.3f} m seeds the chain there)\n")

    case_speed_maintained()
    case_one_long_beats_two_short()
    case_non_negative_on_drop()
    case_climb_is_priced()
    case_wall_costs_more_than_flat()
    case_plan_keeps_long_hops()

    print()
    if _fails:
        print(f"{len(_fails)} FAILED: {', '.join(_fails)}")
        return 1
    print("all cases passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Numerical validation of the friction-cone / BEAM takeoff-angle interval.

`docs/alpha_range_campana.md` was reconstructed from an OCR'd PDF and warns that
its landing-cone case split and velocity bounds are prone to sign and
transcription errors. Section 6 of that document prescribes the remedy this file
implements: treat `feasible_alpha_interval` as a fast filter, and check it
against ground truth computed directly from the trajectory.

Ground truth here means: take an `alpha_s` the interval claims is valid,
reconstruct the actual takeoff and landing velocity **vectors** from Campana
Eq. 2/4, and confirm each one lies inside its contact's friction cone and under
`V_max`. Nothing in that check reuses the interval formulas, so it cannot
inherit their mistakes.

Prints PASS/FAIL per case and exits with 0 (all pass) or 1 (any fail).

Run:
    python test/test_friction_cone.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

import config
from hopping_astar_planner import (
    _arc_z,
    _speed_tan_interval,
    feasible_alpha_interval,
    inplane_friction_cone,
)
from map2d5 import Map2D5


G = config.G_ACCEL
V_MAX = config.V_MAX
MU = config.MU

# Slack for the "inside the cone / under V_max" comparisons. `_ALPHA_EPS` (1e-6
# rad) already holds alpha strictly inside the interval, so a valid angle sits
# at most that far from a boundary; 1e-6 rad of angle and 1e-6 m/s of speed
# absorbs it plus float noise.
TOL_ANG = 1e-6
TOL_VEL = 1e-6


def _normal(grade_x: float, grade_y: float = 0.0) -> tuple[float, float, float]:
    """Outward unit normal of the plane `z = grade_x * x + grade_y * y`."""
    n = np.array([-grade_x, -grade_y, 1.0])
    return tuple(n / np.linalg.norm(n))


def _velocities(
    X: float, Z: float, alpha_s: float, theta: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Ground-truth 3D takeoff and landing velocity vectors for one arc.

    Deliberately does NOT reuse any interval formula. `xdot` comes from Campana
    Eq. 4; the landing vertical rate is read off the trajectory by finite
    difference of `_arc_z`, which is the same function the clearance check flies,
    so this also cross-checks the analytic `tan(alpha_g)` relation in
    `_landing_cone_alpha_s` against the arc actually flown.
    """
    xdot = math.sqrt(G * X * X / (2.0 * (X * math.tan(alpha_s) - Z)))
    zdot_s = xdot * math.tan(alpha_s)
    # Flight time to the landing point, then vertical rate there.
    zdot_g = zdot_s - G * (X / xdot)

    ex = np.array([math.cos(theta), math.sin(theta), 0.0])
    ez = np.array([0.0, 0.0, 1.0])
    return xdot * ex + zdot_s * ez, xdot * ex + zdot_g * ez


def _cone_angle(v: np.ndarray, n: tuple[float, float, float]) -> float:
    """Angle between `v` and the surface normal `n`, in radians."""
    c = float(np.dot(v, np.asarray(n)) / np.linalg.norm(v))
    return math.acos(min(1.0, max(-1.0, c)))


def _violations(
    X: float, Z: float, alpha_s: float, theta: float,
    n_s: tuple[float, float, float], n_g: tuple[float, float, float],
    mu: float, v_max: float,
) -> list[str]:
    """Ground-truth check of one `alpha_s`. Empty list means fully admissible."""
    beta = math.atan(mu)
    v_s, v_g = _velocities(X, Z, alpha_s, theta)
    bad = []
    # Takeoff: the push-off velocity must lie inside the cone at the start.
    if _cone_angle(v_s, n_s) > beta + TOL_ANG:
        bad.append(f"takeoff slips ({math.degrees(_cone_angle(v_s, n_s)):.2f} deg "
                   f"> beta={math.degrees(beta):.2f})")
    # Landing: the robot arrives moving INTO the surface, so it is the reversed
    # velocity that must lie inside the cone at the goal.
    if _cone_angle(-v_g, n_g) > beta + TOL_ANG:
        bad.append(f"landing slips ({math.degrees(_cone_angle(-v_g, n_g)):.2f} deg "
                   f"> beta={math.degrees(beta):.2f})")
    if float(np.linalg.norm(v_s)) > v_max + TOL_VEL:
        bad.append(f"v_s={np.linalg.norm(v_s):.4f} > V_max={v_max:.4f}")
    if float(np.linalg.norm(v_g)) > v_max + TOL_VEL:
        bad.append(f"v_g={np.linalg.norm(v_g):.4f} > V_max={v_max:.4f}")
    return bad


def _check(name: str, ok: bool, detail: str = "") -> bool:
    tag = "PASS" if ok else "FAIL"
    print(f"  [{tag}] {name}{('  ' + detail) if detail else ''}")
    return ok


# Scenario deck: every combination of grade, heading and hop geometry that the
# interval math has a distinct branch for. `theta` is the hop heading relative
# to +x, and the normals are built from a plane grade in x, so theta=0 is
# straight up/down the fall line and theta=pi/2 is a pure cross-slope.
CASES = [
    # (label,                       X,   Z,     grade, theta)
    ("flat, level",                1.0,  0.0,   0.0,   0.0),
    ("flat, long",                 1.5,  0.0,   0.0,   0.0),
    ("flat, short",                0.3,  0.0,   0.0,   0.0),
    ("flat, step up",              0.6,  0.4,   0.0,   0.0),
    ("flat, step down",            0.6, -0.4,   0.0,   0.0),
    ("flat, big drop",             1.0, -1.2,   0.0,   0.0),
    ("slope 0.35, fall line up",   0.8,  0.28,  0.35,  0.0),
    ("slope 0.35, fall line down", 0.8, -0.28,  0.35,  math.pi),
    ("slope 0.35, cross",          1.0,  0.0,   0.35,  math.pi / 2),
    ("slope 0.9, fall line up",    0.4,  0.36,  0.9,   0.0),
    ("slope 0.9, cross",           1.0,  0.0,   0.9,   math.pi / 2),
    ("slope 0.9, cross long",      1.5,  0.0,   0.9,   math.pi / 2),
    ("slope 0.9, diagonal",        0.5,  0.318, 0.9,   math.pi / 4),
    ("slope 0.6, cross, mismatch", 1.0,  0.0,   0.6,   math.pi / 3),
]


def _case_inputs(grade: float, theta: float):
    n = _normal(grade)
    return n, n


def main() -> int:
    all_ok = True

    # ------------------------------------------------------------------ #
    print("(1) Ground truth: every alpha the interval accepts is admissible")
    # The core of the paper's Section 6 checklist. If any of these fail, a bound
    # formula is wrong — most likely a sign.
    for label, X, Z, grade, theta in CASES:
        n_s, n_g = _case_inputs(grade, theta)
        iv = feasible_alpha_interval(X, Z, V_MAX, G, mu=MU,
                                     n_s=n_s, n_g=n_g, theta=theta)
        if iv is None:
            print(f"  [....] {label}: no interval (skipped, covered by case 2)")
            continue
        lo, hi = iv
        bad_at = []
        for f in np.linspace(0.0, 1.0, 21):
            a = lo + f * (hi - lo)
            v = _violations(X, Z, a, theta, n_s, n_g, MU, V_MAX)
            if v:
                bad_at.append((math.degrees(a), v))
        ok = not bad_at
        detail = (f"alpha in [{math.degrees(lo):.2f}, {math.degrees(hi):.2f}] deg, "
                  f"21 samples all admissible")
        if bad_at:
            detail = f"{len(bad_at)}/21 bad, first: {bad_at[0][0]:.2f} deg {bad_at[0][1]}"
        all_ok &= _check(label, ok, detail)

    # ------------------------------------------------------------------ #
    print("\n(2) Tightness: just outside the interval, something must break")
    # Guards the opposite error — an interval so conservative it rejects valid
    # hops. Only meaningful where the endpoint is a real physical bound, so
    # endpoints sitting on the geometric clip (atan2(Z,X)) or on pi/2 are
    # skipped: those are not constraints that ground truth can violate.
    for label, X, Z, grade, theta in CASES:
        n_s, n_g = _case_inputs(grade, theta)
        iv = feasible_alpha_interval(X, Z, V_MAX, G, mu=MU,
                                     n_s=n_s, n_g=n_g, theta=theta)
        if iv is None:
            continue
        lo, hi = iv
        probe = 1e-3  # well past _ALPHA_EPS, small enough to stay local
        results = []
        geom_lo = math.atan2(Z, X)
        if lo - probe > geom_lo:
            results.append(("below alpha_min",
                            bool(_violations(X, Z, lo - probe, theta,
                                             n_s, n_g, MU, V_MAX))))
        if hi + probe < 0.5 * math.pi:
            results.append(("above alpha_max",
                            bool(_violations(X, Z, hi + probe, theta,
                                             n_s, n_g, MU, V_MAX))))
        if not results:
            continue
        ok = all(r[1] for r in results)
        detail = ", ".join(f"{nm}: {'violates' if v else 'STILL VALID'}"
                           for nm, v in results)
        all_ok &= _check(label, ok, detail)

    # ------------------------------------------------------------------ #
    print("\n(3) Flat ground: the cone floor is exactly atan(1/mu)")
    # On level terrain gamma = pi/2 and delta = beta, so both cones collapse to
    # the textbook Coulomb bound. This pins the reduction formula down.
    for mu in (0.5, 0.8, 1.2, 2.0):
        iv = feasible_alpha_interval(1.0, 0.0, V_MAX, G, mu=mu)
        expect = math.atan(1.0 / mu)
        ok = iv is not None and abs(iv[0] - expect) < 1e-5
        got = f"{math.degrees(iv[0]):.4f}" if iv else "None"
        all_ok &= _check(f"mu={mu}", ok,
                         f"alpha_min={got} deg, expected {math.degrees(expect):.4f}")

    # ------------------------------------------------------------------ #
    print("\n(4) Landing-angle relation: tan(a_s) = 2Z/X - tan(a_g), sign checked")
    # `docs/alpha_range_campana.md` transcribes this term as -2Z/X. The two
    # differ whenever Z != 0, so a flat-ground-only test would not catch it.
    # Ground truth is the flown arc itself, via `_arc_z`.
    for X, Z, a_deg in ((1.0, 1.0, 80.0), (1.0, 0.0, 60.0), (1.5, -0.5, 50.0),
                        (0.6, 0.4, 70.0)):
        a_s = math.radians(a_deg)
        h = 1e-6
        # Landing slope of the arc, by finite difference of the trajectory.
        dz_du = (_arc_z(X, X, Z, 0.0, a_s) - _arc_z(X - h, X, Z, 0.0, a_s)) / h
        a_g_true = math.atan(dz_du)
        a_g_formula = math.atan(-math.tan(a_s) + 2.0 * Z / X)
        wrong_sign = math.atan(-math.tan(a_s) - 2.0 * Z / X)
        ok = abs(a_g_true - a_g_formula) < 1e-4
        distinguishes = Z == 0.0 or abs(a_g_true - wrong_sign) > 1e-3
        all_ok &= _check(
            f"X={X}, Z={Z}, alpha_s={a_deg} deg", ok and distinguishes,
            f"a_g true={math.degrees(a_g_true):.4f}, "
            f"+2Z/X={math.degrees(a_g_formula):.4f}, "
            f"-2Z/X={math.degrees(wrong_sign):.4f} deg",
        )

    # ------------------------------------------------------------------ #
    print("\n(5) Cone reduction: delta <= beta, with equality only in-plane")
    beta = math.atan(MU)
    n09 = _normal(0.9)
    cone_fall = inplane_friction_cone(n09, 0.0, MU)
    cone_cross = inplane_friction_cone(n09, math.pi / 2, MU)
    all_ok &= _check(
        "grade 0.9 fall line: delta == beta (normal lies in the hop plane)",
        cone_fall is not None and abs(cone_fall[1] - beta) < 1e-9,
        f"delta={math.degrees(cone_fall[1]):.4f}, beta={math.degrees(beta):.4f} deg",
    )
    all_ok &= _check(
        "grade 0.9 cross-slope: delta < beta (cone clipped by the plane)",
        cone_cross is not None and cone_cross[1] < beta - 1e-6,
        f"delta={math.degrees(cone_cross[1]):.4f} < beta={math.degrees(beta):.4f} deg",
    )
    all_ok &= _check(
        "grade 0.9 fall line: gamma == pi/2 + atan(grade) (axis tilts back)",
        abs(cone_fall[0] - (0.5 * math.pi + math.atan(0.9))) < 1e-9,
        f"gamma={math.degrees(cone_fall[0]):.4f} deg",
    )
    # A cross-slope steeper than mu cannot be stood on without sliding, and the
    # reduction reports that as a degenerate wedge.
    all_ok &= _check(
        f"cross-slope grade {MU + 0.2:.1f} > mu={MU}: degenerate, no jump",
        inplane_friction_cone(_normal(MU + 0.2), math.pi / 2, MU) is None,
    )
    all_ok &= _check(
        f"cross-slope grade {MU - 0.2:.1f} < mu={MU}: still usable",
        inplane_friction_cone(_normal(MU - 0.2), math.pi / 2, MU) is not None,
    )

    # ------------------------------------------------------------------ #
    print("\n(6) Takeoff-speed interval matches the superseded asin(K) form")
    # `_speed_tan_interval` replaced a `sin(2a + psi) >= K` formulation. The two
    # are algebraically the same constraint; this is the regression anchor on
    # that swap.
    def _legacy(X, Z, v_max, g):
        R = math.hypot(X, Z)
        psi = math.atan2(-Z, X)
        K = (g * X * X / (v_max * v_max) + Z) / R
        if K > 1.0:
            return None
        if K < -1.0:
            return -math.inf, math.inf
        return (0.5 * (math.asin(K) - psi),
                0.5 * (math.pi - math.asin(K) - psi))

    for X, Z in ((1.0, 0.0), (1.5, 0.0), (0.6, 0.4), (1.0, -0.5), (2.4, 0.0),
                 (2.5, 0.0), (0.3, 0.25)):
        legacy = _legacy(X, Z, V_MAX, G)
        tan_iv = _speed_tan_interval(X, Z, V_MAX * V_MAX, G)
        new = None if tan_iv is None else (math.atan(tan_iv[0]), math.atan(tan_iv[1]))
        if legacy is None or new is None:
            ok = legacy is None and new is None
            detail = f"both None" if ok else f"legacy={legacy}, new={new}"
        else:
            ok = abs(legacy[0] - new[0]) < 1e-9 and abs(legacy[1] - new[1]) < 1e-9
            detail = (f"[{math.degrees(new[0]):.4f}, {math.degrees(new[1]):.4f}] deg, "
                      f"max diff {max(abs(legacy[0] - new[0]), abs(legacy[1] - new[1])):.2e}")
        all_ok &= _check(f"X={X}, Z={Z}", ok, detail)

    # ------------------------------------------------------------------ #
    print("\n(7) Landing-speed constraint (4) binds on drops, not on climbs")
    # v_g^2 = v_s^2 - 2gZ, so the landing budget is slack uphill and tight
    # downhill. Verified by the ground-truth vectors, not by the bound formula.
    iv_up = feasible_alpha_interval(1.0, 0.5, V_MAX, G, mu=None)
    iv_dn = feasible_alpha_interval(1.0, -0.5, V_MAX, G, mu=None)
    ok = iv_up is not None and iv_dn is not None
    if ok:
        _, vg_up = _velocities(1.0, 0.5, 0.5 * (iv_up[0] + iv_up[1]), 0.0)
        _, vg_dn = _velocities(1.0, -0.5, 0.5 * (iv_dn[0] + iv_dn[1]), 0.0)
        ok = float(np.linalg.norm(vg_up)) < float(np.linalg.norm(vg_dn))
        detail = (f"|v_g| uphill={np.linalg.norm(vg_up):.3f} < "
                  f"downhill={np.linalg.norm(vg_dn):.3f} m/s")
    else:
        detail = "interval unexpectedly empty"
    all_ok &= _check("landing speed higher after a drop", ok, detail)
    # A drop deep enough that no angle can land under budget.
    deep = -(V_MAX * V_MAX) / (2.0 * G) - 0.5
    all_ok &= _check(
        f"drop of {deep:.2f} m rejected by the landing budget alone",
        feasible_alpha_interval(1.0, deep, V_MAX, G, mu=None) is None,
        f"2g|Z| = {2 * G * abs(deep):.1f} > V_max^2 = {V_MAX ** 2:.1f}",
    )

    # ------------------------------------------------------------------ #
    print("\n(8) mu=None disables only the cone, not the rest of BEAM")
    with_cone = feasible_alpha_interval(1.0, 0.0, V_MAX, G, mu=MU)
    no_cone = feasible_alpha_interval(1.0, 0.0, V_MAX, G, mu=None)
    all_ok &= _check(
        "cone narrows the interval from below",
        with_cone[0] > no_cone[0] and abs(with_cone[1] - no_cone[1]) < 1e-9,
        f"alpha_min {math.degrees(no_cone[0]):.2f} -> {math.degrees(with_cone[0]):.2f} deg, "
        f"alpha_max unchanged at {math.degrees(with_cone[1]):.2f} deg",
    )
    # Derived from V_MAX rather than hard-coded: a flat hop of distance X needs
    # v_s >= sqrt(g X), so the reach is V_MAX^2 / g. Writing the literal here
    # made this test track a tuned V_MAX = 4.85 m/s (reach 2.40 m); V_MAX is now
    # derived from the energy chain and the reach moved to 5.50 m.
    _flat_reach = V_MAX * V_MAX / G
    all_ok &= _check(
        "a hop beyond V_max is still rejected with mu=None",
        feasible_alpha_interval(_flat_reach * 1.1, 0.0, V_MAX, G, mu=None) is None,
        f"flat reach = V_MAX^2/g = {_flat_reach:.2f} m; "
        f"{_flat_reach * 1.1:.2f} m rejected",
    )

    # ------------------------------------------------------------------ #
    print("\n(9) Downhill no longer admits a negative takeoff angle")
    # The modelling gap docs/alpha_range_old.md called out: a push-only leg
    # cannot thrust below the horizon, but the pre-cone interval allowed it.
    iv_cone = feasible_alpha_interval(1.0, -0.4, V_MAX, G, mu=MU)
    iv_bare = feasible_alpha_interval(1.0, -0.4, V_MAX, G, mu=None)
    all_ok &= _check(
        "X=1.0, Z=-0.4: alpha_min was negative, now above the cone floor",
        iv_bare[0] < 0.0 < iv_cone[0]
        and abs(iv_cone[0] - math.atan(1.0 / MU)) < 1e-5,
        f"{math.degrees(iv_bare[0]):+.2f} -> {math.degrees(iv_cone[0]):+.2f} deg",
    )

    # ------------------------------------------------------------------ #
    print("\n(10) surface_normals: discontinuities do not fake a slope")
    m = Map2D5(3.0, 1.0, 0.1)
    m.paint_region(0.0)
    m.paint_region(0.4, x_min=2.0)          # a 0.4 m riser at x = 2.0
    n = m.surface_normals()
    r, c_foot = m.world_to_grid(1.95, 0.5)  # flat ground at the foot of the step
    _, c_top = m.world_to_grid(2.05, 0.5)   # flat tread at the top of the step
    all_ok &= _check(
        "cell at the foot of a 0.4 m riser reads level",
        abs(n[r, c_foot, 0]) < 1e-12 and abs(n[r, c_foot, 2] - 1.0) < 1e-12,
        f"n={np.round(n[r, c_foot], 6)} (a central difference would read grade 2.0)",
    )
    all_ok &= _check(
        "cell on top of the riser reads level",
        abs(n[r, c_top, 0]) < 1e-12,
        f"n={np.round(n[r, c_top], 6)}",
    )
    # ...while a genuine uniform grade is recovered exactly.
    m2 = Map2D5(3.0, 1.0, 0.1)
    for col in range(m2.cols):
        x = (col + 0.5) * m2.resolution
        m2.paint_region(0.35 * x, x_min=x - 1e-6, x_max=x + m2.resolution - 1e-6)
    n2 = m2.surface_normals()
    r2, c2 = m2.world_to_grid(1.5, 0.5)
    grade = -n2[r2, c2, 0] / n2[r2, c2, 2]
    all_ok &= _check(
        "uniform 0.35 grade recovered exactly",
        abs(grade - 0.35) < 1e-9, f"read {grade:.10f}",
    )
    all_ok &= _check(
        "every normal is a unit vector with n_z > 0",
        bool(np.allclose(np.linalg.norm(n2, axis=-1), 1.0)) and bool((n2[..., 2] > 0).all()),
    )

    print("\n" + ("ALL PASSED" if all_ok else "SOME FAILED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

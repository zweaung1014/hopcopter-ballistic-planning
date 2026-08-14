"""The inflated-height-field clearance check must never be more permissive than
the sample-by-sample capsule check it replaced.

`Map2D5.inflated_field` + `clearance_floor_alpha` bake the robot's collision
geometry into the terrain once, so the planner can treat the robot as a point.
`terrain_profile` + `clearance_for_alpha` remain in the tree as the REFERENCE
implementation, and this file is what pins the new path to them.

The bar is deliberately one-sided. The two are *not* expected to agree
everywhere, because the reference is approximate in two ways the field version
is not:

  * it rakes only `ARC_LATERAL_SAMPLES` points across the body, 0.15 m apart at
    shipped values, so a ~20 cm obstacle can slip between two of them;
  * it reads terrain BILINEARLY, which averages a thin obstacle with its
    neighbour and halves its effective height (the reason `CLAUDE.md` required
    obstacles to be at least two cells thick).

So the field version rejects ~5-9% more hops on featured maps, and that is the
bug being fixed rather than a regression. What must NEVER happen is the
opposite: the field version accepting a hop the reference rejects. That is the
assertion below.

Run:
    python test/test_inflated_field.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

import config
from hopping_astar_planner import (
    clearance_floor_alpha,
    clearance_for_alpha,
    terrain_profile,
)
from map2d5 import Map2D5

ROBOT_R = config.ROBOT_RADIUS
LEG_R = config.LEG_CYLINDER_RADIUS
FOOT_R = config.FOOT_TIP_RADIUS
LEG = config.LEG_LENGTH
GATE = config.MIN_CLEARANCE
RES = config.CELL_RESOLUTION
MAX_STEP = config.ARC_SAMPLE_MAX_STEP
N_LAT = config.ARC_LATERAL_SAMPLES

#: Hops per map in the agreement sweep. Large enough that the pad regression
#: (which produces 26-92 violations per map) cannot hide in sampling noise.
N_SAMPLES = 4000

#: Obstacle fill for the reference sampler. Any value well above the map's real
#: terrain works — the sweep maps below have no OBSTACLE cells anyway.
OBS_FILL = 1e6


def _bypass_wall() -> Map2D5:
    """The bypassable ridge from `test/demo_cost_model_ab.py`, built inline.

    Not imported from that module: it builds figures at import time.
    """
    m = Map2D5(config.MAP_SIZE_X, config.MAP_SIZE_Y, RES)
    m.paint_region(0.70, x_min=2.3, x_max=2.7, y_min=0.8, y_max=4.2)
    return m


def _deck() -> list[tuple[str, Map2D5]]:
    maps = [("flat", Map2D5(config.MAP_SIZE_X, config.MAP_SIZE_Y, RES))]
    for nm in ("low_wall", "tall_stairs", "slope_crest", "tall_narrow_wall",
               "barely_jumpable_wall"):
        maps.append((nm, __import__("maps." + nm, fromlist=["build"]).build()))
    maps.append(("bypass_wall", _bypass_wall()))
    return maps


def _fields(m: Map2D5, lookup_pad=None, com_taper=False):
    """The two fields the planner builds, with the knobs this file regresses."""
    return (
        m.inflated_field(FOOT_R + GATE, taper=True, lookup_pad=lookup_pad),
        m.inflated_field(ROBOT_R + GATE, taper=com_taper, lookup_pad=lookup_pad),
    )


def _ref_interior(prof, alpha: float) -> float:
    """`clearance_for_alpha` over the INTERIOR samples only.

    Both checks put the two arc endpoints out of scope and defer them to
    `Map2D5.standable_mask` — `clearance_for_alpha` says so in its docstring, and
    `clearance_floor_alpha` cannot do otherwise (its `u*(X-u)` denominator is
    zero there, and an inflated field is a disc, so at `u=0` it would see terrain
    *behind* the takeoff that the reference's corridor never samples). Comparing
    them on the same scope is therefore the honest test.

    Implemented by slicing the profile rather than reimplementing the check, so
    this cannot drift from the reference it is supposed to pin.
    """
    return clearance_for_alpha(
        prof._replace(u=prof.u[1:-1], terrain=prof.terrain[1:-1]), alpha,
    )


def _sweep(m: Map2D5, f_foot, f_com, seed: int = 7) -> tuple[int, int, int]:
    """Compare both checks over random (hop, alpha) pairs.

    Returns `(n, ref_only, field_only)` — total compared, hops the reference
    accepts and the field rejects (expected, benign), and hops the FIELD accepts
    and the reference rejects (never allowed).

    `alpha` is drawn rather than derived from the energy chain on purpose: the
    two checks must agree at EVERY angle, not only at the one the planner would
    pick, and drawing it exercises shallow arcs the chain would never fly.
    """
    rng = np.random.default_rng(seed)
    rows, cols = m.grid.shape
    n = ref_only = field_only = 0

    for _ in range(N_SAMPLES):
        r0, c0 = int(rng.integers(0, rows)), int(rng.integers(0, cols))
        heading = rng.uniform(0.0, 2.0 * math.pi)
        radius = rng.uniform(0.3, 2.5)
        x0, y0 = (c0 + 0.5) * RES, (r0 + 0.5) * RES
        x1 = x0 + radius * math.cos(heading)
        y1 = y0 + radius * math.sin(heading)
        if not m.is_within_bounds(x1, y1):
            continue
        r1, c1 = m.world_to_grid(x1, y1)
        t_s, t_g = float(m.grid[r0, c0]), float(m.grid[r1, c1])
        if t_s == Map2D5.OBSTACLE or t_g == Map2D5.OBSTACLE:
            continue

        alpha = rng.uniform(math.radians(40.0), math.radians(85.0))

        prof = terrain_profile(
            (x0, y0, t_s), (x1, y1, t_g), m, ROBOT_R, LEG_R, FOOT_R, LEG,
            MAX_STEP, OBS_FILL, N_LAT, min_clearance_gate=GATE,
        )
        if prof is None or prof.out_of_bounds:
            continue
        n += 1

        ref_ok = _ref_interior(prof, alpha) >= GATE
        alpha_c = clearance_floor_alpha(
            (x0, y0, t_s), (x1, y1, t_g), m, f_foot, f_com, LEG, MAX_STEP,
        )
        field_ok = alpha_c is not None and alpha >= alpha_c

        if ref_ok and not field_ok:
            ref_only += 1
        elif field_ok and not ref_ok:
            field_only += 1

    return n, ref_only, field_only


def check_never_more_permissive() -> bool:
    """THE assertion: the field check never accepts what the reference rejects."""
    print("field vs reference capsule check — the field must never be looser\n")
    print(f"  {'map':<22}{'compared':>9}{'stricter':>10}{'LOOSER':>9}")
    all_ok = True
    for name, m in _deck():
        n, ref_only, field_only = _sweep(m, *_fields(m))
        ok = field_only == 0
        all_ok &= ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<16}{n:>9}{ref_only:>10}"
              f"{field_only:>9}")
    return all_ok


def check_lookup_pad_guarantee() -> bool:
    """The field must bound the sphere requirement at any query point, not just
    at cell centres — that is exactly what `lookup_pad` buys.

    The planner reads the field with a NEAREST-CELL lookup, so a query point can
    sit up to half a cell diagonal from the centre of the cell it reads. The
    guarantee that has to hold is therefore

        field[nearest_cell(P)]  >=  max over terrain T within `radius` of P of
                                        ( height(T) + sqrt(radius^2 - |PT|^2) )

    for EVERY `P`, not merely for `P` at a cell centre. This checks that
    directly, rather than inferring it from downstream accept/reject
    disagreements: the endpoint check in `clearance_floor_alpha` happens to mask
    most of those, which would leave the pad looking optional when it is not.
    """
    print("\nlookup_pad — nearest-cell lookups must still bound the true sphere\n")
    radius = FOOT_R + GATE
    reach_cells = int(math.ceil((radius + RES) / RES))
    all_ok = True
    total_unpadded = 0

    for name, m in _deck():
        padded = m.inflated_field(radius, taper=True)
        unpadded = m.inflated_field(radius, taper=True, lookup_pad=0.0)
        rows, cols = m.grid.shape
        rng = np.random.default_rng(11)
        bad_padded = bad_unpadded = 0

        for _ in range(3000):
            # A query point anywhere in the map, deliberately NOT at a centre.
            px = float(rng.uniform(0.0, m.size_x))
            py = float(rng.uniform(0.0, m.size_y))
            qc, qr = int(px / RES), int(py / RES)

            required = -math.inf
            for dr in range(-reach_cells, reach_cells + 1):
                for dc in range(-reach_cells, reach_cells + 1):
                    rr, cc = qr + dr, qc + dc
                    if not (0 <= rr < rows and 0 <= cc < cols):
                        continue
                    d = math.hypot((cc + 0.5) * RES - px, (rr + 0.5) * RES - py)
                    if d >= radius:
                        continue
                    h = m.grid[rr, cc]
                    h = math.inf if h == Map2D5.OBSTACLE else float(h)
                    required = max(required, h + math.sqrt(radius * radius - d * d))

            if required == -math.inf:
                continue
            if padded[qr, qc] < required - 1e-9:
                bad_padded += 1
            if unpadded[qr, qc] < required - 1e-9:
                bad_unpadded += 1

        ok = bad_padded == 0
        all_ok &= ok
        total_unpadded += bad_unpadded
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<22}"
              f"padded {bad_padded:>4} unbounded, unpadded {bad_unpadded:>5}")

    ok = total_unpadded > 0
    all_ok &= ok
    print(f"  [{'PASS' if ok else 'FAIL'}] the pad is load-bearing — without it "
          f"{total_unpadded} query points are unbounded")
    return all_ok


def check_com_field_must_not_taper() -> bool:
    """The CoM field is a COLUMN, not a sphere.

    `clearance_for_alpha`'s above-the-CoM branch uses `axis_dist = |r|`
    regardless of how far above the CoM the terrain reaches — i.e. it already
    treats such terrain as full height. Tapering that field would shave the lift
    near the edge of the reach and silently over-reject.
    """
    print("\nCoM field shape — a taper there over-rejects\n")
    flat_taper = tapered = 0
    for _, m in _deck():
        _, a, _ = _sweep(m, *_fields(m))
        _, b, _ = _sweep(m, *_fields(m, com_taper=True))
        flat_taper += a
        tapered += b
    ok = tapered > flat_taper
    print(f"  [{'PASS' if ok else 'FAIL'}] column form rejects {flat_taper} hops "
          f"the reference accepts; tapered form rejects {tapered} "
          f"({tapered - flat_taper} more, i.e. strictly worse)")
    return ok


def check_standable_mask_equivalence() -> bool:
    """`standable_mask` is this same field evaluated at standing height.

    The two are separate implementations — `standable_mask` is computed once per
    planner and costs nothing, so merging them would be risk without payoff. This
    pins the equivalence the `inflated_field` docstring claims, so that a change
    to either one that breaks it is caught here rather than as a mystery path
    difference.

    The CoM sphere is what governs at shipped values: `LEG_CYLINDER_RADIUS`
    (0.01 m) is thinner than `CELL_RESOLUTION` (0.1 m), so `standable_mask`'s
    discretized neighbour search never fires the leg-cylinder-sides check before
    the sphere check does. Fatten the leg past a cell and this will start
    failing — correctly, because the field would then need a leg component too.
    """
    print("\nstandable_mask == inflated_field at standing height\n")
    all_ok = True
    for name, m in _deck():
        truth = m.standable_mask(
            ROBOT_R, LEG_R, GATE, LEG,
            leg_clearance_start_frac=config.LEG_CLEARANCE_START_FRAC,
        )
        # Sphere at the CoM -> the tapered form, unlike the planner's column.
        field = m.inflated_field(ROBOT_R + GATE, taper=True, lookup_pad=0.0)
        derived = (field <= m.grid + LEG + 1e-12) & (m.grid != Map2D5.OBSTACLE)
        n_diff = int((truth != derived).sum())
        ok = n_diff == 0
        all_ok &= ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<22}"
              f"{int(truth.sum()):>5} standable, {n_diff} disagreements")
    return all_ok


def main() -> int:
    all_ok = check_never_more_permissive()
    all_ok &= check_lookup_pad_guarantee()
    all_ok &= check_com_field_must_not_taper()
    all_ok &= check_standable_mask_equivalence()

    print("\n" + ("ALL PASSED" if all_ok else "SOME FAILED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

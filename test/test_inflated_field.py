"""The planner's clearance check must never accept a hop that collides.

`Map2D5.inflated_field` + `clearance_floor_alpha` bake the robot's collision
geometry (a sharp-edged vertical cylinder plus a uniform safety margin) into the
terrain once, so the planner can treat the robot as a POINT. That is an
optimisation, and optimisations need something to be checked against.

They used to be checked against `terrain_profile` + `clearance_for_alpha`, a
second, slower implementation kept in the source tree for the purpose. That
reference is gone — it answered the same question twice, with a bisection
instead of a closed form, and its corridor read terrain BILINEARLY at the outer
edge of the body's width, which dragged in cells up to a full cell beyond the
body's true reach and reported collisions with walls the robot misses (51 of
18592 sampled hops across this deck).

What replaces it is not a third implementation but a deliberately dumb
assertion, `_collides`, written out longhand below: for every point along the
arc, look at every real map cell within the body's reach, and require the foot
to be `MIN_CLEARANCE` above all of them. No sampling pattern, no interpolation,
no closed form. It is far too slow to plan with and exactly right, which is what
a test wants.

The bar is one-sided. The two are *not* expected to agree everywhere: the field
is read with a NEAREST-CELL lookup, so `lookup_pad` widens it to cover any query
point inside the cell, and that conservatism makes it reject a little more than
the truth requires. What must NEVER happen is the opposite.

Run:
    python test/test_inflated_field.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

import config
from hopping_astar_planner import clearance_floor_alpha
from map2d5 import Map2D5

ROBOT_R = config.ROBOT_RADIUS
LEG = config.LEG_LENGTH
GATE = config.MIN_CLEARANCE
RES = config.CELL_RESOLUTION
MAX_STEP = config.ARC_SAMPLE_MAX_STEP

#: The body's total lateral reach — the radius the terrain is dilated by.
REACH = ROBOT_R + GATE

#: Hops per map in the agreement sweep.
N_SAMPLES = 1500


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


def _disc_max(m: Map2D5, px: float, py: float) -> float:
    """Tallest terrain cell whose CENTRE lies within `REACH` of `(px, py)`.

    Brute force on purpose — this is the definition the field is an
    optimisation of, so it must not share any code with it.
    """
    rc = int(math.ceil(REACH / RES)) + 1
    qc, qr = int(px / RES), int(py / RES)
    best = -math.inf
    for dr in range(-rc, rc + 1):
        for dc in range(-rc, rc + 1):
            rr, cc = qr + dr, qc + dc
            if not (0 <= rr < m.rows and 0 <= cc < m.cols):
                continue
            if math.hypot((cc + 0.5) * RES - px, (rr + 0.5) * RES - py) >= REACH:
                continue
            h = m.grid[rr, cc]
            best = max(best, math.inf if h == Map2D5.OBSTACLE else float(h))
    return best


def _collides(m: Map2D5, c_s, c_g, alpha: float) -> bool:
    """Ground truth: does the body hit anything on this arc, at this angle?

    The model spelled out with no shortcuts. The body is a cylinder with a flat
    bottom at the foot, so the whole question at each point along the arc is
    whether the foot clears every terrain cell within the body's reach by
    `MIN_CLEARANCE`.

    Endpoints and near-endpoint terrain are exempt for the same reason the
    planner exempts them: `u = 0` and `u = X` are stance configurations that
    `Map2D5.standable_mask` gates, and terrain at or below the taller endpoint
    cannot reject a hop that starts and ends standing on it.
    """
    x_s, y_s, t_s = c_s
    x_g, y_g, t_g = c_g
    dx, dy = x_g - x_s, y_g - y_s
    X = math.hypot(dx, dy)

    step = min(MAX_STEP, RES / 3.0)
    n = max(3, int(math.ceil(X / step)) + 1)
    u = np.linspace(0.0, X, n)
    Z = t_g - t_s
    T = math.tan(alpha)
    endpoint_max = max(t_s, t_g)

    for i in range(1, n - 1):
        ui = u[i]
        px, py = x_s + ui * (dx / X), y_s + ui * (dy / X)
        if not (0.0 <= px < m.size_x and 0.0 <= py < m.size_y):
            return True
        h_max = _disc_max(m, px, py)
        if h_max <= endpoint_max:
            continue
        foot_h = t_s + Z * ui * ui / (X * X) + T * ui * (X - ui) / X
        if foot_h < h_max + GATE - 1e-12:
            return True
    return False


def _sweep(m: Map2D5, field, seed: int = 7) -> tuple[int, int, int]:
    """Compare the field check against `_collides` over random (hop, alpha).

    Returns `(n, strict, LOOSE)` — hops compared, hops the field rejects that
    truth allows (expected, benign: the `lookup_pad`'s conservatism), and hops
    the FIELD accepts that truth says collide (never allowed).

    `alpha` is drawn rather than derived from the energy chain on purpose: the
    check must hold at EVERY angle, not only at the one the planner would pick,
    and drawing it exercises shallow arcs the chain would never fly.
    """
    rng = np.random.default_rng(seed)
    rows, cols = m.grid.shape
    n = strict = loose = 0

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
        c_s, c_g = (x0, y0, t_s), (x1, y1, t_g)

        alpha_c = clearance_floor_alpha(
            c_s, c_g, m, field, GATE, LEG, MAX_STEP,
        )
        if alpha_c is None:
            continue
        n += 1

        field_ok = alpha >= alpha_c
        truth_ok = not _collides(m, c_s, c_g, alpha)

        if truth_ok and not field_ok:
            strict += 1
        elif field_ok and not truth_ok:
            loose += 1

    return n, strict, loose


def check_never_accepts_a_collision() -> bool:
    """THE assertion: the field check never accepts a hop that collides."""
    print("field check vs brute-force truth — the field must never be looser\n")
    print(f"  {'map':<22}{'compared':>9}{'stricter':>10}{'LOOSER':>9}")
    all_ok = True
    for name, m in _deck():
        n, strict, loose = _sweep(m, m.inflated_field(REACH))
        ok = loose == 0
        all_ok &= ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<16}{n:>9}{strict:>10}"
              f"{loose:>9}")
    return all_ok


def check_lookup_pad_guarantee() -> bool:
    """The field must bound the true reach at ANY query point, not just centres.

    That is exactly what `lookup_pad` buys, and it is not optional. The planner
    reads the field with a NEAREST-CELL lookup, so a query point can sit up to
    half a cell diagonal from the centre of the cell it reads. The guarantee
    that has to hold is therefore

        field[nearest_cell(P)]  >=  height of every terrain cell within
                                    `REACH` of P

    for EVERY `P`, not merely for `P` at a cell centre. This checks that
    directly rather than inferring it from downstream accept/reject
    disagreements: the endpoint exemption in `clearance_floor_alpha` masks most
    of those, which would leave the pad looking optional when it is not.
    """
    print("\nlookup_pad — nearest-cell lookups must still bound the true reach\n")
    all_ok = True
    total_unpadded = 0

    for name, m in _deck():
        padded = m.inflated_field(REACH)
        unpadded = m.inflated_field(REACH, lookup_pad=0.0)
        rng = np.random.default_rng(11)
        bad_padded = bad_unpadded = 0

        for _ in range(3000):
            # A query point anywhere in the map, deliberately NOT at a centre.
            px = float(rng.uniform(0.0, m.size_x))
            py = float(rng.uniform(0.0, m.size_y))
            qc, qr = int(px / RES), int(py / RES)

            required = _disc_max(m, px, py)
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


def check_field_is_untapered() -> bool:
    """Flat ground must read back as flat ground — nothing added.

    The field is a plain sideways dilation: the tallest terrain in reach, and no
    more. A TAPERED field (the config-space form of a SPHERE) would instead lift
    every cell by up to a full `REACH` even over bare ground, which charges the
    body's width as vertical clearance underneath a body that has no underside.
    That was briefly the implementation. This pins it shut: over flat terrain the
    field must equal the terrain exactly, and over a wall it must equal the
    wall's own height, never more.
    """
    print("\nno taper — the field adds nothing to the terrain it reports\n")
    all_ok = True
    for name, m in _deck():
        field = m.inflated_field(REACH)
        finite = np.isfinite(field)
        ok = bool(np.all(field[finite] <= m.grid.max() + 1e-12))
        flat = Map2D5(2.0, 2.0, RES)
        flat_field = flat.inflated_field(REACH)
        ok &= bool(np.allclose(flat_field, 0.0))
        all_ok &= ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<22}"
              f"peak {float(field[finite].max()):+.3f} m "
              f"(terrain peak {float(m.grid.max()):+.3f} m)")
    return all_ok


def check_standable_mask_wraps_inflated_field() -> bool:
    """`standable_mask` must be exactly `inflated_field` read at standing height.

    There is no independent geometry left in `standable_mask` to regress against
    — it IS `inflated_field(radius + clearance) <= grid + leg_length`, ANDed
    with not-OBSTACLE — so this pins the wrapper against a hand-built version of
    that same expression. A future edit that quietly reintroduces bespoke
    geometry there will show up here as the two diverging.
    """
    print("\nstandable_mask == inflated_field at standing height\n")
    all_ok = True
    for name, m in _deck():
        truth = m.standable_mask(ROBOT_R, GATE, LEG)
        derived = (m.inflated_field(REACH) <= m.grid + LEG) \
            & (m.grid != Map2D5.OBSTACLE)
        n_diff = int((truth != derived).sum())
        ok = n_diff == 0
        all_ok &= ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<22}"
              f"{int(truth.sum()):>5} standable, {n_diff} disagreements")
    return all_ok


def main() -> int:
    all_ok = check_never_accepts_a_collision()
    all_ok &= check_lookup_pad_guarantee()
    all_ok &= check_field_is_untapered()
    all_ok &= check_standable_mask_wraps_inflated_field()

    print("\n" + ("ALL PASSED" if all_ok else "SOME FAILED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

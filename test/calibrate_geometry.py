"""Search for map geometry where the *clearance* gate genuinely rejects hops.

Why this exists
---------------
`min_clearance` skips any arc sample that has not risen `robot_radius` above
BOTH endpoints (hopping_astar_planner.py, the body-guard inside the sample
loop).  On a "climb up onto a plateau" geometry the arc rises only ~0.07–0.24 m
above the landing, so every interior sample is skipped and `min_clearance`
returns `+inf` — the clearance gate cannot reject anything.  `maps/tall_stairs.py`
already concedes this in its docstring.

For clearance to bite, the obstruction must be a LOCAL MAXIMUM exceeding both
endpoints by more than `robot_radius`.  The usable windows are narrow, so this
script sweeps candidate geometries and reports which ones produce a real
contrast: at least one takeoff with finite `mc > 0` and at least one with
finite `mc < 0`, where the rejection comes from clearance rather than from
`feasible_alpha_interval` returning None.

Run:
    python test/calibrate_geometry.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

import config
from demo_common import HOP_RADIUS, V_MAX, make_planner
from hopping_astar_planner import (
    alpha_for_clearance,
    feasible_alpha_interval,
    terrain_profile,
)
from map2d5 import Map2D5


RES = config.CELL_RESOLUTION
PROBE_Y = 2.5                  # centre row for all probes


# ---------------------------------------------------------------------------
# Parameterised map builders (inline — the winning params get frozen into maps/)
# ---------------------------------------------------------------------------

def build_slope(
    ramp_x0: float,
    ramp_x1: float,
    crest_z: float,
    crest_x1: float,
    top_x0: float,
    top_z: float,
) -> Map2D5:
    """Ramp → flat crest (local max) → short back slope → summit plateau."""
    m = Map2D5(size_x=config.MAP_SIZE_X, size_y=config.MAP_SIZE_Y, resolution=RES)
    for col in range(m.cols):
        x = (col + 0.5) * RES
        if x < ramp_x0:
            z = 0.0
        elif x <= ramp_x1:
            z = crest_z * (x - ramp_x0) / (ramp_x1 - ramp_x0)
        elif x <= crest_x1:
            z = crest_z
        elif x < top_x0:
            t = (x - crest_x1) / (top_x0 - crest_x1)
            z = crest_z + (top_z - crest_z) * t
        else:
            z = top_z
        m.grid[:, col] = z
    return m


def build_curbed_stairs(curb1: float, curb2: float, curb3: float) -> Map2D5:
    """`tall_stairs` geometry, with a raised curb at each tread's leading edge.

    Boundaries are world metres, matching `maps/stairs_with_curb.py`, so the
    physical staircase is the same at any resolution. (Column slices would tie
    the curb width to the cell size and silently halve the whole staircase when
    the grid is refined.)
    """
    m = Map2D5(size_x=config.MAP_SIZE_X, size_y=config.MAP_SIZE_Y, resolution=RES)
    m.paint_region(0.0,   x_max=2.0)
    m.paint_region(curb1, x_min=2.0, x_max=2.2)
    m.paint_region(0.40,  x_min=2.2, x_max=2.6)
    m.paint_region(curb2, x_min=2.6, x_max=2.8)
    m.paint_region(0.80,  x_min=2.8, x_max=3.2)
    m.paint_region(curb3, x_min=3.2, x_max=3.4)
    m.paint_region(1.20,  x_min=3.4)
    return m


# ---------------------------------------------------------------------------
# Probe
# ---------------------------------------------------------------------------

def probe(m: Map2D5, x_takeoff: float, x_landing: float, planner) -> dict:
    """Evaluate one hop along y=PROBE_Y, snapped to cell centres like the planner."""
    r0, c0 = m.world_to_grid(x_takeoff, PROBE_Y)
    r1, c1 = m.world_to_grid(x_landing, PROBE_Y)
    p0 = m.grid_to_world(r0, c0)
    p1 = m.grid_to_world(r1, c1)
    z0 = float(m.grid[r0, c0])
    z1 = float(m.grid[r1, c1])

    X = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    Z = z1 - z0
    res = {"x_s": p0[0], "x_g": p1[0], "z0": z0, "z1": z1, "X": X, "Z": Z}

    if X < 1e-9:
        res.update(gate="degenerate", mc=None, alpha_s=None)
        return res
    if X > HOP_RADIUS + 1e-9:
        res.update(gate="out of range", mc=None, alpha_s=None)
        return res

    iv = feasible_alpha_interval(X, Z, planner.V_max, planner.g)
    if iv is None:
        res.update(gate="physics", mc=None, alpha_s=None)
        return res

    profile = terrain_profile(
        (p0[0], p0[1], z0), (p1[0], p1[1], z1),
        m, planner.robot_radius, planner.leg_length,
        planner.arc_max_step, planner._obstacle_fill, planner.n_lateral,
        min_clearance_gate=planner.min_clearance_gate,
    )
    a, mc = alpha_for_clearance(
        profile, iv[0], iv[1],
        planner.min_clearance_gate, planner.alpha_margin_frac,
    )
    if math.isinf(mc):
        gate = "off-map"
    elif mc < planner.min_clearance_gate:
        gate = "clearance"
    else:
        gate = "accept"
    res.update(gate=gate, mc=mc, alpha_s=a)
    return res


def evaluate(m: Map2D5, takeoffs, x_landing: float) -> tuple[list[dict], bool]:
    """Probe every takeoff; report whether a usable accept/reject contrast exists."""
    planner = make_planner(m, disable_clearance=False,
                           start=(0.5, PROBE_Y), goal=(x_landing, PROBE_Y))
    rows = [probe(m, xt, x_landing, planner) for xt in takeoffs]
    has_accept = any(r["gate"] == "accept" for r in rows)
    has_clear_reject = any(r["gate"] == "clearance" for r in rows)
    return rows, (has_accept and has_clear_reject)


def show(title: str, rows: list[dict], usable: bool) -> None:
    flag = "USABLE" if usable else "  --  "
    print(f"\n[{flag}] {title}")
    print(f"    {'x_s':>5} {'x_g':>5} {'X':>5} {'Z':>6} {'alpha':>6} "
          f"{'mc':>9}  gate")
    for r in rows:
        a = f"{math.degrees(r['alpha_s']):.1f}" if r["alpha_s"] is not None else "  -  "
        mc = f"{r['mc']:+.4f}" if r["mc"] is not None and math.isfinite(r["mc"]) else \
             ("   +inf" if r["mc"] is not None else "    -   ")
        print(f"    {r['x_s']:5.2f} {r['x_g']:5.2f} {r['X']:5.2f} {r['Z']:+6.2f} "
              f"{a:>6} {mc:>9}  {r['gate']}")


# ---------------------------------------------------------------------------
# Sweeps
# ---------------------------------------------------------------------------

def sweep_slope() -> list[tuple]:
    print("\n" + "=" * 78)
    print("SLOPE WITH CONVEX CREST")
    print("=" * 78)
    print("Goal: a hop from low on the ramp onto the summit plateau must CLIP the")
    print("crest (finite mc < 0), while a hop from higher up must CLEAR it.")

    winners = []
    # crest_z must exceed top_z by > robot_radius for the body-guard to keep
    # the crest samples in play at all.
    for crest_z, top_z in [
        (1.05, 0.80), (1.00, 0.75), (0.95, 0.70),
        (0.90, 0.60), (0.80, 0.55), (0.70, 0.45),
        (0.60, 0.35), (0.50, 0.25),
    ]:
        for ramp_x0, ramp_x1 in [(1.6, 3.0), (1.2, 3.0), (2.0, 3.0)]:
            m = build_slope(ramp_x0, ramp_x1, crest_z, 3.3, 3.7, top_z)
            landing_x = 4.1
            takeoffs = [round(x, 2) for x in np.arange(2.7, 4.05, 0.2)]
            takeoffs = [t for t in takeoffs if landing_x - t <= HOP_RADIUS + 1e-9]
            rows, usable = evaluate(m, takeoffs, landing_x)
            title = (f"crest_z={crest_z} top_z={top_z} ramp=[{ramp_x0},{ramp_x1}] "
                     f"landing_x={landing_x}")
            if usable:
                winners.append(("slope", crest_z, top_z, ramp_x0, ramp_x1, landing_x))
                show(title, rows, usable)
    if not winners:
        print("\n  No usable slope geometry in this sweep.")
    return winners


def sweep_stairs() -> list[tuple]:
    print("\n" + "=" * 78)
    print("STAIRS WITH CURBED TREAD EDGES")
    print("=" * 78)
    print("Goal: a hop onto tread 1 (z=0.40) must CLIP curb 1 from some takeoffs")
    print("(finite mc < 0) and CLEAR it from others.")

    winners = []
    for curb1 in [0.50, 0.52, 0.55, 0.58, 0.60, 0.65, 0.70, 0.75, 0.80]:
        m = build_curbed_stairs(curb1, curb1 + 0.40, curb1 + 0.80)
        landing_x = 2.3            # tread 1, just behind the curb at col 10
        takeoffs = [round(x, 2) for x in np.arange(0.9, 2.05, 0.2)]
        takeoffs = [t for t in takeoffs if landing_x - t <= HOP_RADIUS + 1e-9]
        rows, usable = evaluate(m, takeoffs, landing_x)
        title = f"curb1={curb1} (tread1=0.40) landing_x={landing_x}"
        if usable:
            winners.append(("stairs", curb1))
            show(title, rows, usable)
    if not winners:
        print("\n  No usable curb height in this sweep.")
    return winners


def main() -> int:
    print("Calibrating geometry for clearance-driven rejection")
    print(f"hop_radius={HOP_RADIUS} m  V_max={V_MAX} m/s  "
          f"robot_radius={config.ROBOT_RADIUS} m  resolution={RES} m")

    slope_winners = sweep_slope()
    stairs_winners = sweep_stairs()

    print("\n" + "=" * 78)
    print("SUMMARY")
    print("=" * 78)
    print(f"  slope geometries with a real accept/reject contrast:  {len(slope_winners)}")
    print(f"  curb heights with a real accept/reject contrast:      {len(stairs_winners)}")
    if slope_winners:
        print(f"\n  suggested slope:  {slope_winners[0]}")
    if stairs_winners:
        print(f"  suggested curb:   {stairs_winners[0]}")

    if not slope_winners and not stairs_winners:
        print("\n  Neither sweep found a usable window — widen the sweep or fall back")
        print("  to labelling the stairs demo as physics-driven (see the plan).")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

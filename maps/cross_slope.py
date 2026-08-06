"""A side-hill: a constant-grade slope the robot must cross to reach the goal.

This map exists for the friction cone (Campana's BEAM constraints 1 and 2). It
is the only map here whose terrain is graded steeply enough, and traversed in
enough different directions, for the cone to be the constraint that decides
whether a hop exists.

Profile (constant in y — the hillside spans the full y range, so there is no way
around it, only across it):

    x in [0.0, 1.0)    z = 0.00                   low bench (start sits here)
    x in [1.0, 3.0)    z = 0.00 -> 1.80  linear   side-hill, grade 0.9
    x in [3.0, 5.0]    z = 1.80                   high bench (goal sits here)

Why grade 0.9
-------------
It has to clear two ceilings from opposite directions.

Below the *standability* ceiling, or the stance gate rejects the hillside before
the cone is ever consulted and the map tests nothing. `Map2D5.standable_mask`
gives two ceilings — the leg-cylinder-sides one at `(L * frac) / (leg_radius +
MIN_CLEARANCE)` ~ 1.21 and the CoM-sphere one at `sqrt((L / (ROBOT_RADIUS +
MIN_CLEARANCE))^2 - 1)` ~ 1.25 — and 0.9 sits comfortably under both.

Below `MU` (1.2), or the cross-slope cone degenerates (`cos(beta)/A > 1`) and no
contact on the hillside is usable at all, which would make the map unplannable
rather than interesting.

And high enough that the cone actually bites: at 0.9 the in-plane cone axis
`gamma` tilts back by `atan(0.9)` = 42 degrees when hopping up the fall line,
putting the takeoff floor at `pi/2 + atan(0.9) - atan(MU)` = 81.8 degrees.

What the cone does here — it is direction-dependent, and nothing else is
--------------------------------------------------------------------------
Every other gate in the planner (leg energy, clearance, stance) depends only on
`X` and `Z`. The friction cone is the only one that also depends on *heading*,
because only the component of the surface normal lying in the hop plane matters.
On this hillside that shows up as a hop-length budget that varies with direction:

  * straight **up the fall line**, the cone axis has tilted 42 degrees back, so
    the takeoff floor (81.8 deg) nearly collides with the leg-energy ceiling;
    only short hops survive;
  * straight **across the fall line**, the normal is perpendicular to the hop
    plane's horizontal axis, so `gamma` stays at 90 degrees and the wedge merely
    narrows (`delta` = 30.5 deg < `beta` = 50.2 deg) — this is the cone/plane
    clipping the paper mentions but does not derive. Full-radius hops are fine;
  * **downhill**, the axis tilts forward and the cone is at its most permissive.

So the robot can cover far more ground per hop traversing the hillside than
climbing it directly, and the planner has to spend more, shorter hops on the
climb. That asymmetry is what `test/demo_friction_cone.py` plots.

A note on what this map deliberately does NOT show: the cone can never make a
standable slope impassable. Standing at all requires grade <= MU, and the
fall-line takeoff floor is `pi/2 + atan(grade) - atan(MU)`, which stays below
pi/2 exactly when `grade < MU`, while the leg-energy ceiling tends to pi/2 as
the hop gets short. So arbitrarily short uphill hops always remain feasible.
The cone caps hop *length* by heading; it does not gate reachability.
"""

import config
from map2d5 import Map2D5

# --- profile constants (exported so demos can draw reference lines) ---------
BENCH_X1 = 1.0    # m; toe of the hillside / end of the low bench
SLOPE_X1 = 3.0    # m; brow of the hillside / start of the high bench
GRADE = 0.9       # dz/dx on the hillside (see module docstring for the bounds)
TOP_Z = GRADE * (SLOPE_X1 - BENCH_X1)   # 1.80 m; high-bench elevation

# Start low and near one edge, goal high and near the other, so the direct route
# crosses the hillside diagonally and the planner has a genuine choice between
# climbing the fall line and traversing across it.
START = (0.5, 0.5)
GOAL = (4.5, 4.5)


def profile_z(x: float) -> float:
    """Terrain elevation at world x (constant in y)."""
    if x < BENCH_X1:
        return 0.0
    if x < SLOPE_X1:
        return GRADE * (x - BENCH_X1)
    return TOP_Z


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )
    res = env_map.resolution
    for col in range(env_map.cols):
        x = (col + 0.5) * res
        # One column at a time, addressed by world bounds rather than by index,
        # so the physical profile is independent of CELL_RESOLUTION.
        env_map.paint_region(profile_z(x), x_min=x - 0.5 * res, x_max=x + 0.5 * res)
    return env_map

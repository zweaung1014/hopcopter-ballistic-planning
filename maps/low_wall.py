"""Low wall map: a short terrain ridge dropped in the middle of flat ground.

Wall dimensions:
  x ∈ [1.7, 1.9]  (0.2 m thick)
  y — the full map (no way around it)
  z = 0.40 m      (low: clearable, but not for free)

This is `maps/flat.py` with one thing added, and it exists for one reason: on
flat ground the takeoff-angle interval is decided entirely by the energy floor
(E1), the flown angle sits exactly on `alpha_min`, and the injection budget is
never touched. Nothing downstream of E1 gets exercised.

A wall changes which angle gets flown. The clearance gate has to lift the arc
above `alpha_min` to get the foot over 0.40 m of terrain, and paying for that
lift is what turns injection on — so this is the scenario where the E2 ceiling
and the least-injection angle selection actually do something visible. See
`test/demo_takeoff_angle_range.py`, which plans across both maps and dumps the
per-constraint construction of every hop's interval.

Geometry notes
--------------
The wall straddles x = 1.75, the midpoint between that demo's START (0.5, 2.5)
and GOAL (3.0, 2.5), so neither endpoint is favoured and the crossing hop has
roughly a full `HOP_RADIUS` of run-up on either side.

It spans the full map in y deliberately. A finite wall would let A* hop around
the end, and the demo would then be measuring a routing decision instead of the
takeoff-angle interval it is built to inspect.

0.2 m is two cells at `CELL_RESOLUTION = 0.1` (centres 1.75 and 1.85), which is
the minimum that works: a one-cell wall gets averaged 50/50 with its flat
neighbour by the bilinear clearance lookup and only blocks arcs at half its
height. Painted from world-metre bounds via `Map2D5.paint_region`, so the
physical wall is the same at any resolution.
"""

import config
from map2d5 import Map2D5

START = (0.5, 2.5)
GOAL  = (4.5, 2.5)


WALL_HEIGHT = 0.4    # m — terrain elevation of the ridge
WALL_XMIN   = 1.7    # m
WALL_XMAX   = 1.9    # m


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )

    # y unbounded: the wall runs the full width of the map.
    env_map.paint_region(
        WALL_HEIGHT,
        x_min=WALL_XMIN, x_max=WALL_XMAX,
    )

    return env_map

"""Tall narrow wall map: a thin terrain ridge that blocks direct east-west travel.

Wall dimensions:
  x ∈ [2.3, 2.7]  (0.4 m thick)
  y ∈ [0.8, 4.2]  (3.4 m — spans almost the full map in y)
  z = 1.40 m      (clearable from most takeoff distances, but not all)

Painted from world-metre bounds via `Map2D5.paint_region`, so the physical wall
is the same at any `CELL_RESOLUTION`.

Parameters below are the DEMO's values (`test/demo_narrow_wall_showcase.py`,
which takes them from `test/demo_common.py`), not `config.py`'s — config ships
`HOP_RADIUS = 1.0`.

Calibration (HOP_RADIUS = 1.5 m, LEG_LENGTH = 0.40 m, ROBOT_RADIUS = 0.15 m,
MIN_CLEARANCE = 0.10 m)
-----------------------------------------------------------------------------
The wall was 0.70 m, calibrated against a tuned `V_MAX = 4.85 m/s` and a
max-margin takeoff angle. Both of those are gone: `V_MAX` is now derived from
the energy chain (7.35 m/s) and the planner flies the least-injection angle. A
robot that cannot shed the speed it arrives with is forced steep, and a steep
arc clears a 0.70 m wall from *every* takeoff — the scenario stopped
demonstrating anything, with both planners returning the identical clean path.

At 1.40 m the contrast is back, and it is the same contrast as before:

    planner    takeoff x   crossing hop
    baseline      2.05      clips the wall (1 bad hop)
    ballistic     1.95      backs off and clears

The baseline takes off too close, so its arc is still rising when it reaches the
wall; the ballistic planner backs the takeoff off by 0.10 m to put the wall
under the arc's flatter middle. Measured sweep, wall height vs. what the
ballistic planner does:

    1.1 – 1.4 m  →  shifts the takeoff, still crosses  (this demo)
    1.6 m and up →  gives up on crossing and detours around the y-end
                    (that is `test/demo_planner_reroute.py`'s scenario)

The wall y-span prevents a single-hop bypass: at y = 2.4 the nearest wall edges
(y = 0.8 and y = 4.2) are 1.6 m and 1.8 m away, both beyond one 1.5 m hop.
"""

import config
from map2d5 import Map2D5


WALL_HEIGHT = 1.40   # m — terrain elevation of the narrow ridge (calibrated; see above)
WALL_XMIN   = 2.3    # m
WALL_XMAX   = 2.7    # m
WALL_YMIN   = 0.8    # m
WALL_YMAX   = 4.2    # m


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )

    env_map.paint_region(
        WALL_HEIGHT,
        x_min=WALL_XMIN, x_max=WALL_XMAX,
        y_min=WALL_YMIN, y_max=WALL_YMAX,
    )

    return env_map

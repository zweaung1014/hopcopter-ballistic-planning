"""Tall narrow wall map: a thin terrain ridge that blocks direct east-west travel.

Wall dimensions:
  x ∈ [2.3, 2.7]  (0.4 m thick)
  y ∈ [0.8, 4.2]  (3.4 m — spans almost the full map in y)
  z = 0.70 m      (clearable from most takeoff distances, but not all)

Painted from world-metre bounds via `Map2D5.paint_region`, so the physical wall
is the same at any `CELL_RESOLUTION`.

Parameters below are the DEMO's values (`test/demo_narrow_wall_showcase.py`,
which takes them from `test/demo_common.py`), not `config.py`'s — config ships
`HOP_RADIUS = 1.0`.

Calibration (HOP_RADIUS = 1.5 m, V_MAX = 4.85 m/s, LEG_LENGTH = 0.40 m,
ROBOT_RADIUS = 0.20 m, MIN_CLEARANCE = 0.15 m)
-----------------------------------------------------------------------------
Clearance of a 1.5 m hop crossing the wall, by takeoff x:

    1.1 → +0.20  accept        1.7 → +0.15  accept
    1.3 → -0.08  REJECT        1.9 → +0.15  accept
    1.5 → +0.15  accept        2.1 → +0.12  REJECT

Two distinct failures bracket the usable band. Taking off at 2.1 m the robot is
too close: the arc is still rising when it reaches the wall. Taking off at 1.3 m
it is too far: the wall lands under the descending limb. The planner has to pick
a takeoff in between, which is what separates its path from the baseline's.

The `+0.15` entries are hops where the default (max-margin) takeoff angle did
not clear and the planner escalated to a steeper one — `alpha_for_clearance`
stops as soon as the gate is met, so the clearance reads exactly at the gate.

The wall y-span prevents a single-hop bypass: at y = 2.4 the nearest wall edges
(y = 0.8 and y = 4.2) are 1.6 m and 1.8 m away, both beyond one 1.5 m hop.
"""

import config
from map2d5 import Map2D5


WALL_HEIGHT = 0.70   # m — terrain elevation of the narrow ridge (calibrated)
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

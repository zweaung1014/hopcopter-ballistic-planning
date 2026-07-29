"""Barely-jumpable wall: a narrow ridge tuned so exactly one takeoff cell clears it.

Wall dimensions
---------------
  x ∈ [2.3, 2.7]  (0.4 m thick)
  y ∈ [0.8, 4.2]  (3.4 m — spans almost the full map in y)
  z = 1.00 m

Painted from world-metre bounds via `Map2D5.paint_region`, so the physical wall
is the same at any `CELL_RESOLUTION`. The taller sibling of
`maps/tall_narrow_wall.py` (0.70 m), which leaves a comfortable band of working
takeoffs; this one leaves a single cell.

Calibration (HOP_RADIUS = 1.5 m, V_MAX = 4.85 m/s, LEG_LENGTH = 0.40 m,
ROBOT_RADIUS = 0.20 m, MIN_CLEARANCE = 0.15 m)
-----------------------------------------------------------------------------
These are the DEMO's parameters (`test/demo_barely_jumpable.py`, sourced from
`test/demo_common.py`), not `config.py`'s — config ships `HOP_RADIUS = 1.0`.

Clearance of a 1.5 m hop crossing the wall, by takeoff x:

    1.3 → -0.38  REJECT   (wall under the descending limb)
    1.5 → -0.01  REJECT   (marginal — clips the far face)
    1.7 → +0.15  accept   ← the only takeoff that works
    1.9 → +0.12  REJECT   (arc still rising at the near face)
    2.1 → -0.18  REJECT   (much too close)

Only the hop from x = 1.7 m clears both wall faces, so the planner has to work
its way to that cell before attempting the crossing rather than approaching as
close as it can. The accepted hop reads exactly `+0.15` because the default
max-margin takeoff angle does not clear and `alpha_for_clearance` escalates to
the shallowest steeper angle that does — this wall demands a near-maximal leg
effort, which the demo plots as leg-energy utilisation.

Wall y-span prevents bypass in a single 1.5 m hop: from the default path at
y = 2.4, the nearest wall edges (y = 0.8 and y = 4.2) are 1.6 m and 1.8 m away.
"""

import config
from map2d5 import Map2D5

WALL_HEIGHT = 1.00   # m (calibrated — see module docstring)
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

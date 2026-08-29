"""Tall ascending staircase spanning the full y-width of the map.

Three steps running east (increasing x):

  Ground: z = 0.0 m   x ∈ [0.0, 2.0)
  Step 1: z = 0.4 m   x ∈ [2.0, 2.6)
  Step 2: z = 0.8 m   x ∈ [2.6, 3.2)
  Top   : z = 1.2 m   x ∈ [3.2, 5.0]

The full y-span means there is no lateral bypass — the robot must ascend the
stairs in the x direction to reach any cell on the top platform.

Geometry is specified in world metres and painted with `Map2D5.paint_region`,
so the physical staircase is identical at any `CELL_RESOLUTION`; refining the
grid only samples the same terrain more finely.

What the gates do here
----------------------
The 0.4 m risers are well inside the leg's budget, so the physics gate is
inert. What bites instead is the *stance* check. The robot's collision volume
is a single cylinder of `ROBOT_RADIUS`, checked at the foot, so the
un-standable band in front of each riser is `ROBOT_RADIUS + MIN_CLEARANCE`
wide — any foot position within that lateral distance of the riser has the
riser's terrain within its stance-check reach. The planner has to place its
takeoff outside those bands — which is what separates the ballistic path from
the clearance-disabled baseline.
"""

import config
from map2d5 import Map2D5

START = (0.5, 2.5)
GOAL  = (4.5, 2.5)

# --- elevation constants ---------------------------------------------------
STEP1_Z = 0.4   # m
STEP2_Z = 0.8   # m
TOP_Z   = 1.2   # m

# --- world-x boundaries of each step ---------------------------------------
STEP1_XMIN = 2.0   # m
STEP2_XMIN = 2.6   # m
TOP_XMIN   = 3.2   # m


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )
    # Applied across the full y-span (no lateral bypass).
    env_map.paint_region(STEP1_Z, x_min=STEP1_XMIN, x_max=STEP2_XMIN)
    env_map.paint_region(STEP2_Z, x_min=STEP2_XMIN, x_max=TOP_XMIN)
    env_map.paint_region(TOP_Z,   x_min=TOP_XMIN)
    return env_map

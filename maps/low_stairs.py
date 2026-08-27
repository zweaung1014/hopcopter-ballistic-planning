"""Three ascending 0.2 m steps in x, inside a bypassable y-band.

Contrast with `maps/tall_stairs.py` (0.4 m risers, full y-span). Here the
risers are shallow enough that stance-standability alone is the interesting
gate (the un-standable band in front of each riser is `ROBOT_RADIUS +
MIN_CLEARANCE = 0.30 m` wide), and the y-band leaves the planner free to
detour around the whole staircase.
"""

import config
from map2d5 import Map2D5

START = (0.5, 2.5)
GOAL  = (4.5, 2.5)

# --- riser elevations ------------------------------------------------------
STEP1_Z = 0.2   # m
STEP2_Z = 0.4   # m
STEP3_Z = 0.6   # m

# --- world-x boundaries of each step (0.5 m treads) ------------------------
STEP1_XMIN = 2.0   # m
STEP2_XMIN = 2.5   # m
STEP3_XMIN = 3.0   # m
STEP3_XMAX = 3.5   # m — beyond this, ground drops back to z = 0

# --- y-band (bypassable in y at either end) --------------------------------
STAIR_YMIN = 1.5   # m
STAIR_YMAX = 3.5   # m


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )

    env_map.paint_region(
        STEP1_Z,
        x_min=STEP1_XMIN, x_max=STEP2_XMIN,
        y_min=STAIR_YMIN, y_max=STAIR_YMAX,
    )
    env_map.paint_region(
        STEP2_Z,
        x_min=STEP2_XMIN, x_max=STEP3_XMIN,
        y_min=STAIR_YMIN, y_max=STAIR_YMAX,
    )
    env_map.paint_region(
        STEP3_Z,
        x_min=STEP3_XMIN, x_max=STEP3_XMAX,
        y_min=STAIR_YMIN, y_max=STAIR_YMAX,
    )

    return env_map

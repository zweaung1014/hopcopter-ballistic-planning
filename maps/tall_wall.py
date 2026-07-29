"""Tall wall map: a uniform obstacle region of height 1.0 m blocking the path.

The wall is bypassable in y — it spans only x in [2.0, 3.0], y in [1.8, 3.0] —
so this is the "go around" scenario rather than a "hop over" one.
"""

import config
from map2d5 import Map2D5

WALL_HEIGHT = 1.0    # m
WALL_XMIN   = 2.0    # m
WALL_XMAX   = 3.0    # m
WALL_YMIN   = 1.8    # m
WALL_YMAX   = 3.0    # m


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

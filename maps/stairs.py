"""Stair-like elevation map: two raised rectangular regions forming steps."""

import config
from map2d5 import Map2D5


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )

    env_map.paint_region(0.4, x_min=2.0, x_max=3.0, y_min=2.4, y_max=3.0)
    env_map.paint_region(0.2, x_min=2.0, x_max=3.0, y_min=1.8, y_max=2.4)

    return env_map

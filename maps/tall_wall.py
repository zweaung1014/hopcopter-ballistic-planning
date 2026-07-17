"""Tall wall map: a uniform obstacle region of height 1.0 m blocking the path."""

import config
from map2d5 import Map2D5


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )

    r_min, c_min = env_map.world_to_grid(2.0, 1.8)
    r_max, c_max = env_map.world_to_grid(3.0, 3.0)
    env_map.grid[r_min:r_max + 1, c_min:c_max + 1] = 1.0

    return env_map

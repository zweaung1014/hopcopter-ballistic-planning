"""Infinite-height wall: a thin OBSTACLE ridge with narrow bypass gaps at the y-ends.

Uses `set_obstacle_region` (not `paint_region`), so the cells carry
`Map2D5.OBSTACLE = -1.0`. `Map2D5.inflated_field` reports `+inf` at OBSTACLE
cells, so no arc can clear this wall — the planner must route around through
the 0.8 m corridors at y ∈ [0.0, 0.8] and y ∈ [4.2, 5.0].

Same y-layout as `maps/tall_narrow_wall.py`; the difference is height —
infinite vs. finite 0.50 m.
"""

import config
from map2d5 import Map2D5

START = (0.5, 2.5)
GOAL  = (4.5, 2.5)

WALL_XMIN = 2.0    # m
WALL_XMAX = 2.3    # m — 0.3 m thick
WALL_YMIN = 0.8    # m
WALL_YMAX = 4.2    # m — 0.8 m bypass corridor at each y-end


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )

    env_map.set_obstacle_region(
        x_min=WALL_XMIN, y_min=WALL_YMIN,
        x_max=WALL_XMAX, y_max=WALL_YMAX,
    )

    return env_map

"""Finite-height platform: a 0.4 m rectangular block wide enough to land on top of.

Contrast with `maps/low_wall.py` (0.2 m thick, full y-span — hop-over-only) and
`maps/tall_wall.py` (1.0 m tall — geometry gate matters). Here the platform is
low enough that `feasible_alpha_interval` is inert, and thick enough in x
(1.0 m ≈ one flat steady-state hop) that landing on the top surface is a real
option alongside routing around it in y.
"""

import config
from map2d5 import Map2D5

START = (0.5, 2.5)
GOAL  = (4.5, 2.5)


PLATFORM_HEIGHT = 0.4    # m
PLATFORM_XMIN   = 2.0    # m
PLATFORM_XMAX   = 3.0    # m — 1.0 m thick
PLATFORM_YMIN   = 1.5    # m
PLATFORM_YMAX   = 3.5    # m — 1.5 m bypass corridor at each y-end


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )

    env_map.paint_region(
        PLATFORM_HEIGHT,
        x_min=PLATFORM_XMIN, x_max=PLATFORM_XMAX,
        y_min=PLATFORM_YMIN, y_max=PLATFORM_YMAX,
    )

    return env_map

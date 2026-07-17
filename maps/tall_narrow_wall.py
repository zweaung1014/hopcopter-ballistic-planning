"""Tall narrow wall map: a thin terrain ridge that blocks direct east-west travel.

Wall dimensions:
  x ∈ [2.3, 2.7]  (0.4 m — three 0.2 m grid cells wide)
  y ∈ [0.8, 4.2]  (3.4 m — spans almost the full map in y)
  z = 0.15 m      (low enough to arc over from the right takeoff distance,
                   too high for a hop that starts too close to the wall)

With hop_radius = 1.5 m and the 0.2 m grid resolution, the east ring
candidate from a node at x ≈ 2.1 enters the wall at u ≈ 0.2 m where the
arc is only 0.175 m above ground — below the clearance threshold of 0.25 m.
The 22.5° ring candidate from the same start ends up at x ≈ 1.9 m, which
enters the wall at u ≈ 0.4 m where the arc is 0.30 m — above the threshold.
The ballistic planner therefore shifts the intermediate waypoint 0.2 m
(one grid cell) to the left so the arc peaks over the wall instead of
clipping it on the way up.

The wall y-span is chosen so the robot cannot bypass it north or south
in a single hop (max Δy per hop = 1.5 m puts the nearest off-wall y at
0.9 m, which is still inside [0.8, 4.2]).
"""

import config
from map2d5 import Map2D5


WALL_HEIGHT = 0.15   # m — terrain elevation of the narrow ridge
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

    r_min, c_min = env_map.world_to_grid(WALL_XMIN, WALL_YMIN)
    r_max, c_max = env_map.world_to_grid(WALL_XMAX, WALL_YMAX)
    env_map.grid[r_min:r_max + 1, c_min:c_max + 1] = WALL_HEIGHT

    return env_map

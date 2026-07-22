"""Barely-jumpable wall: a narrow ridge tuned to the ballistic planner's arc apex.

Wall dimensions
---------------
  x ∈ [2.3, 2.7]  (0.4 m centre-to-centre span — 3 grid cells at resolution 0.2 m)
  y ∈ [0.8, 4.2]  (3.4 m — spans almost the full map in y)
  z = 0.22 m

Physics (HOP_RADIUS = 1.5 m, V_MAX = 6.0 m/s, flat terrain, g = 9.81 m/s²)
----------------------------------------------------------------------------
The midpoint angle for a flat hop is always α_s = 45°.  Due to grid snapping,
a 1.5 m ring hop in the east direction gives an effective cell-to-cell distance
X = 1.6 m (both takeoff and landing sit on cell centres).  The arc equation is:

    z(u) = u − 0.625 u²,    apex at u = 0.8 m → z_apex = 0.400 m

The minimum arc height over the wall cells (centres at x = 2.3 and x = 2.7)
occurs at the entry and exit faces:

    From takeoff cx = 1.7 m:
        u_entry = 0.6 m,  z(0.6) = 0.375 m,  clearance = 0.375 − 0.22 − 0.10 = +0.055 m  ✓

Shifting the takeoff by ±0.2 m (one grid cell) moves the wall to u = 0.4 m or
u = 0.8/1.2 m, where:

    From cx = 1.9 m:  u_entry = 0.4,  z(0.4) = 0.300 m,  clearance = −0.020 m  ✗
    From cx = 1.5 m:  u_entry = 0.8 (apex),  z(0.8) = 0.400 m,
                       but u_exit = 1.2,       z(1.2) = 0.300 m,  clearance = −0.020 m  ✗

Only the hop from cx = 1.7 m clears both wall faces.  The planner must
navigate to x ≈ 1.7 m before attempting the crossing — two grid cells further
west than the greedy straight-line approach would land (cx ≈ 2.1 m).

Wall y-span prevents bypass in a single 1.5 m hop: the nearest wall edges
(y = 0.8 m and y = 4.2 m) are 1.6 m and 1.8 m away from the default path
y = 2.4 m, both beyond a single hop.
"""

import config
from map2d5 import Map2D5

WALL_HEIGHT = 0.22   # m
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

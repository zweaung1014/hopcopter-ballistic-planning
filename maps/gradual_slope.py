"""Continuous 30° ramp inside a bypassable y-band, then a plateau to the map edge.

Grade `tan(30°) ≈ 0.577` sits well under `MU = 1.2` (friction cone permissive)
and well under `STEEP_INFLATE_GRADE = 1.732` (no cells inflate as steep edges,
so `standable_mask` and `inflated_field` are inert on the ramp itself). This
is the "gentle slope" baseline — contrast with `maps/slope_crest.py`
(grade 0.35 with a convex crest, clearance-limited) and `maps/cross_slope.py`
(grade 0.9, cone-limited).

The ramp is painted only inside `y ∈ [1.5, 3.5]`; outside that band the terrain
stays flat at z = 0, so the planner may either climb the ramp or detour in y.
"""

import math

import config
from map2d5 import Map2D5


# --- profile constants (exported so demos can draw reference lines) --------
RAMP_ANGLE_DEG = 30.0
RAMP_GRADE = math.tan(math.radians(RAMP_ANGLE_DEG))   # ≈ 0.5774
RAMP_X0 = 2.0    # m; ramp toe
RAMP_X1 = 3.0    # m; ramp shoulder / start of plateau
TOP_Z = RAMP_GRADE * (RAMP_X1 - RAMP_X0)              # ≈ 0.5774 m

# --- y-band (bypassable in y at either end) --------------------------------
RAMP_YMIN = 1.5   # m
RAMP_YMAX = 3.5   # m


def profile_z(x: float) -> float:
    """Terrain elevation at world x, inside the ramp's y-band."""
    if x < RAMP_X0:
        return 0.0
    if x <= RAMP_X1:
        return RAMP_GRADE * (x - RAMP_X0)
    return TOP_Z


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )
    res = env_map.resolution
    # One column at a time, addressed by world bounds so the physical profile
    # is independent of CELL_RESOLUTION. Painted only inside the y-band; the
    # rest of the map stays at the initial z = 0.
    for col in range(env_map.cols):
        x = (col + 0.5) * res
        env_map.paint_region(
            profile_z(x),
            x_min=x - 0.5 * res, x_max=x + 0.5 * res,
            y_min=RAMP_YMIN, y_max=RAMP_YMAX,
        )
    return env_map

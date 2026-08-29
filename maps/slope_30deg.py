"""Continuous 30° slope inside a bypassable y-band, then a flat platform.

The slope runs east (increasing x) from x = 1.0 m, rising at grade tan(30°) ≈
0.577 until it reaches SLOPE_TOP_Z ≈ 2.02 m at x = 4.5 m.  The flat platform
at ≈ 2.02 m then extends to the map edge.

  Ground:   z = 0.0 m                   x ∈ [0.0, 1.0)
  Slope:    z = (x − 1.0) × tan(30°)   x ∈ [1.0, 4.5],  y ∈ [1.0, 4.0]
  Platform: z ≈ 2.02 m                  x ∈ (4.5, 5.0],  y ∈ [1.0, 4.0]

Grade tan(30°) ≈ 0.577 is well under MU = 1.2 (friction permissive) and under
STEEP_INFLATE_GRADE = 1.732 (no cells inflate as steep edges), so the slope is
comfortably traversable — a gentler counterpart to the 50° variant.

The 1.0 m ground margins north (y ∈ [4.0, 5.0]) and south (y ∈ [0.0, 1.0])
stay flat at z = 0, providing a lateral bypass route identical to stairs.py.
"""

import math

import config
from map2d5 import Map2D5

START = (0.5, 4.5)
GOAL  = (4.75, 2.5)  # centre of the 0.5 m platform at x ∈ (4.5, 5.0]

# --- profile constants (exported so demos can draw reference lines) --------
SLOPE_ANGLE_DEG = 30.0
SLOPE_GRADE = math.tan(math.radians(SLOPE_ANGLE_DEG))  # ≈ 0.5774
SLOPE_X0   = 1.0                                       # m; slope toe
SLOPE_X1   = 4.5                                       # m; slope shoulder / start of platform
SLOPE_TOP_Z = SLOPE_GRADE * (SLOPE_X1 - SLOPE_X0)     # ≈ 2.021 m; platform height

# --- y-band (bypassable in y at either end) --------------------------------
SLOPE_YMIN = 1.0   # m  (matches stairs.py STAIR_YMIN)
SLOPE_YMAX = 4.0   # m  (matches stairs.py STAIR_YMAX)


def profile_z(x: float) -> float:
    """Terrain elevation at world x, inside the slope's y-band."""
    if x < SLOPE_X0:
        return 0.0
    if x <= SLOPE_X1:
        return SLOPE_GRADE * (x - SLOPE_X0)
    return SLOPE_TOP_Z


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )
    res = env_map.resolution
    # One column at a time so the physical profile is independent of
    # CELL_RESOLUTION.  Painted only inside the y-band; the rest of the map
    # (lateral bypass margins) stays at the initial z = 0.
    for col in range(env_map.cols):
        x = (col + 0.5) * res
        env_map.paint_region(
            profile_z(x),
            x_min=x - 0.5 * res, x_max=x + 0.5 * res,
            y_min=SLOPE_YMIN, y_max=SLOPE_YMAX,
        )

    return env_map

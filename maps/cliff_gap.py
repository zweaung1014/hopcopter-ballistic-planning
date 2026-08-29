"""Slope up to a platform, a cliff-gap, then a matching platform with stairs down.

Layout (15 m x 5 m), all inside the y-band y in [1.0, 4.0]:

  Ground:         z = 0.0 m                 x in [0.0, 2.0)
  Slope (30 deg): z = (x - 2.0) * tan(30)  x in [2.0, 5.118]
  Left platform:  z = 1.80 m                x in (5.118, 6.5]
  Cliff / gap:    z = 0.0 m                 x in (6.5, 8.0)   (1.5 m wide)
  Right platform: z = 1.80 m                x in [8.0, 11.5]
  Stair tread 1:  z = 1.20 m                x in (11.5, 11.8]
  Stair tread 2:  z = 0.60 m                x in (11.8, 12.1]
  Ground:         z = 0.0 m                 x in (12.1, 15.0]

The slope reaches exactly SLOPE_TOP_Z = 1.80 m at x = 5.118 m (tan(30 deg) *
3.118 = 1.80018 m, matched to the right platform to 4 decimals) so both
platforms sit at the same height. Grade tan(30 deg) ~= 0.577 is well under
MU = 1.2 (friction permissive) and under STEEP_INFLATE_GRADE = 1.732 (no
cells inflate as steep edges), so the slope is comfortably traversable --
same flavour as maps/slope_30deg.py.

Stair pattern mirrors maps/stairs.py (0.6 m risers, 0.3 m treads) but only
three risers are needed to descend from 1.80 m to ground, so there are two
intermediate treads (at 1.20 m and 0.60 m) plus the ground level east of
them.

The 1.0 m ground margins north (y in [4.0, 5.0]) and south (y in [0.0, 1.0])
stay flat at z = 0, providing a lateral bypass route in the style of
maps/maze.py, maps/slope_30deg.py, and maps/stairs.py.

MAP_SIZE_X is a module-local constant (15.0 m) rather than pulled from
`config` -- the rest of the deck uses `config.MAP_SIZE_X = 5.0`, and bumping
that globally would enlarge every other scenario's search space. Same trick
as maps/maze.py.
"""

import math

import config
from map2d5 import Map2D5

START = (0.5, 2.5)
GOAL  = (10.0, 3.0)

# --- module-local map size (deliberate departure from config; see docstring) --
MAP_SIZE_X = 15.0
MAP_SIZE_Y = config.MAP_SIZE_Y  # 5.0 m, matches maps/maze.py

# --- slope constants (mirrors maps/slope_30deg.py, sized to hit 1.80 m) -------
SLOPE_ANGLE_DEG = 30.0
SLOPE_GRADE = math.tan(math.radians(SLOPE_ANGLE_DEG))  # ~= 0.57735
SLOPE_X0 = 2.0     # m; slope toe
SLOPE_X1 = 5.118   # m; slope shoulder / left platform start
PLATFORM_Z = 1.80  # m; both platforms sit at exactly this height
# Sanity: SLOPE_GRADE * (SLOPE_X1 - SLOPE_X0) ~= 1.80018 m ~= PLATFORM_Z
# (Slope run 3.118 m = 1.80 / tan(30 deg), matched to right platform / stairs.)

# --- left platform ------------------------------------------------------------
LEFT_PLATFORM_XMIN = SLOPE_X1   # 5.118 m
LEFT_PLATFORM_XMAX = 6.5        # m; east edge = west edge of the cliff gap

# --- cliff / ground gap (no paint needed; ground stays at z = 0) --------------
GAP_XMIN = LEFT_PLATFORM_XMAX   # 6.5 m
GAP_XMAX = 8.0                  # m; 1.5 m wide

# --- right platform (top of the descending stairs) ----------------------------
RIGHT_PLATFORM_XMIN = GAP_XMAX  # 8.0 m
RIGHT_PLATFORM_XMAX = 11.5      # m; east edge = start of the first descending tread

# --- descending stairs (mirrors maps/stairs.py: 0.6 m risers, 0.3 m treads) ---
STEP_WIDTH = 0.3   # m, x extent of each tread
STEP_HEIGHT = 0.6  # m, riser height
STAIR_XMIN = RIGHT_PLATFORM_XMAX  # 11.5 m; first tread starts here
# Tread 1: z = 1.20 m, x in [11.5, 11.8]
# Tread 2: z = 0.60 m, x in [11.8, 12.1]
# Ground east of x = 12.1 m -- reached in three risers from PLATFORM_Z.

# --- y-band (bypassable in y at either end; matches maps/slope_30deg.py) ------
BAND_YMIN = 1.0   # m
BAND_YMAX = 4.0   # m


def profile_z(x: float) -> float:
    """Terrain elevation at world x, inside the structured y-band."""
    if x < SLOPE_X0:
        return 0.0
    if x <= SLOPE_X1:
        return SLOPE_GRADE * (x - SLOPE_X0)
    if x <= LEFT_PLATFORM_XMAX:
        return PLATFORM_Z
    if x < GAP_XMAX:
        return 0.0
    if x <= RIGHT_PLATFORM_XMAX:
        return PLATFORM_Z
    # Descending stairs: two intermediate treads, then ground.
    if x <= STAIR_XMIN + STEP_WIDTH:
        return PLATFORM_Z - STEP_HEIGHT           # 1.20 m
    if x <= STAIR_XMIN + 2 * STEP_WIDTH:
        return PLATFORM_Z - 2 * STEP_HEIGHT       # 0.60 m
    return 0.0


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=MAP_SIZE_X,
        size_y=MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )
    res = env_map.resolution

    # Paint the slope column-by-column so the physical profile is independent
    # of CELL_RESOLUTION -- same idiom as maps/slope_30deg.py.
    for col in range(env_map.cols):
        x = (col + 0.5) * res
        if x < SLOPE_X0 or x > SLOPE_X1:
            continue
        env_map.paint_region(
            SLOPE_GRADE * (x - SLOPE_X0),
            x_min=x - 0.5 * res, x_max=x + 0.5 * res,
            y_min=BAND_YMIN, y_max=BAND_YMAX,
        )

    # Left platform.
    env_map.paint_region(
        PLATFORM_Z,
        x_min=LEFT_PLATFORM_XMIN, x_max=LEFT_PLATFORM_XMAX,
        y_min=BAND_YMIN, y_max=BAND_YMAX,
    )

    # Right platform (top of the descending stairs).
    env_map.paint_region(
        PLATFORM_Z,
        x_min=RIGHT_PLATFORM_XMIN, x_max=RIGHT_PLATFORM_XMAX,
        y_min=BAND_YMIN, y_max=BAND_YMAX,
    )

    # Descending stair treads (three 0.6 m risers => two intermediate treads).
    env_map.paint_region(
        PLATFORM_Z - STEP_HEIGHT,      # 1.20 m
        x_min=STAIR_XMIN,
        x_max=STAIR_XMIN + STEP_WIDTH,
        y_min=BAND_YMIN, y_max=BAND_YMAX,
    )
    env_map.paint_region(
        PLATFORM_Z - 2 * STEP_HEIGHT,  # 0.60 m
        x_min=STAIR_XMIN + STEP_WIDTH,
        x_max=STAIR_XMIN + 2 * STEP_WIDTH,
        y_min=BAND_YMIN, y_max=BAND_YMAX,
    )

    return env_map

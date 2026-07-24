"""Tall ascending staircase spanning the full y-width of the map.

Three steps running east (increasing x):

  Ground: z = 0.0 m  cols  0– 9   x ∈ [0.0, 2.0)
  Step 1: z = 0.4 m  cols 10–12   x ∈ [2.0, 2.6)   cell centres 2.1, 2.3, 2.5 m
  Step 2: z = 0.8 m  cols 13–15   x ∈ [2.6, 3.2)   cell centres 2.7, 2.9, 3.1 m
  Top   : z = 1.2 m  cols 16–24   x ∈ [3.2, 5.0]   cell centres 3.3 … 4.9 m

The full y-span (all 25 rows) means there is no lateral bypass — the robot
must ascend the stairs in the x direction to reach any cell on the top platform.

Physics note (HOP_RADIUS=1.5 m, V_MAX=6.0 m/s)
------------------------------------------------
With 0.4 m risers each hop that tries to skip two or more steps at full radius
has K > 1 (Campana infeasibility) or goes out of the map in backward
directions.  The ring visualisation highlights these rejected candidates at
the most-constrained node (usually the start or a near-step node).

For steep uphill arcs (large Δz) the planner's min_clearance body-guard
skips most interior samples (z_arc stays below z_g + robot_radius), so
these hops produce mc = +∞ rather than mc < 0.  The “readjustment” shown
here is therefore physics-infeasibility driven, not terrain-clearance driven.
"""

import config
from map2d5 import Map2D5

# --- elevation constants ---------------------------------------------------
STEP1_Z = 0.4   # m
STEP2_Z = 0.8   # m
TOP_Z   = 1.2   # m

# --- world-x boundaries of each step (left cell-edge = col * resolution) ---
_RES        = config.CELL_RESOLUTION   # 0.2 m
STEP1_XMIN  = 10 * _RES   # 2.0 m
STEP2_XMIN  = 13 * _RES   # 2.6 m
TOP_XMIN    = 16 * _RES   # 3.2 m

# --- column slices ---------------------------------------------------------
# Using explicit integer slices avoids floating-point issues with world_to_grid.
_STEP1_COLS = slice(10, 13)   # cols 10, 11, 12
_STEP2_COLS = slice(13, 16)   # cols 13, 14, 15
_TOP_COLS   = slice(16, 25)   # cols 16 … 24


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=_RES,
    )
    # Apply to ALL rows (full y-span = no lateral bypass).
    env_map.grid[:, _STEP1_COLS] = STEP1_Z
    env_map.grid[:, _STEP2_COLS] = STEP2_Z
    env_map.grid[:, _TOP_COLS]   = TOP_Z
    return env_map

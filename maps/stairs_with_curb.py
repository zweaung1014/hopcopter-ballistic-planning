"""Ascending staircase whose treads carry a raised curb (nosing) at each edge.

Same column layout as `maps/tall_stairs.py`, but the leading cell of every tread
is raised into a curb that stands 0.10 m proud of the tread behind it:

    Ground   z = 0.00   cols  0– 9   x ∈ [0.0, 2.0)
    Curb 1   z = 0.50   col  10      x ∈ [2.0, 2.2)   ← local max over tread 1
    Tread 1  z = 0.40   cols 11–12   x ∈ [2.2, 2.6)
    Curb 2   z = 0.90   col  13      x ∈ [2.6, 2.8)   ← local max over tread 2
    Tread 2  z = 0.80   cols 14–15   x ∈ [2.8, 3.2)
    Curb 3   z = 1.30   col  16      x ∈ [3.2, 3.4)   ← local max over the top
    Top      z = 1.20   cols 17–24   x ∈ [3.4, 5.0]

Applied to all 25 rows, so there is no lateral bypass.

Why the curbs exist
-------------------
On plain stairs the clearance gate is inert.  `min_clearance` discards any arc
sample that has not risen `robot_radius` above BOTH endpoints, and a hop climbing
onto a tread rises only ~0.07–0.24 m above its landing, so every interior sample
is skipped and `min_clearance` returns `+inf`.  `maps/tall_stairs.py` says as much
in its own docstring: its rerouting is driven by the physics feasibility gate,
not by terrain clearance.

A curb that is a genuine local maximum above its tread keeps those samples in
play, so a hop that comes in too flat is rejected on *clearance*.  The usable
window is narrow — `test/calibrate_geometry.py` finds only 0.50 m and 0.52 m
work for a 0.40 m tread; below 0.50 the body-guard swallows the curb, above 0.54
no takeoff clears it at all.  0.50 m is chosen for the larger accept margin:

    takeoff x=0.90 → tread 1 (x=2.30) :  mc = -0.079 m   REJECT (clips curb)
    takeoff x=1.10 → tread 1 (x=2.30) :  mc = -0.091 m   REJECT (clips curb)
    takeoff x=1.30 → tread 1 (x=2.30) :  mc = +0.027 m   accept

The contrast runs the opposite way from `maps/barely_jumpable_wall.py`, where the
planner must step *back* from the wall.  Here an over-long approach comes in on
the descending part of its arc and catches the curb, so the planner has to start
*closer* to the stairs.  Both are the same underlying behaviour: choose the
takeoff distance that puts the obstruction under the useful part of the parabola.
"""

import config
from map2d5 import Map2D5

# --- elevation constants ---------------------------------------------------
TREAD1_Z = 0.40
TREAD2_Z = 0.80
TOP_Z    = 1.20

CURB1_Z  = 0.50   # calibrated; see module docstring
CURB2_Z  = 0.90
CURB3_Z  = 1.30

CURB_RISE = CURB1_Z - TREAD1_Z   # 0.10 m proud of the tread

# --- world-x boundaries (left cell edge = col * resolution) ----------------
_RES       = config.CELL_RESOLUTION   # 0.2 m
CURB1_XMIN  = 10 * _RES   # 2.0 m
TREAD1_XMIN = 11 * _RES   # 2.2 m
CURB2_XMIN  = 13 * _RES   # 2.6 m
TREAD2_XMIN = 14 * _RES   # 2.8 m
CURB3_XMIN  = 16 * _RES   # 3.2 m
TOP_XMIN    = 17 * _RES   # 3.4 m

# --- column slices ---------------------------------------------------------
# Explicit integer slices avoid floating-point edge cases in world_to_grid.
_GROUND_COLS = slice(0, 10)
_CURB1_COL   = 10
_TREAD1_COLS = slice(11, 13)
_CURB2_COL   = 13
_TREAD2_COLS = slice(14, 16)
_CURB3_COL   = 16
_TOP_COLS    = slice(17, 25)


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=_RES,
    )
    env_map.grid[:, _GROUND_COLS] = 0.0
    env_map.grid[:, _CURB1_COL]   = CURB1_Z
    env_map.grid[:, _TREAD1_COLS] = TREAD1_Z
    env_map.grid[:, _CURB2_COL]   = CURB2_Z
    env_map.grid[:, _TREAD2_COLS] = TREAD2_Z
    env_map.grid[:, _CURB3_COL]   = CURB3_Z
    env_map.grid[:, _TOP_COLS]    = TOP_Z
    return env_map

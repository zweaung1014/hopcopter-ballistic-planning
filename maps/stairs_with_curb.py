"""Ascending staircase whose treads carry a raised curb (nosing) at each edge.

Same layout as `maps/tall_stairs.py`, but the leading strip of every tread is
raised into a curb standing `CURB_RISE` proud of the tread behind it:

    Ground   z = 0.00                x ∈ [0.0, 2.0)
    Curb 1   z = TREAD1_Z + RISE     x ∈ [2.0, 2.2)   ← local max over tread 1
    Tread 1  z = 0.40                x ∈ [2.2, 2.6)
    Curb 2   z = TREAD2_Z + RISE     x ∈ [2.6, 2.8)   ← local max over tread 2
    Tread 2  z = 0.80                x ∈ [2.8, 3.2)
    Curb 3   z = TOP_Z    + RISE     x ∈ [3.2, 3.4)   ← local max over the top
    Top      z = 1.20                x ∈ [3.4, 5.0]

Applied across the full y-span, so there is no lateral bypass. Painted from
world-metre bounds, so the physical staircase is resolution-invariant.

Why the curbs exist
-------------------
On plain stairs a climbing hop passes well above the terrain and nothing is
ever rejected on clearance. A curb that is a genuine local maximum above its
tread puts an obstruction under the descending part of the arc, so a hop that
comes in too flat is rejected on *clearance* rather than on physics.

The rise has to clear the robot's own body. With the leg model the arc runs
`LEG_LENGTH` above the terrain and the body extends `ROBOT_RADIUS` below the
arc, so an obstruction only registers once it stands more than
`LEG_LENGTH - ROBOT_RADIUS - MIN_CLEARANCE` above the surface the robot is
landing on. `CURB_RISE` is calibrated against that by
`test/calibrate_geometry.py`; see its output for the accept/reject window.

The contrast runs the opposite way from `maps/barely_jumpable_wall.py`, where
the planner must step *back* from the wall. Here an over-long approach comes in
on the descending part of its arc and catches the curb, so the planner has to
start *closer* to the stairs. Both are the same underlying behaviour: choose the
takeoff distance that puts the obstruction under the useful part of the parabola.
"""

import config
from map2d5 import Map2D5

# --- elevation constants ---------------------------------------------------
TREAD1_Z = 0.40
TREAD2_Z = 0.80
TOP_Z    = 1.20

# Height each curb stands proud of the tread behind it. Must exceed
# LEG_LENGTH - ROBOT_RADIUS - MIN_CLEARANCE (= 0.05 m at shipped config) by a
# real margin or the curb passes under the robot's body unnoticed.
#
# Calibrated at HOP_RADIUS = 1.5 m, V_MAX = 4.85 m/s, LEG_LENGTH = 0.40 m,
# ROBOT_RADIUS = 0.20 m, MIN_CLEARANCE = 0.15 m. Clearance climbing onto
# tread 1 (landing x = 2.35), by takeoff x:
#
#     0.9 → -0.05  REJECT        1.5 → +0.15  accept
#     1.1 → +0.02  REJECT        1.7 → +0.15  accept
#     1.3 → +0.10  REJECT        1.9 → +0.15  accept
#
# Monotone in takeoff distance: an over-long approach arrives on the descending
# part of its arc and catches the curb, so the planner has to start CLOSER to
# the stairs. Below ~0.45 the curb passes under the body and nothing is ever
# rejected; above ~0.70 no takeoff clears it at all.
CURB_RISE = 0.60

CURB1_Z = TREAD1_Z + CURB_RISE
CURB2_Z = TREAD2_Z + CURB_RISE
CURB3_Z = TOP_Z + CURB_RISE

# --- world-x boundaries ----------------------------------------------------
CURB1_XMIN  = 2.0   # m
TREAD1_XMIN = 2.2   # m
CURB2_XMIN  = 2.6   # m
TREAD2_XMIN = 2.8   # m
CURB3_XMIN  = 3.2   # m
TOP_XMIN    = 3.4   # m


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )
    env_map.paint_region(0.0,      x_max=CURB1_XMIN)
    env_map.paint_region(CURB1_Z,  x_min=CURB1_XMIN,  x_max=TREAD1_XMIN)
    env_map.paint_region(TREAD1_Z, x_min=TREAD1_XMIN, x_max=CURB2_XMIN)
    env_map.paint_region(CURB2_Z,  x_min=CURB2_XMIN,  x_max=TREAD2_XMIN)
    env_map.paint_region(TREAD2_Z, x_min=TREAD2_XMIN, x_max=CURB3_XMIN)
    env_map.paint_region(CURB3_Z,  x_min=CURB3_XMIN,  x_max=TOP_XMIN)
    env_map.paint_region(TOP_Z,    x_min=TOP_XMIN)
    return env_map

"""Continuous slope rising to a convex crest, then dropping to a lower shelf.

Unlike every other map here, elevation varies continuously with x rather than in
constant-z blocks, so the clearance check samples a genuinely graded surface
along each arc.

Profile (applied to ALL rows — the full y-span means no lateral bypass, so the
robot has to climb):

    x ∈ [0.0, 3.0]    z = 0.00 → 1.05  linear     ramp, grade 0.35
    x ∈ (3.0, 3.3]    z = 1.05                    crest — LOCAL MAXIMUM
    x ∈ (3.3, 3.7)    z = 1.05 → 0.20  linear     back side of the brow
    x ∈ [3.7, 5.0]    z = 0.20                    far shelf (goal sits here)

Why the ramp grade is 0.35
--------------------------
There is a hard ceiling on how steep a slope this robot can stand on. Its body
is a sphere of `ROBOT_RADIUS` PLUS the upper `(1 - LEG_CLEARANCE_START_FRAC)`
of a leg cylinder of the same radius. On a constant grade `g`, the terrain
`(R + MIN_CLEARANCE)` metres uphill of the foot rises by `g * (R + MIN_CLEARANCE)`.
The leg-cylinder-sides check kicks in whenever that rise exceeds
`L * LEG_CLEARANCE_START_FRAC` (the exempt-slab height), so

    g_max = (L * LEG_CLEARANCE_START_FRAC) / (R + MIN_CLEARANCE)
          = (0.4 * 1/3) / (0.2 + 0.15)
          ≈ 0.38

At the shipped geometry anything steeper is un-standable everywhere, and the
planner returns no path at all. The ramp toe therefore reaches all the way to
x = 0.0 (spreading the 1.05 m climb over 3.0 m for a 0.35 grade), leaving a
0.03 slack under the 0.38 ceiling. (`Map2D5.standable_mask` verifies this
empirically.)

The earlier sphere-only stance model allowed `g_max = 0.553`, so this map used
to ship at 0.50; the leg-cylinder check tightens the ceiling significantly.

Why the crest overshoots the shelf
----------------------------------
With the leg model the robot's body underside rides `LEG_LENGTH - ROBOT_RADIUS`
= 0.20 m above the terrain even at rest, so a bump has to be a substantial local
maximum before it can register at all. A crest level with the far side would
never gate anything.

Dropping the shelf to 0.20 m makes the crest stand 0.85 m proud of it — the
original 0.25 m overshoot, scaled up for the taller robot.

Calibration (HOP_RADIUS = 1.5 m, V_MAX = 4.85 m/s, LEG_LENGTH = 0.40 m,
ROBOT_RADIUS = 0.20 m, MIN_CLEARANCE = 0.15 m), hopping to x = 4.10:

    2.3 → +0.15  accept        3.1 → +0.20  accept
    2.5 → +0.15  accept        3.3 → +0.15  REJECT (reads at the gate, then
    2.7 → +0.15  accept                             the back side clips)
    2.9 → +0.20  accept        3.5 → +0.09  REJECT

Note the direction: the *late* takeoffs fail. Launching from on top of the brow
gives a flat arc that catches the far side on the way down, so the planner has
to commit to a longer hop from further back on the ramp rather than shuffling
over the crest. That is the opposite failure mode from
`maps/barely_jumpable_wall.py`, where the planner must back *away* from the
obstacle — both are the same underlying choice of where to put the obstruction
relative to the useful part of the parabola.

A *uniform* ramp would not work: on a linear surface the takeoff→landing chord
lies exactly on the terrain and the parabola always bulges above it, so clearance
is trivially satisfied everywhere. Convexity is what makes the check bite.
"""

import config
from map2d5 import Map2D5

# --- profile constants (exported so demos can draw reference lines) ---------
RAMP_X0  = 0.0    # m; ramp toe (starts at the map's left edge)
RAMP_X1  = 3.0    # m; ramp shoulder / start of crest
CREST_Z  = 1.05   # m; crest elevation — the local maximum
CREST_X1 = 3.3    # m; end of the flat crest
TOP_X0   = 3.7    # m; start of the far shelf
TOP_Z    = 0.20   # m; far shelf elevation (calibrated; see module docstring)

RAMP_GRADE = CREST_Z / (RAMP_X1 - RAMP_X0)   # 0.35 — stays under the 0.38
                                             # standability ceiling that comes
                                             # from the leg-cylinder-sides
                                             # stance check.


def profile_z(x: float) -> float:
    """Terrain elevation at world x (constant in y)."""
    if x < RAMP_X0:
        return 0.0
    if x <= RAMP_X1:
        return CREST_Z * (x - RAMP_X0) / (RAMP_X1 - RAMP_X0)
    if x <= CREST_X1:
        return CREST_Z
    if x < TOP_X0:
        t = (x - CREST_X1) / (TOP_X0 - CREST_X1)
        return CREST_Z + (TOP_Z - CREST_Z) * t
    return TOP_Z


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )
    res = env_map.resolution
    for col in range(env_map.cols):
        # Cell centres sit at (col + 0.5) * resolution, matching grid_to_world.
        env_map.grid[:, col] = profile_z((col + 0.5) * res)
    return env_map

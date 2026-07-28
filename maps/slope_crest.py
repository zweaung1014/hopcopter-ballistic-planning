"""Continuous slope rising to a convex crest, then a summit plateau.

Unlike every other map here, elevation varies continuously with x rather than in
constant-z blocks, so `get_elevation_bilinear` samples a genuinely graded surface
along each arc.

Profile (applied to ALL rows — the full y-span means no lateral bypass, so the
robot has to climb):

    x ∈ [0.0, 1.6)    z = 0.00                    approach apron
    x ∈ [1.6, 3.0]    z = 0.00 → 1.05  linear     ramp, grade 0.75
    x ∈ (3.0, 3.3]    z = 1.05                    crest — LOCAL MAXIMUM
    x ∈ (3.3, 3.7)    z = 1.05 → 0.80  linear     back side of the brow
    x ∈ [3.7, 5.0]    z = 0.80                    summit plateau (goal sits here)

Why the crest overshoots the plateau
------------------------------------
`min_clearance` ignores any arc sample that has not risen `robot_radius` above
BOTH endpoints.  On a plain ramp-onto-plateau climb the arc rises only ~0.1–0.2 m
above the landing, so every interior sample is skipped, `min_clearance` returns
`+inf`, and the clearance gate can never reject anything — this is the same
limitation `maps/tall_stairs.py` documents.

Making the crest a local maximum 0.25 m ABOVE the summit plateau (well over
`ROBOT_RADIUS = 0.1`) keeps the crest samples in play, so an over-long hop that
would clip the brow is rejected on clearance rather than on physics.  Verified by
`test/calibrate_geometry.py` at HOP_RADIUS=1.5 m, V_MAX=6.0 m/s:

    takeoff x=2.70 → landing x=4.10 :  mc = -0.082 m   REJECT (clips crest)
    takeoff x=2.90 → landing x=4.10 :  mc = -0.046 m   REJECT (clips crest)
    takeoff x=3.10 → landing x=4.10 :  mc = +0.012 m   accept
    takeoff x=3.30 → landing x=4.10 :  mc = +0.149 m   accept

A *uniform* ramp would not work: on a linear surface the takeoff→landing chord
lies exactly on the terrain and the parabola always bulges above it, so clearance
is trivially satisfied everywhere.  Convexity is what makes the check bite.
"""

import config
from map2d5 import Map2D5

# --- profile constants (exported so demos can draw reference lines) ---------
RAMP_X0  = 1.6    # m; ramp toe
RAMP_X1  = 3.0    # m; ramp shoulder / start of crest
CREST_Z  = 1.05   # m; crest elevation — the local maximum
CREST_X1 = 3.3    # m; end of the flat crest
TOP_X0   = 3.7    # m; start of the summit plateau
TOP_Z    = 0.80   # m; summit plateau elevation

RAMP_GRADE = CREST_Z / (RAMP_X1 - RAMP_X0)   # 0.75


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
    res = config.CELL_RESOLUTION
    for col in range(env_map.cols):
        # Cell centres sit at (col + 0.5) * resolution, matching grid_to_world.
        env_map.grid[:, col] = profile_z((col + 0.5) * res)
    return env_map

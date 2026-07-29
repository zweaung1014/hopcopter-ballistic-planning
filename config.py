"""Configuration parameters for the 2.5D map hopping-robot simulation."""

import math


# Map parameters
MAP_SIZE_X = 5.0  # meters
MAP_SIZE_Y = 5.0  # meters
CELL_RESOLUTION = 0.1  # meters per cell (10 cm)

# Robot start position (x, y) in meters
START = (0.0, 0.0)

# Goal position (x, y) in meters
GOAL = (4.5, 4.5)

# RRT* parameters
STEP_SIZE = 0.2  # meters
SEARCH_RADIUS = 0.5  # meters
MAX_ITERATIONS = 5000
GOAL_TOLERANCE = 0.1  # meters

# Height-aware planning parameters
MAX_JUMP_HEIGHT = 0.5  # meters; edges with dz > this are impassable
ALPHA_UPHILL = 5.0  # cost weight for uphill elevation changes
ALPHA_DOWNHILL = 0.0  # cost weight for downhill elevation changes (landing)

# Hopping-robot A* parameters
HOP_RADIUS = 1.0  # meters; the robot hops to points on a circle of this radius
HOP_N_ANGLES = 16  # number of evenly spaced candidate hop directions per expansion
HOP_SCAN_STEP = 0.1  # m; spacing of the radius ladder the inward ray-search walks
                     # when generating candidate landing cells. Along any one
                     # direction, candidates sit at HOP_RADIUS, HOP_RADIUS - step,
                     # ... so this is what quantizes how far a hop can go.
                     #
                     # It is a separate parameter from CELL_RESOLUTION (rather than
                     # just reading the grid) so it can be tuned for speed: the
                     # branching factor is directly proportional to 1/step, and
                     # coarsening it is the cheapest way to make planning faster.
                     #
                     # But coarsening it also makes straight-line progress lumpy —
                     # at 0.3 m, straight-ahead landings are 0.3 m apart while
                     # diagonal ones are not, so A* picks up lateral doglegs to
                     # reach x-positions the straight ladder skips. Keeping it at
                     # CELL_RESOLUTION avoids that: candidates are then as fine as
                     # the map itself. Measured on slope_crest: 0.3 -> a 5-hop path
                     # with a 0.3 m y-detour in 2.6 s; 0.1 -> a straight 4-hop path
                     # in 7.2 s.
HOP_FIXED_COST = 0.05  # per-hop constant added to every edge. Without it, N short
                       # hops along a straight line cost exactly the same as one
                       # long hop (triangle equality), so A* is indifferent and
                       # tie-breaks on heap order, producing jittery micro-hop
                       # chains. Admissible: it only ever adds cost.

# Robot geometry
# -----------------------------------------------------------------------------
# The robot is a sphere of ROBOT_RADIUS whose center (the CoM that the ballistic
# arc actually tracks) sits LEG_LENGTH above the contact point.
ROBOT_RADIUS = 0.2   # m; body radius — both lateral half-width and vertical half-height
LEG_LENGTH = 0.4     # m; CoM height above the foot. Hop arcs start and end at
                     # `terrain_z + LEG_LENGTH`, not at terrain level.

# Ballistic (parabolic hop) physics + clearance parameters
# Based on Campana & Laumond (2016), "Ballistic motion planning."
G_ACCEL = 9.81  # m/s^2; gravitational acceleration
MAX_APEX_HEIGHT = 1.2  # m; how tall a parabola the leg can produce — the CoM rise
                       # above the takeoff CoM on a vertical in-place hop.
V_MAX = math.sqrt(2.0 * G_ACCEL * MAX_APEX_HEIGHT)  # 4.852 m/s
        # Derived, not tuned: apex rise = V_MAX^2 / (2g). The longest flat hop
        # this affords is V_MAX^2 / g = 2.40 m, and a flat hop of distance X
        # needs v_s >= sqrt(g*X), so HOP_RADIUS must stay well under that.

MIN_CLEARANCE = 0.15  # m; HARD gate — an arc whose body-to-terrain clearance ever
                      # drops below this is rejected outright. Clearance does NOT
                      # enter the edge cost; it is purely a feasibility test.
ALPHA_MARGIN_FRAC = 0.5  # where in [alpha_min, alpha_max] the default takeoff angle
                         # sits. 0.5 = Campana's max-margin midpoint. The planner
                         # escalates above this only when the gate demands it.
ARC_SAMPLE_MAX_STEP = 0.05  # m; upper bound on sampling step along the arc's XY line
ARC_LATERAL_SAMPLES = 3  # terrain samples across the body's width, spanning
                         # [-ROBOT_RADIUS, +ROBOT_RADIUS] perpendicular to travel.
                         # 1 collapses to the old point-mass centreline check.
OBSTACLE_WALL_EXTRA = 1.5  # m; added on top of map_max_z to treat OBSTACLE cells as
                           # tall walls. Must exceed the tallest arc the robot can
                           # fly over a cell, else OBSTACLEs become jumpable:
                           #   LEG_LENGTH + MAX_APEX_HEIGHT - ROBOT_RADIUS - MIN_CLEARANCE
                           #   = 0.4 + 1.2 - 0.2 - 0.15 = 1.25 m

# A hop's clearance at takeoff/landing is exactly LEG_LENGTH - ROBOT_RADIUS (the
# body's underside above its own contact point). If that ever drops below
# MIN_CLEARANCE, every edge in the graph is rejected and plan() silently returns
# None everywhere — a failure mode with no obvious symptom, so assert it here.
assert LEG_LENGTH - ROBOT_RADIUS > MIN_CLEARANCE, (
    f"LEG_LENGTH - ROBOT_RADIUS = {LEG_LENGTH - ROBOT_RADIUS:.3f} m must exceed "
    f"MIN_CLEARANCE = {MIN_CLEARANCE} m, or no hop can ever satisfy the gate."
)
assert OBSTACLE_WALL_EXTRA >= (
    LEG_LENGTH + MAX_APEX_HEIGHT - ROBOT_RADIUS - MIN_CLEARANCE
), "OBSTACLE_WALL_EXTRA too small — obstacle cells would be jumpable."

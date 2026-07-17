"""Configuration parameters for the 2.5D map RRT* simulation."""


# Map parameters
MAP_SIZE_X = 5.0  # meters
MAP_SIZE_Y = 5.0  # meters
CELL_RESOLUTION = 0.2  # meters per cell (10 cm)

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

# Ballistic (parabolic hop) physics + clearance parameters
# Based on Campana & Laumond (2016), "Ballistic motion planning."
G_ACCEL = 9.81  # m/s^2; gravitational acceleration
V_MAX = 4.5  # m/s; maximum takeoff speed the leg can produce (leg-energy limit).
             # Minimum feasible v_s for a flat X-meter hop is sqrt(g*X), so
             # V_MAX must exceed sqrt(9.81 * HOP_RADIUS) with margin.
ROBOT_RADIUS = 0.1  # m; robot geometric radius; subtracted from arc clearance
CLEARANCE_MARGIN = 0.15  # m; smooth-penalty scale for arc-to-terrain proximity
CLEARANCE_WEIGHT = 2.0  # unitless; multiplier `w` on the proximity penalty
ARC_SAMPLE_MAX_STEP = 0.05  # m; upper bound on sampling step along the arc's XY line
ARC_ENDPOINT_EPSILON = 0.05  # m; trim near takeoff/landing so arc meeting ground doesn't register as collision
OBSTACLE_WALL_EXTRA = 1.0  # m; added on top of map_max_z to treat OBSTACLE cells as tall walls

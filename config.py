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

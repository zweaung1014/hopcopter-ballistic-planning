"""Five-step ascending staircase centered in y, with lateral bypass available.

Five steps running east (increasing x), spanning y ∈ [1.0, 4.0]:

  Ground: z = 0.0 m   x ∈ [0.0, 2.0)
  Step 1: z = 0.6 m   x ∈ [2.0, 2.3),  y ∈ [1.0, 4.0]
  Step 2: z = 1.2 m   x ∈ [2.3, 2.6),  y ∈ [1.0, 4.0]
  Step 3: z = 1.8 m   x ∈ [2.6, 2.9),  y ∈ [1.0, 4.0]
  Step 4: z = 2.4 m   x ∈ [2.9, 3.2),  y ∈ [1.0, 4.0]
  Step 5: z = 3.0 m   x ∈ [3.2, 5.0],  y ∈ [1.0, 4.0]

Each step is 0.3 m wide and 0.6 m tall. The 1.0 m ground margins north and
south of the staircase provide a lateral bypass route.
"""

import config
from map2d5 import Map2D5

START = (0.5, 4.5)
GOAL  = (4.5, 2.5)

STEP_WIDTH = 0.3   # m, x extent of each tread
STEP_HEIGHT = 0.6  # m, riser height

STAIR_XMIN = 2.0   # m, x where first step begins
STAIR_YMIN = 1.0   # m
STAIR_YMAX = 4.0   # m


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )

    for i in range(5):
        x_min = STAIR_XMIN + i * STEP_WIDTH
        x_max = x_min + STEP_WIDTH if i < 4 else None  # last step runs to map edge
        z = (i + 1) * STEP_HEIGHT
        env_map.paint_region(z, x_min=x_min, x_max=x_max,
                             y_min=STAIR_YMIN, y_max=STAIR_YMAX)

    return env_map

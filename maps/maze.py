"""Long maze map: scattered infinitely-tall square poles + a mid-map 0.5 m wall.

Layout (15 m x 5 m):

  Ground: z = 0.0 m everywhere except the regions below.

  Wall (paint_region, height 0.5 m):
    x in [7.35, 7.65),  y in [1.0, 4.0]
  Bypass corridors ~1.0 m wide at y in [0.0, 1.0] and y in [4.0, 5.0].

  Poles (set_obstacle_region, infinitely tall OBSTACLE cells):
    17 scattered 0.2 m x 0.2 m footprints, none within 1.0 m of another and
    none inside the wall zone or the start/goal discs. See `POLES` below.

`MAP_SIZE_X` and `MAP_SIZE_Y` are hardcoded module-local constants rather
than pulled from `config` — the rest of the deck uses `config.MAP_SIZE_X = 5.0`,
and bumping that globally would enlarge every other scenario's search space.

`START` and `GOAL` are PLACEHOLDERS: near the west and east edges of the map,
centred in y. Retune them once obstacle placement settles.
"""

import config
from map2d5 import Map2D5

# Placeholders — tune once obstacle layout settles.
START = (0.5, 2.5)
GOAL  = (14.5, 2.5)

# Module-local map size (deliberate departure from config; see docstring).
MAP_SIZE_X = 15.0
MAP_SIZE_Y = config.MAP_SIZE_Y  # 5.0 m, matches `maps/stairs.py`

# Wall spec — same style as maps/tall_wall.py.
WALL_HEIGHT = 0.5    # m
WALL_XMIN   = 7.35   # m  (0.3 m thick, centred on x = 7.5)
WALL_XMAX   = 7.65   # m
WALL_YMIN   = 1.0    # m
WALL_YMAX   = 4.0    # m  (~1.0 m bypass corridors at each y-end)

# Square-pole footprint half-width. 0.1 m => 0.2 m x 0.2 m (2x2 cells at
# CELL_RESOLUTION = 0.1). Each pole is un-flyable because
# `Map2D5.inflated_field` reports `+inf` at OBSTACLE cells.
POLE_HALF = 0.1

# Pole centres (x, y) in metres. Chosen to keep every pair >= 1.0 m apart, to
# stay clear of the wall region and the start/goal discs, and to leave feasible
# routing corridors both around and over the wall.
POLES = [
    # Pre-wall field.
    (2.0, 1.5),
    (2.5, 3.5),
    (3.5, 2.5),
    (4.0, 4.2),
    (4.5, 0.8),
    (5.5, 3.0),
    (5.8, 1.5),
    (6.5, 4.0),
    (6.7, 2.2),
    # Post-wall field.
    (8.5, 2.8),
    (8.7, 4.2),
    (9.5, 1.5),
    (10.0, 3.5),
    (10.5, 2.0),
    (11.5, 4.0),
    (12.0, 1.2),
    (12.5, 3.0),
]


def build() -> Map2D5:
    env_map = Map2D5(
        size_x=MAP_SIZE_X,
        size_y=MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )

    env_map.paint_region(
        WALL_HEIGHT,
        x_min=WALL_XMIN, x_max=WALL_XMAX,
        y_min=WALL_YMIN, y_max=WALL_YMAX,
    )

    for cx, cy in POLES:
        env_map.set_obstacle_region(
            x_min=cx - POLE_HALF, y_min=cy - POLE_HALF,
            x_max=cx + POLE_HALF, y_max=cy + POLE_HALF,
        )

    return env_map

"""Flat map: level ground everywhere, no obstacles. The control scenario.

Every other map in this directory tests how the planner reacts to *terrain* —
whether an arc clears a wall, whether a slope is standable, whether a riser is
climbable. This one deliberately removes all of that, and it is the more useful
map for a specific job: checking the takeoff-angle interval itself.

On level ground with `Z = 0` every constraint in `feasible_alpha_interval`
collapses to a form you can evaluate by hand, so the interval it produces is
checkable rather than merely plausible:

    Eq. 4 validity     alpha in (0, 90 deg)                  — always the full range
    (E1) energy floor  tan a = (W + sqrt(W^2 - g^2 X^2)) / (g X),  W = v_s_min^2
    (E3) min apex      tan a = 4 * MIN_APEX_HEIGHT / X       — since apex = X tan a / 4
    (1) takeoff cone   alpha >= atan(1 / MU)                 — the textbook Coulomb bound
    (2) landing cone   symmetric to (1), because Z = 0

The friction cones are also at their most permissive here (both normals are
vertical, so the wedges are the textbook ones and heading does not matter),
which means anything that *does* narrow the interval on this map is either the
energy chain or the min-apex floor — the two newest and least-tested gates.

See `test/demo_takeoff_angle_range.py`, which plans across this map and dumps
the per-constraint construction of every hop's interval.

There is nothing to paint, so there is no `paint_region` call: `Map2D5`
initialises its grid to zeros.
"""

import config
from map2d5 import Map2D5


def build() -> Map2D5:
    return Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )

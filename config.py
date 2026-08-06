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
# The robot's collision volume is a capsule from the foot to the CoM (the point
# the ballistic arc actually tracks), which sits LEG_LENGTH above the contact
# point. The capsule has three independently-sized parts: a sphere of
# ROBOT_RADIUS at the CoM (the robot's actual body), a much thinner cylinder of
# LEG_CYLINDER_RADIUS along the leg, and a small hemisphere of FOOT_TIP_RADIUS
# at the foot. See `hopping_astar_planner.clearance_for_alpha` for how the three
# regions are distinguished and `Map2D5.standable_mask` for the stance-time
# sphere + leg-cylinder-sides check (no foot-tip component at stance — the foot
# is on the ground by definition).
ROBOT_RADIUS = 0.15         # m; CoM sphere radius — the robot's actual body.
LEG_CYLINDER_RADIUS = 0.01  # m; leg cylinder sides — the leg is thin.
FOOT_TIP_RADIUS = 0.02      # m; hemispherical foot tip — slightly fatter than
                            # the leg (a rounded foot pad), but still thin.
LEG_LENGTH = 0.4     # m; CoM height above the foot. Hop arcs start and end at
                     # `terrain_z + LEG_LENGTH`, not at terrain level.
LEG_CLEARANCE_START_FRAC = 1.0 / 3.0
    # Fraction of LEG_LENGTH from the foot upward that is exempt from the
    # STANCE leg-cylinder-sides check. The lower `frac * L` slab of the leg is
    # not required to clear terrain — the foot is on the ground by definition
    # and requiring the lower leg to also clear terrain would condemn every
    # non-flat cell. Only the upper `(1 - frac) * L` of the leg's sides are
    # checked at stance (in addition to the CoM sphere).
    #
    # This sets the maximum standable constant grade under a rigid vertical
    # leg, and there are now two independent ceilings depending on which
    # component (leg-cylinder sides or CoM sphere) fires first. On a slope of
    # grade `g`, the leg-cylinder-sides ceiling is
    #
    #     g_max_leg = (L * frac) / (LEG_CYLINDER_RADIUS + MIN_CLEARANCE)
    #               = (0.4 * 1/3) / (0.01 + 0.10)
    #               ≈ 1.21
    #
    # and the CoM-sphere-alone ceiling (from the sphere reaching the terrain
    # at horizontal distance `ROBOT_RADIUS + MIN_CLEARANCE`) is
    #
    #     g_max_sphere = sqrt((L / (ROBOT_RADIUS + MIN_CLEARANCE))^2 - 1)
    #                  = sqrt((0.4 / 0.25)^2 - 1)
    #                  ≈ 1.25
    #
    # The tighter of the two (g_max_leg ≈ 1.21) governs, since standability
    # requires clearing BOTH components. Both ceilings now sit above grade 1.0
    # (45°) — steeper than any realistic map grade in this repo (the steepest,
    # `maps/slope_crest.py`, ships at 0.35) — so neither is actually binding
    # for real terrain; this stance check now mainly guards against near-
    # vertical walls, not graded slopes.
    #
    # DOES NOT apply during flight: `terrain_profile` uses the full capsule
    # (sphere + full cylinder + bottom hemisphere at the foot). The exempt
    # slab is a stance-only relaxation.

# Ballistic (parabolic hop) physics + clearance parameters
# Based on Campana & Laumond (2016), "Ballistic motion planning."
G_ACCEL = 9.81  # m/s^2; gravitational acceleration
MAX_APEX_HEIGHT = 1.2  # m; how tall a parabola the leg can produce — the CoM rise
                       # above the takeoff CoM on a vertical in-place hop.
V_MAX = math.sqrt(2.0 * G_ACCEL * MAX_APEX_HEIGHT)  # 4.852 m/s
        # Derived, not tuned: apex rise = V_MAX^2 / (2g). The longest flat hop
        # this affords is V_MAX^2 / g = 2.40 m, and a flat hop of distance X
        # needs v_s >= sqrt(g*X), so HOP_RADIUS must stay well under that.

MU = 1.2  # Coulomb friction coefficient, uniform over the environment (as in
          # Campana & Laumond). Sets the friction cone's half-angle
          # `beta = atan(MU)` = 50.19 deg at this value.
          #
          # This is a real constraint on the takeoff angle, not a safety factor.
          # On FLAT ground the cone alone forces `alpha >= atan(1/MU)` = 39.79
          # deg at both contact points — push any shallower and the foot slips.
          # For reference, the paper benchmarks MU in {0.5, 1.2}; MU = 0.5 would
          # put the flat-ground floor at 63.43 deg.
          #
          # It also caps how steep a surface the robot can contact at all: a
          # cross-slope of grade > MU makes the cone-plane intersection
          # degenerate (see `hopping_astar_planner.inplane_friction_cone`), so
          # no hop can start or land there. At MU = 1.2 that limit (1.2) sits
          # just below `standable_mask`'s geometric ceiling (~1.21), so friction
          # is now the binding standability limit — by a hair.

MIN_CLEARANCE = 0.10  # m; HARD gate — an arc whose body-to-terrain clearance ever
                      # drops below this is rejected outright. Clearance does NOT
                      # enter the edge cost; it is purely a feasibility test.
ALPHA_MARGIN_FRAC = 0.5  # where in [alpha_min, alpha_max] the default takeoff angle
                         # sits. 0.5 = Campana's max-margin midpoint. The planner
                         # escalates above this only when the gate demands it.
ARC_SAMPLE_MAX_STEP = 0.05  # m; upper bound on sampling step along the arc's XY line
ARC_LATERAL_SAMPLES = 5  # terrain samples across the body's width, spanning
                         # [-(R_max + MIN_CLEARANCE), +(R_max + MIN_CLEARANCE)]
                         # perpendicular to travel, where R_max is the largest
                         # of the three capsule radii (ROBOT_RADIUS, since the
                         # CoM sphere is by far the widest part — see the
                         # ROBOT_RADIUS >= ... assert below). The corridor
                         # half-width includes MIN_CLEARANCE because the
                         # safety margin extends past the body radius. 1
                         # collapses to a centreline-only check.
                         #
                         # Bumped from 3 to 5 when the corridor widened
                         # (R_max → R_max + MIN_CLEARANCE) so the inter-sample
                         # gap stays ≤ 0.2 m: with 3 samples across a 0.7 m
                         # corridor the gap is 0.35 m, wide enough for a
                         # narrow (~20 cm) obstacle to slip through undetected.
OBSTACLE_WALL_EXTRA = 1.5  # m; added on top of map_max_z to treat OBSTACLE cells as
                           # tall walls. Must exceed the tallest arc the robot can
                           # fly over a cell, else OBSTACLEs become jumpable. The
                           # binding case is the foot-tip bottom-cap check (the
                           # lowest point of the capsule, closest to terrain):
                           #   LEG_LENGTH + MAX_APEX_HEIGHT - FOOT_TIP_RADIUS - MIN_CLEARANCE
                           #   = 0.4 + 1.2 - 0.02 - 0.10 = 1.48 m

# A hop's clearance at takeoff/landing is exactly LEG_LENGTH - ROBOT_RADIUS (the
# CoM sphere's underside above its own contact point — this is the stance
# self-check `standable_mask` runs against a robot's own flat cell). If that
# ever drops below MIN_CLEARANCE, every edge in the graph is rejected and
# plan() silently returns None everywhere — a failure mode with no obvious
# symptom, so assert it here.
assert LEG_LENGTH - ROBOT_RADIUS > MIN_CLEARANCE, (
    f"LEG_LENGTH - ROBOT_RADIUS = {LEG_LENGTH - ROBOT_RADIUS:.3f} m must exceed "
    f"MIN_CLEARANCE = {MIN_CLEARANCE} m, or no hop can ever satisfy the gate."
)
assert OBSTACLE_WALL_EXTRA >= (
    LEG_LENGTH + MAX_APEX_HEIGHT - FOOT_TIP_RADIUS - MIN_CLEARANCE
), "OBSTACLE_WALL_EXTRA too small — obstacle cells would be jumpable."

# terrain_profile's lateral sampling corridor (ARC_LATERAL_SAMPLES, above) and
# clearance_for_alpha's above-CoM branch both assume the CoM sphere is the
# widest part of the capsule.
assert ROBOT_RADIUS >= LEG_CYLINDER_RADIUS and ROBOT_RADIUS >= FOOT_TIP_RADIUS, (
    "ROBOT_RADIUS (CoM sphere) must be the largest of the three capsule radii."
)

# Same silent-failure hazard as the MIN_CLEARANCE assert above, one gate later.
# The friction cone raises the takeoff-angle floor to `pi/2 - atan(MU)` on flat
# ground; if that ever rises above the leg-energy ceiling at HOP_RADIUS, the
# feasible interval is empty for *every* flat hop and plan() returns None
# everywhere with no obvious symptom. At shipped values the floor is 39.79 deg
# against a ceiling of 77.66 deg.
assert MU > 0.0, "MU must be positive."
_FLAT_DISC = V_MAX**4 - G_ACCEL**2 * HOP_RADIUS**2
assert _FLAT_DISC >= 0.0, (
    f"HOP_RADIUS = {HOP_RADIUS} m exceeds the flat-hop reach of V_MAX "
    f"({V_MAX**2 / G_ACCEL:.2f} m) — no flat hop is feasible at any angle."
)
assert math.pi / 2 - math.atan(MU) < math.atan(
    (V_MAX**2 + math.sqrt(_FLAT_DISC)) / (G_ACCEL * HOP_RADIUS)
), (
    f"MU = {MU} is too low — the friction cone's flat-ground floor "
    f"({math.degrees(math.pi / 2 - math.atan(MU)):.2f} deg) sits above the "
    f"leg-energy ceiling at HOP_RADIUS, so no flat hop is feasible."
)

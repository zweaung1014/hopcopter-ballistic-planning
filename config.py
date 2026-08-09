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
                       #
                       # Largely superseded by W_ENERGY below: once landings cost
                       # energy, micro-hop chains stop being free on their own.
                       # Kept at 0.05 for tie-breaking hygiene, not load-bearing.

W_ENERGY = 0.84  # m per J; the exchange rate between energy and distance in the
                 # edge cost. See `HoppingAStarPlanner._edge_cost`:
                 #
                 #     cost = xy_dist + W_ENERGY * (e_inject + momentum_lost)
                 #
                 # DERIVED, not tuned. Holding speed steady through one hop costs
                 # `0.5 * ROBOT_MASS * v^2 * (1 - ETA_HOP)`, which at the flat
                 # steady state (v = 3.158 m/s on 1 m hops) is 1.197 J. Setting
                 # W_ENERGY = 1 / 1.197 = 0.84 makes that hop's energy cost equal
                 # its distance cost, so the default means "weigh energy and
                 # distance exactly as the physics says". Raise it to bias the
                 # planner toward detouring around obstacles, lower it to bias
                 # toward hopping over them.
                 #
                 # Note the derivation has no hop LENGTH in it — holding speed
                 # costs the same 1.197 J whether the hop is 0.2 m or 1.0 m. That
                 # is what makes the energy term penalise hop COUNT without any
                 # separate per-hop constant, and it is why HOP_FIXED_COST above
                 # is no longer doing real work.
                 #
                 # W_ENERGY is also a RUNTIME dial: `_heuristic` estimates
                 # distance only, so every Joule it adds is cost the heuristic
                 # cannot see, and A* loses focus as it rises. Measured on the
                 # flat map, 0 -> 0.84 took expansions from 186 to ~1800.

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

# The energy chain
# -----------------------------------------------------------------------------
# The robot HOPS, it does not jump: every hop's takeoff speed is set by the
# previous hop's landing speed, so hops are not independent and the planner
# carries energy along the path. One cycle is three phases:
#
#   1. descent  — attitude control aims the leg; ballistic, lossless.
#   2. stance   — inverted-pendulum swing from touchdown to takeoff. The CoM is
#                 at `terrain + LEG_LENGTH` at BOTH ends (same foot, same leg),
#                 so there is no net change in gravitational PE across stance
#                 and the entire energy balance is kinetic. That is why the loss
#                 is bookkept on full CoM kinetic energy at touchdown, `m v^2/2`,
#                 rather than on apex height: `m g h_apex` counts only the
#                 vertical share and would leave the horizontal speed — the very
#                 thing the pendulum redirects — undamped.
#   3. thrust   — four propellers inject up to `E_INJECT_MAX` along the body's
#                 own "up". Injection can only ADD energy; the robot cannot shed
#                 it, which is what makes the takeoff speed a hard FLOOR and not
#                 merely a preference.
#
#     flight:    v_g^2 = v_s^2 - 2 g Z          (conservation, Z = z_g - z_s)
#     stance:    v_s_min' = sqrt(ETA_HOP) * v_g
#     thrust:    v_s' in [v_s_min', sqrt(v_s_min'^2 + 2 E_INJECT_MAX / MASS)]
#
# See `hopping_astar_planner.feasible_alpha_interval` for how the band becomes a
# takeoff-angle interval, and `docs/alpha_range_new.md` for the derivations.
ROBOT_MASS = 0.8  # kg
ETA_HOP = 0.7     # fraction of kinetic energy surviving one stance phase (30%
                  # lost). Note sqrt(0.7) = 0.837, so SPEED drops ~16% per hop
                  # while ENERGY drops 30%.
H_INITIAL = 1.0   # m; apex of the robot's very first hop. Seeds the chain: the
                  # start node is given a virtual incoming landing speed
                  # `sqrt(2 g H_INITIAL / ETA_HOP)` chosen so the uniform stance
                  # rule reproduces a first takeoff speed of
                  # `sqrt(2 g H_INITIAL)` = 4.43 m/s. Seeding it this way means
                  # the first hop needs no special case in the planner.
MIN_APEX_HEIGHT = 0.3  # m; minimum APEX-TO-LANDING drop. Below this the elastic
                       # leg does not compress enough for the controller to
                       # detect the stance phase at all, so the whole cycle
                       # fails. Measured apex -> landing, not takeoff -> apex:
                       # it is the fall that compresses the leg, and on an
                       # uphill hop the robot can rise 0.3 m and still land
                       # while barely descending.
INJECT_MAX_HEIGHT = 1.0  # m; the injection budget expressed as a height.
E_INJECT_MAX = ROBOT_MASS * G_ACCEL * INJECT_MAX_HEIGHT  # 7.848 J per hop

MAX_LANDING_APEX = 2.0  # m; the tallest fall the leg can absorb — sized for
                        # hopping off a platform, not for ordinary hops.
V_G_MAX = math.sqrt(2.0 * G_ACCEL * MAX_LANDING_APEX)  # 7.004 m/s
        # THE cap that actually bounds the chain, and the one that gates real
        # hops: reject any hop arriving faster than the leg can absorb. It also
        # bounds every takeoff speed recursively, via V_MAX below.

V_MAX = math.sqrt(ETA_HOP * V_G_MAX**2 + 2.0 * E_INJECT_MAX / ROBOT_MASS)
        # 7.345 m/s — the fastest takeoff the robot can EVER produce, reached
        # only by landing at exactly V_G_MAX and then injecting the full budget,
        # i.e. only immediately after a MAX_LANDING_APEX platform drop.
        #
        # NOT a per-hop cap. Each individual hop is capped by its own parent:
        # `sqrt(v_s_min^2 + 2 E_INJECT_MAX / MASS)`, which on flat ground settles
        # near 5.4 m/s. V_MAX exists to size MAX_APEX_HEIGHT (and hence
        # OBSTACLE_WALL_EXTRA) for the platform-drop worst case, and as a
        # never-binding backstop on Campana's takeoff-speed constraint.
        #
        # The longest flat hop it affords is V_MAX^2 / g = 5.50 m, and a flat hop
        # of distance X needs v_s >= sqrt(g*X), so HOP_RADIUS must stay under
        # that. (It was 2.40 m when V_MAX was a tuned 4.852 m/s.)

MAX_APEX_HEIGHT = V_MAX**2 / (2.0 * G_ACCEL)  # 2.750 m
        # Derived, not tuned: the CoM rise above the takeoff CoM on a vertical
        # in-place hop at V_MAX. Read only by OBSTACLE_WALL_EXTRA's assert.

SPEED_BIN = 0.25  # m/s; quantisation of landing speed in the A* state. Energy
                  # now depends on the path taken, so the search state is
                  # (cell, round(v_g / SPEED_BIN)) rather than just `cell`.
                  # Neither more nor less energy dominates — more raises the
                  # floor and shuts off shallow hops, less lowers the ceiling —
                  # so there is no valid dominance pruning and binning is the
                  # honest way to keep the state space finite. Rounding to
                  # nearest costs at most +-SPEED_BIN/2 = 0.125 m/s of error in
                  # the propagated speed. In practice the chain converges to a
                  # fixed point within ~3 hops, so few bins per cell stay live.
                  #
                  # THE lever on planning time, now that the energy axis is what
                  # dominates it: the deck runs 3-9x more expansions than it did
                  # before the chain, and no amount of caching removes that (see
                  # `_profile_cache`, which already recovers the redundant
                  # terrain sampling). Measured on slope_crest at HOP_RADIUS=1.5:
                  #   0.25 -> 30.0 s, 6198 expansions
                  #   0.50 -> 20.1 s, 3914 expansions, IDENTICAL path and angles
                  #   1.00 -> 17.3 s, 2884 expansions, first hop's angle shifts
                  #                                    84 -> 75 deg
                  # 0.25 ships as the conservative choice. 0.5 looks free on that
                  # scenario, but one scenario is not evidence that it is free in
                  # general — if you coarsen it, diff the paths before trusting
                  # the figures.

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
OBSTACLE_WALL_EXTRA = 3.1  # m; added on top of map_max_z to treat OBSTACLE cells as
                           # tall walls. Must exceed the tallest arc the robot can
                           # fly over a cell, else OBSTACLEs become jumpable. The
                           # binding case is the foot-tip bottom-cap check (the
                           # lowest point of the capsule, closest to terrain):
                           #   LEG_LENGTH + MAX_APEX_HEIGHT - FOOT_TIP_RADIUS - MIN_CLEARANCE
                           #   = 0.4 + 2.75 - 0.02 - 0.10 = 3.03 m
                           #
                           # Was 1.5 m. It rose with MAX_APEX_HEIGHT (1.2 -> 2.75)
                           # when V_MAX became a derived worst case of the energy
                           # chain: a robot fresh off a MAX_LANDING_APEX platform
                           # drop can fly far higher than one limited to a tuned
                           # 4.85 m/s, and obstacles must stay un-flyable for it.

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

# The same silent-failure hazard once more, now for the energy chain — and this
# is the one most likely to trip, because FOUR parameters have to agree.
# The very first hop is the tightest in the whole plan: H_INITIAL hands the robot
# more energy than a HOP_RADIUS hop needs, and it cannot shed any of it, so the
# energy FLOOR (not the ceiling) is what nearly empties the interval. If it does
# empty, plan() returns None everywhere with no other symptom.
#
# All four lower bounds on tan(alpha), evaluated for a flat HOP_RADIUS hop:
_W_LO = 2.0 * G_ACCEL * H_INITIAL                     # v_s^2 leaving the start
_W_HI = min(_W_LO + 2.0 * E_INJECT_MAX / ROBOT_MASS, V_MAX**2)


def _tan_hi(W: float) -> float | None:
    """Upper `tan(alpha)` root of `v_s^2 <= W` for a flat HOP_RADIUS hop.

    Mirrors `hopping_astar_planner._speed_tan_interval` at `Z = 0`; inlined
    because `config` must not import the planner.
    """
    D = W * W - G_ACCEL**2 * HOP_RADIUS**2
    if D < 0.0:
        return None
    return (W + math.sqrt(D)) / (G_ACCEL * HOP_RADIUS)


_T_CEIL = _tan_hi(_W_HI)
assert _T_CEIL is not None, (
    f"the start's full-thrust budget cannot reach HOP_RADIUS = {HOP_RADIUS} m at "
    f"any angle — raise H_INITIAL or E_INJECT_MAX, or lower HOP_RADIUS."
)
_T_FLOOR = max(
    _tan_hi(_W_LO) or 0.0,                        # energy floor (the binding one)
    4.0 * MIN_APEX_HEIGHT / HOP_RADIUS,           # min-apex floor, flat-ground form
    1.0 / MU,                                     # friction cone floor, tan(atan(1/MU))
)
assert _T_FLOOR < _T_CEIL, (
    f"no takeoff angle survives the first hop: floor "
    f"{math.degrees(math.atan(_T_FLOOR)):.2f} deg >= ceiling "
    f"{math.degrees(math.atan(_T_CEIL)):.2f} deg for a flat HOP_RADIUS hop. The "
    f"start carries v_s = {math.sqrt(_W_LO):.2f} m/s and cannot shed it."
)

# The chain is seeded with a virtual landing speed (see H_INITIAL); it has to be
# one the leg could actually have absorbed, or the start state is unreachable by
# the robot's own physics.
assert math.sqrt(2.0 * G_ACCEL * H_INITIAL / ETA_HOP) <= V_G_MAX, (
    f"H_INITIAL = {H_INITIAL} m implies a seed landing speed of "
    f"{math.sqrt(2.0 * G_ACCEL * H_INITIAL / ETA_HOP):.2f} m/s, above V_G_MAX = "
    f"{V_G_MAX:.2f} m/s — the robot could not survive its own initial condition."
)

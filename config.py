"""Configuration parameters for the 2.5D map hopping-robot simulation."""

import math


# Map parameters
MAP_SIZE_X = 5.0  # meters
MAP_SIZE_Y = 5.0  # meters
CELL_RESOLUTION = 0.1  # meters per cell (10 cm)

# RRT* parameters
STEP_SIZE = 0.2  # meters
SEARCH_RADIUS = 0.5  # meters
MAX_ITERATIONS = 5000
GOAL_TOLERANCE = 0.1  # meters

# Height-aware planning parameters
MAX_JUMP_HEIGHT = 0.5  # meters; edges with dz > this are impassable

# Hopping-robot A* parameters
# HOP_RADIUS is not a config constant any more: the robot hops to points on a
# circle whose radius is derived per-state from the energy it is carrying
# (the flat-ground 45-degree ballistic range at the current takeoff-speed
# ceiling — see `hopping_astar_planner.max_hop_radius`), not a single tuned
# number. `test/demo_hop_radius_headroom.py` is what originally showed a
# fixed radius was the wrong model: at a fixed 1.0 m, essentially every hop
# saturated the ring (X_taken == hop_radius) while the physics gates
# (feasibility + energy chain + clearance) tolerated 2-4x more -- the ring,
# not the robot's physics, was choosing hop length. Sizing it from the actual
# energy state fixes that at the source instead of hand-picking a bigger
# constant.
HOP_N_ANGLES = 16  # NO LONGER READ BY THE PLANNER. `_generate_hop_neighbors`
                   # fills its candidate disk with a `HOP_SCAN_STEP`-spaced
                   # lattice (a scanline circle fill) rather than sampling
                   # fixed directions, so this constant now configures only the
                   # single-ring diagnostic visualizations —
                   # `test/demo_common.py::enumerate_ring_candidates` and
                   # `test/demo_planner_reroute.py::enumerate_ring_candidates`
                   # — which still sample one ring of evenly spaced directions
                   # for figure readability, deliberately simpler than what the
                   # planner itself does.
HOP_SCAN_STEP = 0.1  # m; lattice spacing of the scanline circle-fill that
                     # generates candidate landing cells, AT the reference
                     # radius `HOP_SCAN_STEP_REF_RADIUS` below (see
                     # `HoppingAStarPlanner._generate_hop_neighbors`).
                     #
                     # It is a separate parameter from CELL_RESOLUTION (rather than
                     # just reading the grid) so it can be tuned for speed. At a
                     # FLAT spacing, candidate count scales as
                     # ~pi * hop_radius^2 / step^2 (an area fill) — measured on
                     # stairs/tall_stairs/slope_crest, that made the scanline fill
                     # run 5.4-6.4x slower than the old ray-search (up to 572.8s vs
                     # 94.1s per plan), because hop_radius reaches ~5.5 m right
                     # after a big drop. HOP_SCAN_STEP_REF_RADIUS fixes the SHAPE
                     # of that scaling (back to ~linear in hop_radius, like the old
                     # ray-search); this parameter still controls the overall
                     # density, and coarsening it is still the cheapest lever on
                     # planning time.
                     #
                     # Coarsening it also makes straight-line progress lumpy —
                     # candidate spacing along any lattice axis is exactly `step`,
                     # so a coarse step can make A* pick up lateral doglegs to
                     # reach x-positions the lattice skips. Keeping it at
                     # CELL_RESOLUTION avoids that: candidates are then as fine as
                     # the map itself.
HOP_SCAN_STEP_REF_RADIUS = 1.0  # m; hop_radius below which HOP_SCAN_STEP is the
                     # spacing actually used. Above it, the effective spacing
                     # widens as `HOP_SCAN_STEP * sqrt(hop_radius /
                     # HOP_SCAN_STEP_REF_RADIUS)`, trading candidate DENSITY at
                     # large radii for keeping candidate COUNT growth linear
                     # rather than quadratic in hop_radius. 1.0 m is the flat
                     # steady-state hop length referenced throughout this
                     # codebase (see W_ENERGY's derivation above), so ordinary
                     # hops see no change; only the rare large-radius hops that
                     # were driving the 5-6x regression get coarsened. Measured
                     # on stairs/tall_stairs/slope_crest WITH this widening:
                     # 1.3-1.7x slower than the old ray-search (down from
                     # 5.4-6.4x with a flat HOP_SCAN_STEP) — the residual gap is
                     # the scanline fill's inherently higher candidate density
                     # even at/below the reference radius (a disk fill visits
                     # more points than 16 rays do), not further large-radius
                     # blowup.

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
                 # separate per-hop constant.
                 #
                 # W_ENERGY is also a RUNTIME dial: `_heuristic` estimates
                 # distance only, so every Joule it adds is cost the heuristic
                 # cannot see, and A* loses focus as it rises. Measured on the
                 # flat map, 0 -> 0.84 took expansions from 186 to ~1800.

# Robot geometry
# -----------------------------------------------------------------------------
# The robot's collision volume is a single uniform vertical cylinder of radius
# ROBOT_RADIUS, spanning the foot (the contact point) to the top of the body,
# LEG_LENGTH + ROBOT_RADIUS above it. The CoM (the point the ballistic arc
# actually tracks) sits LEG_LENGTH above the foot. There used to be three
# independently-sized capsule regions (CoM sphere, thin leg cylinder, foot
# hemisphere) with two separate inflated terrain fields; a single field now
# governs both stance (`Map2D5.standable_mask`) and flight
# (`hopping_astar_planner.clearance_floor_alpha`), since terrain in this
# codebase is always a solid column rising from the ground, so a flat-capped
# cylinder's clearance test reduces to checking only its bottom (the foot).
ROBOT_RADIUS = 0.01  # m; the robot's single collision-cylinder radius.
LEG_LENGTH = 0.4     # m; CoM height above the foot. Hop arcs start and end at
                     # `terrain_z + LEG_LENGTH`, not at terrain level.

# What counts as an EDGE — the only terrain that feeds the inflated field
# -----------------------------------------------------------------------------
# `Map2D5.inflated_field` dilates terrain sideways so the robot can be treated as
# a point. The purpose of that dilation is EDGE AVOIDANCE: keep the body off the
# rim of a wall or a step. It is not "charge uphill height everywhere", and when
# it was applied to every cell that is what it did — on a grade-`g` ramp the
# field read `z + 0.32*g` at every cell, so `clearance_floor_alpha`'s
# `field > max(t_s, t_g)` test lifted the takeoff angle for hops along a ramp
# that never came near anything. (The same over-reach already forced the
# endpoint exemption documented in `clearance_floor_alpha`.)
#
# So only cells that are part of a genuine edge are dilation sources: cells with
# an 8-neighbour whose elevation differs by more than this grade over the
# distance to it. At CELL_RESOLUTION = 0.1 m that is a 0.173 m step to an
# orthogonal neighbour, 0.245 m to a diagonal one.
STEEP_INFLATE_ANGLE_DEG = 60.0
STEEP_INFLATE_GRADE = math.tan(math.radians(STEEP_INFLATE_ANGLE_DEG))  # 1.732
# Max standable constant grade under this model is exactly STEEP_INFLATE_GRADE
# = 1.73, and the transition is a STEP, not a smooth limit. Below the threshold
# no cell is a dilation source, so `inflated_field == grid` and every cell is
# standable however steep; one hair above it every cell is a source at once and
# the field jumps to `z + 0.30*g` ≥ 0.52 m, well past LEG_LENGTH. The old
# geometric bound `LEG_LENGTH / (ROBOT_RADIUS + MIN_CLEARANCE)` = 1.33 therefore
# never binds any more — the threshold reaches it first. (Before steep-only
# inflation that geometric bound WAS the ceiling; before the capsule→cylinder
# change it was ~0.88.) MU = 1.2 remains the binding standability limit, now
# with more headroom than it had.
#
# THE TRADE THIS BUYS AND WHAT IT COSTS: in flight the inflated field is the
# only thing representing the body's ROBOT_RADIUS of width — `_arc_samples`
# reads one nearest-cell value along a bare centreline. Sub-threshold terrain
# may still rise at grade 1.73, so terrain (ROBOT_RADIUS + MIN_CLEARANCE) to the
# side of the centreline can sit up to 0.43 m above what the field reports. A
# hop down a 55° gully, or along a staircase of sub-threshold risers, flies
# clean on paper and could clip laterally. Deliberate, not an oversight: lower
# STEEP_INFLATE_ANGLE_DEG to buy that protection back at the price of
# re-inflating slopes.

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
        # near 5.4 m/s. V_MAX exists to size MAX_APEX_HEIGHT for the
        # platform-drop worst case, and as a never-binding backstop on
        # Campana's takeoff-speed constraint.
        #
        # The longest flat hop it affords is V_MAX^2 / g = 5.50 m, and a flat hop
        # of distance X needs v_s >= sqrt(g*X) -- this is what caps
        # `hopping_astar_planner.max_hop_radius`'s per-state ring radius, since
        # its v_s_max is itself capped at V_MAX. (It was 2.40 m when V_MAX was
        # a tuned 4.852 m/s.)

MAX_APEX_HEIGHT = V_MAX**2 / (2.0 * G_ACCEL)  # 2.750 m
        # Derived, not tuned: the CoM rise above the takeoff CoM on a vertical
        # in-place hop at V_MAX. Documentation only now — nothing reads it.

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
                  # before the chain, and no amount of caching removes that.
                  # (The terrain-profile cache this used to point at is gone:
                  # the planner reads precomputed inflated height fields now, so
                  # there is nothing per-hop left to cache.) Measured on
                  # slope_crest at HOP_RADIUS=1.5, BEFORE that change:
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
          # no hop can start or land there. At MU = 1.2 that limit sits well
          # below the standability ceiling set by STEEP_INFLATE_GRADE (1.73, see
          # ROBOT_RADIUS above), so friction is the binding standability limit,
          # not geometry. The assert below pins that ordering.

MIN_CLEARANCE = 0.01  # m; HARD gate — an arc whose body-to-terrain clearance ever
                      # drops below this is rejected outright. Clearance does NOT
                      # enter the edge cost; it is purely a feasibility test.
ARC_SAMPLE_MAX_STEP = 0.05  # m; upper bound on sampling step along the arc's XY
                            # line, clamped to CELL_RESOLUTION/3 by
                            # `hopping_astar_planner._arc_samples`.
                            #
                            # DO NOT COARSEN THIS. It looks like it should be
                            # safe to, since the planner reads an inflated field
                            # whose features are at least a dilation radius
                            # (0.30 m) wide. It is not: the closed form in
                            # `clearance_floor_alpha` has a POLE at each end of
                            # the hop (its `u*(X-u)` denominator), so the
                            # required takeoff angle is far more sensitive to
                            # sample spacing near the endpoints than the field
                            # is. Measured across the whole map deck, marching at
                            # CELL_RESOLUTION let 333 hops through that a
                            # sample-by-sample check rejects, CELL_RESOLUTION/2
                            # let 20 through, and CELL_RESOLUTION/3 let none.

# NOTE: the old sphere model had a `LEG_LENGTH - ROBOT_RADIUS > MIN_CLEARANCE`
# assert here, guarding against the CoM sphere's rounded underside clipping its
# own foot's ground contact. That failure mode does not exist for a flat-capped
# cylinder — its bottom face sits exactly at the foot's height by construction,
# contributing zero self-lift regardless of ROBOT_RADIUS (verified: a flat map
# is standable under the new model for any ROBOT_RADIUS, even ROBOT_RADIUS >
# LEG_LENGTH) — so there is nothing left for that assert to protect against.
#
# There is likewise no OBSTACLE_WALL_EXTRA any more. It was a "tall wall" height
# substituted for OBSTACLE cells when terrain was sampled BILINEARLY, and it had
# to be calibrated against MAX_APEX_HEIGHT or obstacles became jumpable.
# `Map2D5.inflated_field` gives OBSTACLE cells `+inf` outright, which is strictly
# stronger and needs no calibration at all.

# Reference "first hop" ballistic reach, used only to sanity-check that the
# shipped ENERGY/FRICTION/APEX parameters leave a non-empty takeoff-angle
# interval at start-up. Hop radius itself is no longer a free config
# parameter: the planner derives it per-state from the incoming speed, as the
# flat-ground 45-degree range `v_s_max^2 / g` (see
# `hopping_astar_planner.max_hop_radius`). This mirrors that formula (inlined
# rather than imported -- config must not import the planner) evaluated at
# the seeded first-hop speed, which is deterministic and known here.
_V_S_MIN_FIRST_HOP = math.sqrt(2.0 * G_ACCEL * H_INITIAL)  # = sqrt(ETA_HOP) * v_g_initial
_V_S_MAX_FIRST_HOP = math.sqrt(min(
    _V_S_MIN_FIRST_HOP**2 + 2.0 * E_INJECT_MAX / ROBOT_MASS, V_MAX**2,
))
_HOP_RADIUS_FIRST_HOP = _V_S_MAX_FIRST_HOP**2 / G_ACCEL

# Same silent-failure hazard as the MIN_CLEARANCE assert above, one gate later.
# The friction cone raises the takeoff-angle floor to `pi/2 - atan(MU)` on flat
# ground; if that ever rises above the leg-energy ceiling at the first hop's
# own radius, the feasible interval is empty for *every* flat hop and plan()
# returns None everywhere with no obvious symptom. At shipped values the
# floor is 39.79 deg against a ceiling of exactly 45.00 deg -- the ceiling is
# ALWAYS exactly 45 deg now, by construction: _HOP_RADIUS_FIRST_HOP is defined
# as exactly the distance that needs the full injection budget at 45 deg, so
# every hop's own ring edge sits at this same tight margin.
assert MU > 0.0, "MU must be positive."
assert STEEP_INFLATE_GRADE > MU, (
    f"STEEP_INFLATE_ANGLE_DEG = {STEEP_INFLATE_ANGLE_DEG} puts the inflation "
    f"threshold at grade {STEEP_INFLATE_GRADE:.3f}, at or below MU = {MU}. "
    f"That silently makes the inflated field, rather than friction, the binding "
    f"standability limit: slopes the friction cone accepts would be rejected by "
    f"`standable_mask` instead, and the failure looks like `plan()` returning "
    f"None on graded maps with no other symptom."
)
_FLAT_DISC = V_MAX**4 - G_ACCEL**2 * _HOP_RADIUS_FIRST_HOP**2
# Structurally >= 0 now (no assert needed): _HOP_RADIUS_FIRST_HOP is capped so
# V_MAX can always reach it (the `min(..., V_MAX**2)` above), so this can no
# longer go negative. Kept as a value only because the friction-floor check
# below reuses its square root.
assert math.pi / 2 - math.atan(MU) < math.atan(
    (V_MAX**2 + math.sqrt(_FLAT_DISC)) / (G_ACCEL * _HOP_RADIUS_FIRST_HOP)
), (
    f"MU = {MU} is too low — the friction cone's flat-ground floor "
    f"({math.degrees(math.pi / 2 - math.atan(MU)):.2f} deg) sits above the "
    f"leg-energy ceiling at the first hop's radius, so no flat hop is feasible."
)

# The same silent-failure hazard once more, now for the energy chain — and this
# is the one most likely to trip, because several parameters have to agree.
# The seeded first hop is the one deterministic case checkable at import time
# (every later hop's own ring edge sits at the same tight margin by
# construction, but those depend on where the search actually goes).
#
# All three lower bounds on tan(alpha), evaluated for a flat first-hop-radius hop:
_W_LO = _V_S_MIN_FIRST_HOP**2   # v_s^2 leaving the start, no injection
_W_HI = _V_S_MAX_FIRST_HOP**2


def _tan_hi(W: float) -> float | None:
    """Upper `tan(alpha)` root of `v_s^2 <= W` for a flat first-hop-radius hop.

    Mirrors `hopping_astar_planner._speed_tan_interval` at `Z = 0`; inlined
    because `config` must not import the planner.
    """
    D = W * W - G_ACCEL**2 * _HOP_RADIUS_FIRST_HOP**2
    if D < 0.0:
        return None
    return (W + math.sqrt(D)) / (G_ACCEL * _HOP_RADIUS_FIRST_HOP)


_T_CEIL = _tan_hi(_W_HI)
assert _T_CEIL is not None, (
    "the start's full-thrust budget cannot reach its own first-hop radius at "
    "any angle -- this should be structurally impossible; raise H_INITIAL or "
    "E_INJECT_MAX if it trips."
)
_T_FLOOR = max(
    _tan_hi(_W_LO) or 0.0,                          # energy floor (rarely binding at the ring edge)
    4.0 * MIN_APEX_HEIGHT / _HOP_RADIUS_FIRST_HOP,  # min-apex floor, flat-ground form
    1.0 / MU,                                       # friction cone floor, tan(atan(1/MU))
)
assert _T_FLOOR < _T_CEIL, (
    f"no takeoff angle survives the first hop: floor "
    f"{math.degrees(math.atan(_T_FLOOR)):.2f} deg >= ceiling "
    f"{math.degrees(math.atan(_T_CEIL)):.2f} deg for a flat hop at the first "
    f"hop's own radius. The start carries v_s = {math.sqrt(_W_LO):.2f} m/s and "
    f"cannot shed it."
)

# The chain is seeded with a virtual landing speed (see H_INITIAL); it has to be
# one the leg could actually have absorbed, or the start state is unreachable by
# the robot's own physics.
assert math.sqrt(2.0 * G_ACCEL * H_INITIAL / ETA_HOP) <= V_G_MAX, (
    f"H_INITIAL = {H_INITIAL} m implies a seed landing speed of "
    f"{math.sqrt(2.0 * G_ACCEL * H_INITIAL / ETA_HOP):.2f} m/s, above V_G_MAX = "
    f"{V_G_MAX:.2f} m/s — the robot could not survive its own initial condition."
)

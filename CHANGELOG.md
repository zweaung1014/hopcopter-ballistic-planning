# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed — hops now carry energy between them (hopping, not jumping)

Every hop was validated in isolation: `feasible_alpha_interval(X, Z, V_max, g)` asked only
"does *some* takeoff angle exist within the leg's budget?", so a landing cell was equally
reachable however the robot got there. That is a **jumping** robot — land, stop, jump again.

The real robot hops in a three-phase cycle (descent with attitude control, an
inverted-pendulum stance, then powered climbing thrust), so takeoff speed is a function of
landing speed, which is a function of the hop before it. Two things the old model could not
express:

1. **The robot already has a takeoff speed when it leaves the ground**, and propellers can
   only *add* energy. Every parabola shallower than the one that speed produces is
   unreachable, not merely suboptimal — the planner used to pick those freely.
2. **The mechanism needs a minimum drop.** Falling less than `MIN_APEX_HEIGHT` does not
   compress the elastic leg enough for the controller to detect a stance phase at all.

- **The chain** — bookkeeping is full CoM kinetic energy at touchdown, `½mv²`. The CoM sits
  at `terrain + LEG_LENGTH` at touchdown *and* takeoff, so stance involves no net change in
  gravitational PE and the whole 30 % loss is kinetic, with no PE term to track. `m·g·h_apex`
  was rejected as the alternative: it counts only the vertical share and would leave the
  horizontal speed — exactly what the inverted pendulum redirects — undamped.

  ```
  flight (lossless):   v_g² = v_s² − 2gZ
  stance (η loss):     v_s_min′ = √η · v_g                    η = ETA_HOP = 0.7
  thrust (injection):  v_s′ ∈ [v_s_min′, √(v_s_min′² + 2·E_INJECT_MAX/m)]
  ```

- **Three new α bounds**, applied *before* Eq. 4 validity and the friction cones, in
  `feasible_alpha_interval` — all behind keyword args defaulting to `None`, so bare
  `(X, Z, V_max, g)` calls keep their old behaviour and `test/test_friction_cone.py` still
  exercises BEAM in isolation:
  - **(E1) energy floor.** `_speed_tan_interval` already returns where `v_s² ≤ W`; the floor
    is its complement, and the implementation keeps the upper branch (the higher parabola).
    Vacuous when `v_s_min` cannot reach the target at all, where the bound falls back to
    (E2)'s lower root.
  - **(E2) injection ceiling**, `v_s ≤ √(v_s_min² + 2·E_INJECT_MAX/m)`.
  - **(E3) minimum drop**, new free function `min_apex_tan`. Measured apex → landing, not
    takeoff → apex: it is the fall that compresses the leg, and on an uphill hop the robot
    can rise 0.3 m and still land while barely descending. `h_drop = (XT−2Z)²/(4(XT−Z))` is
    monotone in `T = tan α` on the valid domain, so it inverts in closed form to
    `T ≥ 2(Z + h + √(h(h+Z)))/X` — no bisection. It is `max`'d against (E1), making it a
    pure fallback that changes nothing unless the incoming speed's parabola is too low.

- **A\* state is now `(cell, speed_bin)`.** A cell reached two ways is two situations, and
  one may have no feasible continuation. Neither more nor less energy dominates — more
  raises the floor and shuts off shallow hops, less lowers the ceiling — so there is no
  valid dominance pruning and the speed axis is quantised (`SPEED_BIN` = 0.25 m/s) instead.
  Measured on `stairs`: 827 → 3285 expansions (~4 live bins per cell), 5.1 s → 11.2 s.
  Most of that was clawed back with a new bounded `_profile_cache`: terrain profiles depend
  only on the cell pair, not on energy, so the bins of one cell were re-sampling identical
  terrain (21.6 s before the cache; 20k entries thrashes at 15.0 s, 60k reaches the
  uncapped 11.2 s at ~170 MB peak RSS).

- **Takeoff angle is chosen to minimise injected energy**, not to maximise clearance margin
  — energy spent now is energy the next hop lacks. `alpha_for_clearance` loses
  `margin_frac` and returns `clamp(α*, α_c, α_max)`, where `α_c` is the shallowest clearing
  angle (clearance is monotone in `tan α`) and `α*` = `min_energy_tan(X, Z)` =
  `(Z + hypot(Z,X))/X` is the argmin of required speed (U-shaped in α). Exact, not a search.
  **The accept/reject verdict is unchanged** by the policy switch — both reject only when
  even `α_max` fails — so only reported angles and clearances moved.
  `ALPHA_MARGIN_FRAC` is deleted.

- **`V_MAX` is now derived from the chain, and `V_G_MAX` replaces it as the real cap.**
  `V_G_MAX` = `√(2g·MAX_LANDING_APEX)` = 7.00 m/s is the fastest touchdown the leg can
  absorb (sized at a 2.5 m fall — hopping off a platform); since `v_s_min = √η·v_g` on the
  next hop, bounding `v_g` recursively bounds every takeoff speed. `V_MAX` =
  `√(η·V_G_MAX² + 2·E_INJECT_MAX/m)` = 7.35 m/s is what remains: the global worst case,
  reachable only immediately after a max-height drop, and a never-binding backstop on
  constraint (3). Constraint (4) now reads `v_g ≤ V_G_MAX`.

  Knock-on: `MAX_APEX_HEIGHT` becomes derived (1.20 → 2.75 m) and **`OBSTACLE_WALL_EXTRA`
  had to rise 1.5 → 3.1 m** to keep obstacles un-flyable for a robot fresh off a platform.
  Its existing assert catches this.

- **New config**: `ROBOT_MASS`, `ETA_HOP`, `H_INITIAL`, `MIN_APEX_HEIGHT`,
  `INJECT_MAX_HEIGHT`/`E_INJECT_MAX`, `MAX_LANDING_APEX`/`V_G_MAX`, `SPEED_BIN`. The start
  cell is seeded with a *virtual* landing speed `√(2g·H_INITIAL/η)` = 5.29 m/s, chosen so
  the uniform stance rule reproduces a first takeoff speed of `√(2g·H_INITIAL)` = 4.43 m/s
  — so the first hop needs no special case anywhere in the search. Two new asserts guard
  the silent-`None`-everywhere failure mode: that some angle survives the first hop (the
  tightest in any plan, because `H_INITIAL` hands the robot a surplus it cannot shed), and
  that the seed speed is one the leg could have absorbed.

- **`HoppingAStarPlanner.path_hops`** records per-hop `alpha_s`, `v_s`, `v_g`, `e_inject`,
  `apex_drop`, `X`, `Z` for the returned path. Callers needing the takeoff angle must read
  it from there: with the chain, the angle depends on the whole path leading up to the hop
  and cannot be recovered from the endpoints. `main.py` prints the table.

- **Diagnostics chain, and bin.** `demo_common.diagnose_path` now threads `v_g` forward
  hop by hop instead of mapping `diagnose_edge` over the pairs; `planner_alpha_interval`,
  `diagnose_edge`, `enumerate_ring_candidates` and `demo_decision_sweep.path_cost` take a
  `v_g_in`. Scoring hops independently would judge every one of them against the
  start-of-chain energy. It also *quantises* `v_g` between hops, because the search state
  carries a binned speed and that — not the exact value — is what chose the angle;
  chaining the exact value drifts by up to `SPEED_BIN/2` per hop, enough to move the
  reported clearance across the gate. That defect was visible as `diagnose_path` accusing
  the ballistic planner's own path of containing a hop the planner had accepted
  (`stairs_with_curb`, mc 0.088 vs the 0.10 gate).

- **Map recalibration: `maps/tall_narrow_wall.py` 0.70 → 1.40 m.** Its 0.70 m ridge was
  sized against a tuned `V_MAX` and a max-margin angle; under the energy chain a forced-
  steep arc clears it from every takeoff, and both planners returned the identical clean
  path. At 1.40 m the original contrast is back — baseline takes off at x=2.05 and clips,
  ballistic backs off to 1.95 and clears. Measured: 1.1–1.4 m shifts the takeoff, 1.6 m
  and up makes the planner detour around the y-end instead (which is
  `demo_planner_reroute`'s scenario, not this one). `demo_clearance_sweep`'s pillar moved
  0.9 → 1.6 m to stay in step with `test_clearance_rejection`, and now passes the energy
  band rather than a bare interval.

- **Test recalibration.** `test/test_clearance_rejection.py`'s pillar sweep needed
  `PILLAR_H` 0.45 → 1.6 m and its wall case 1.10 → 1.90 m: a robot that cannot shed energy
  is forced steep on short hops (39.8° of floor at `X = 2.1`, 78.2° at `X = 0.8`) and clears
  a low pillar trivially. Both files' hard-coded reach boundaries are now derived from
  `V_MAX²/g` so they cannot silently encode a tuned value again. New
  `test/test_hop_energy_chain.py` covers the closed forms, the interval bounds, and
  end-to-end chain closure on a real plan.

- Demo output moved to `results/energy_aware_planning/`. The code default had already
  drifted to `results/with_leg_capsule_margin` while the docs still said
  `results/after_LS_recs`; all now agree.

### Added — friction cone at both contacts (Campana BEAM constraints 1–4)

`feasible_alpha_interval` was pure and map-free: it saw only `X`, `Z`, `V_max` and
`g`, and applied two constraints (Campana Eq. 4 validity + the takeoff leg-energy
limit). Nothing about the terrain's surface normal reached it, so the interval
admitted takeoff angles a real push-only leg cannot produce — most starkly on a
downhill hop, where `alpha_min` went *negative*, i.e. thrust aimed into the ground.
`docs/alpha_range_old.md` had flagged this as the largest open modelling gap.

It now implements Campana & Laumond's full **BEAM**: five constraints intersected on
`alpha`, of which four are physical and one is the pre-existing parabola-validity
bound. See [docs/alpha_range_new.md](docs/alpha_range_new.md) for the derivations.

- **Physics** — [hopping_astar_planner.py](hopping_astar_planner.py) gains three pure
  free functions beside `feasible_alpha_interval`, kept unit-testable in isolation per
  the repo convention:
  - `inplane_friction_cone(n, theta, mu)` reduces the 3D Coulomb cone (half-angle
    `beta = atan(mu)`) to the 2D wedge `(gamma, delta)` it cuts in the hop plane, with
    `delta = acos(cos(beta)/A)`. The paper states this reduction (Sec. IV-A) but omits
    the formula (its footnote 1); this is the standard cone/plane intersection and
    reproduces the property the paper does state, `delta <= beta`. Returns `None` when
    the plane meets the cone only at its apex — a cross-slope steeper than `mu`.
  - `_speed_tan_interval(X, Z, W, g)` covers **both** velocity constraints, since
    `v_g^2 = v_s^2 - 2 g Z` makes the landing bound the same quadratic in `tan(alpha)`
    with `W = V_max^2 + 2 g Z`. Expanding that reproduces the paper's `Delta` and
    `Lambda` exactly. It replaces the old `asin(K)/psi` formulation, which is
    algebraically the same constraint (its `K < -1` "unbounded" branch was provably
    unreachable for `X > 0`).
  - `_landing_cone_alpha_s` maps the landing wedge back onto `alpha_s` through
    `tan(alpha_s) = 2Z/X - tan(alpha_g)`. Replaces the paper's Algorithm 1 case split
    on `sign(gamma_g)` with a single branch: heightmap normals always give
    `gamma_g in (0, pi)`, so only one branch can occur, and clamping `alpha_g` to what
    a real descending parabola produces subsumes the algorithm's `no_solution` and
    `undefined` flags.
- **Surface normals** — [map2d5.py](map2d5.py) gains `surface_normals()` (memoised
  alongside the existing fill cache, which `_invalidate_fill_cache` was renamed to
  `_invalidate_caches` to cover). It uses **min-|slope| one-sided differences**, not a
  central difference, and that is a correctness requirement: a central difference
  straddles a discontinuity and invents a ramp, reading grade 2.0 at the foot of
  `tall_stairs`' 0.4 m riser. That makes `cos(beta)/A = 1.43 > 1` — a degenerate cone
  at exactly the cell the planner needs to take off from, which would have made the
  stair maps unplannable. The one-sided rule picks the flat tread the foot actually
  rests on, while recovering a uniform grade exactly (`slope_crest`'s ramp reads 0.35).
- **Config** — `MU = 1.2` (the paper benchmarks 0.5 and 1.2). Asserted `> 0`, and
  asserted that the flat-ground cone floor `pi/2 - atan(MU)` stays below the leg-energy
  ceiling at `HOP_RADIUS` — otherwise the interval is empty for *every* flat hop and
  `plan()` returns None everywhere with no other symptom, the same silent-failure class
  the `MIN_CLEARANCE` assert already guards.
- **What it changes.** On flat ground both cones collapse to `alpha >= atan(1/MU)` =
  39.81°, up from 12.31° for a 1 m hop; the chosen angle moves 45.0° → 58.7°. On uphill
  landings the *landing* cone binds hardest (stair hop `X=0.6, Z=0.4`: `[41.7°, 82.0°]`
  → `[65.2°, 82.0°]`, chosen 61.9° → 73.6°). Because clearance is monotone in
  `tan(alpha)`, steeper arcs mean clearance only improves. Constraint (4) is
  implemented for correctness but never binds on current maps (`Lambda >= 0` needs
  `Z >= -7.2 m` at `X = 1.5`).
- **What it deliberately does not change.** The cone cannot make a standable slope
  impassable — standing requires grade `<= MU`, and the fall-line floor
  `pi/2 + atan(grade) - atan(MU)` stays under `pi/2` exactly when `grade < MU`, while
  the energy ceiling tends to `pi/2` as hops shorten. It caps hop *length* by heading,
  not reachability. `alpha_for_clearance` is untouched: its monotonicity argument
  depends only on `_arc_z`, not on where the interval came from.
- **Threaded through**: `mu=config.MU` into all four `HoppingAStarPlanner`
  construction sites — [main.py](main.py),
  [test/demo_common.py](test/demo_common.py) `make_planner` (which every other demo
  goes through), [test/benchmark_tall_stairs.py](test/benchmark_tall_stairs.py) and
  [test/demo_planner_reroute.py](test/demo_planner_reroute.py).

  Separately, every diagnostic that re-scores an edge outside the planner now goes
  through a new `demo_common.planner_alpha_interval` helper. The cone needs the two
  surface normals and the heading, none of which are recoverable from `X` and `Z`, so
  a bare `feasible_alpha_interval(X, Z, V_max, g)` call silently falls back to level
  ground — meaning every diagnostic on sloped terrain would quietly disagree with the
  planner it exists to explain. Fixed in `diagnose_edge` and
  `enumerate_ring_candidates` ([test/demo_common.py](test/demo_common.py)),
  `enumerate_ring_candidates` ([test/demo_planner_reroute.py](test/demo_planner_reroute.py),
  its own copy) and `classify` ([test/calibrate_geometry.py](test/calibrate_geometry.py)).
- **New scenario**: [maps/cross_slope.py](maps/cross_slope.py), a grade-0.9 side-hill —
  the only map here traversed in enough directions for the cone to decide anything.
  [test/demo_friction_cone.py](test/demo_friction_cone.py) plots the result: at
  mid-hillside the longest feasible hop is 1.50 m across the fall line but only 0.55 m
  along it, and the planned path takes 7 waypoints with the cone against 5 without,
  with the extra hops landing on the grade — the effect the paper reports in Table I.
- **Tests**: [test/test_friction_cone.py](test/test_friction_cone.py) implements the
  validation checklist that `docs/alpha_range_campana.md` Section 6 asks for, and it
  earned its keep (see errata below). For each of 14 geometries it samples the returned
  interval, reconstructs the takeoff and landing velocity **vectors** from Eq. 2/4, and
  confirms each lies inside its contact's cone and under `V_max` — ground truth that
  reuses none of the interval formulas. A tightness pass checks just outside each
  endpoint to catch over-conservatism.
  [test/test_clearance_rejection.py](test/test_clearance_rejection.py) sections (2) and
  (4) were rewritten for the new physics; case (4) used to assert "downhill widens the
  interval", which is no longer true, and its old geometry (`X=2.0, Z=-0.5, V_max=4.5`)
  is now rejected outright because landing would need 5.01 m/s.
- **Errata found in [docs/alpha_range_campana.md](docs/alpha_range_campana.md)** (noted
  in a banner at the top of that file, since it was reconstructed from an OCR'd PDF):
  1. the landing-cone mapping's `2*Z/X_theta` term is **positive**, not negative — the
     two agree only when `Z = 0`, so a flat-ground test would not have caught it. For
     `X=1, Z=1, alpha_s=80°` the flown arc lands at −74.76°; the correct sign gives
     −74.76°, the document's gives −82.57°;
  2. that file's suspicion about the missing `arctan()` on the velocity bounds was
     correct — those fractions are `tan(alpha)` values.
- **Measured on the existing deck** (`mu=None` vs `mu=1.2`, same map and endpoints):

  | scenario | without cone | with cone |
  |---|---|---|
  | `stairs_with_curb` | 6 waypoints, α = [45 75 62 61 45]° | 6 waypoints, α = [59 78 74 61 62]° |
  | `slope_crest` | 5 waypoints, α = [55 55 79 **29**]° | 5 waypoints, α = [68 63 52 61]°, **different waypoints** |
  | `cross_slope` | 5 waypoints | 7 waypoints, extra hops on the grade |

  `stairs_with_curb` keeps its route and only steepens. `slope_crest` genuinely
  reroutes, and the reason is visible in the table: its old path contained a 29°
  takeoff, below the flat-ground cone floor of 39.81°, so that hop is now illegal.
- **Not yet revisited** (deferred): `test/test_clearance_rejection.py` case (1) and
  cases (6b)/(6c) still fail on constants left stale by the capsule-radius split. That
  failure predates this change and was verified unchanged against `HEAD` (same three
  cases, same `mc` values).

### Changed — split the capsule into three independent radii (CoM, leg, foot)

The collision capsule previously used one shared `ROBOT_RADIUS` for the CoM
sphere, the leg-cylinder sides, and the foot-tip hemisphere. It now has three
independent radii, since a 0.2 m-thick leg and foot were unrealistically fat:
`ROBOT_RADIUS = 0.15` m (CoM sphere, revised down from 0.2), new
`LEG_CYLINDER_RADIUS = 0.01` m (leg sides), new `FOOT_TIP_RADIUS = 0.02` m
(foot-tip hemisphere). `MIN_CLEARANCE` is unchanged.

- **Flight** — [hopping_astar_planner.py](hopping_astar_planner.py)
  `clearance_for_alpha` now selects both the axis distance *and* the radius
  per terrain sample from three regions instead of one: below the foot uses
  `FOOT_TIP_RADIUS` (bottom hemisphere, unchanged formula), at-or-above the
  foot now splits by height into the leg segment (`LEG_CYLINDER_RADIUS`) vs.
  above the CoM (`ROBOT_RADIUS`) — previously that whole region shared one
  radius, which would have silently shrunk the CoM sphere to leg-radius
  during flight for any terrain taller than the current arc height, exactly
  the case that matters most for obstacle detection. `ArcProfile` and
  `terrain_profile` carry `com_radius`/`leg_radius`/`foot_radius` instead of
  one `robot_radius`; the endpoint-transition mask now sizes off
  `foot_radius`. Monotonicity in `tan(alpha)` still holds except for a
  sub-centimeter, single-crossing dip where the radius switches from
  `LEG_CYLINDER_RADIUS` to the larger `FOOT_TIP_RADIUS` — well below
  `MIN_CLEARANCE`, not worth correcting with exact nearest-point-on-cone
  geometry.
- **Stance** — [map2d5.py](map2d5.py) `standable_mask` takes `com_radius` and
  `leg_radius` separately and combines them as
  `min(sphere_dist - com_radius, leg_dist - leg_radius) >= clearance` instead
  of subtracting one shared radius after the min. At `CELL_RESOLUTION = 0.1 m`,
  `LEG_CYLINDER_RADIUS (0.01 m)` is smaller than a grid cell, so in practice
  the leg-cylinder-sides check no longer binds before the CoM sphere does —
  the max standable grade is now governed by the sphere ceiling
  (`sqrt((LEG_LENGTH / (ROBOT_RADIUS + MIN_CLEARANCE))^2 - 1) ≈ 1.25`, not the
  leg-cylinder formula's ≈1.21), both now far steeper than any map in this
  repo (steepest ships at 0.35).
- **Config**: `OBSTACLE_WALL_EXTRA`'s assert is re-derived against
  `FOOT_TIP_RADIUS` (the bottom-cap/obstacle-height concern) instead of
  `ROBOT_RADIUS`; new assert that `ROBOT_RADIUS` is the largest of the three
  radii, since `terrain_profile`'s lateral sampling corridor is sized off the
  largest one.
- **Threaded through**: [main.py](main.py), [visualizer.py](visualizer.py)
  (`draw_arc_side_view` gained keyword-only `leg_radius`/`foot_radius`
  defaults so its "authoritative" clearance number stays correct without
  updating every call site), and all `test/demo_*.py` scripts that construct
  `HoppingAStarPlanner` or call `terrain_profile` directly.
- **Tests**: [test/test_clearance_rejection.py](test/test_clearance_rejection.py)
  call signatures updated for the new `standable_mask`/`terrain_profile`
  params; case (6a)'s stance-ceiling comparison rewritten to reflect the
  sphere-governs-in-practice finding above. Case (1)'s `PILLAR_H = 0.45`
  calibration and cases (6b)/(6c) still assume the old, much larger foot-tip
  radius and have **not** been re-tuned — the suite currently fails on case
  (1) until those constants are re-derived against the new geometry.
- **Not yet revisited** (deferred): the numeric constants in
  `maps/barely_jumpable_wall.py`, `maps/slope_crest.py`, `maps/tall_stairs.py`,
  `maps/stairs_with_curb.py`, and the demos built around them were calibrated
  assuming a single 0.2 m radius everywhere; their margins loosen substantially
  under the new geometry and have not been re-verified.

### Changed — leg safety margin (stance leg-cylinder sides + flight capsule)

The body's collision volume is now the full leg-to-CoM capsule of radius
`ROBOT_RADIUS`, not just the sphere at the CoM. Stance and flight enforce it
differently.

- **Stance** — [map2d5.py](map2d5.py) `standable_mask` gains a leg-cylinder-
  sides channel: the upper `(1 - LEG_CLEARANCE_START_FRAC) = 2/3` of a cylinder
  of radius `ROBOT_RADIUS` from foot to CoM must clear terrain too. The bottom
  `LEG_CLEARANCE_START_FRAC = 1/3` is exempt so graded slopes remain standable.
  Max standable grade tightens from ~0.55 (sphere-only) to
  `(LEG_LENGTH * frac) / (ROBOT_RADIUS + MIN_CLEARANCE) ≈ 0.38`.
- **Flight** — [hopping_astar_planner.py](hopping_astar_planner.py)
  `clearance_for_alpha` models the full capsule (top hemisphere at the CoM +
  full cylinder + bottom hemisphere at the foot). Terrain directly under the
  foot must clear the foot tip by `ROBOT_RADIUS + MIN_CLEARANCE = 0.35 m`, not
  just `MIN_CLEARANCE`. The `terrain_profile` corridor is widened from
  `[-R, R]` to `[-(R + gate), +(R + gate)]`, and per-sample terrain is stored
  without max-collapse so the capsule distance can be computed cell-by-cell.
- **Endpoint-transition mask** — samples where `terrain <= endpoint_max AND
  foot_h < endpoint_max + R + gate` are masked to `+inf`. This suppresses the
  rigid-vertical-leg model's spurious "foot skimming endpoint terrain" artifact
  at near-endpoint samples, without ever masking wall samples (terrain above
  endpoint height).
- **Lateral samples**: `ARC_LATERAL_SAMPLES` bumped 3 → 5 so the inter-sample
  gap stays ≤ 0.175 m under the widened corridor.
- **Config**: new `LEG_CLEARANCE_START_FRAC = 1.0/3.0` in
  [config.py](config.py). Threaded through `HoppingAStarPlanner.__init__`.
- **Scenarios**: [maps/slope_crest.py](maps/slope_crest.py) regraded 0.50 →
  0.35 to stay under the new 0.38 ceiling. [maps/tall_stairs.py](maps/tall_stairs.py)
  docstring updated for the widened `R + MIN_CLEARANCE = 0.35 m` un-standable
  band in front of each riser.
- **Visualizer**: [visualizer.py](visualizer.py) `draw_arc_side_view` now plots
  foot-tip trajectory and bottom-cap envelope; new
  `Visualizer.draw_robot_pose()` overlays top-down body + envelope rings at
  start/goal, wired up in [main.py](main.py).
- **Tests**: [test/test_clearance_rejection.py](test/test_clearance_rejection.py)
  gains case (6) covering capsule-specific stance and flight geometry
  (grade-0.35 standable / grade-0.50 not, bump-clearing hop accepts,
  wall-scraping hop rejects). Pillar height in case (1) recalibrated 0.9 → 0.45
  m to match the new capsule reach.

### Changed — revised robot model (denser map, leg, body radius, hard clearance gate)

The robot is no longer a point mass on a coarse grid. It is a sphere of
`ROBOT_RADIUS` whose centre — the point the ballistic arc actually tracks — sits
`LEG_LENGTH` above the contact foot, moving over a grid at twice the previous
density. Clearance became a hard feasibility gate rather than a cost penalty.

- **Map density doubled**: `CELL_RESOLUTION` 0.2 → 0.1 m (25×25 → 50×50 cells).
  - [map2d5.py](map2d5.py): new `paint_region()` sets cells by *world-metre*
    bounds. Both previous idioms were resolution-dependent: raw column slices
    (`grid[:, 10:13]`) hard-code a cell count, and `world_to_grid(...)` with an
    inclusive `+1` overshoots by up to one cell. At 0.1 m the former halved the
    staircases in `tall_stairs`/`stairs_with_curb` and left the map's right half
    at z=0; the latter shrank every world-painted obstacle by 0.1 m per axis.
    All seven maps now paint from world bounds and are resolution-invariant.
- **Jump height is now a stated capability, not a tuned constant**:
  `MAX_APEX_HEIGHT = 1.2 m` (the CoM rise on a vertical in-place hop) derives
  `V_MAX = sqrt(2 g h) = 4.852 m/s`. The longest feasible flat hop is
  `V_MAX^2 / g = 2.40 m`. `test/demo_common.py` no longer overrides `V_MAX`
  (was 6.0), and `demo_clearance_sweep` / `demo_planner_reroute` no longer
  hardcode 7.0 — their geometry was rescaled to spans the robot can reach.
- **`LEG_LENGTH = 0.4 m`**: hop arcs start and end at `terrain_z + LEG_LENGTH`.
  `Z = z_g - z_s` is unchanged, so `feasible_alpha_interval` is unaffected.
  This retired two workarounds that only existed because the arc used to start
  *on* the ground: the endpoint body-guard in `min_clearance`, and
  `ARC_ENDPOINT_EPSILON`. Both are deleted. Dropping the epsilon also closed a
  hole where hops shorter than `2 * epsilon` returned `+inf` and were accepted
  with no clearance check at all.
- **`ROBOT_RADIUS` 0.1 → 0.2 m, and the body now has lateral extent**: the
  clearance check samples `ARC_LATERAL_SAMPLES` points across the body's width
  perpendicular to travel and takes the maximum terrain, so a ridge *beside* the
  centreline blocks the hop. New `Map2D5.standable_mask()` rejects landing cells
  where the body could not rest, measuring true 3D distance to the terrain
  rather than a vertical drop — a flat-underside test would condemn every graded
  slope. There is a closed-form ceiling on traversable grade:
  `sqrt((LEG_LENGTH / (ROBOT_RADIUS + MIN_CLEARANCE))^2 - 1)` = 0.553.
- **Clearance is a hard gate**: `MIN_CLEARANCE = 0.15 m` rejects outright.
  `CLEARANCE_MARGIN` and `CLEARANCE_WEIGHT` are deleted and the smooth proximity
  penalty is gone — clearance no longer enters edge cost at all. This makes the
  `disable_clearance=True` A/B a *feasibility* comparison ("which candidates
  exist") rather than a cost-shaping one.
- **Takeoff angle: minimum sufficient effort**. Clearance is monotone
  nondecreasing in `tan(alpha)` (`dz/d tan a = u(X-u)/X >= 0` at every `u`), so
  the max-clearance angle is *always* `alpha_max` — which is exactly where
  `v_s = V_max`, the leg at 100% of its budget. Rather than always flying at the
  limit, `alpha_for_clearance()` takes the max-margin midpoint when it already
  clears, gives up only if `alpha_max` cannot, and otherwise bisects for the
  shallowest angle that does. Typical hops cost one clearance evaluation.
- **`min_hop_radius` default 0** (was `hop_radius / 2`), with two changes that
  make it affordable and well-behaved:
  - `HOP_FIXED_COST = 0.05` per hop. Without it, N short hops along a straight
    line cost *exactly* as much as one long hop, leaving A* indifferent and
    tie-breaking on heap order — the observed result was paths containing runs
    of consecutive 0.1 m micro-hops. The old radius floor existed to prevent
    exactly this; a fixed cost does it without imposing a floor.
  - `HOP_SCAN_STEP` makes the inward ray-search step an explicit parameter
    rather than reading `map.resolution`. It is the dominant cost knob — the
    branching factor is proportional to `1/step` — but coarsening it also
    quantizes how far a hop can travel, which shows up as lateral doglegs where
    the straight-ahead ladder cannot reach a wanted x. Shipped at 0.1 m
    (= `CELL_RESOLUTION`), i.e. the pre-change behaviour, after 0.3 m was found
    to put a visible 0.30 m y-detour into the `slope_crest` path.
- **Performance**: a naive implementation of the above measured 13.7 s per
  `plan()` (vs 284 ms before). Two-phase evaluation now separates the
  alpha-independent terrain sampling (`terrain_profile`) from the cheap
  per-angle arc evaluation (`clearance_for_alpha`), backed by a vectorized
  `Map2D5.sample_bilinear()` and a memoised obstacle-substituted grid. The
  cached grid lives on `Map2D5`, not the planner, so the six external callers of
  `min_clearance` get it for free. Deck total: **2.46 s → 24.4 s** for 14 plans
  (the full demo suite, which runs more than one plan per script, takes ~73 s).
- **Obstacle maps recalibrated ~3×**. The leg offset lifts the arc 0.4 m, which
  made every previously-tuned obstacle invisible. Heights re-derived empirically:
  `tall_narrow_wall` 0.15 → 0.70 m, `barely_jumpable_wall` 0.22 → 1.00 m,
  `stairs_with_curb` curb rise 0.10 → 0.60 m, `slope_crest` crest-above-shelf
  0.25 → 0.85 m with the ramp regraded 0.75 → 0.50 to stay under the 0.553
  standability ceiling. Map docstrings quote the fresh sweeps.
- **`enumerate_ring_candidates` gained a `stance` gate**, and `n_bad_hops` now
  counts a hop bad if its landing cell is un-standable. Several demos asserted
  "the baseline path contains clipping hops"; under the new model the baseline
  more often fails by *landing where the body cannot rest*, which the old
  arc-only diagnosis missed entirely.
- **Demo output moved** from `test/` to `results/after_LS_recs/`, via
  `demo_common.out_dir()` with a `$PLANNER_OUT_DIR` override (relative paths
  resolve against the repo root). All ten demos now route through `out_path()`.
- [test/time_deck.py](test/time_deck.py): new timing harness recording
  wall-clock, expansions, edge checks, hop count and leg-energy utilisation per
  scenario. Written against `make_planner`'s default interface so the same file
  runs against both old and new code; baselines are in
  `results/before_LS_recs/timings.md`.
- `test/benchmark_tall_stairs.py`: `N_TRIALS` 30 → 5, since a `tall_stairs` plan
  is now ~6.5 s (the benchmark itself takes ~40 s).

### Added
- **Presentation demo suite** — six scenarios covering go-around, hop-over,
  takeoff readjustment, stair climbing and slope climbing, all sharing one set
  of physics parameters so a single cost model visibly produces every behaviour.
  - [test/demo_common.py](test/demo_common.py): shared scaffolding for the
    `demo_*.py` scripts. `make_planner`, `diagnose_edge` and
    `enumerate_ring_candidates` were previously copy-pasted byte-identically
    across five demos; all demos now import them. Also holds the deck-wide
    constants (`HOP_RADIUS = 1.5`, `V_MAX = 6.0`, `N_ANGLES = 16`), the colour
    vocabulary, `param_caption()` for stamping physics on every figure, and
    `draw_topdown_compact()` for multi-panel figures where per-cell elevation
    text would be illegible.
  - `enumerate_ring_candidates` now records a `gate` field naming which check
    rejected each candidate (`bounds` / `obstacle` / `physics` / `clearance`),
    so a figure can state which gate did the work instead of assuming.
  - [maps/slope_crest.py](maps/slope_crest.py): the first map with continuously
    varying elevation rather than constant-z blocks — a 0.75-grade ramp rising
    to a crest at z=1.05 m, then a summit plateau at z=0.80 m.
  - [maps/stairs_with_curb.py](maps/stairs_with_curb.py): `tall_stairs` with a
    raised curb at each tread edge, standing 0.10 m proud of its tread.
  - [test/calibrate_geometry.py](test/calibrate_geometry.py): sweeps candidate
    map geometries and reports which produce a genuine clearance-driven
    accept/reject contrast. Both new maps' constants come from its output.
  - New demos: [test/demo_slope_crest.py](test/demo_slope_crest.py),
    [test/demo_stairs_curb.py](test/demo_stairs_curb.py),
    [test/demo_decision_sweep.py](test/demo_decision_sweep.py) (identical
    geometry, wall height swept 0.15 → 1.40 m, strategy flips from crossing to
    detouring between 1.20 m and 1.40 m with no per-panel tuning), and
    [test/demo_overview.py](test/demo_overview.py) (contact sheet).

### Fixed
- **Documented that neither rejection gate fires on `tall_stairs`.** Its
  docstring claimed hops skipping two or more steps are rejected as Campana-
  infeasible. Enumerating the full-radius ring at all 625 cells gives
  `accept 6518 · off-map 3482 · physics 0 · clearance 0`: 0.4 m risers sit well
  inside the leg's budget, and the `min_clearance` body-guard (which discards
  samples that have not risen `robot_radius` above *both* endpoints) skips
  nearly every interior sample on a steep ascent, returning `+inf`. The
  baseline/ballistic path difference there comes from the clearance *penalty*
  shaping cost, not from any candidate being ruled out.
  [maps/stairs_with_curb.py](maps/stairs_with_curb.py) exists to give a stair
  scenario where the clearance gate genuinely rejects hops.
- `maps/tall_narrow_wall.py` docstring claimed the ballistic planner shifts its
  takeoff 0.2 m (one grid cell); the demo actually shifts it 0.60 m (three
  cells), from x=2.10 to x=1.50.
- `maps/tall_narrow_wall.py` and `maps/barely_jumpable_wall.py` docstrings quoted
  `HOP_RADIUS = 1.5` / `V_MAX = 6.0` as though from `config.py`, which ships
  `1.0` / `4.5`. Both now say the values are demo-local.
- `test/demo_tall_stairs.py` docstring described a 1.8 m top platform and three
  0.6 m risers; the map has a 1.2 m platform and 0.4 m risers.
- Removed a dead `_add_wall_markings` stub from
  [test/demo_barely_jumpable.py](test/demo_barely_jumpable.py) whose body was
  `pass` and whose markings were already drawn by its caller.

- **Variable-radius neighbor generation** in
  [hopping_astar_planner.py](hopping_astar_planner.py): replaces the
  fixed-radius ring sampler with a per-direction ray search.
  - For each of the `n_angles` directions, the planner scans radii from
    `hop_radius` down to `min_hop_radius` (default `hop_radius / 2`) in
    steps of `map.resolution`, adding **every** valid (ballistically
    feasible, clearance-passing) landing cell it finds.
  - Generating all valid radii per direction — not just the farthest —
    allows A* to consider shorter landings in the same direction. A
    full-radius hop may deposit the robot too close to an obstacle for the
    next arc to clear it, while a shorter hop along the same ray gives
    sufficient launch distance.
  - Two deduplication sets (`attempted`, `in_results`) prevent redundant
    validation calls and duplicate result entries while allowing different
    directions to scan past a previously-failed cell to shorter radii.
  - `min_hop_radius` constructor parameter (default `hop_radius / 2`)
    prevents degenerate sub-cell hops that would make perimeter-walking
    artificially cheap relative to direct wall-crossing arcs.
  - Admissibility of the Euclidean heuristic is preserved: shorter hops
    only add valid candidates with higher-or-equal g-cost per unit
    distance, so the heuristic never over-estimates.

- `HoppingAStarPlanner` in [hopping_astar_planner.py](hopping_astar_planner.py):
  a new A*-based planner tailored for a hopping robot. Instead of stepping to
  one of 8 adjacent grid cells, the robot hops to points sampled on a circle
  of radius `hop_radius` around its current position.
  - Ring-based neighbor generation: `n_angles` evenly spaced directions are
    sampled on the reachable circle each expansion and snapped to grid cells.
  - Goal-snap edge: when the goal lies within `hop_radius` of the current
    cell, an explicit direct-hop-to-goal edge is added so the planner can
    land exactly on the goal without requiring angular alignment.
  - Only the landing cell is validated (bounds, non-obstacle, `|Δz| ≤
    max_jump_height`); the robot flies over intermediate terrain and
    obstacles.
  - Cost model unchanged from the original planner: Euclidean xy-distance
    between takeoff and landing plus an asymmetric elevation penalty
    (`alpha_uphill` for climbing, `alpha_downhill` for descending).
  - Heuristic unchanged: Euclidean distance to the goal (still admissible).
- Hopping-robot parameters in [config.py](config.py):
  - `HOP_RADIUS = 1.0` — hop distance in meters.
  - `HOP_N_ANGLES = 16` — number of candidate hop directions per expansion.

### Changed
- [main.py](main.py) now instantiates `HoppingAStarPlanner` (with
  `hop_radius` and `n_angles` from config) instead of `AStarPlanner`.

### Retained
- [astar_planner.py](astar_planner.py) is kept unchanged as a reference
  implementation for comparison against the hopping variant.

---

## [0.2.0] – 2026-05-28

### Added
- Initial A* planner implementation ([astar_planner.py](astar_planner.py))
  with 8-connected grid neighbors, height-aware edge costs, and an
  admissible Euclidean heuristic.
- Configuration tuning for cost weights (`ALPHA_UPHILL`, `ALPHA_DOWNHILL`)
  and the `MAX_JUMP_HEIGHT` hard constraint.

## [0.1.0] – 2026-05-25

### Added
- Initial project scaffold: 2.5D map representation
  ([map2d5.py](map2d5.py)), visualizer ([visualizer.py](visualizer.py)),
  and RRT*-based prototype.
- Stair-like elevation regions for testing height-aware planning.
- README with project overview.

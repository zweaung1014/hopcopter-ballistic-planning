# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

A simulation/research codebase for path planning on a 2.5D map (a 2D grid where each
cell stores an elevation z) for a hopping robot. The active planner
(`HoppingAStarPlanner`) models each move as a ballistic (parabolic) hop rather than a
step to an adjacent grid cell, using the physics from Campana & Laumond (2016),
"Ballistic motion planning." See `docs/planner.md` for the full algorithm writeup and
`CHANGELOG.md` for the history of how the planner evolved (RRT* → grid A* → hopping A*
→ ballistic feasibility/clearance gating).

## Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt   # numpy, matplotlib
```

## Common commands

There is no build step, package manifest, or test runner config (no pytest/tox) — this
is a set of standalone scripts run directly with `python`.

```bash
# Run the main planner + visualizer on a map (default map: "stairs")
python main.py
python main.py tall_stairs   # positional arg selects maps/<name>.py; run with an
                              # invalid name to print the list of available maps

# Numeric correctness checks (prints PASS/FAIL per case, exits 1 on any failure)
python test/test_clearance_rejection.py

# Timing/scaling benchmark (ballistic vs. baseline planner on the same scenario)
python test/benchmark_tall_stairs.py

# Per-scenario timing table across the whole deck (writes markdown)
python test/time_deck.py results/after_LS_recs/timings.md

# Visual demos — each writes PNGs into results/after_LS_recs/ (side-view arcs,
# top-down paths, ring-candidate panels, etc.); set PLANNER_OUT_DIR to redirect.
# Open the generated PNGs to inspect results.
python test/demo_tall_stairs.py
python test/demo_clearance_sweep.py
python test/demo_barely_jumpable.py
python test/demo_narrow_wall_showcase.py
python test/demo_planner_reroute.py
python test/demo_prof_showcase.py
```

`test/*.py` scripts insert the repo root onto `sys.path` themselves
(`sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))`), so
run them directly with `python test/<file>.py` from anywhere — no `PYTHONPATH` or
package install needed.

## Architecture

```
config.py                ← all tunable parameters in one place
map2d5.py                ← Map2D5: 2D grid of elevations, obstacle + bilinear queries
maps/                     ← map builders; each module exposes build() -> Map2D5
astar_planner.py         ← original 8-connected grid A* (kept as reference, unused by main.py)
hopping_astar_planner.py ← HoppingAStarPlanner: the active planner (ballistic hops)
visualizer.py            ← Visualizer: matplotlib rendering (grid, arcs, path)
main.py                  ← entry point: load_map(name) → plan() → visualize
test/                     ← demo scripts (produce PNGs), a benchmark, a timing
                            harness, and one assertion-based numeric test (no pytest)
results/                  ← generated figures + timing tables, one dir per run
```

### Map2D5 (`map2d5.py`)

Stores terrain as a numpy 2D array of elevations (meters). `OBSTACLE = -1.0` is a
sentinel value, not a real elevation. Key methods: `world_to_grid`/`grid_to_world`
(cell centers are at `((col+0.5)*res, (row+0.5)*res)`), `is_obstacle`,
`get_elevation`, `sample_bilinear` (vectorized; the clearance check samples a whole
arc in one call, with obstacle cells substituted for a tall "wall" fill value so
they block arcs instead of producing bilinear artifacts — the substituted grid is
memoised), `get_elevation_bilinear` (scalar wrapper over it), `set_obstacle_region`,
`paint_region`, `standable_mask` (sphere-at-CoM, radius `ROBOT_RADIUS`, + upper
`(1 - LEG_CLEARANCE_START_FRAC)` of the leg cylinder sides, radius
`LEG_CYLINDER_RADIUS` — at shipped values `LEG_CYLINDER_RADIUS` is thinner than
`CELL_RESOLUTION`, so in practice the CoM sphere, not the leg-cylinder-sides
channel, governs the max standable grade, ~1.25 at shipped values).

**Always paint terrain with `paint_region`,** which takes world-metre bounds. Raw
column slices (`grid[:, 10:13]`) hard-code a cell count, and `world_to_grid(...)`
plus an inclusive `+1` overshoots by up to one cell; both silently rescale the
physical terrain when `CELL_RESOLUTION` changes.

### Maps (`maps/`)

Each module (`stairs.py`, `tall_stairs.py`, `tall_wall.py`, `tall_narrow_wall.py`,
`barely_jumpable_wall.py`) exposes `build() -> Map2D5` and hand-paints elevation
regions directly into `env_map.grid`. `main.py` dynamically imports `maps.<name>` based
on the CLI arg. Add a new scenario by adding a new module here with a `build()`
function — no registration step needed.

### HoppingAStarPlanner (`hopping_astar_planner.py`) — the core algorithm

Standard A* (`heapq` open set, `g_cost`/`came_from` dicts, Euclidean-distance
heuristic — admissible because elevation and proximity penalties only add cost), but
neighbor generation and edge validation are non-trivial:

- **Neighbor generation** (`_generate_hop_neighbors`): for each of `n_angles` evenly
  spaced directions around the current cell, a ray-search scans radii from
  `hop_radius` down to `min_hop_radius` (default 0) in steps of `hop_scan_step`
  (world metres), adding *every* valid landing cell found along the ray — not
  just the farthest. This matters because a full-radius hop may land too close to an obstacle
  for the *next* arc to clear it, while a shorter hop in the same direction gives a
  better launch position. Two dedup sets (`attempted`, `in_results`) prevent
  redundant validation while still letting different directions scan past a
  previously *failed* cell to reach a valid shorter-radius one. A "goal-snap" edge is
  added whenever the goal is within `hop_radius`, so reaching it doesn't depend on
  angular alignment with the ring sampling.
- **Edge validation** (`_validate_and_cost`), cheapest test first:
  1. reject if the landing cell is an obstacle;
  2. **stance gate** (`Map2D5.standable_mask`, precomputed once): reject if the
     robot's body cannot rest at the landing cell without overlapping nearby
     terrain;
  3. **feasibility gate** (`feasible_alpha_interval`): reject unless some takeoff
     angle `alpha` satisfies both Campana Eq. 4 validity and the leg-energy limit
     `v_s = xdot/cos(alpha) <= V_max`;
  4. **clearance gate**: `terrain_profile` samples the corridor the body sweeps,
     then `alpha_for_clearance` picks a takeoff angle and reports the resulting
     clearance; reject if it is below `min_clearance_gate`;
  5. cost = XY distance + asymmetric elevation penalty (`alpha_uphill` /
     `alpha_downhill`) + `hop_fixed_cost`. **Clearance does not enter the cost** —
     it is purely a feasibility test.
- `disable_clearance=True` skips the stance and clearance gates but keeps the
  physics feasibility gate — used only for A/B baseline comparisons (e.g. in
  `test/benchmark_tall_stairs.py`), never for real planning. Note this is now a
  *feasibility* A/B ("which candidates exist"), not a cost-shaping one.

### The robot model — three things that are easy to get wrong

- **The arc is the CoM, not the feet.** Hops start and end at
  `terrain_z + leg_length`. `Z = z_g - z_s` is unchanged (both ends shift
  equally), so `feasible_alpha_interval` is untouched; only the absolute arc
  height moves. Anything comparing an arc to terrain must add the leg offset.
- **Clearance is monotone in `tan(alpha)`.** `dz/d tan a = u(X-u)/X >= 0` at every
  `u`, so a steeper angle lifts the whole arc at once and the max-clearance angle
  is *always* `alpha_max` — which is exactly where `v_s = V_max`. That is why
  `alpha_for_clearance` does not simply maximise: it takes the max-margin midpoint
  when that clears, and bisects for the shallowest sufficient angle otherwise.
  Monotonicity is what makes the bisection valid.
- **Collision geometry is a capsule, not just a sphere, and it's checked
  differently at stance than in flight — and it has three different radii,
  not one.** The full collision volume is a capsule from foot to CoM with
  three independently-sized regions: the CoM sphere (`ROBOT_RADIUS` = 0.15 m,
  the robot's actual body), the leg-cylinder sides (`LEG_CYLINDER_RADIUS` =
  0.01 m, thin), and the foot-tip hemisphere (`FOOT_TIP_RADIUS` = 0.02 m,
  slightly fatter than the leg). `standable_mask` (stance) checks the sphere
  at the CoM PLUS the upper `1 - LEG_CLEARANCE_START_FRAC` of the leg cylinder
  sides — the bottom `frac` is exempt so graded slopes stay standable; no
  foot-tip component at stance, since the foot is on the ground by
  definition. `clearance_for_alpha` (flight) checks the full capsule (top
  hemisphere at the CoM + full cylinder + bottom hemisphere at the foot), so
  terrain right under the foot must clear the foot tip by `FOOT_TIP_RADIUS +
  MIN_CLEARANCE`, not just `MIN_CLEARANCE`. Terrain *above* the CoM height
  during flight is checked against `ROBOT_RADIUS`, not `LEG_CYLINDER_RADIUS`
  — getting this wrong would silently let the CoM sphere shrink to leg-radius
  for any obstacle taller than the current arc height, exactly the case that
  matters most. Near-endpoint samples where the arc has barely lifted are
  masked (the rigid-vertical-leg model would spuriously report the foot
  grazing endpoint terrain there); `standable_mask` handles those.
- **Max standable grade** at shipped values is governed by the CoM sphere, not
  the leg-cylinder-sides formula you'd naively write down. The closed-form
  leg-cylinder ceiling is `(LEG_LENGTH * LEG_CLEARANCE_START_FRAC) /
  (LEG_CYLINDER_RADIUS + MIN_CLEARANCE)` ≈ 1.21, but `LEG_CYLINDER_RADIUS`
  (0.01 m) is thinner than `CELL_RESOLUTION` (0.1 m), so `standable_mask`'s
  discretized neighbour search never actually fires that check before the
  sphere-alone ceiling `sqrt((LEG_LENGTH / (ROBOT_RADIUS +
  MIN_CLEARANCE))^2 - 1)` ≈ 1.25 does. Both ceilings are far steeper than any
  map grade in this repo (`maps/slope_crest.py` ships at 0.35, chosen back
  when the ceiling was ~0.38 under the old shared-radius model — that
  reasoning is now stale, though the grade itself is still safely standable).
- Instrumentation counters (`n_expansions`, `n_edge_checks`, `n_edges_accepted`) are
  reset at the top of `plan()` and read by the benchmark script.

`astar_planner.py` is the earlier, simpler 8-connected grid A* (no ballistic model,
hard `MAX_JUMP_HEIGHT` cutoff instead of a physics gate). It's retained as a reference
implementation for comparison and is not used by `main.py` anymore.

### Config (`config.py`)

All tunables live here and are threaded explicitly through constructors (no globals
read deep in planner code, no config object passed around — `main.py` and the
`test/*.py` scripts import `config` and pass fields in by name). When adding a new
ballistic/clearance parameter, add it to `config.py` and thread it through both
`main.py` and any test/demo script that constructs a `HoppingAStarPlanner`.

Notable non-obvious parameters:
- `V_MAX` is **derived**, not tuned: `sqrt(2 * G_ACCEL * MAX_APEX_HEIGHT)`, i.e. the
  speed needed to raise the CoM `MAX_APEX_HEIGHT` on a vertical in-place hop. The
  longest feasible flat hop it affords is `V_MAX^2 / G_ACCEL`, and a flat hop of
  distance X needs `sqrt(g*X)`, so `HOP_RADIUS` must stay well under that. Change
  `MAX_APEX_HEIGHT` to change the robot, never `V_MAX` directly.
- `LEG_LENGTH - ROBOT_RADIUS` must exceed `MIN_CLEARANCE`, or *every* edge is
  rejected and `plan()` silently returns `None` everywhere. `config.py` asserts
  it. This is specifically about the CoM sphere's own-column stance check
  (`ROBOT_RADIUS`, not `LEG_CYLINDER_RADIUS` or `FOOT_TIP_RADIUS`).
- `OBSTACLE_WALL_EXTRA` is added on top of the map's max real elevation to get the
  height used for obstacle cells in the clearance check — obstacles aren't just
  "tall," they're "taller than anything else on the map," so bilinear interpolation
  near an obstacle edge doesn't accidentally produce a below-wall reading. It must
  exceed `LEG_LENGTH + MAX_APEX_HEIGHT - FOOT_TIP_RADIUS - MIN_CLEARANCE` or
  obstacles become jumpable — this is a foot-tip/bottom-cap concern, not a CoM
  one, since the foot is the lowest point of the flight capsule; `config.py`
  asserts this too, along with `ROBOT_RADIUS >= LEG_CYLINDER_RADIUS` and
  `ROBOT_RADIUS >= FOOT_TIP_RADIUS` (the lateral sampling corridor in
  `terrain_profile` is sized off the largest of the three radii).
- `HOP_SCAN_STEP` is a separate parameter from `CELL_RESOLUTION` so it can be
  tuned for speed — branching factor is proportional to `1/step`, making it the
  cheapest lever on planning time. But it also quantizes how far a hop can go:
  straight ahead you can only ever land on `x + step*k`, while diagonals project
  to different increments, so a coarse step makes A* reach for lateral doglegs to
  land on x-values the straight ladder skips. It ships equal to `CELL_RESOLUTION`
  for that reason. If you coarsen it for speed, check the paths for zig-zags
  before trusting the figures.
- `HOP_FIXED_COST` exists because `min_hop_radius` defaults to 0. Without it, N
  short hops along a straight line cost exactly as much as one long hop, so A* is
  indifferent and tie-breaks on heap order, producing jittery micro-hop chains.

Obstacles must be **at least two cells thick** across any arc that should be
blocked. A one-cell obstacle sampled along its own boundary is averaged 50/50 with
its neighbour by the bilinear lookup, halving its effective height.

### Visualizer (`visualizer.py`)

matplotlib rendering: colored elevation grid, obstacles, path, hop circles/arcs,
start/goal markers. Demo scripts largely bypass this and build their own custom
matplotlib figures (side-view arc strips, ring-candidate accept/reject panels) because
they need more specialized plots than the generic `Visualizer` provides.

## Working conventions in this repo

- No test framework — new numeric checks follow the `test/test_clearance_rejection.py`
  pattern: plain assertions, PASS/FAIL prints, `sys.exit(1)` on failure. New visual
  demos follow the `test/demo_*.py` pattern: build a scenario, run both the ballistic
  and baseline (`disable_clearance=True`) planners, save annotated PNGs via
  `demo_common.out_path()` (`results/after_LS_recs/` by default, override with
  `$PLANNER_OUT_DIR`).
- `CHANGELOG.md` (Keep a Changelog format) is actively maintained — update it under
  `[Unreleased]` when making a notable change to the planner or its parameters.
- Pure, unit-testable physics functions (`feasible_alpha_interval`, `predict_trajectory`,
  `min_clearance`) are kept as free functions in `hopping_astar_planner.py`, separate
  from the `HoppingAStarPlanner` class, specifically so they can be imported and tested
  in isolation (see `test/test_clearance_rejection.py`).

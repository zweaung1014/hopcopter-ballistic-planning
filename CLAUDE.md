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

# Visual demos — each produces PNGs into test/ (side-view arcs, top-down paths,
# ring-candidate panels, etc.); open the generated PNGs to inspect results
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
test/                     ← demo scripts (produce PNGs), a benchmark, and one
                            assertion-based numeric test (no pytest)
```

### Map2D5 (`map2d5.py`)

Stores terrain as a numpy 2D array of elevations (meters). `OBSTACLE = -1.0` is a
sentinel value, not a real elevation. Key methods: `world_to_grid`/`grid_to_world`
(cell centers are at `((col+0.5)*res, (row+0.5)*res)`), `is_obstacle`,
`get_elevation`, `get_elevation_bilinear` (used by the clearance check to sample
terrain height continuously along an arc, with obstacle cells substituted for a tall
"wall" fill value so they block arcs instead of producing bilinear artifacts),
`set_obstacle_region`.

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
  `hop_radius` down to `min_hop_radius` (default `hop_radius / 2`) in steps of one
  grid cell, adding *every* valid landing cell found along the ray — not just the
  farthest. This matters because a full-radius hop may land too close to an obstacle
  for the *next* arc to clear it, while a shorter hop in the same direction gives a
  better launch position. Two dedup sets (`attempted`, `in_results`) prevent
  redundant validation while still letting different directions scan past a
  previously *failed* cell to reach a valid shorter-radius one. A "goal-snap" edge is
  added whenever the goal is within `hop_radius`, so reaching it doesn't depend on
  angular alignment with the ring sampling.
- **Edge validation** (`_validate_and_cost`), in order:
  1. reject if the landing cell is an obstacle;
  2. **feasibility gate** (`feasible_alpha_interval`): reject unless some takeoff
     angle `alpha` satisfies both Campana Eq. 4 validity and the leg-energy limit
     `v_s = xdot/cos(alpha) <= V_max`;
  3. pick `alpha_s` as the midpoint of the feasible interval (max-margin choice);
  4. **clearance gate** (`min_clearance`): march along the arc's XY segment,
     comparing the parabola's height (Campana Eq. 2) against the terrain (bilinear
     lookup, obstacles treated as tall walls) minus `robot_radius`; reject if the arc
     ever intersects the terrain;
  5. cost = XY distance + asymmetric elevation penalty (`alpha_uphill` /
     `alpha_downhill`) + a smooth penalty that grows linearly as clearance shrinks
     below `clearance_margin` (zero once clearance ≥ margin).
- `disable_clearance=True` keeps the physics feasibility gate but skips the terrain
  clearance gate and its cost penalty — used only for A/B baseline comparisons (e.g.
  in `test/benchmark_tall_stairs.py`), never for real planning.
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
- `V_MAX` must exceed `sqrt(G_ACCEL * HOP_RADIUS)` with margin, or no hop of length
  `HOP_RADIUS` is physically reachable (the minimum feasible takeoff speed for a flat
  hop of distance X is `sqrt(g*X)`).
- `OBSTACLE_WALL_EXTRA` is added on top of the map's max real elevation to get the
  height used for obstacle cells in the clearance check — obstacles aren't just
  "tall," they're "taller than anything else on the map," so bilinear interpolation
  near an obstacle edge doesn't accidentally produce a below-wall reading.

### Visualizer (`visualizer.py`)

matplotlib rendering: colored elevation grid, obstacles, path, hop circles/arcs,
start/goal markers. Demo scripts largely bypass this and build their own custom
matplotlib figures (side-view arc strips, ring-candidate accept/reject panels) because
they need more specialized plots than the generic `Visualizer` provides.

## Working conventions in this repo

- No test framework — new numeric checks follow the `test/test_clearance_rejection.py`
  pattern: plain assertions, PASS/FAIL prints, `sys.exit(1)` on failure. New visual
  demos follow the `test/demo_*.py` pattern: build a scenario, run both the ballistic
  and baseline (`disable_clearance=True`) planners, save annotated PNGs into `test/`.
- `CHANGELOG.md` (Keep a Changelog format) is actively maintained — update it under
  `[Unreleased]` when making a notable change to the planner or its parameters.
- Pure, unit-testable physics functions (`feasible_alpha_interval`, `predict_trajectory`,
  `min_clearance`) are kept as free functions in `hopping_astar_planner.py`, separate
  from the `HoppingAStarPlanner` class, specifically so they can be imported and tested
  in isolation (see `test/test_clearance_rejection.py`).

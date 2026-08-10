# Ballistic Path Planning on a 2.5D Map

Path planning for a **hopping robot** on a 2.5D map — a 2D grid where each cell stores an
elevation (z) value. Each move is a ballistic (parabolic) hop rather than a step to an
adjacent cell, using the physics from Campana & Laumond (2016), *Ballistic motion planning*.

See [docs/planner.md](docs/planner.md) for the algorithm writeup and
[CHANGELOG.md](CHANGELOG.md) for how the planner evolved (RRT* → grid A* → hopping A* →
ballistic feasibility/clearance gating).

## Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Usage

```bash
python main.py                # default map ("stairs")
python main.py tall_stairs    # any module in maps/; an invalid name lists them
```

Opens a matplotlib window showing the elevation grid, the planned path and the hop arcs.

## The robot

The robot is a sphere of `ROBOT_RADIUS` whose centre — the point the parabola actually
tracks — sits `LEG_LENGTH` above the contact foot. A hop is accepted only if:

1. the landing cell is not an obstacle;
2. the body can **rest** there without overlapping nearby terrain;
3. some takeoff angle satisfies both Campana Eq. 4 and the leg-energy limit `v_s <= V_max`;
4. the arc keeps the body at least `MIN_CLEARANCE` clear of the terrain along the whole
   corridor the body sweeps.

Clearance is a pure feasibility gate — it does not shape cost. Edge cost is
`xy distance + W_ENERGY × (injected energy + momentum thrown away)`.

The energy term replaced an elevation penalty, which could not price the case it was
written for: a hop that arcs *over* an obstacle and lands at the same height has
`dz = 0` and was charged nothing. Injected energy rises on its own when the clearance
gate lifts the takeoff angle over terrain, so "hop over vs go around" is decided by
physics. The momentum term (`max(0, KE_in − KE_out)`) stops the planner chopping paths
into short hops, which need no thrust at all and would otherwise price as free. See
`results/energy_based_edge_cost/`.

## Configuration

Everything tunable lives in `config.py` and is threaded explicitly through constructors.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAP_SIZE_X` / `MAP_SIZE_Y` | 5.0 m | Map extent |
| `CELL_RESOLUTION` | 0.1 m | Grid cell size |
| `START` / `GOAL` | (0,0) / (4.5,4.5) | Endpoints, in world metres |
| `MAX_APEX_HEIGHT` | 1.2 m | CoM rise on a vertical in-place hop |
| `V_MAX` | *derived* | `sqrt(2 g h)` = 4.85 m/s — change the apex, not this |
| `LEG_LENGTH` | 0.4 m | CoM height above the foot |
| `ROBOT_RADIUS` | 0.2 m | Body radius, lateral and vertical |
| `MIN_CLEARANCE` | 0.15 m | Hard clearance gate |
| `HOP_RADIUS` | 1.0 m | Ring-sampling radius for candidate landings |
| `HOP_SCAN_STEP` | 0.1 m | Inward ray-search step; the dominant speed/quality knob |
| `W_ENERGY` | 0.84 m/J | Energy/distance exchange rate in the edge cost. Derived, not tuned |
| `ALPHA_MARGIN_FRAC` | 0.5 | Default takeoff angle within the feasible interval |
| `OBSTACLE_WALL_EXTRA` | 1.5 m | Height obstacle cells read as during clearance checks |

Two invariants are asserted at import: `LEG_LENGTH - ROBOT_RADIUS > MIN_CLEARANCE`
(else every edge is rejected and `plan()` silently returns `None`), and
`OBSTACLE_WALL_EXTRA >= LEG_LENGTH + MAX_APEX_HEIGHT - ROBOT_RADIUS - MIN_CLEARANCE`
(else obstacles become jumpable).

## Adding a map

Add a module to `maps/` exposing `build() -> Map2D5`. No registration step.

```python
env_map.paint_region(0.4, x_min=2.0, x_max=3.0, y_min=1.8, y_max=3.0)  # terrain
env_map.set_obstacle_region(2.0, 2.0, 3.0, 3.0)                        # impassable
```

Always use `paint_region` (world metres) rather than raw column slices — index-based
painting silently rescales the terrain when `CELL_RESOLUTION` changes. Obstacles should
be at least two cells thick, or bilinear sampling halves their effective height.

## Tests and demos

No test framework; these are standalone scripts. Each inserts the repo root onto
`sys.path`, so run them from anywhere.

```bash
python test/test_clearance_rejection.py   # numeric checks, PASS/FAIL, exit 1 on failure
python test/benchmark_tall_stairs.py      # ballistic vs. baseline timing
python test/time_deck.py results/x.md     # per-scenario timing table
python test/calibrate_geometry.py         # sweeps obstacle heights for the maps
python test/demo_overview.py              # contact sheet of all six scenarios
```

Demos write PNGs to `results/energy_aware_planning/`; set `$PLANNER_OUT_DIR` to redirect
(relative paths resolve against the repo root).

## Project Structure

```
├── config.py                 # All tunable parameters
├── map2d5.py                 # Map2D5: elevation grid, sampling, painting, stance mask
├── maps/                     # Scenario builders, each exposing build() -> Map2D5
├── hopping_astar_planner.py  # HoppingAStarPlanner + ballistic primitives (active)
├── astar_planner.py          # Earlier 8-connected grid A*, kept as reference
├── visualizer.py             # Visualizer: matplotlib rendering
├── main.py                   # Entry point
├── test/                     # Demos, benchmark, timing harness, numeric test
├── results/                  # Generated figures and timing tables, one dir per run
└── requirements.txt          # numpy, matplotlib
```

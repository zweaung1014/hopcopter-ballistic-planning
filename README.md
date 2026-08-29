# Ballistic Path Planning on a 2.5D Map

Path planning for a hopping robot on a 2.5D map — a 2D grid where each cell stores an
elevation. Each move is a ballistic (parabolic) hop rather than a step to an adjacent cell.

For how it works: [docs/planner.md](docs/planner.md), [CLAUDE.md](CLAUDE.md) and
[CHANGELOG.md](CHANGELOG.md).

## Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run it

```bash
python main.py              # default map: stairs
python main.py cliff_gap    # any module in maps/
```

You get a summary table in the terminal, a matplotlib window, and a PNG saved under
`results/demonstration_scenarios/`.

Available maps (an invalid name prints this list):

```
barely_jumpable_wall, cliff_gap, cross_slope, flat, gradual_slope, infinite_obstacle,
low_stairs, low_wall, maze, platform_0p4m, slope_30deg, slope_crest, stairs,
stairs_with_curb, tall_narrow_wall, tall_stairs, tall_wall
```

Settings live in `config.py`. Start and goal live in the map module, not in `config.py`.

## Tests and demos

Standalone scripts — no test framework. They set up their own paths, so you can run them
from anywhere.

```bash
python test/test_clearance_rejection.py   # prints PASS/FAIL, exits 1 on failure
python test/test_friction_cone.py
python test/test_hop_energy_chain.py
python test/test_edge_cost_energy.py
python test/test_inflated_field.py

python test/demo_clearance_sweep.py       # these write PNGs
python test/demo_takeoff_angle_range.py
python test/visualize_inflated_map.py
python test/time_deck.py results/timings.md
```

Demo PNGs go to `results/energy_aware_planning/`; set `$PLANNER_OUT_DIR` to redirect.

## Project layout

```
├── main.py                   # entry point
├── config.py                 # tunable parameters
├── map2d5.py                 # the elevation grid
├── maps/                     # scenarios, one module each
├── hopping_astar_planner.py  # the active planner
├── astar_planner.py          # earlier grid A*, kept as reference
├── visualizer.py             # matplotlib rendering
├── test/                     # tests and demo scripts
├── results/                  # generated figures and tables
├── docs/                     # algorithm write-ups
├── CHANGELOG.md
└── CLAUDE.md
```

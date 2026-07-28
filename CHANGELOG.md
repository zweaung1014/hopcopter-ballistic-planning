# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

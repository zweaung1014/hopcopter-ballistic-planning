# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
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

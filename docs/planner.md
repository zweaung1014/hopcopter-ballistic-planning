# Height-Aware A* Planner

> **Scope note.** This document describes `astar_planner.py`, the 8-connected grid A*
> that was the original planner. It is retained as a reference implementation and is
> **not** what `main.py` runs. The active planner is `HoppingAStarPlanner` in
> `hopping_astar_planner.py`, which models each move as a ballistic hop and replaces the
> hard `MAX_JUMP_HEIGHT` cutoff described below with a physics feasibility gate (Campana's
> BEAM: parabola validity, a Coulomb friction cone at both contacts, and the leg-speed
> budget at takeoff and landing), a stance check and an arc-clearance gate. See
> `CLAUDE.md` for the current design, `docs/alpha_range_new.md` for the takeoff-angle
> math, and `CHANGELOG.md` for how it got there.
>
> **One structural difference is worth flagging up front**, because everything below
> assumes otherwise: hops are no longer independent. The active planner carries energy
> along the path — each hop's takeoff speed is floored by the previous hop's landing
> speed, since the robot cannot shed energy — so its search state is
> `(cell, speed_bin)`, not `cell`, and the same cell reached two ways is two different
> nodes. The grid A* described here has no such notion.

## Overview

This planner implements A* (A-Star) with elevation-aware cost computation, designed for a hopping robot navigating a 2.5D terrain map.

The key insight is that a hopping robot can traverse elevated terrain (unlike a wheeled robot), but doing so has an energy cost. The planner balances:
- **Distance cost**: longer paths are more expensive
- **Elevation cost**: jumping up/down incurs energy penalties
- **Hard limits**: some height differences are simply too large to jump

A* is used instead of sampling-based planners (e.g. RRT*) because the terrain is a fully known discrete grid. A* on a grid is:
- **Guaranteed optimal** — finds the globally best path every time
- **Deterministic** — same map, same result
- **Fast** — a 50×50 grid (5 m map, 0.1 m cells) solves in under 1 ms, well within a 1 Hz
  replanning budget. (This figure is for *this* 8-connected planner. The ballistic planner
  on the same grid takes ~2.5 s, because every edge requires validating a parabola against
  the terrain rather than reading one neighbouring cell — see `results/*/timings.md`.)

## Algorithm

### Core A* Components

A* maintains an open set (priority queue) of cells to explore, ordered by estimated total cost $f = g + h$:

- $g(\text{cell})$ = best known cost from start to this cell
- $h(\text{cell})$ = heuristic estimate of cost from this cell to goal
- $f(\text{cell}) = g + h$ = estimated total path cost through this cell

At each step, the lowest-$f$ cell is expanded. When the goal cell is popped, the optimal path is reconstructed by tracing back through `came_from` pointers.

### Grid Connectivity

The map is searched with **8-connected neighbors** (cardinal + diagonal moves). This allows the planner to route at any angle, not just axis-aligned.

| Move type | XY distance |
|-----------|-------------|
| Cardinal (N/S/E/W) | `resolution` |
| Diagonal (NE/NW/SE/SW) | `resolution × √2` |

### Height-Aware Edge Cost

The cost of moving from cell $A$ to adjacent cell $B$ is:

$$\text{edge\_cost}(A \to B) = d_{xy} + \alpha \cdot |\Delta z|$$

Where:
- $d_{xy}$ is the XY distance (resolution or resolution × √2)
- $\Delta z = z_B - z_A$ is the height difference between the two cells
- $\alpha = \alpha_{\text{uphill}}$ if $\Delta z > 0$ (jumping up)
- $\alpha = \alpha_{\text{downhill}}$ if $\Delta z < 0$ (landing down)

The $\alpha$ values set the **exchange rate** between elevation and distance. For example, with `alpha_uphill = 5.0`, climbing 1m costs as much as traveling 5m horizontally. This means the planner prefers to go around an obstacle only if the detour is less than 5× the elevation gained by going over it.

> **These are now `astar_planner.py` constructor arguments only.** `ALPHA_UPHILL` and
> `ALPHA_DOWNHILL` were removed from `config.py` when the active planner replaced this
> elevation penalty with an energy cost,
> `xy_dist + W_ENERGY · (e_inject + max(0, KE_in − KE_out))`. The penalty below could not
> price the case it existed for: a hop that arcs **over** an obstacle and lands at the same
> height has $\Delta z = 0$ and is charged nothing. Only paths that land *on* the obstacle
> were ever priced. See `CLAUDE.md` § "The edge cost" and
> `results/energy_based_edge_cost/`.

### Hard Jump Constraint

A move to an adjacent cell is **rejected entirely** (treated like an obstacle) if:

$$|\Delta z| > \text{MAX\_JUMP\_HEIGHT}$$

This models the physical limit of the robot's hopping capability. Cells that require too large a jump are simply not considered, regardless of their distance savings.

### Heuristic

The heuristic is **Euclidean distance** to the goal in world coordinates:

$$h(\text{cell}) = \sqrt{(x - x_{\text{goal}})^2 + (y - y_{\text{goal}})^2}$$

This is **admissible** (never overestimates) because:
- The actual edge cost is always ≥ the XY distance
- Elevation penalties only add cost, never reduce it

An admissible heuristic guarantees A* finds the optimal path.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_JUMP_HEIGHT` | 0.5 m | Maximum height difference between adjacent cells. Moves exceeding this are impassable. Still in `config.py`. |
| `alpha_uphill` | 1.0 | Cost multiplier for uphill moves. Higher values make the planner prefer flat paths over elevated ones. **Constructor arg only** — no longer in `config.py`. |
| `alpha_downhill` | 0.5 | Cost multiplier for downhill moves (landings). Lower than uphill because landing is generally easier than jumping up. **Constructor arg only.** |

## Tuning Guide

> For the **active** planner these knobs do not exist; the equivalent dial is
> `config.W_ENERGY` (m per J). Raise it to prefer going around, lower it to prefer hopping
> over. Unlike the $\alpha$ values it has a derived default — `1 / 1.197` J, the cost of
> holding speed through one flat steady-state hop — so `0.84` means "weigh energy and
> distance exactly as the physics says" rather than an arbitrary starting point.

### Making the robot prefer flat paths (go around)
- Increase `alpha_uphill` and `alpha_downhill`
- The planner will accept longer 2D distances to avoid elevation changes

### Making the robot prefer short paths (hop over)
- Decrease `alpha_uphill` and `alpha_downhill` (toward 0)
- At 0, the planner ignores elevation entirely and finds the shortest 2D path

### Adjusting asymmetry
- `alpha_uphill > alpha_downhill`: jumping up costs more than landing down (default — realistic for a hopping robot)
- `alpha_uphill == alpha_downhill`: symmetric cost for up and down

### Hard constraint
- Lower `MAX_JUMP_HEIGHT` to make more terrain impassable (stricter robot limits)
- Set it very high (e.g., 100.0) to effectively disable the hard constraint and rely only on soft costs

## Architecture

```
config.py           ← All parameters in one place
map2d5.py           ← Map2D5: 2D grid with z-values, obstacle queries
astar_planner.py    ← AStarPlanner: A* search with height-aware edge costs
visualizer.py       ← Visualizer: matplotlib rendering
main.py             ← Entry point: build map → plan → visualize
```

### Key Classes

- **`Map2D5`**: Stores the terrain as a numpy array. Each cell holds an elevation (z in meters). Value -1 means obstacle. Provides `is_obstacle()`, `get_elevation()`, coordinate transforms.
- **`AStarPlanner`**: The planner. Runs A* on the grid, computes height-aware edge costs, returns the optimal path as world-coordinate waypoints.
- **`Visualizer`**: Draws the grid (colored by elevation), obstacles, path, and start/goal markers.

## Future Extensions

- **D\* Lite**: If the map changes incrementally between replans (a few cells updated), D* Lite reuses previous search results and only re-expands affected nodes, reducing replanning cost
- **Anisotropic cost**: Factor in approach angle for the hopping robot
- **Kinodynamic constraints**: Limit consecutive jump heights based on momentum
- **Elevation-colored path**: Color-code the path segments by their elevation cost contribution

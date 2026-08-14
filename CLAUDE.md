# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

A simulation/research codebase for path planning on a 2.5D map (a 2D grid where each
cell stores an elevation z) for a hopping robot. The active planner
(`HoppingAStarPlanner`) models each move as a ballistic (parabolic) hop rather than a
step to an adjacent grid cell, using the physics from Campana & Laumond (2016),
"Ballistic motion planning." See `docs/planner.md` for the full algorithm writeup and
`CHANGELOG.md` for the history of how the planner evolved (RRT* → grid A* → hopping A*
→ ballistic feasibility/clearance gating → energy-carrying hop chain).

**The robot HOPS, it does not jump.** Hops are not independent: the takeoff speed of
each one is set by the landing speed of the last, through a three-phase cycle
(ballistic descent with attitude control, an inverted-pendulum stance that returns
`ETA_HOP` of the arriving kinetic energy, then powered climbing thrust that may inject
up to `E_INJECT_MAX`). Thrust can only *add* energy, never shed it, so the incoming
speed is a hard FLOOR on what the next hop can do — and since `(X, Z, alpha)` fixes
`v_s` outright, that floor deletes every takeoff angle producing a lower parabola.
This is why the A* state is `(cell, speed_bin)` and not `cell`, and why anything
re-scoring a hop outside the planner has to know how the robot arrived.

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
python test/test_friction_cone.py       # BEAM / friction-cone ground-truth validation
python test/test_hop_energy_chain.py    # energy chain: closed forms, alpha bounds,
                                        # and end-to-end chain closure on a real plan
python test/test_edge_cost_energy.py    # energy-based edge cost: the momentum term,
                                        # non-negativity, and the over-a-wall case
python test/test_inflated_field.py      # the planner's inflated-field clearance check
                                        # vs. the reference capsule check it replaced:
                                        # asserts it is never MORE PERMISSIVE, plus
                                        # the lookup-pad guarantee and the
                                        # standable_mask equivalence

# Timing/scaling benchmark (ballistic vs. baseline planner on the same scenario)
python test/benchmark_tall_stairs.py

# Per-scenario timing table across the whole deck (writes markdown)
python test/time_deck.py results/energy_aware_planning/timings.md

# Visual demos — each writes PNGs into results/energy_aware_planning/ (side-view arcs,
# top-down paths, ring-candidate panels, etc.); set PLANNER_OUT_DIR to redirect.
# Open the generated PNGs to inspect results.
python test/demo_tall_stairs.py
python test/demo_clearance_sweep.py
python test/demo_friction_cone.py
python test/demo_barely_jumpable.py
python test/demo_narrow_wall_showcase.py
python test/demo_planner_reroute.py
python test/demo_prof_showcase.py
python test/demo_cost_model_ab.py    # 3 cost models x 3 scenarios: what each
                                     # energy term buys (~2 min)
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
                            harness, and three assertion-based numeric tests (no pytest)
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
channel, governs the max standable grade, ~1.25 at shipped values),
`surface_normals` (per-cell outward unit normals for the friction cone; memoised),
`inflated_field` (terrain inflated by a sphere or column of a given radius, in
absolute heights — the planner's flight-clearance primitive; memoised).

**`inflated_field` is what makes the robot a POINT.** It answers, per cell, "how
high must a body of this radius be when passing over here" — so the whole capsule
test becomes one scalar comparison, and the body's width lives in the map rather
than in a lateral sampling pattern. Two properties are load-bearing and both are
regression-tested in `test/test_inflated_field.py`: the `lookup_pad`
(`resolution*sqrt(2)/2`, without which a nearest-cell lookup can read a cell
whose value does not bound the query point), and `taper=False` for the CoM field
(the reference check treats terrain above the CoM as a full-height column, so a
sphere there over-rejects). `standable_mask` is provably this same object
evaluated at standing height — `inflated_field(com_radius+gate) <= grid +
leg_length` — and the two are kept separate only because `standable_mask` runs
once per planner and costs nothing.

**`surface_normals` uses min-|slope| one-sided differences, not a central
difference,** and that is a correctness requirement, not a refinement. A central
difference straddles a discontinuity and reports a fictitious ramp: at the foot of
`tall_stairs`' 0.4 m riser it reads grade 2.0, which makes the friction cone
degenerate and kills takeoff from exactly the cell the planner needs. Taking the
smaller-magnitude one-sided difference per axis picks the flat tread the foot
actually rests on, while still recovering a uniform grade exactly.

**Always paint terrain with `paint_region`,** which takes world-metre bounds. Raw
column slices (`grid[:, 10:13]`) hard-code a cell count, and `world_to_grid(...)`
plus an inclusive `+1` overshoots by up to one cell; both silently rescale the
physical terrain when `CELL_RESOLUTION` changes.

### Maps (`maps/`)

Each module (`stairs.py`, `tall_stairs.py`, `tall_wall.py`, `tall_narrow_wall.py`,
`barely_jumpable_wall.py`, `slope_crest.py`, `cross_slope.py`) exposes `build() -> Map2D5` and hand-paints elevation
regions directly into `env_map.grid`. `main.py` dynamically imports `maps.<name>` based
on the CLI arg. Add a new scenario by adding a new module here with a `build()`
function — no registration step needed.

### HoppingAStarPlanner (`hopping_astar_planner.py`) — the core algorithm

A* over **`(cell, speed_bin)` states**, not cells (`heapq` open set,
`g_cost`/`came_from` dicts, Euclidean-distance heuristic on the cell — still
admissible because elevation and proximity penalties only add cost). Neighbor
generation and edge validation are non-trivial:

- **The state carries energy.** `speed_bin = round(v_g / SPEED_BIN)`, and the
  canonical speed for a state is `speed_bin * SPEED_BIN`, not the exact speed that
  produced it — that is what keeps the graph deterministic and finite. Neither more
  nor less energy dominates (more raises the takeoff floor and shuts off shallow
  hops; less lowers the ceiling), so **there is no valid dominance pruning** and
  binning is the honest alternative. The start is seeded with a *virtual* landing
  speed `sqrt(2*g*H_INITIAL/ETA_HOP)`, chosen so the uniform stance rule
  `v_s = sqrt(eta)*v_g` yields a first takeoff speed of `sqrt(2*g*H_INITIAL)` — so
  the first hop needs no special case. Costs ~4x the expansions in practice,
  which is affordable because the clearance check is a read off a precomputed
  field rather than a per-hop terrain sample (below).
- **`plan()` populates `self.path_hops`** with per-hop `alpha_s`, `v_s`, `v_g`,
  `e_inject`, `apex_drop`, `X`, `Z`. **Read the takeoff angle from there, never
  re-derive it from the endpoints** — with the energy chain the angle depends on the
  entire path leading up to the hop.

- **Neighbor generation** (`_generate_hop_neighbors`): candidate landing cells are
  generated by a **scanline circle fill** — a `hop_scan_step`-spaced lattice
  (world metres) centred on the current cell, walked one row at a time, where
  each row's column span is bounded in closed form by the circle equation
  (`x_max = sqrt(reach^2 - y^2)`) rather than by a per-point distance test. Every
  point produced is inside the disk by construction, so unlike a "walk the
  bounding square, discard the corners" approach there is nothing generated and
  then thrown away, and unlike the fixed-angle ray-search this replaced,
  candidate spacing is uniform everywhere in the disk instead of dense near the
  centre and coarse at the rim (16 rays at a 2 m hop radius spaced candidates
  ~0.78 m apart at the rim against 0.1 m along each ray). Both bounds are padded
  by `hop_scan_step * sqrt(2) / 2` — mirroring `Map2D5.inflated_field`'s
  `lookup_pad` — so a cell the true circle merely *touches* isn't excluded on a
  rounding technicality: the outer bound (`hop_radius`) widens and the inner
  bound (`min_hop_radius`, default 0, an annulus floor) shrinks, both moves
  making the swept region strictly larger, never smaller. A single `attempted`
  dedup set is sufficient (no second set is needed the way the old ray-search's
  was): because a flat lattice sweep visits each grid cell once, a cell's
  validity never depends on which lattice point reached it first —
  `_validate_and_cost(current, current_z, neighbor, v_g_in)` is a pure function
  of those three arguments, all fixed for the expansion. A "goal-snap" edge is
  added whenever the goal is within `hop_radius`, so reaching it doesn't depend
  on lattice alignment.
- **Edge validation** (`_validate_and_cost`), cheapest test first:
  1. **stance gate** (`Map2D5.standable_mask`, precomputed once): reject if the
     robot's body cannot rest at the landing cell without overlapping nearby
     terrain. This also rejects OBSTACLE cells outright — `standable_mask` ANDs
     in `grid != OBSTACLE` unconditionally — so there is no separate obstacle
     check;
  2. **feasibility gate** (`feasible_alpha_interval`) — reject unless some takeoff
     angle `alpha` satisfies all six of: the **energy floor** from the parent hop
     (`v_s >= sqrt(eta)*v_g_in`, unshedable), the **injection ceiling**
     (`v_s <= sqrt(v_s_min^2 + 2*E_INJECT_MAX/m)`, capped at `V_max`), the
     **min-apex floor** (`min_apex_tan`), the Coulomb friction cone at the
     takeoff contact, the cone at the landing contact (mapped back through the
     arc), and `v_g <= V_g_max`. **The three energy bounds are evaluated
     first**, before the cones, because they are the ones that can wipe out
     most of the interval. Campana's Eq. 4 validity and his takeoff-speed bound
     `v_s <= V_max` are *not* checked separately: they are algebraically
     implied once the energy band is supplied — `_speed_tan_interval`'s roots
     satisfy Eq. 4's `X*tan(alpha) - Z > 0` on their own, and the injection
     ceiling's own `min(..., V_max^2)` already subsumes `v_s <= V_max`. (Because
     of this, `v_s_min`/`e_inject_max`/`mass` are required arguments to
     `feasible_alpha_interval` now — there is no bare-BEAM fallback mode.) This
     is the only gate that depends on the hop's *heading* (only it sees the
     surface normals) and the only one that depends on the hop's *history*. See
     `docs/alpha_range_new.md`;
  3. **clearance gate**: `clearance_floor_alpha` reads the **precomputed
     inflated height fields** along the hop's centreline and solves for
     `alpha_c`, the shallowest angle that clears, in closed form; reject if it
     exceeds `alpha_max`. The flown angle is then the least-injection one at or
     above it, so this step still decides the successor's energy and is not
     skippable;
  4. cost = XY distance + `w_energy * (e_inject + max(0, KE_in - KE_out))`. Clearance still does **not** enter the cost (it is purely a
     feasibility test), but injected energy now does — see "The edge cost" below.
     Note injection plays two distinct roles: step 3 minimises it *within* an edge
     (which angle to fly), step 4 prices it *across* edges (which hop to take).
- **The clearance check is precomputed per CELL, not derived per HOP,** and that
  distinction is the whole design. `terrain_profile` was keyed on a `(takeoff,
  landing)` **pair** — ~7M of them — so it could never be precomputed; a 60k FIFO
  cache compensated at a 17-34% hit rate, and the search still made **155M grid
  reads over a 2,500-cell map, ~62,000 per cell of terrain that never changes**.
  `Map2D5.inflated_field` is keyed on a **cell** instead, so all 2,500 answers
  are computed once in 0.26 ms. Two fields are built (`_inflated_foot`,
  `_inflated_com`), because the capsule's regions differ in radius *and* shape:
  the foot tip is a sphere (tapered) while terrain above the CoM is treated as a
  full-height column (untapered). **Do not add a taper to the CoM field** — it
  silently over-rejects. The cache is gone; nothing per-hop is derived any more.
- `disable_clearance=True` skips the stance and clearance gates but keeps the
  physics feasibility gate — used only for A/B baseline comparisons (e.g. in
  `test/benchmark_tall_stairs.py`), never for real planning. Note this is now a
  *feasibility* A/B ("which candidates exist"), not a cost-shaping one. It still
  picks a flown angle (the least-injection one, without a terrain profile to
  consult), because the successor state's energy depends on it.
- `mu=None` is the analogous knob for the friction cone: it drops BEAM constraints
  (1) and (2) while keeping the energy band and the landing-speed limit. A/B
  baselines only (`test/demo_friction_cone.py`), never real planning.
- `charge_momentum=False` is the third of these knobs: `_edge_cost` then charges
  injected energy but not the momentum a hop throws away. **Unlike the other two
  it is not a weaker model but a known-broken one**, and worse than dropping the
  energy term entirely (`w_energy=0`) — short hops need no thrust, so on injection
  alone they price as free and A* chops paths into stubs that arrive drained. It
  exists so `test/demo_cost_model_ab.py` can show that; do not reach for it to
  claw back planning time.
- **`v_s_min`/`e_inject_max`/`mass` are required arguments** to
  `feasible_alpha_interval`, not optional. There is no bare-BEAM fallback mode:
  Eq. 4 validity and the takeoff-speed bound are algebraically implied by the
  energy band (see "Edge validation" above), so a caller with no energy state
  to supply cannot use this function. `test/test_friction_cone.py` and parts of
  `test/test_clearance_rejection.py` previously exercised BEAM in isolation
  with a bare `(X, Z, V_max, g)` call; that mode no longer exists and those
  tests are expected to fail until rewritten.

### The edge cost — why it is energy and not elevation

`cost = xy_dist + w_energy * (e_inject + max(0, KE_in - KE_out))`. Both energy
terms are `>= 0` by construction, so the Euclidean-XY heuristic stays admissible.

- **`e_inject` was already computed on every edge and thrown away.** It rises on
  its own when the clearance gate lifts `alpha` over terrain, and it prices
  climbing without help (`Z = +0.4` costs 3.73 J against 1.20 J flat). That is
  why there is no elevation term any more: `alpha_uphill * dz` was not
  miscalibrated, it was *blind* — arcing over a wall and landing on the flat
  beyond is `dz = 0`, so it charged nothing for exactly the manoeuvre it existed
  to price.
- **The momentum term is load-bearing, not a refinement.** Charging `e_inject`
  alone is worse than the old cost: short hops need no thrust (the robot already
  carries the speed from its last landing), so they price as FREE and A* chops
  paths into stubs — `[1.0, 1.0, 1.0]` became `[0.8, 1.0, 0.8, 0.4]`, arriving at
  2.56 m/s instead of 3.16. It did not save energy, it spent momentum. Summing
  `e_inject` counts the battery but not the bank account.
- **It is also what regulates hop count**, which is why there is no per-hop
  energy constant. Holding speed steady costs `0.5*m*v^2*(1-eta)` = 1.197 J per
  hop — an expression with **no hop length in it** — so N hops over the same
  ground cost N times as much.
- **`KE_out` uses the BINNED landing speed**, matching `KE_in` and the speed the
  successor state stores. Mixing binned-in with exact-out leaves ~0.32 J of
  quantisation noise per hop against a ~1.2 J signal.
- **The `max(0, ...)` cap is required, not defensive.** The uncapped form
  `e_inject + KE_in - KE_out` is exact potential shaping and telescopes neatly,
  but reaches **-1.36 J** on a 0.4 m drop. Negative edge costs break A*.
- **Do not switch to charging stance dissipation** instead of injection. It looks
  attractive (monotone in hop count, no cap needed) but makes climbing *cheaper*
  than flat ground — 1.02 J for a 0.4 m climb against 1.20 J flat, because a
  climb lands slower — so the robot seeks out hills.
- **Do not re-add a raw obstacle-height penalty.** It over-counts ~3x: a 0.5 m
  wall costs 1.67 J on the crossing hop but only 0.50 J once chained, because
  launching harder means landing harder and the next hop needs no injection
  (`[1.20, 2.86, 0.03, 1.20]`). Only a path sum can credit that refund.
- **`W_ENERGY` is a runtime dial as well as a behaviour one.** `_heuristic`
  estimates distance only, so the whole energy term is cost it cannot anticipate
  and A* degrades toward Dijkstra as `w_energy` rises. Turning it on cost 4-6x on
  the two maps measured. The known fix is a per-hop energy constant, which unlike
  the momentum charge can be lower-bounded from `ceil(dist / hop_radius)`.

See `results/energy_based_edge_cost/` for the measurements behind all of the
above.

### The robot model — four things that are easy to get wrong

- **Energy is bookkept as full CoM kinetic energy at touchdown (`m*v^2/2`), not as
  apex height (`m*g*h`).** This is deliberate and load-bearing. The CoM sits at
  `terrain + LEG_LENGTH` at touchdown *and* at takeoff — same foot, same leg — so
  net ΔPE across stance is exactly zero and the whole balance is kinetic, with no
  PE term to get wrong. `m*g*h_apex` counts only the *vertical* share and would
  leave horizontal speed undamped, which is backwards: redirecting horizontal
  momentum is the entire job of the inverted-pendulum stance. It is also the only
  version that makes "delete the lower parabolas" well-posed — `v_g` plus `eta`
  pins `v_s` to one number, and `v_s` with `(X, Z)` pins `alpha` through Eq. 4,
  whereas apex energy only constrains `v_s*sin(alpha)`. The apex picture is not
  lost: `h_drop = v_z_land^2 / (2g)` is a projection of the same state, which is
  what `min_apex` gates on.
- **The arc is the CoM, not the feet.** Hops start and end at
  `terrain_z + leg_length`. `Z = z_g - z_s` is unchanged (both ends shift
  equally), so `feasible_alpha_interval` is untouched; only the absolute arc
  height moves. Anything comparing an arc to terrain must add the leg offset.
- **Clearance is monotone in `tan(alpha)`,** and so is apex height and everything
  else about how high the arc sits. `dz/d tan a = u(X-u)/X >= 0` at every `u`, so a
  steeper angle lifts the whole arc at once — which is what makes "higher parabola"
  and "larger alpha" the same statement, lets every energy constraint reduce to a
  bound on `tan(alpha)`, and makes the clearing set an upward-closed interval
  `[alpha_c, alpha_max]`. Required *speed*, by contrast, is U-shaped in alpha with
  its minimum at `min_energy_tan`. Together those give an exact answer rather
  than a search: `clamp(alpha*, alpha_c, alpha_max)`. In practice this collapses
  to `alpha_c` — `alpha*` sits below `alpha_min` whenever the energy floor binds,
  since that floor starts on the steep branch above `alpha*`.

  Linearity is the stronger statement, and it is what `clearance_floor_alpha`
  uses: `arc_z` is not merely monotone in `T = tan(alpha)`, it is **affine** in
  it, so `alpha_c` inverts outright —
  `T_req(u) = (H(u) - t_s - Z*u^2/X^2) * X / (u*(X-u))`, maximised over `u`. The
  reference `alpha_for_clearance` still bisects (monotonicity is all a bisection
  needs); the planner does not. **The `u*(X-u)` denominator is a pole at each
  end,** which is why `ARC_SAMPLE_MAX_STEP` cannot be coarsened even though the
  inflated fields are smooth — see its note in `config.py`.
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
- `MU` (Coulomb friction, 1.2) is a **hard physical constraint**, not a safety
  factor. On flat ground the cone alone forces `alpha >= atan(1/MU)` = 39.8°, well
  above the 12.3° the pre-cone interval allowed for a 1 m hop, so it changes the
  chosen angle on essentially every edge. It also caps the steepest contactable
  cross-slope at `MU` itself — which at 1.2 sits just under `standable_mask`'s
  geometric ceiling (~1.21), making friction the binding standability limit by a
  hair. `config.py` asserts the flat-ground cone floor stays below the leg-energy
  ceiling at `HOP_RADIUS`; violating it empties the interval for *every* flat hop
  and `plan()` returns None everywhere with no other symptom.
- **`V_G_MAX` is the cap that matters; `V_MAX` and `MAX_APEX_HEIGHT` are derived from
  it.** `V_G_MAX = sqrt(2*G_ACCEL*MAX_LANDING_APEX)` = 7.00 m/s is the fastest
  touchdown the leg can absorb, sized for hopping off a 2.5 m platform. Because
  `v_s_min = sqrt(ETA_HOP)*v_g` on the next hop, bounding `v_g` recursively bounds
  every takeoff speed in the plan. Then
  `V_MAX = sqrt(ETA_HOP*V_G_MAX^2 + 2*E_INJECT_MAX/ROBOT_MASS)` = 7.35 m/s and
  `MAX_APEX_HEIGHT = V_MAX^2/(2g)` = 2.75 m.
  **`V_MAX` is NOT a per-hop cap** — each hop is capped by its own parent
  (`sqrt(v_s_min^2 + 2*E_INJECT_MAX/m)`, ~5.4 m/s in the flat steady state). `V_MAX`
  is the arithmetic worst case, reached only right after a max-height drop; its jobs
  are sizing `OBSTACLE_WALL_EXTRA` and acting as a never-binding backstop on
  Campana's constraint (3). To change the robot, change `MAX_LANDING_APEX`,
  `E_INJECT_MAX` or `ETA_HOP` — never `V_MAX` or `MAX_APEX_HEIGHT` directly.
  The longest feasible flat hop is `V_MAX^2 / G_ACCEL` = 5.50 m, and a flat hop of
  distance X needs `sqrt(g*X)`, so `HOP_RADIUS` must stay well under that.
- `H_INITIAL` (1.0 m) is the first hop's apex, and the **first hop is the tightest in
  any plan** — it hands the robot more energy than a `HOP_RADIUS` hop needs and the
  robot cannot shed it, so the energy *floor* (not the ceiling) is what nearly
  empties the interval. At shipped values that floor is 75.0° against an 82.8°
  ceiling. `config.py` asserts the interval survives; violating it returns `None`
  from `plan()` everywhere with no other symptom.
- `MIN_APEX_HEIGHT` (0.3 m) is a **mechanism requirement, not a safety margin**:
  below it the elastic leg does not compress enough for the controller to detect a
  stance phase, so the cycle fails outright. It is what the chain converges to —
  on flat 1 m hops the steady state sits exactly at a 0.30 m drop (50.2°).
- `ETA_HOP` (0.7) is the fraction of kinetic energy surviving stance. Note
  `sqrt(0.7)` = 0.837, so **speed** drops ~16% per hop while **energy** drops 30%;
  reasoning about it in speed terms when the parameter is defined in energy terms is
  an easy way to be wrong by a square root.
- `SPEED_BIN` (0.25 m/s) quantises the search state's energy axis. Too coarse and
  the propagated speed drifts from the arc actually flown; too fine and the state
  space multiplies for no modelling gain. Rounding to nearest costs at most
  ±0.125 m/s.
- `LEG_LENGTH - ROBOT_RADIUS` must exceed `MIN_CLEARANCE`, or *every* edge is
  rejected and `plan()` silently returns `None` everywhere. `config.py` asserts
  it. This is specifically about the CoM sphere's own-column stance check
  (`ROBOT_RADIUS`, not `LEG_CYLINDER_RADIUS` or `FOOT_TIP_RADIUS`).
- `OBSTACLE_WALL_EXTRA` **no longer gates the planner** — `inflated_field` gives
  OBSTACLE cells `+inf`, which is strictly stronger and needs no calibration
  against `MAX_APEX_HEIGHT`. It still governs the reference implementation
  (`terrain_profile`) and every `test/demo_*.py` that reads
  `planner._obstacle_fill`, so the assert below is still live. It is added on top
  of the map's max real elevation to get the
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
- `W_ENERGY` (0.84 m/J) is the energy/distance exchange rate in the edge cost.
  Derived, not tuned: `1 / 1.197` J, so one flat steady-state hop's energy cost
  equals its distance cost. Raise it to bias toward detouring around obstacles,
  lower it to bias toward hopping over. **It also governs planning time** — see
  "The edge cost" above.

Obstacles **no longer have to be two cells thick.** That rule existed because the
old flight check read terrain bilinearly, which averaged a one-cell obstacle
50/50 with its neighbour and halved its effective height. The planner now reads
`Map2D5.inflated_field`, which is a `max` and so never under-reports. The rule
still applies to anything going through `sample_bilinear` directly — including
`terrain_profile` / `clearance_for_alpha`, retained as the reference
implementation.

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
  `demo_common.out_path()` (`results/energy_aware_planning/` by default, override with
  `$PLANNER_OUT_DIR`).
- **Diagnostics that re-score an edge outside the planner must go through
  `demo_common.planner_alpha_interval`,** never a bare `feasible_alpha_interval(X, Z,
  V_max, g)`. Two things are invisible in `X` and `Z` alone: the friction cone needs
  both surface normals and the heading (the bare call silently assumes level ground,
  so on sloped terrain the diagnostic disagrees with the planner it is explaining),
  and the energy band needs the speed the robot *arrived* at.
- **Diagnostics over a whole path must CHAIN, not map.** Use
  `demo_common.diagnose_path`, which threads `v_g` forward hop by hop; calling
  `diagnose_edge` in a loop scores every hop against the start-of-chain energy and
  reports angles the robot could not have flown. Same for
  `enumerate_ring_candidates` at a cell partway along a path — pass the `v_g_in` from
  `path_hops` or it will show a different robot's options than the figure does.
- `CHANGELOG.md` (Keep a Changelog format) is actively maintained — update it under
  `[Unreleased]` when making a notable change to the planner or its parameters.
- Pure, unit-testable physics functions (`feasible_alpha_interval`, `predict_trajectory`,
  `min_clearance`, `min_apex_tan`, `min_energy_tan`, `takeoff_speed`, `landing_speed`,
  `injection_energy`) are kept as free functions in `hopping_astar_planner.py`, separate
  from the `HoppingAStarPlanner` class, specifically so they can be imported and tested
  in isolation (see `test/test_clearance_rejection.py`,
  `test/test_hop_energy_chain.py`).

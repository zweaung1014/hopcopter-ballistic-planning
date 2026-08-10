# after_LS_recs — deck re-run under the revised robot model

Regenerated from `results/before_LS_recs/` after six changes to the robot and planner.
Same 21 figures, same scenarios, same start/goal pairs.

## What changed

| | before | after |
|---|---|---|
| Grid resolution | 0.2 m (25×25) | **0.1 m (50×50)** |
| Jump capability | `V_MAX = 6.0` m/s (tuned per demo) | **`MAX_APEX_HEIGHT = 1.2 m` → `V_MAX = 4.85` m/s (derived)** |
| Robot | point mass, radius subtracted vertically | **sphere, `r = 0.2 m`, checked laterally too** |
| Leg | none — arcs started on the ground | **`LEG_LENGTH = 0.4 m`; arcs track the CoM** |
| Clearance | soft penalty below 0.15 m | **hard reject below `MIN_CLEARANCE = 0.15 m`** |
| Min hop radius | `hop_radius / 2` | **0** |

## Read this before comparing the figures

**Terrain changed too, so this is not a controlled A/B.** The 0.4 m leg lifts every arc,
which made all four calibrated obstacles invisible — the clearance gate accepted every
takeoff position on `barely_jumpable_wall`, `tall_narrow_wall`, `stairs_with_curb` and
`slope_crest`. Obstacle heights were re-derived empirically to keep the gate live:

| map | before | after |
|---|---|---|
| `tall_narrow_wall` | 0.15 m | 0.70 m |
| `barely_jumpable_wall` | 0.22 m | 1.00 m |
| `stairs_with_curb` curb rise | 0.10 m | 0.60 m |
| `slope_crest` crest above shelf | 0.25 m | 0.85 m (ramp regraded 0.75 → 0.50) |
| `tall_wall`, `tall_stairs`, `stairs` | — | unchanged |

Density, physics and terrain therefore all move at once. Attribute a figure's change to
the planner only where the map is one of the unchanged three.

## Timings

Full table in [timings.md](timings.md); baseline in
[../before_LS_recs/timings.md](../before_LS_recs/timings.md).

| scenario | before (s) | after (s) | ratio | hops b→a | expansions b→a | max v_s/V_max |
|---|---:|---:|---:|:--:|:--:|---:|
| tall_wall | 0.201 | 1.720 | 8.6× | 3→4 | 173→256 | 0.938 |
| tall_narrow_wall | 0.175 | 0.569 | 3.3× | 4→3 | 133→78 | 0.913 |
| barely_jumpable_wall | 0.148 | 0.520 | 3.5× | 4→3 | 117→76 | 0.970 |
| tall_stairs | 0.295 | 6.547 | 22.2× | 4→3 | 321→1196 | 0.902 |
| stairs_with_curb | 0.314 | 5.478 | 17.4× | 3→5 | 363→1035 | 0.950 |
| slope_crest | 0.259 | 7.355 | 28.4× | 4→4 | 273→1053 | 0.861 |
| stairs | 0.037 | 0.718 | 19.4× | 4→4 | 27→96 | 0.819 |

**Ballistic total: 1.43 s → 22.9 s (16×).** The whole demo suite runs in ~73 s.

Most of that is unavoidable: 4× the cells, a 3-wide swept corridor instead of a
centreline, and a stance check per candidate. The parts that were optimised:

- **Two-phase edge evaluation.** The corridor the body sweeps does not depend on the
  takeoff angle — only the arc's height does. Terrain is sampled once per edge
  (`terrain_profile`), then each candidate angle costs a few array ops
  (`clearance_for_alpha`). This is what makes the angle search essentially free, and
  it is why the takeoff-angle escalation added almost nothing to the bill.
- **Vectorized bilinear sampling with a memoised obstacle-substituted grid**, on
  `Map2D5` rather than the planner, so all six external callers of `min_clearance`
  benefit.
- **A cheap stance mask** consulted before any arc work, killing invalid landings in
  ~1 µs instead of ~35 µs.

A direct implementation without these measured **13.7 s for `tall_stairs` alone**.

### A note on HOP_SCAN_STEP

`HOP_SCAN_STEP` (0.1 m) is the dominant speed knob — the branching factor is
proportional to `1/step`, so coarsening it is by far the cheapest way to make
planning faster. It was briefly shipped at 0.3 m, which cut the deck to ~8 s.

That was reverted, because coarsening it also quantizes how far a hop can travel:
straight ahead you can only land on `x + step*k`, while diagonal ring samples project
to different increments. At 0.3 m that produced a visible 0.30 m lateral dogleg in the
`slope_crest` path — A* stepping sideways purely to reach an x-position the
straight-ahead ladder skipped. Measured:

| `HOP_SCAN_STEP` | slope_crest | plan time |
|---|---|---|
| 0.3 m | 5 hops, 0.30 m y-detour | 2.6 s |
| 0.2 m | 5 hops, straight | 4.0 s |
| **0.1 m (shipped)** | **4 hops, straight** | **7.2 s** |

Worth knowing if you tune it for speed later: check the paths for zig-zags before
trusting the figures.

## Notable behavioural findings

**The takeoff angle now adapts.** Clearance is monotone in `tan(α)`, so the
maximum-clearance angle is always `α_max` — which is exactly where the leg runs at 100% of
`V_max`. Rather than always flying at the limit, the planner takes the max-margin midpoint
when it clears and bisects for the shallowest sufficient angle otherwise. The
`max v_s/V_max` column shows the result: `stairs` and `slope_crest` top out at 82–86% of
the leg's budget, while `barely_jumpable_wall` pushes to 97% at its one viable crossing —
the planner spends the extra energy only where the terrain demands it.

**What prunes the search is mostly the body, not the physics.** On `tall_stairs` the old
model rejected *nothing* (14,794 edge checks, 14,794 accepted). Now ~20% of edges are
rejected — and on plain risers the rejections are `stance`, not `clearance`: the robot
cannot stand within 0.2 m of a step without its body overlapping the riser. The A/B
against `disable_clearance=True` is now a feasibility comparison, not a cost-shaping one.

**The baseline fails differently.** Previously the clearance-off planner produced paths
with *clipping arcs*. Now it more often produces paths that **land where the body cannot
rest** — the arc is fine, the standing pose is not. `n_bad_hops` was extended to count
this; without it the A/B looked falsely clean on three maps.

**A 0.75 grade is untraversable for this robot.** There is a closed form for the steepest
standable slope, `sqrt((LEG/(R + MIN_CLEARANCE))² − 1)` = **0.553**, confirmed empirically
(0.55 → 100% standable, 0.60 → 0%). `slope_crest`'s original ramp sat above it and the
planner returned no path at all until the ramp was regraded.

## Figures

| file | scenario | source |
|---|---|---|
| `overview_contact_sheet.png` | all six scenarios | `test/demo_overview.py` |
| `prof_showcase_{topdown,arcs}.png` | `tall_wall` — go around | `test/demo_prof_showcase.py` |
| `prof_narrow_wall_{topdown,arcs}.png` | `tall_narrow_wall` — hop over | `test/demo_narrow_wall_showcase.py` |
| `barely_jumpable_{topdown,arcs,crossing}.png` | `barely_jumpable_wall` — one viable takeoff | `test/demo_barely_jumpable.py` |
| `stairs_{topdown,arcs,ring_candidates}.png` | `tall_stairs` | `test/demo_tall_stairs.py` |
| `stairs_curb_{topdown,arcs,ring_candidates}.png` | `stairs_with_curb` | `test/demo_stairs_curb.py` |
| `slope_crest_{topdown,arcs,profile}.png` | `slope_crest` | `test/demo_slope_crest.py` |
| `clearance_sweep.png` | takeoff sweep past a pillar | `test/demo_clearance_sweep.py` |
| `decision_sweep.png` | over-vs-around as wall height rises | `test/demo_decision_sweep.py` |
| `planner_reroute_{topdown,sideview}.png` | reroute around an OBSTACLE pillar | `test/demo_planner_reroute.py` |

## Reproducing

```bash
source .venv/bin/activate
for d in test/demo_*.py; do python "$d"; done
python test/time_deck.py results/after_LS_recs/timings.md
```

Output location is `demo_common.out_dir()`; set `$PLANNER_OUT_DIR` to write elsewhere.

# Takeoff angle (α) selection in `hopping_astar_planner.py`

Supersedes [`alpha_range_old.md`](alpha_range_old.md), which describes the pre-friction-cone
state. The implementation is now Campana & Laumond's **BEAM** (IROS 2016, "Ballistic motion
planning", Secs. III–V) — see [`alpha_range_campana.md`](alpha_range_campana.md) for the
spec this was built from, **including two errata recorded at the bottom of this file**.

α is measured from the world horizontal, not from the local surface.

---

## Stage 1 — build the feasible interval

`feasible_alpha_interval(X, Z, V_max, g, *, mu, n_s, n_g, theta, v_s_min,
e_inject_max, mass, min_apex, V_g_max)`

Given `X` and `Z`, choosing α fixes the parabola completely (Campana Eq. 4:
`ẋ² = g·X² / (2·(X·tan α − Z))`). So every physical constraint reduces to an interval on α,
and feasibility is their intersection. There are eight: three from this robot's energy
chain, then Campana's five.

Arc height is monotone increasing in `tan α` (`∂z/∂ tan α = u(X−u)/X ≥ 0` at every `u`), so
"higher parabola" and "larger α" are the same statement, and every row below is literally a
bound on how high the arc may be.

| # | constraint | interval | binds when |
|---|---|---|---|
| (E1) | **Energy floor** — the robot arrives with a takeoff speed it cannot shed, so every parabola below the one `v_s_min` produces is unreachable | complement of `v_s² ≤ v_s_min²`, upper branch | short hops, and right after a tall drop |
| (E2) | **Injection ceiling**, `v_s ≤ √(v_s_min² + 2·E_inject_max/m)` — one stance-plus-thrust cycle | same quadratic as (3), with the band's `W` | long hops from a slow arrival |
| (E3) | **Minimum drop** `min_apex` — below it the elastic leg never compresses enough for the controller to register a stance phase | `tan α ≥ 2(Z + h + √(h(h+Z)))/X` | the steady state on flat ground |
| (i) | **Eq. 4 validity** — a projectile falls below its launch ray, so you must aim above the chord | `α ∈ (atan2(Z, X), π/2)` | steep uphill |
| (1) | **Takeoff non-sliding** — the push-off direction *is* α | `α ∈ [γ_s − δ_s, γ_s + δ_s]` | flat ground, when the energy floor is slack |
| (2) | **Landing non-sliding** — the *reversed* arrival direction must lie in the goal's cone | mapped through the arc, see below | uphill landings |
| (3) | **Takeoff speed**, `v_s ≤ V_max` | `atan` of the roots of a quadratic in `tan α` | never, once (E2) is supplied — `V_max` is the global worst case over all parents, so (E2) is strictly tighter |
| (4) | **Landing speed**, `v_g ≤ V_g_max` | same quadratic with `V_g_max² → V_g_max² + 2gZ` | deep drops |

The energy rows are evaluated **first**, before Eq. 4 validity and before the cones,
because they are the ones that can wipe out most of the interval. Passing `v_s_min=None`
skips them entirely and recovers the pre-energy-chain behaviour — which is what lets
`test/test_friction_cone.py` keep exercising BEAM in isolation with a bare
`(X, Z, V_max, g)` call.

This is **not** map-free: (1) and (2) need the surface normal at each contact. Nor is it
history-free any more: (E1) and (E2) need the speed the robot arrived with, which is why
the A* state is `(cell, speed_bin)` and why diagnostics must go through
`demo_common.diagnose_path` (which chains `v_g` forward) rather than scoring hops
independently.

### The energy chain

Three phases, of which only stance is lossy and only thrust adds:

```
flight  (lossless):   v_g² = v_s² − 2·g·Z
stance  (η loss):     v_s_min′ = √η · v_g                     η = ETA_HOP = 0.7
thrust  (injection):  v_s′ ∈ [v_s_min′, √(v_s_min′² + 2·E_inject_max/m)]
```

Stance carries no potential-energy term because the CoM sits at `terrain + LEG_LENGTH` at
touchdown *and* at takeoff — same foot, same leg — so ΔPE across it is exactly zero and the
whole balance is kinetic. That is why the bookkeeping is on full CoM kinetic energy at
touchdown rather than on apex height: `m·g·h_apex` counts only the vertical share and would
leave the horizontal speed, which is precisely what the inverted-pendulum stance redirects,
undamped.

The floor is **hard**: propellers only add. This is what makes (E1) a deletion rather than a
preference. Since `_speed_tan_interval` returns where `v_s² ≤ W`, the floor is that
interval's complement, which has two branches; the implementation keeps the upper one (the
higher parabola). When `v_s_min` cannot reach `(X, Z)` at any angle the floor is vacuous and
the lower bound falls back to (E2)'s own lower root.

(E3) is `max`'d against (E1), so it is a pure fallback: it changes nothing unless the
incoming speed's own parabola is too low.

### The minimum-drop bound

Differentiating Eq. 2 and substituting Eq. 4 gives the vertical velocity at landing,
`v_z_g = v_s·cos α·(2Z/X − tan α)` — so the robot is descending at all only when
`X·tan α > 2Z`. The fall from apex is `h = v_z_g²/(2g)`; substituting Eq. 4 once more clears
`v_s` out entirely and leaves, with `T = tan α` and `s = X·T − Z`:

```
h_drop = (X·T − 2Z)² / (4(X·T − Z)) = (s − Z)² / (4s)
d/ds = (s² − Z²) / (4s²) ≥ 0    whenever s ≥ |Z| — which holds on the whole valid domain
```

Monotone, so it inverts to a single lower bound with no bisection:
`s = (Z + 2h) + 2√(h(h+Z))`, hence `T = (s + Z)/X`. Vacuous when `h + Z < 0` (the terrain
already drops further than `h`). Level-ground check: `T = 4h/X`, matching `apex = X·tan α/4`.

Measured apex → landing, not takeoff → apex: it is the *fall* that compresses the leg, and
on an uphill hop the robot can rise 0.3 m and still land while barely descending.

### The two velocity constraints are one formula

`_speed_tan_interval(X, Z, W, g)` returns the `tan α` interval satisfying `v_s² ≤ W`.
Substituting Eq. 4 into `v_s = ẋ / cos α` and writing `T = tan α`:

```
v_s² = g·X²·(1 + T²) / (2·(X·T − Z))            ≤ W
  ⟺  g·X²·T² − 2·W·X·T + (g·X² + 2·W·Z) ≤ 0
  ⟹  T = (W ± √D) / (g·X),   D = W² − g²X² − 2gZW
```

Energy conservation on the CoM gives `v_g² = v_s² − 2gZ`, so `v_g ≤ V_max` is the *same*
constraint with `W = V_max² + 2gZ`. That reproduces the paper's `Δ` and `Λ` exactly:

* `W = V_max²` → `D = V_max⁴ − 2gZV_max² − g²X²` = the paper's `Δ`
* `W = V_max² + 2gZ` → `D = V_max⁴ + 2gZV_max² − g²X²` = the paper's `Λ`

`D < 0` means no angle meets the bound. `W ≤ 0` (only reachable for (4), on a drop where
`2g|Z|` alone exceeds the budget) likewise.

### The friction cone → a 2D wedge

`inplane_friction_cone(n, theta, mu)`. Coulomb friction says a contact velocity does not
slip iff it lies within the cone of half-angle `β = atan(μ)` around the normal. The whole
parabola lives in one vertical plane, so only that cone's intersection with the plane
matters — itself a wedge `(γ, δ)`:

```
n_xθ = n_x·cos θ + n_y·sin θ        # normal's component along the in-plane horizontal
γ    = atan2(n_z, n_xθ)             # wedge axis, in-plane
A    = hypot(n_xθ, n_z)   ≤ 1       # length of the normal's projection into the plane
δ    = acos(cos β / A)    ≤ β
```

`cos β / A > 1` ⇒ the plane meets the cone only at its apex ⇒ **no jump at this contact**.
Concretely that is a cross-slope steeper than `μ` — terrain the robot could not stand on
without sliding.

The paper states this reduction (Sec. IV-A) but omits the `δ` formula (its footnote 1). The
derivation above is the standard cone/plane intersection and reproduces the property the
paper does state, `δ ≤ β`.

**This is the only gate in the whole planner that depends on heading.** Leg energy,
clearance and stance all see only `X` and `Z`. The cone sees the normal, and only the part
of the normal lying in the hop plane enters — which is what makes `δ` vary with θ.

### The landing cone, mapped back to α_s

`_landing_cone_alpha_s(X, Z, gamma_g, delta_g)`. The robot arrives moving *into* the
surface, so it is `−v_g` that must lie in the landing cone, i.e.
`α_g + π ∈ [γ_g − δ_g, γ_g + δ_g]`. Differentiating Eq. 2 and substituting Eq. 4:

```
tan α_g = tan α_s − 2·(X·tan α_s − Z)/X = −tan α_s + 2Z/X
  ⟹  tan α_s = 2Z/X − tan α_g            (strictly DECREASING: endpoints swap)
```

The implementation clamps the `α_g` interval to what a real descending parabola can produce,
`(−π/2, atan2(Z, X))`, then maps the endpoints. That replaces the paper's Algorithm 1 case
split on `sign(γ_g)` with a single branch — heightmap normals always have `n_z > 0`, so
`γ_g ∈ (0, π)` always and only the paper's `γ_g > 0` branch can occur. The clamp also
subsumes its `no_solution` / `undefined` flags and keeps `tan` finite.

### Surface normals from a 2.5D grid

`Map2D5.surface_normals()`. A grid stores no normals, so they are inferred by differencing
neighbouring elevations — and the estimator matters. A **central** difference is wrong at
every discontinuity: on `maps/tall_stairs.py` the cell at `x ∈ [1.9, 2.0)` (flat ground at
the foot of a 0.4 m riser) reads grade `0.4/0.2 = 2.0`, a 63° surface, giving `A = 0.447`
and `cos β / A = 1.43 > 1` — a degenerate cone at precisely the cell the planner most needs
to take off from.

So each axis takes the **smaller-magnitude of the two one-sided differences**. At the foot
of a riser, forward reads 4.0 and backward reads 0, so 0 wins: the foot rests on a flat
tread, and the riser belongs to the neighbouring cell's contact plane. On a uniform grade
both agree, so the grade is recovered exactly (`slope_crest`'s ramp reads 0.35, not an
average).

### Reference numbers (`g = 9.81`, `MU = 1.2`, `β = 50.19°`, `η = 0.7`, `min_apex = 0.3 m`)

Flat ground, at the chain's seed state (`v_g = 5.294`, so `v_s_min = 4.429 m/s`) — the
robot at the start of a plan, holding more energy than it ever holds again:

| case | pre-cone | + BEAM cone (`V_MAX = 4.852`) | + energy chain (`V_MAX` now derived) | decided by |
|---|---|---|---|---|
| flat, `X = 1.0` | `[12.31°, 77.69°]` | `[39.81°, 77.69°]` | `[75.00°, 82.76°]` | **(E1)** |
| flat, `X = 2.0` | — | `[39.81°, 61.80°]` | `[45.00°, 75.00°]` | **(E1)** |
| flat, `X = 0.8` | — | `[39.81°, 80.20°]` | `[78.16°, 84.16°]` | **(E1)** |
| stair, `X = 0.6, Z = 0.4` | `[41.70°, 81.99°]` | `[65.22°, 81.99°]` | | (2) |
| downhill, `X = 1.0, Z = −0.4` | `[−4.87°, 73.07°]` | `[39.81°, 73.07°]` | | (1) |
| grade 0.9, cross-slope | `X ≤ 1.50 m` | `X ≤ 1.50 m` (`δ` = 30.5° < `β`) | | unchanged |

The pattern in the first three rows is the energy floor: the *shorter* the hop, the more
surplus speed there is to dispose of, and the only way to dispose of it is to go steeper.
At the seed state a 0.8 m hop is forced past 78°.

That surplus decays by `(1 − η)` per hop. Once it is gone, `min_apex` takes over as the
binding floor and the chain settles into a fixed point — on flat 1.0 m hops, 50.2° with a
0.30 m drop, injecting ~1.2 J per hop to hold it. See
[`test/test_hop_energy_chain.py`](../test/test_hop_energy_chain.py) §8.

On flat ground both cones collapse to the textbook bound `α ≥ atan(1/μ)` = 39.81°, but with
the energy chain in place the cone is usually no longer the binding floor.

---

## Stage 2 — pick α within the interval

`alpha_for_clearance(profile, alpha_min, alpha_max, gate)`. Two monotonicity facts make the
choice exact rather than a search.

**Clearance is monotone nondecreasing in `tan α`** — the same `∂z/∂ tan α ≥ 0` as above, so
a steeper angle lifts the whole arc at once and never trades height at one point for height
at another. The pointwise minimum inherits it, so the clearing set is always an
upward-closed interval `[α_c, α_max]`, making `α_c` unique and bisectable.

**Required speed is U-shaped in α**, with its minimum at `min_energy_tan(X, Z) =
(Z + hypot(Z, X))/X` (the root of `X·T² − 2Z·T − X = 0`; `T = 1`, i.e. 45°, on flat ground).

So the least-injection angle over the clearing set is exactly `clamp(α*, α_c, α_max)` — no
search. In practice the clamp returns `α_c` itself, because `α*` sits below `α_min` whenever
the energy floor binds: that floor starts on the steep branch *above* the minimum-energy
angle. `α*` lands inside the interval only when the incoming speed is too low to reach the
target at all.

This replaced a **max-margin midpoint** policy (`ALPHA_MARGIN_FRAC = 0.5`, now deleted). The
accept/reject verdict is identical either way — both reject only when even `α_max` fails to
clear — but the flown angle is now the shallowest sufficient one rather than the most
comfortable one, because energy spent here is energy the *next* hop does not have.

One consequence worth noting: because both the cone and the energy floor raise `α_min`, and
clearance is monotone nondecreasing in `tan α`, both make arcs *higher*. Clearance never
gets worse from adding either.

---

## Three properties worth knowing

1. **The cone can never make a standable slope impassable.** Standing at all requires
   grade ≤ `μ`. The fall-line takeoff floor is `π/2 + atan(grade) − atan(μ)`, which stays
   below `π/2` exactly when `grade < μ`, while the leg-energy ceiling tends to `π/2` as the
   hop gets short. So arbitrarily short uphill hops always survive. The cone caps hop
   *length* by heading; it does not gate reachability.
2. **The landing-velocity gate (4) is now the cap that bounds the whole chain.** It was
   implemented for correctness, not effect, back when it read `v_g ≤ V_max` with a tuned
   `V_max`. It now reads `v_g ≤ V_G_MAX` = 7.00 m/s (a 2.5 m fall — sized for hopping off a
   platform), and since `v_s_min = √η·v_g` on the next hop, bounding `v_g` recursively
   bounds every takeoff speed in the plan. Its drop ceiling is `V_G_MAX²/(2g)` = 2.50 m,
   attained only in the limit of a straight-down hop; at `X = 1.0` the cap is already
   2.63 m under the *derived* `V_MAX`, because horizontal speed competes for the same
   budget. Constraint (3) inherited the leftover role of a never-binding backstop.
3. **Friction is now (just barely) the binding standability limit.** A cross-slope steeper
   than `μ` = 1.2 gives a degenerate cone, against `standable_mask`'s geometric ceiling of
   ~1.21. No separate stance-friction check was added: BEAM already rejects every hop into
   or out of such a cell, from both the degeneracy branch (cross-slope headings) and the
   `π/2` clip (fall-line headings, where `A = 1` so the wedge never degenerates).

---

## Remaining modelling gaps

The cone closes the largest one from `alpha_range_old.md`. Still assumed, not verified:

* **No leg kinematics** — no joint limits, stroke, torque curve, or extension time. The
  energy chain replaced the tuned `V_MAX` with a derived one, but stance is still a scalar
  damping law (`v_s = √η·v_g`), not an integrated inverted-pendulum swing. The
  full-kinetic-energy bookkeeping is the formulation that *extends* there: modelling the
  actual redirection (touchdown leg angle → takeoff leg angle) needs `v_g`'s magnitude and
  direction, and this carries the magnitude while the arc geometry supplies the direction.
  Apex-height bookkeeping would discard the direction and have to be redone.
* **η is a constant.** A real stance loss depends on touchdown speed, leg compression and
  the incidence angle. `ETA_HOP` is read in exactly two places (`_validate_and_cost` and
  the seed), so a per-contact model would slot in without touching the interval math.
* **Point-mass ballistics in flight** — no drag, no body rotation, no landing-attitude
  angular-momentum budget. Phase 1's attitude control is assumed to deliver whatever
  landing orientation constraint (2) asks for, at no energy cost.
* **μ is uniform over the environment**, as in the paper. Per-cell friction would slot in
  wherever `self.mu` is read, without touching the interval math.
* **Normals are per-cell, not per-contact-patch.** `FOOT_TIP_RADIUS` (0.02 m) is well under
  `CELL_RESOLUTION` (0.1 m), so a foot-sized plane fit would collapse to the single-cell
  case anyway — but this is what makes the min-|slope| choice a modelling decision rather
  than an approximation of something better-defined.

---

## Errata in `alpha_range_campana.md`

Both were found while implementing and are covered by cases (4) and (6) of
[`test/test_friction_cone.py`](../test/test_friction_cone.py).

**1. Sign of the landing-cone mapping.** That document writes
`alpha2 = arctan(-2*Z/X_theta - tan(alpha_g))`. The `2Z/X` term is **positive**; see the
derivation above. The two agree only when `Z = 0`, which is why the error is invisible on
flat ground. Numerically, for `X = 1, Z = 1, α_s = 80°` the flown arc lands at
`α_g = −74.76°`; `+2Z/X` gives −74.76°, `−2Z/X` gives −82.57°.

**2. The `arctan` really is required.** That document flags the paper's α₃/α₄ notation as
ambiguous and guesses that the fractions are `tan` values. They are — the quadratic
derivation above yields `T = tan α`, not α. Confirmed independently for both (3) and (4).

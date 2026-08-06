# Takeoff angle (α) selection in `hopping_astar_planner.py`

Supersedes [`alpha_range_old.md`](alpha_range_old.md), which describes the pre-friction-cone
state. The implementation is now Campana & Laumond's **BEAM** (IROS 2016, "Ballistic motion
planning", Secs. III–V) — see [`alpha_range_campana.md`](alpha_range_campana.md) for the
spec this was built from, **including two errata recorded at the bottom of this file**.

α is measured from the world horizontal, not from the local surface.

---

## Stage 1 — build the feasible interval

`feasible_alpha_interval(X, Z, V_max, g, *, mu, n_s, n_g, theta)`
([hopping_astar_planner.py:64](../hopping_astar_planner.py#L64))

Given `X` and `Z`, choosing α fixes the parabola completely (Campana Eq. 4:
`ẋ² = g·X² / (2·(X·tan α − Z))`). So every physical constraint reduces to an interval on α,
and feasibility is their intersection. There are five.

| # | constraint | interval | binds when |
|---|---|---|---|
| (i) | **Eq. 4 validity** — a projectile falls below its launch ray, so you must aim above the chord | `α ∈ (atan2(Z, X), π/2)` | steep uphill |
| (1) | **Takeoff non-sliding** — the push-off direction *is* α | `α ∈ [γ_s − δ_s, γ_s + δ_s]` | always; it is the floor on flat ground |
| (2) | **Landing non-sliding** — the *reversed* arrival direction must lie in the goal's cone | mapped through the arc, see below | uphill landings |
| (3) | **Takeoff leg energy**, `v_s ≤ V_max` | `atan` of the roots of a quadratic in `tan α` | long hops |
| (4) | **Landing leg energy**, `v_g ≤ V_max` | same quadratic with `V_max² → V_max² + 2gZ` | deep drops |

Unlike the old two-constraint version, this is **not** map-free: (1) and (2) need the
surface normal at each contact.

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

### Reference numbers (`V_MAX = 4.852 m/s`, `g = 9.81`, `MU = 1.2`, `β = 50.19°`)

| case | before (no cone) | after (BEAM) | decided by |
|---|---|---|---|
| flat, `X = 1.0` | `[12.31°, 77.69°]`, picks 45.0° | `[39.81°, 77.69°]`, picks 58.7° | (1) |
| flat, `X = 1.5` | `[19.34°, 70.66°]` | `[39.81°, 70.66°]` | (1) |
| stair, `X = 0.6, Z = 0.4` | `[41.70°, 81.99°]`, picks 61.9° | `[65.22°, 81.99°]`, picks 73.6° | **(2)** |
| downhill, `X = 1.0, Z = −0.4` | `[−4.87°, 73.07°]` | `[39.81°, 73.07°]` | (1) |
| grade 0.9, fall line | `X ≤ 1.05 m` | `X ≤ 0.55 m` | (1) |
| grade 0.9, cross-slope | `X ≤ 1.50 m` | `X ≤ 1.50 m` (`δ` = 30.5° < `β`) | unchanged |

On flat ground both cones collapse to the textbook bound `α ≥ atan(1/μ)` = 39.81°.

---

## Stage 2 — pick α within the interval

`alpha_for_clearance(...)` — **unchanged by this work.** Its monotonicity argument depends
only on `_arc_z`, not on where the interval came from, so narrowing the interval from either
end leaves it valid. Policy is still minimum sufficient effort: try the midpoint, escalate
to `α_max` only if the clearance gate demands it, then bisect for the shallowest sufficient
angle.

One consequence worth noting: because the cone raises `α_min` and clearance is monotone
nondecreasing in `tan α`, the cone makes arcs *higher*. Clearance never gets worse from
adding it.

---

## Three properties worth knowing

1. **The cone can never make a standable slope impassable.** Standing at all requires
   grade ≤ `μ`. The fall-line takeoff floor is `π/2 + atan(grade) − atan(μ)`, which stays
   below `π/2` exactly when `grade < μ`, while the leg-energy ceiling tends to `π/2` as the
   hop gets short. So arbitrarily short uphill hops always survive. The cone caps hop
   *length* by heading; it does not gate reachability.
2. **The landing-velocity gate (4) never binds on current maps.** `Λ ≥ 0` needs
   `Z ≥ −7.2 m` at `X = 1.5`. Implemented for correctness, not for effect. Its ceiling is
   `V_max²/(2g)` = 1.20 m, attained only in the limit of a straight-down hop; at `X = 1.0`
   the cap is already 0.96 m because horizontal speed competes for the same budget.
3. **Friction is now (just barely) the binding standability limit.** A cross-slope steeper
   than `μ` = 1.2 gives a degenerate cone, against `standable_mask`'s geometric ceiling of
   ~1.21. No separate stance-friction check was added: BEAM already rejects every hop into
   or out of such a cell, from both the degeneracy branch (cross-slope headings) and the
   `π/2` clip (fall-line headings, where `A = 1` so the wedge never degenerates).

---

## Remaining modelling gaps

The cone closes the largest one from `alpha_range_old.md`. Still assumed, not verified:

* **No leg kinematics** — no joint limits, stroke, torque curve, or extension time.
  `V_MAX` is still just `√(2·g·MAX_APEX_HEIGHT)`.
* **Point-mass ballistics in flight** — no drag, no body rotation, no landing-attitude
  angular-momentum budget.
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

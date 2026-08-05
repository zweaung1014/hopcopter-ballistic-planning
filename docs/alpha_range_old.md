Takeoff angle (α) selection in hopping_astar_planner.py
Repo: planning_2p5Dmap — ballistic hop planner on a 2.5D grid, physics from Campana & Laumond (2016). α is measured from the world horizontal, not from the local surface.
Stage 1 — build the feasible interval: feasible_alpha_interval(X, Z, V_max, g) (line 64)
Inputs are only: horizontal distance X, endpoint-to-endpoint elevation change Z, leg speed budget V_max, gravity g. It never sees the map.
Given X and Z, picking α fixes the parabola completely — Campana Eq. 4: ẋ² = g·X² / (2·(X·tan α − Z)). So feasibility reduces to which α give a producible speed. Two constraints:
(i) Geometric validity — denominator must be > 0, i.e. α ∈ (atan2(Z, X), π/2). Physically: a projectile falls below its launch ray, so you must aim above the chord to the target. Z < 0 makes the lower bound negative, widening the interval.
(ii) Leg energy — v_s = ẋ/cos α ≤ V_max. Since v_s² = g·X²/(X·sin2α − Z·cos2α − Z), this becomes sin(2α + ψ) ≥ K with R = √(X²+Z²), ψ = atan2(−Z, X), K = (g·X²/V_max² + Z)/R. Then: K > 1 → None (leg too weak); K < −1 → unbounded; else α ∈ [(asin K − ψ)/2, (π − asin K − ψ)/2]. Two-sided — too shallow and too steep both cost speed.
Final: α_min = max(geom_lo, vel_lo) + ε, α_max = min(π/2, vel_hi) − ε; empty ⇒ None. In practice α_max is essentially always the energy bound (v_s = V_max exactly), not π/2. Interval midpoint = minimum-speed angle (45° when Z = 0; generally (π/2 − ψ)/2).
Reference numbers (V_max = 4.852 m/s, g = 9.81, HOP_RADIUS = 1.0):
* Flat, X = 1.0: α ∈ [12.31°, 77.69°], mid 45°. Flat hops die at X = V_max²/g = 2.40 m.
* Uphill 45° grade, X = 0.9, Z = 0.9: α ∈ [60.74°, 74.26°] — only 13.5° wide; infeasible past X ≈ 0.95 m.
Stage 2 — pick α within the interval: alpha_for_clearance(...) (line 468)
Called from _validate_and_cost step (d), line 834. Policy is minimum sufficient effort — steeper α costs leg energy, so escalate only when the clearance gate demands it:
1. Evaluate α_default = α_min + margin_frac·(α_max − α_min), margin_frac = ALPHA_MARGIN_FRAC = 0.5. If clearance ≥ MIN_CLEARANCE gate → return it. (1 evaluation, common case.)
2. Else evaluate α_max. If that fails → reject the edge. (2 evaluations.) A collision at the midpoint alone never rejects a candidate.
3. Else bisect 8× on [α_default, α_max] for the shallowest clearing angle, return it. (9 evaluations.)
Why bisection is valid, not interval sampling: clearance is monotone nondecreasing in tan α — dz/d(tan α) = u(X−u)/X ≥ 0 at every point u ∈ [0, X], so a steeper α lifts the whole arc; the pointwise minimum inherits the monotonicity. Consequences: testing α_max is a complete test for "does any feasible angle clear?", angles below the midpoint are never searched (correctly — they'd only be lower), and n_bisect affects only the reported angle, never accept/reject.
Alpha-independent early rejections happen before any of this: feasible_alpha_interval returning None (line 812) and terrain_profile returning None (line 832, terrain too high for any arc).
Modeling gaps — what is assumed, not verified
The entire actuation model is the single scalar v_s ≤ V_max. Every α inside the interval is assumed equally achievable. Not modeled:
* No friction cone / no surface normal. Nothing about the local terrain gradient reaches feasible_alpha_interval. On a uniform grade the endpoint chord accidentally makes atan2(Z, X) equal the surface angle, which masks the problem — but that coincidence breaks whenever start and goal don't share a grade. Failure cases: standing on a 45° slope hopping to equal elevation gives α_min ≈ 12° (ground reaction ~33° off the normal, needs μ ≳ 0.65); hopping downhill gives negative α_min (−34.7° at X = 1.0), a thrust direction a push-only leg cannot produce.
* No leg kinematics — no joint limits, stroke, torque curve, or extension time. V_max is derived as √(2·g·MAX_APEX_HEIGHT), nothing more.
* Point-mass ballistics in flight: no drag, no body rotation, no landing-attitude angular momentum budget.
If adding a friction cone: express it as an interval on α (same shape as (i) and (ii)) and intersect at line 123. It needs the local surface normal threaded into feasible_alpha_interval, which is currently pure and map-free. Everything downstream survives unchanged — clipping from above shrinks α_max, from below raises α_min, and alpha_for_clearance's monotonicity logic holds either way.

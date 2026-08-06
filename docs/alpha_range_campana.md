> **STATUS: implemented. Two errata found — read before using this document.**
>
> This spec was the input to the friction-cone work; the shipped implementation lives in
> `hopping_astar_planner.feasible_alpha_interval` and is described in
> `docs/alpha_range_new.md`. Section 6 below correctly warned that the OCR'd formulas needed
> numeric validation. They did. Two are wrong as written:
>
> 1. **Section 3, constraint (2): the sign of the `2*Z/X_theta` term is wrong.** It is
>    `arctan(+2*Z/X_theta - tan(alpha_g))`, not `-2*Z/X_theta`. Derivation and numbers in
>    `docs/alpha_range_new.md`; the error is invisible when `Z = 0`, which is presumably how
>    it survived. Case (4) of `test/test_friction_cone.py` pins it against the flown arc.
> 2. **Section 3, constraints (3) and (4): the `arctan()` suspicion was correct.** Those
>    fractions are `tan(alpha)` values and do need `arctan`. Confirmed by deriving the
>    quadratic in `tan(alpha)` directly; case (6) of the test anchors it against the
>    superseded `asin(K)` formulation.
>
> Also note: the implementation replaces Algorithm 1's case split on `sign(gamma_g)` with a
> single branch, because heightmap normals always give `gamma_g` in `(0, pi)`.

Task: Implement friction-cone-constrained ballistic jump validity check
Implement the BEAM(cs, cg, mu, Vmax) and STEER(cs, cg, mu, Vmax, n_limit) functions from Campana & Laumond, "Ballistic Motion Planning" (IROS 2016), Sections III–V. These determine whether a parabolic jump between two contact points is admissible (non-sliding + bounded velocity) and, if so, pick a concrete takeoff angle.
1. Setup per candidate jump (cs -> cg)
* cs, cg: 3D contact points, each with an outward surface normal n_s, n_g (unit vectors).
* g: gravity magnitude (9.81).
* Horizontal direction: theta = atan2(cg.y - cs.y, cg.x - cs.x)
* Plane basis: e_xtheta = [cos(theta), sin(theta), 0], vertical axis e_z = [0,0,1]. The whole parabola lives in the vertical plane spanned by these two vectors.
* X_theta = horizontal distance = sqrt((cg.x-cs.x)^2 + (cg.y-cs.y)^2)
* Z = cg.z - cs.z
All of the following angles are the takeoff angle alpha_s measured in this plane (angle of initial velocity above e_xtheta, at cs). Everything reduces to finding a valid interval for alpha_s.
2. Friction cone -> 2D angular cone (per contact point)
Physical basis: Coulomb friction. A contact velocity vector doesn't slip iff it lies inside a 3D cone around the surface normal, with half-angle beta = atan(mu) (mu = friction coefficient, uniform over the environment in the paper).
Since the jump trajectory is confined to the plane above, only the intersection of this 3D cone with the plane matters. That intersection is itself a 2D angular wedge: center direction gamma, half-angle delta <= beta.
Note: the paper states this reduction exists (Sec. IV-A) but explicitly omits the formula for delta ("due to space limitation... does not present any particular difficulty" — footnote 1). Use this derivation (standard cone/plane intersection):
For a contact point with normal n = (nx, ny, nz):
  n_xtheta = nx*cos(theta) + ny*sin(theta)      # normal's component along e_xtheta
  n_z      = nz                                  # normal's component along e_z
  gamma = atan2(n_z, n_xtheta)                   # cone axis direction, IN-PLANE
  A = sqrt(n_xtheta^2 + n_z^2)                   # <= 1, since n has an out-of-plane part too

  beta = atan(mu)
  if cos(beta) / A > 1:
      # plane only touches the cone at the apex -> no jump possible here
      cone_degenerate = True
  else:
      delta = arccos( cos(beta) / A )            # <= beta, matches paper's stated property
      cone_degenerate = False
Compute (gamma_s, delta_s, degenerate_s) for cs and (gamma_g, delta_g, degenerate_g) for cg. If either is degenerate, return "no admissible jump" immediately (this matches Fig. 1 / Sec. IV-A: "if one of both intersection sets is reduced to a point, there is no possible jump").
3. Four constraint intervals on alpha_s
(1) Takeoff non-sliding constraint — direct:
alpha1_minus = gamma_s - delta_s
alpha1_plus  = gamma_s + delta_s
(2) Landing non-sliding constraint — this is Algorithm 1 in the paper (case split on sign of gamma_g). Implement it, but treat it as a first draft and validate numerically (see Section 5 below) since it's a compact case-based derivation:
if gamma_g > 0:
    alphag_minus = gamma_g - pi - delta_g
    alphag_plus  = gamma_g - pi + delta_g
    if alphag_plus < -pi/2:
        no_solution = True
    else:
        alpha2_minus = arctan( -2*Z/X_theta - tan(alphag_plus) )
        if alphag_minus > -pi/2:
            alpha2_plus = arctan( -2*Z/X_theta - tan(alphag_minus) )
        else:
            alpha2_plus = undefined
else:
    alphag_minus = gamma_g + pi - delta_g
    alphag_plus  = gamma_g + pi + delta_g
    if alphag_plus > pi/2:
        no_solution = True
    else:
        alpha2_plus = arctan( -2*Z/X_theta - tan(alphag_minus) )
        if alphag_plus < pi/2:
            alpha2_minus = arctan( -2*Z/X_theta - tan(alphag_plus) )
        else:
            alpha2_minus = undefined
(3) Takeoff velocity constraint (v_s <= Vmax), from quadratic in tan(alpha_s):
Delta = Vmax^4 - 2*g*Z*Vmax^2 - g^2*X_theta^2
if Delta < 0:
    no_solution = True   # goal unreachable within velocity limit
else:
    tan_a3_minus = (Vmax^2 - sqrt(Delta)) / (g*X_theta)
    tan_a3_plus  = (Vmax^2 + sqrt(Delta)) / (g*X_theta)
    alpha3_minus = arctan(tan_a3_minus)
    alpha3_plus  = arctan(tan_a3_plus)
(4) Landing velocity constraint (v_g <= Vmax), same pattern:
Lambda = Vmax^4 + 2*g*Z*Vmax^2 - g^2*X_theta^2
if Lambda < 0:
    no_solution = True
else:
    tan_a4_minus = (Vmax^2 + 2*g*Z - sqrt(Lambda)) / (g*X_theta)
    tan_a4_plus  = (Vmax^2 + 2*g*Z + sqrt(Lambda)) / (g*X_theta)
    alpha4_minus = arctan(tan_a4_minus)
    alpha4_plus  = arctan(tan_a4_plus)
Note: the paper's text writes alpha3_minus/plus and alpha4_minus/plus as equal to these fractions directly (no explicit arctan). Given the setup (alpha_s is an angle, and eq. (2)/(4) are written in terms of tan(alpha_s)), these fractions are almost certainly tan(alpha) values that still need arctan(). Implement with the arctan() and validate against equation (2) directly (see Section 5) — don't trust the paper's notation blindly here.
4. Combine into BEAM
alpha_s_minus = max(alpha1_minus, alpha2_minus, alpha3_minus, alpha4_minus)
alpha_s_plus  = min(alpha1_plus,  alpha2_plus,  alpha3_plus,  alpha4_plus)

# physical feasibility: only top-curved (gravity-consistent) parabolas, eq. (5)
lower_bound = atan2(Z, X_theta)
alpha_s_minus = max(alpha_s_minus, lower_bound)
alpha_s_plus  = min(alpha_s_plus, pi/2)

if any input marked no_solution/undefined, or alpha_s_minus >= alpha_s_plus:
    return EMPTY_INTERVAL   # no admissible jump between cs and cg
else:
    return (alpha_s_minus, alpha_s_plus)
5. STEER: pick an angle and collision-check
def STEER(cs, cg, mu, Vmax, n_limit=6):
    interval = BEAM(cs, cg, mu, Vmax)
    if interval is empty:
        return empty_path

    alpha_s = 0.5 * (interval.minus + interval.plus)   # center of interval = max margin
                                                          # from slipping / velocity limits
    path = compute_parabola(cs, cg, alpha_s)            # via eq. (1)-(4)
    n = 1
    while path.has_collisions() and n < n_limit:
        alpha_s = dichotomy_sample(interval, n)          # bisect the interval
        path = compute_parabola(cs, cg, alpha_s)
        n += 1

    return path if not path.has_collisions() else empty_path
compute_parabola should reconstruct the actual 3D trajectory using eq. (1): c(t) = -(g/2) t^2 e_z + cs_dot * t + cs, with the takeoff speed x_dot_thetas recovered from eq. (4): x_dot_thetas = sqrt(g*X_theta^2 / (2*(X_theta*tan(alpha_s) - Z))).
6. Validation checklist (important — do this)
The landing-cone algorithm and the velocity-bound arctans above were reconstructed from an OCR'd PDF and are prone to sign/transcription errors. Before trusting BEAM's output, add a numeric sanity check per candidate alpha_s:
1. Reconstruct takeoff velocity vector at cs and landing velocity vector at cg from alpha_s and eq. (2)-(4).
2. Confirm both vectors lie within delta_s / delta_g of gamma_s / gamma_g (non-sliding).
3. Confirm both vector magnitudes are <= Vmax.
4. If any check fails for an alpha_s BEAM claims is valid, the bound formulas need correcting — treat BEAM as a fast filter and this check as ground truth.
7. Parameters used in the paper's benchmarks (for reference/testing)
* n_limit = 6 (dichotomy steps in STEER)
* Test with mu in {0.5, 1.2} and Vmax in {5.3, 6.5, 6.8, 7.0} m/s — narrower cones (mu=0.5) and lower Vmax should produce more waypoints / longer paths, per Table I and Figs. 9-11 of the paper.
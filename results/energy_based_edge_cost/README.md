# Energy-based edge cost

Replaces the elevation penalty (`alpha_uphill * dz`) in
`HoppingAStarPlanner._edge_cost` with

```
cost = xy_dist + W_ENERGY * (e_inject + max(0, KE_in - KE_out))
```

`W_ENERGY = 0.84` m/J, derived rather than tuned: holding speed steady through
one hop costs `0.5 * m * v^2 * (1 - eta)` = 1.197 J at the flat steady state, so
`1 / 1.197` makes that hop's energy cost equal its distance cost.

`ALPHA_UPHILL` and `ALPHA_DOWNHILL` are deleted. `HOP_FIXED_COST` stays at 0.05.

## Why the old penalty could not work

It only saw `dz` between takeoff and landing cells. A hop that arcs **over** a
wall and lands on the flat beyond it has `dz = 0`, so it was charged nothing —
for exactly the manoeuvre it was added to price. It was also blind to how the
robot arrived, charging the same whether the hop was paid for with spare momentum
or with thrust.

`e_inject` was already computed on every edge (`hopping_astar_planner.py:1501`),
already rose when the clearance gate lifted `alpha` over terrain, and was being
thrown away.

## Why the momentum term is not optional

Charging `e_inject` alone is worse than the status quo. Short hops need no thrust
— the robot already carries the speed from its last landing — so they price as
**free**, and A* chops paths into stubs. Measured on `flat`, robot seeded at the
steady state, 3.0 m straight run:

| cost model | hops | inject | exit v_g | lengths |
|---|---|---|---|---|
| distance only (old) | 3 | 3.10 J | 3.16 m/s | `[1.0, 1.0, 1.0]` |
| + `e_inject` only | 4 | 2.45 J | 2.56 m/s | `[0.8, 1.0, 0.8, 0.4]` |
| + `e_inject` + momentum | 3 | 3.10 J | 3.16 m/s | `[1.0, 1.0, 1.0]` |

The middle row genuinely draws less from the battery. It gets there by spending
momentum instead — arriving at 2.56 m/s rather than 3.16. Summing `e_inject`
counts the battery but not the bank account. Covering 1 m:

```
one 1.0 m hop:   battery 1.20 J + momentum 0.00 J  =  1.20 J
two 0.5 m hops:  battery 0.81 J + momentum 1.23 J  =  2.04 J
```

The momentum term is also what regulates hop **count**, which is why no per-hop
energy constant was added: holding speed costs 1.197 J per hop, an expression
with no hop length in it, so N hops over the same ground cost N times as much.

## flat — before / after

`START (0.0, 0.0) → GOAL (4.5, 4.5)`, `HOP_RADIUS = 1.0`.

```
BEFORE  7 hops   9.3 s   2213 expansions   inject 3.63 J   exit v_g 2.97 m/s
  X       0.99  0.99  0.99  0.85  0.85  0.85  0.85
  E_inj   0.00  0.00  0.02  0.57  1.01  1.01  1.01

AFTER   7 hops  44.6 s   7704 expansions   inject 4.00 J   exit v_g 3.15 m/s
  X       0.99  0.85  0.57  0.99  0.99  0.99  0.99
  E_inj   0.00  0.00  0.00  1.00  1.00  1.00  1.00
```

**Total injection went UP (3.63 → 4.00 J), and that is the correct outcome.**
Injection is not the objective; injection plus squandered momentum is. The old
path was cheaper on thrust because it coasted down to 2.97 m/s; the new one holds
3.15 m/s. Using the telescoping identity (speed is non-increasing along both, so
the per-hop `max(0, ·)` terms sum to `KE_start - KE_end` on binned speeds):

```
BEFORE  3.63 J thrust + 0.4·(5.25² - 3.00²) = 3.63 + 7.43 = 11.06 J
AFTER   4.00 J thrust + 0.4·(5.25² - 3.25²) = 4.00 + 6.80 = 10.80 J
```

The steady-state hops also lengthened from 0.85 m to 0.99 m — the anti-chopping
effect. Hops 1–3 inject nothing in both: the chain starts at `H_INITIAL = 1.0` m,
well above the flat steady state, and has to burn the surplus off through stance
losses before thrust matters at all.

## low_wall — before / after

Same endpoints. A 0.4 m ridge at `x ∈ [1.7, 1.9]` spanning the full map in y, so
there is no way around it.

```
BEFORE  7 hops   6.6 s   1652 expansions   inject 4.26 J   exit v_g 2.97 m/s
  X       0.99  0.99  0.99  0.85  0.85  0.85  0.85
  alpha   74.9  68.4  65.0  54.7  54.7  54.7  54.7
  E_inj   0.00  0.00  1.13  0.10  1.01  1.01  1.01

AFTER   7 hops  40.1 s   7421 expansions   inject 3.67 J   exit v_g 2.89 m/s
  X       0.99  0.99  0.76  0.99  0.99  0.98  0.78
  alpha   74.9  68.4  70.3  50.5  50.5  50.6  56.9
  E_inj   0.00  0.00  0.76  0.53  1.00  0.98  0.39
```

Hop 3 is the wall crossing in both — note `Z = 0.00` for every hop in the plan,
which is exactly why the old `dz` penalty saw nothing here. It shows up instead
as an elevated takeoff angle (70.3° against the 50.5° cruise) and an injection
spike. The new cost found a cheaper crossing: 0.76 J against 1.13 J, by taking a
shorter run-up into the wall.

## decision_sweep.png

`test/demo_decision_sweep.py` — one geometry, four wall heights, everything else
fixed. Runs at `demo_common.HOP_RADIUS = 1.5`, not `config.HOP_RADIUS`.

```
h=0.30  OVER    3 hops  cost= 9.81 = 4.00 m travel + 0.84 × (1.03 J thrust + 5.71 J momentum)
h=0.60  OVER    3 hops  cost=10.21 = 4.00 m travel + 0.84 × (2.10 J thrust + 5.11 J momentum)
h=0.90  OVER    3 hops  cost=12.13 = 4.00 m travel + 0.84 × (3.65 J thrust + 5.85 J momentum)
h=1.20  AROUND  4 hops  cost=13.07 = 5.66 m travel + 0.84 × (3.55 J thrust + 5.04 J momentum)
```

Thrust climbs 1.03 → 2.10 → 3.65 J as the wall grows; at 1.20 m the planner pays
1.7 m of extra travel instead. **The flip moved from h ∈ [1.20, 1.40] under the
old penalty to h ∈ [0.90, 1.20].** It moved down because arcing over is now paid
for at all — and every crossing panel now arcs clear rather than landing on the
crest, which is the case the old penalty was blind to.

At `config.HOP_RADIUS = 1.0` the flip instead sits at h ∈ [0.45, 0.60]: a shorter
run-up buys less height for the same energy. The demo's `HEIGHTS` are calibrated
for 1.5.

## cost_model_ab.png / cost_model_ledger.png

`test/demo_cost_model_ab.py` — three cost models across three scenarios. Three,
not two, because the interesting claim is not "energy changes the paths" but the
sharper one: **charging thrust alone is worse than having no energy term at all**,
which a plain before/after would miss.

```
A  distance only       w_energy = 0.0
B  thrust only         w_energy = 0.84, charge_momentum = False    <- broken
C  thrust + momentum   shipped
```

Column A is `w_energy = 0`, **not** the deleted `alpha_uphill * dz` penalty — that
comparison is the flat / low_wall section above, taken by running the real
pre-change code.

Every scenario is seeded at the flat steady state (`h_initial ≈ 0.356 m`). At the
shipped `H_INITIAL = 1.0` the robot opens at 5.29 m/s, injects nothing for three
hops while stance losses burn the surplus off, and ~8 J of forced momentum loss
swamps the ledger — paid identically by every model. A development run showed no
difference between models for exactly that reason.

```
scenario   model  hops  travel  thrust  moment  total E  exit v   exps  lengths
flat       A         3   3.00    3.10    0.00     3.10    3.16     186  [1.0, 1.0, 1.0]
flat       B         4   3.00    2.45    2.35     4.80    2.56    1972  [0.8, 1.0, 0.8, 0.4]
flat       C         3   3.00    3.10    0.00     3.10    3.16     999  [1.0, 1.0, 1.0]

low_wall   A         4   3.00    3.90    3.23     7.13    2.72     259  [0.9, 1.0, 1.0, 0.1]
low_wall   B         4   3.00    3.06    2.50     5.56    2.63    1590  [0.8, 0.9, 0.8, 0.5]
low_wall   C         4   3.16    3.55    1.30     4.85    2.92    2273  [0.98, 0.7, 0.67, 0.81]

bypass     A         4   4.00    6.79    3.68    10.47    3.76     358  [1.0, 1.0, 1.0, 1.0]
bypass     B         5   4.00    6.45    4.83    11.27    2.93    5565  [1.0, 0.8, 0.6, 1.0, 0.6]
bypass     C         6   5.44    5.44    0.62     6.06    2.89    5728  [0.99, 0.99, 0.98, 0.85, 0.85, 0.78]
```

The momentum column is reported for B too, even though B does not pay it — showing
what B spent without being billed is the entire point.

**What to read off it:**

- **`flat`** — the cleanest case. A and C both give `[1.0, 1.0, 1.0]` at 3.10 J and
  exit at 3.16 m/s. B chops to four hops, burns *less* thrust (2.45 J) and arrives
  at 2.56 m/s. It did not save energy; it spent 2.35 J of momentum off the books.
- **`low_wall`** — C is cheapest overall (4.85 J vs 5.56 and 7.13) *and* arrives
  fastest (2.92 m/s). Note A's trailing `0.1` m hop: with nothing but distance in
  the cost, `HOP_FIXED_COST = 0.05` is not enough to suppress a micro-hop stub.
- **`bypass`** — the models split on routing, not just hop pattern. A and B cross a
  0.70 m wall; C walks 1.44 m further around the end and spends **6.06 J against
  B's 11.27 J**. In the ledger this is one 5–7 J spike (A, B) versus six ordinary
  ~1 J hops (C).

B burns less thrust than C while arriving slower on `flat` and `low_wall`. It does
not on `bypass`, because there C takes a completely different route rather than a
different hop pattern over the same one — the comparison is not hop-for-hop there.

The wall height (0.70 m) is calibrated, not chosen: swept at `HOP_RADIUS = 1.0`,
B and C agree at h = 0.30/0.45/0.55 and disagree at h = 0.60/0.70. Re-sweep if
`HOP_RADIUS`, `W_ENERGY`, `ETA_HOP` or `MIN_APEX_HEIGHT` change.

## Tests

`test/test_edge_cost_energy.py` — 11 assertions, all passing. The load-bearing
ones are regressions against specific rejected designs:

- one 1.0 m hop beats two 0.5 m hops (2.055 vs 2.452) — **and would not on
  injected energy alone** (0.646 J vs 1.197 J), which is the whole reason the
  momentum term exists;
- edge cost stays non-negative across drops of 0.2–0.8 m — the uncapped
  potential-shaped form `e_inject + KE_in - KE_out` reaches −1.36 J on a 0.4 m
  drop, and negative edges break A*;
- cost rises monotonically with `Z` (`-0.4: 1.05 … +0.4: 4.51`), i.e. climbing is
  priced with no elevation term present;
- a 0.4 m ridge costs 2.801 against 2.055 flat, **with `Z = 0` on both** — the
  original bug, via 50.2° → 63.2° and 1.20 → 2.08 J.

`test_clearance_rejection.py`, `test_friction_cone.py` and
`test_hop_energy_chain.py` all still pass unchanged.

## Known costs and conservatism

**Planning is 4–6× slower.** `flat` 9.3 → 44.6 s, `low_wall` 6.6 → 40.1 s;
expansions roughly 3.5–4.5×. This is structural, not a bug: `_heuristic`
estimates distance only, so the entire energy term is cost it cannot anticipate,
and A* degrades toward Dijkstra. It scales with `W_ENERGY`. The known mitigation
is a per-hop energy constant, which unlike the momentum charge can be
lower-bounded from `ceil(remaining_distance / hop_radius)` and added to the
heuristic — deliberately not done here, as it is a search optimisation rather
than a cost-model question.

**Climbs are slightly overcharged.** A climb converts kinetic energy into height
rather than wasting it, but the momentum term charges for it anyway — about
0.60 J on top of 3.72 J of real injection for a 0.4 m step, ~15%. Conservative,
not wrong.

**`HOP_FIXED_COST` is probably now dead weight** and was left at 0.05 rather than
removed. The micro-hop chains it was invented to prevent stop being free once
landings cost energy. Not yet verified across the deck.

## Rejected alternatives

Recorded so they are not re-litigated:

- **Raw obstacle height.** Over-counts ~3×. A 0.5 m wall costs 1.67 J on the
  crossing hop but 0.50 J once chained — launching harder means landing harder,
  so the next hop needs no injection (`[1.20, 2.86, 0.03, 1.20]`). Summing
  `e_inject` credits that refund automatically; a per-edge penalty cannot.
- **`m*g*h` at the apex.** Charges for height the robot got free from momentum it
  cannot shed, and counts only the vertical share of the energy.
- **Stance dissipation instead of injection.** Makes climbing *cheaper* than flat
  ground (1.02 J vs 1.20 J for a 0.4 m climb) because climbing lands slower.
- **Uncapped potential shaping.** Negative edge costs; see the test above.
- **A larger `hop_fixed_cost` as the counterweight.** Needs ~1.0 to work, and is
  fitted per scenario.
- **A separate per-hop energy constant.** Tested at 0.2 J: changed neither the
  path nor the expansion count once the momentum term was present.

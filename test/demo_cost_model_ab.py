"""Three cost models, three scenarios — what each energy term actually buys.

The edge cost is now

    xy_dist + W_ENERGY * (e_inject + max(0, KE_in - KE_out))

and the interesting claim is not "energy changes the paths" but the sharper one:
**charging thrust ALONE is worse than having no energy term at all.** A plain
before/after comparison would miss that entirely, so this demo runs three models,
not two:

    A  distance only        w_energy=0.0
    B  thrust only          w_energy=0.84, charge_momentum=False   <- broken
    C  thrust + momentum    shipped defaults

Why B breaks: a short hop needs no thrust, because the robot already carries the
speed from its last landing. On injection alone such a hop prices as FREE, so A*
chops the path into stubs. It is not saving energy, it is spending momentum — and
`Σe_inject` counts what leaves the battery but not what leaves the tank. Term C's
`max(0, KE_in - KE_out)` puts that on the books at the same price.

> **Column A is `w_energy = 0`, i.e. distance only. It is NOT the deleted
> `alpha_uphill * dz` elevation penalty**, which no longer exists in the planner.
> Measured true old-vs-new numbers, taken by running the real pre-change code,
> are in `results/energy_based_edge_cost/README.md`.

Figures produced
----------------
  cost_model_ab.png      — 3x3 top-down paths: does behaviour change?
  cost_model_ledger.png  — 3x3 per-hop energy bars: why?

The ledger is the one that shows the mechanism. Look for two things: B's bars are
mostly momentum where C's are mostly thrust (the same bill, paid two ways), and
the hop after a wall crossing is nearly free (launching hard means landing hard,
so the next hop needs no injection).

Runtime ~3-4 min. Model A is fast; B and C are not.

Run:
    python test/demo_cost_model_ab.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import config
from demo_common import (
    make_planner, draw_topdown_compact, param_caption, save, out_path,
)
from map2d5 import Map2D5


# --------------------------------------------------------------------------- #
# The three cost models
# --------------------------------------------------------------------------- #

#: (key, label, colour, planner kwargs). Order is the figure's row order.
MODELS = [
    ("A", "A — distance only\n(w_energy = 0)", "#757575",
     dict(w_energy=0.0)),
    ("B", "B — thrust only\n(no momentum charge)", "#e65100",
     dict(w_energy=config.W_ENERGY, charge_momentum=False)),
    ("C", "C — thrust + momentum\n(shipped)", "#00695c",
     dict(w_energy=config.W_ENERGY)),
]

#: `h_initial` that seeds the chain at the flat steady state (3.158 m/s), by
#: inverting `v_g_initial = sqrt(2 g h / eta)`. Applied to EVERY scenario.
#
# THIS IS NOT OPTIONAL, for two separate reasons.
#
# 1. It is what makes the effect visible at all. At the shipped H_INITIAL = 1.0 m
#    the robot starts at 5.29 m/s — far above the 3.158 m/s that flat 1 m hops
#    settle at — and cannot shed the surplus except through stance losses. For
#    the first three hops it therefore injects NOTHING, all three models price
#    them identically, and the chopping effect disappears. An A/B run during
#    development showed no difference between the models for exactly this reason.
#
# 2. It stops the burn-down swamping the ledger. Shedding that opening surplus
#    costs ~8 J of momentum, which every model pays identically because it is
#    forced physics, not a choice. Run with the default seed, the low_wall and
#    bypass panels are ~80% one orange block of startup transient with the
#    discretionary difference buried inside it. Seeding at the steady state
#    leaves only the energy each model CHOSE to spend, which is the whole point.
H_STEADY = 3.158 ** 2 * config.ETA_HOP / (2.0 * config.G_ACCEL)

# Bypassable-wall geometry, shared with test/demo_decision_sweep.py.
WALL_XMIN, WALL_XMAX = 2.3, 2.7
WALL_YMIN, WALL_YMAX = 0.8, 4.2

#: CALIBRATED, not guessed — the height at which models B and C disagree about
#: over-vs-around at HOP_RADIUS = 1.0. Swept before being fixed here:
#:   h=0.30/0.45/0.55  all three cross
#:   h=0.60/0.70       B crosses, C detours   <- the separating band
#: 0.70 sits comfortably inside it rather than on the 0.55/0.60 boundary. Re-sweep
#: if HOP_RADIUS, W_ENERGY, ETA_HOP or MIN_APEX_HEIGHT change.
WALL_HEIGHT = 0.70


def flat_map() -> Map2D5:
    return Map2D5(size_x=config.MAP_SIZE_X, size_y=config.MAP_SIZE_Y,
                  resolution=config.CELL_RESOLUTION)


def bypassable_wall() -> Map2D5:
    """A ridge with open ground at both ends, so detouring is an option.

    `paint_region` in world metres — a `world_to_grid` slice would rescale the
    physical wall with `CELL_RESOLUTION` and move the calibrated flip point.
    """
    m = flat_map()
    m.paint_region(WALL_HEIGHT,
                   x_min=WALL_XMIN, x_max=WALL_XMAX,
                   y_min=WALL_YMIN, y_max=WALL_YMAX)
    return m


def low_wall_map() -> Map2D5:
    """`maps/low_wall.py` — 0.4 m ridge spanning the full map in y, no bypass."""
    import maps.low_wall as lw
    return lw.build()


#: (key, title, map builder, start, goal). All are seeded at H_STEADY — see above.
SCENARIOS = [
    ("flat", "flat ground\n(isolates hop chopping)",
     flat_map, (0.5, 2.5), (3.5, 2.5)),
    ("low_wall", "low_wall — 0.4 m ridge, no bypass\n(every hop has dz = 0)",
     low_wall_map, (0.5, 2.5), (3.5, 2.5)),
    ("bypass", f"bypassable wall, h = {WALL_HEIGHT} m\n(over vs around)",
     bypassable_wall, (0.5, 2.4), (4.5, 2.4)),
]


# --------------------------------------------------------------------------- #
# Running and measuring
# --------------------------------------------------------------------------- #

def momentum_charge(planner, hop: dict) -> float:
    """`max(0, KE_in - KE_out)` for one hop — the term C charges and B does not.

    Recomputed here rather than read off the edge cost, because **model B needs
    this reported even though it does not pay it**: showing what B spent without
    being billed is the entire point of the comparison.

    Mirrors `_edge_cost` exactly, including the `_speed_bin` rounding on the
    outgoing speed, so the two reconcile. `v_g_in` is already binned.
    """
    v_out = planner._speed_bin(hop["v_g"]) * planner.speed_bin
    return max(0.0, 0.5 * planner.mass * (hop["v_g_in"] ** 2 - v_out ** 2))


def run(scenario, model) -> dict:
    """Plan one (scenario, model) pair and measure it."""
    _, _, build, start, goal = scenario
    _, _, _, kw = model

    m = build()
    p = make_planner(m, False, start, goal, h_initial=H_STEADY, **kw)
    t0 = time.time()
    path = p.plan()
    dt = time.time() - t0

    if path is None:
        return dict(map=m, path=None, dt=dt, expansions=p.n_expansions)

    hops = p.path_hops
    return dict(
        map=m, path=path, planner=p, hops=hops, dt=dt,
        expansions=p.n_expansions,
        n=len(hops),
        lengths=[h["X"] for h in hops],
        travel=sum(h["X"] for h in hops),
        thrust=[h["e_inject"] for h in hops],
        momentum=[momentum_charge(p, h) for h in hops],
        exit_v=hops[-1]["v_g"],
    )


def crossed(scenario, r) -> str:
    """OVER / AROUND for the bypassable-wall scenario; '' for the others."""
    if scenario[0] != "bypass" or r["path"] is None:
        return ""
    ys = [q[1] for q in r["path"]]
    return "AROUND" if (min(ys) < WALL_YMIN or max(ys) > WALL_YMAX) else "OVER"


# --------------------------------------------------------------------------- #
# Figures
# --------------------------------------------------------------------------- #

def figure_paths(results) -> None:
    """3x3 top-down grid: rows = cost model, cols = scenario."""
    nr, nc = len(MODELS), len(SCENARIOS)
    fig, axes = plt.subplots(nr, nc, figsize=(4.8 * nc, 4.9 * nr), squeeze=False)

    for ri, (mkey, mlabel, colour, _) in enumerate(MODELS):
        for ci, scen in enumerate(SCENARIOS):
            ax = axes[ri][ci]
            r = results[(scen[0], mkey)]
            draw_topdown_compact(r["map"], ax, colorbar_on=None)

            if r["path"] is None:
                ax.set_title("NO PATH", color="#b71c1c", fontsize=9,
                             fontweight="bold")
                continue

            xs = [q[0] for q in r["path"]]
            ys = [q[1] for q in r["path"]]
            ax.plot(xs, ys, color=colour, linewidth=2.4, zorder=8)
            ax.plot(xs, ys, "o", color=colour, markersize=6, zorder=9)
            ax.plot(*scen[3], "go", markersize=10, zorder=10)
            ax.plot(*scen[4], "r*", markersize=14, zorder=10)

            tag = crossed(scen, r)
            ax.set_title(
                (f"{tag}  ·  " if tag else "")
                + f"{r['n']} hops, {r['travel']:.2f} m\n"
                f"thrust {sum(r['thrust']):.2f} J · "
                f"momentum {sum(r['momentum']):.2f} J\n"
                f"exit {r['exit_v']:.2f} m/s",
                color=colour, fontsize=9, fontweight="bold",
            )
            if ci == 0:
                ax.set_ylabel(mlabel + "\n\nY (m)", fontsize=9,
                              fontweight="bold", color=colour)

    for ci, scen in enumerate(SCENARIOS):
        axes[0][ci].text(0.5, 1.28, scen[1], transform=axes[0][ci].transAxes,
                         ha="center", va="bottom", fontsize=10,
                         fontweight="bold")

    fig.suptitle(
        "Three cost models, same scenarios — rows differ ONLY in the edge cost\n"
        "B charges thrust but not momentum: it looks cheaper and arrives slower\n"
        "(A is w_energy=0, i.e. distance only — NOT the deleted elevation penalty)",
        fontsize=12, wrap=True,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    save(fig, out_path("cost_model_ab.png"))
    plt.close(fig)


def figure_ledger(results) -> None:
    """3x3 per-hop energy bars: thrust stacked under momentum.

    This is the panel that shows the mechanism rather than the outcome. The two
    things to read off it: B's bars are mostly momentum where C's are mostly
    thrust (one bill, two ways of paying it), and the hop after a wall crossing
    is nearly free, because launching hard means landing hard.
    """
    nr, nc = len(MODELS), len(SCENARIOS)
    fig, axes = plt.subplots(nr, nc, figsize=(4.8 * nc, 3.5 * nr),
                             squeeze=False, sharey="col")

    for ri, (mkey, mlabel, colour, _) in enumerate(MODELS):
        for ci, scen in enumerate(SCENARIOS):
            ax = axes[ri][ci]
            r = results[(scen[0], mkey)]
            if r["path"] is None:
                ax.text(0.5, 0.5, "NO PATH", ha="center", va="center",
                        transform=ax.transAxes, color="#b71c1c")
                continue

            idx = range(1, r["n"] + 1)
            ax.bar(idx, r["thrust"], color="#1565c0", label="thrust (e_inject)")
            ax.bar(idx, r["momentum"], bottom=r["thrust"], color="#ef6c00",
                   label="momentum thrown away")
            ax.set_xticks(list(idx))
            ax.tick_params(labelsize=8)
            ax.set_title(
                f"total {sum(r['thrust']) + sum(r['momentum']):.2f} J"
                + ("" if mkey != "B" else "   (momentum NOT charged)"),
                fontsize=9, color=colour, fontweight="bold",
            )
            if ci == 0:
                ax.set_ylabel(mlabel + "\n\nJoules", fontsize=9,
                              fontweight="bold", color=colour)
            if ri == nr - 1:
                ax.set_xlabel("hop", fontsize=9)

    for ci, scen in enumerate(SCENARIOS):
        axes[0][ci].text(0.5, 1.32, scen[1], transform=axes[0][ci].transAxes,
                         ha="center", va="bottom", fontsize=10,
                         fontweight="bold")

    axes[0][0].legend(fontsize=7, loc="upper right", framealpha=0.9)
    fig.suptitle(
        "Per-hop energy ledger — the same bill, paid two different ways\n"
        "B's bars are momentum where C's are thrust; B is simply not billed for its column",
        fontsize=12, wrap=True,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    save(fig, out_path("cost_model_ledger.png"))
    plt.close(fig)


# --------------------------------------------------------------------------- #

def main() -> int:
    print(param_caption())
    print(f"hop_radius=dynamic (per-state) · every scenario seeded at "
          f"h_initial={H_STEADY:.3f} m (the flat steady state, 3.158 m/s)\n")

    results = {}
    for scen in SCENARIOS:
        for model in MODELS:
            print(f"  planning {scen[0]:<9} model {model[0]} ...",
                  end="", flush=True)
            r = run(scen, model)
            results[(scen[0], model[0])] = r
            print(f" {r['dt']:5.1f}s", flush=True)
    print()

    # ---------------- summary table ----------------
    hdr = (f"{'scenario':<10}{'model':<7}{'hops':>5}{'travel':>8}{'thrust':>9}"
           f"{'moment':>9}{'total E':>9}{'exit v':>8}{'exps':>7}{'time':>7}   lengths")
    print(hdr)
    print("-" * len(hdr))
    for scen in SCENARIOS:
        for model in MODELS:
            r = results[(scen[0], model[0])]
            if r["path"] is None:
                print(f"{scen[0]:<10}{model[0]:<7}  NO PATH")
                continue
            th, mo = sum(r["thrust"]), sum(r["momentum"])
            print(f"{scen[0]:<10}{model[0]:<7}{r['n']:>5}{r['travel']:>8.2f}"
                  f"{th:>9.2f}{mo:>9.2f}{th + mo:>9.2f}{r['exit_v']:>8.2f}"
                  f"{r['expansions']:>7}{r['dt']:>6.0f}s   "
                  f"{[round(x, 2) for x in r['lengths']]}")
        print()

    figure_paths(results)
    figure_ledger(results)

    # ---------------- did the demo demonstrate anything? ----------------
    differs = [
        s[0] for s in SCENARIOS
        if results[(s[0], "B")]["path"] is not None
        and results[(s[0], "C")]["path"] is not None
        and results[(s[0], "B")]["lengths"] != results[(s[0], "C")]["lengths"]
    ]
    if not differs:
        print("WARNING: models B and C produced identical paths on every "
              "scenario — the demo is not demonstrating anything. Re-calibrate "
              "WALL_HEIGHT, or check that charge_momentum is being threaded.")
        return 2
    print(f"B and C differ on: {', '.join(differs)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

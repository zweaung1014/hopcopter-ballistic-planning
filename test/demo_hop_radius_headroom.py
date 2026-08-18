"""Is `HOP_RADIUS` limiting hop length, or is the physics?

`HOP_RADIUS` only sets the outer edge of the ring `_generate_hop_neighbors`
ray-searches when generating candidates -- it never touches the feasibility
or clearance gates in `_validate_and_cost`. This script settles, with two
independent tests, whether the hop lengths a real plan chooses are actually
capped by that ring or by the physics gates that would reject a longer hop
regardless of the ring's size.

Test 1 -- ray-extension headroom probe (per hop, ground truth)
    For every hop in a baseline plan, scan landing points further out along
    the SAME heading, past the hop actually taken, calling the planner's own
    `_validate_and_cost` (not a reimplementation) at each one. The farthest
    point still accepted is `physics_max_X`; comparing it to `hop_radius`
    is a direct measurement of how much reach the ring is discarding.

Test 2 -- radius sweep, replan (systemic, catches multi-hop chain effects
    Test 1 can't see, since extending one hop changes the arrival speed --
    and hence the feasible interval -- of every hop after it)
    Replans the same start/goal at several `HOP_RADIUS` values (all verified
    safe under config.py's own asserts -- see the conversation this script
    came out of; the real ceiling sits between 4.0 m and 4.2 m, well short
    of the naive V_MAX^2/g = 5.5 m figure) and reports how the resulting
    hop-length distribution moves.

Run:
    python test/demo_hop_radius_headroom.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import config
from demo_common import (
    ANNOT_FS, C_ACCEPT, C_BALL, C_CHOSEN, LABEL_FS, PRESENTATION_DPI,
    TITLE_FS, TOPDOWN_FIGSIZE, draw_topdown_compact, make_planner, out_path,
    planner_alpha_interval, save,
)
from hopping_astar_planner import (
    alpha_for_clearance, feasible_alpha_interval, min_energy_tan, terrain_profile,
)
from map2d5 import Map2D5
from maps import flat, stairs


# --------------------------------------------------------------------------- #
# Test 1 -- ray-extension headroom probe
# --------------------------------------------------------------------------- #

PROBE_STEP = 0.05  # m; finer than HOP_SCAN_STEP for a tight headroom estimate
PROBE_MAX = 5.0    # m; upper bound for the *probe* only -- _validate_and_cost
                   # never reads hop_radius, so this can exceed config.py's
                   # HOP_RADIUS assert ceiling (4.0-4.2 m) without issue. The
                   # 5x5 m map is what actually stops most probes first.


def classify_gate(planner, m, p0, p1, v_g_in) -> str:
    """Which gate rejects the hop p0 -> p1.

    Mirrors `demo_common.enumerate_ring_candidates`'s per-candidate checks
    (obstacle -> stance -> feasibility interval -> clearance), just aimed at
    an arbitrary point along a swept heading instead of the 16-direction ring.
    """
    nb = m.world_to_grid(*p1)
    nz = float(m.grid[nb[0], nb[1]])
    if nz == Map2D5.OBSTACLE:
        return "obstacle"
    if not planner._standable[nb[0], nb[1]]:
        return "stance"

    z0 = float(m.get_elevation(*p0))
    X = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    Z = nz - z0
    iv = planner_alpha_interval(planner, m, p0, p1, X, Z, v_g_in)
    if iv is None:
        return "physics"

    profile = terrain_profile(
        (p0[0], p0[1], z0), (p1[0], p1[1], nz), m,
        planner.robot_radius,
        planner.leg_length, planner.arc_max_step, planner._obstacle_fill,
        planner.n_lateral, min_clearance_gate=planner.min_clearance_gate,
    )
    if profile is None:
        return "clearance(profile)"
    _, mc = alpha_for_clearance(profile, iv[0], iv[1], planner.min_clearance_gate)
    if mc < planner.min_clearance_gate:
        return "clearance"
    return "accepted?"  # shouldn't be reached if _validate_and_cost also failed


def headroom_probe(planner, m, path) -> list[dict]:
    """For each hop in `path`, find the farthest still-feasible landing along
    the SAME heading, using the planner's own `_validate_and_cost`."""
    rows = []
    hop_radius = planner.hop_radius
    for i, hop in enumerate(planner.path_hops):
        p0, p1 = path[i], path[i + 1]
        X = hop["X"]
        v_g_in = hop["v_g_in"]
        current = m.world_to_grid(*p0)
        current_z = float(m.grid[current[0], current[1]])
        dx = (p1[0] - p0[0]) / X
        dy = (p1[1] - p0[1]) / X

        farthest = X
        stopping_gate = ""
        seen = {current}
        r = PROBE_STEP * math.ceil(X / PROBE_STEP)  # first grid point >= X
        hit_edge = False
        while r <= PROBE_MAX:
            tx, ty = p0[0] + r * dx, p0[1] + r * dy
            if not m.is_within_bounds(tx, ty):
                hit_edge = True
                break
            nb = m.world_to_grid(tx, ty)
            if nb not in seen:
                seen.add(nb)
                edge = planner._validate_and_cost(current, current_z, nb, v_g_in)
                if edge is not None:
                    farthest = r
                    stopping_gate = ""
                elif stopping_gate == "":
                    stopping_gate = classify_gate(
                        planner, m, p0, (tx, ty), v_g_in,
                    )
            r += PROBE_STEP

        if stopping_gate == "":
            stopping_gate = "map edge" if hit_edge else "probe cap"

        rows.append({
            "hop": i, "X_taken": X, "hop_radius": hop_radius,
            "physics_max_X": farthest, "headroom": farthest - hop_radius,
            "gate": stopping_gate,
        })
    return rows


def print_headroom_report(rows: list[dict], label: str) -> None:
    print(f"\n--- Test 1: ray-extension headroom probe -- {label} ---")
    print(f"{'hop':>3} {'X_taken':>8} {'hop_radius':>10} {'physics_max_X':>14} "
          f"{'headroom':>9}  stopping gate")
    for row in rows:
        print(f"{row['hop']:>3} {row['X_taken']:>8.3f} {row['hop_radius']:>10.3f} "
              f"{row['physics_max_X']:>14.3f} {row['headroom']:>9.3f}  {row['gate']}")
    max_headroom = max(r["headroom"] for r in rows)
    mean_headroom = sum(r["headroom"] for r in rows) / len(rows)
    if max_headroom < PROBE_STEP:
        verdict = "PHYSICS-BOUND: no hop had meaningful headroom past hop_radius."
    else:
        verdict = (
            f"RING-BOUND: at least one hop had {max_headroom:.2f} m of unused "
            f"physics headroom past hop_radius (mean {mean_headroom:.2f} m) -- "
            f"HOP_RADIUS is truncating candidates the physics would still accept."
        )
    print(f"Verdict: {verdict}")


# --------------------------------------------------------------------------- #
# Test 2 -- radius sweep, replan
# --------------------------------------------------------------------------- #

SWEEP_RADII = [1.0, 1.5, 2.0, 3.0, 4.0]


def sweep_hop_radius(map_builder, start, goal, label: str) -> list[dict]:
    rows = []
    for r in SWEEP_RADII:
        m = map_builder.build()
        planner = make_planner(m, False, start, goal, hop_radius=r)
        path = planner.plan()
        if path is None:
            rows.append({"hop_radius": r, "ok": False})
            continue
        xs = [h["X"] for h in planner.path_hops]
        saturated = sum(1 for x in xs if x >= r - config.HOP_SCAN_STEP)
        rows.append({
            "hop_radius": r, "ok": True, "n_hops": len(xs),
            "X_min": min(xs), "X_mean": sum(xs) / len(xs), "X_max": max(xs),
            "n_saturated": saturated,
        })

    print(f"\n--- Test 2: radius sweep -- {label} ---")
    print(f"{'hop_radius':>10} {'n_hops':>7} {'X_min':>7} {'X_mean':>7} "
          f"{'X_max':>7}  saturated")
    for row in rows:
        if not row["ok"]:
            print(f"{row['hop_radius']:>10.3f}   plan() returned None")
            continue
        print(f"{row['hop_radius']:>10.3f} {row['n_hops']:>7d} "
              f"{row['X_min']:>7.3f} {row['X_mean']:>7.3f} {row['X_max']:>7.3f}  "
              f"{row['n_saturated']}/{row['n_hops']}")

    ok_rows = [r for r in rows if r["ok"]]
    if len(ok_rows) >= 2:
        x_mean_first, x_mean_last = ok_rows[0]["X_mean"], ok_rows[-1]["X_mean"]
        n_first, n_last = ok_rows[0]["n_hops"], ok_rows[-1]["n_hops"]
        moved = abs(x_mean_last - x_mean_first) > 0.1 or n_first != n_last
        if moved:
            verdict = (
                f"RING-BOUND: mean hop length moved {x_mean_first:.2f} -> "
                f"{x_mean_last:.2f} m (hop count {n_first} -> {n_last}) as "
                f"hop_radius grew -- the smaller radius was constraining the plan."
            )
        else:
            verdict = (
                f"PHYSICS-BOUND: mean hop length stayed ~{x_mean_first:.2f} m "
                f"and hop count stayed {n_first} across the whole radius sweep "
                f"-- physics/cost, not hop_radius, is choosing the hop length."
            )
        print(f"Verdict: {verdict}")
    return rows


# --------------------------------------------------------------------------- #
# Test 3 -- why ~2 m: split-cost decomposition
# --------------------------------------------------------------------------- #
#
# `_edge_cost` = xy_dist + w_energy*(e_inject + momentum_lost).
# Hold the hop COUNT fixed at 2 (what A* already found optimal at
# HOP_RADIUS=4.0) and sweep only the intermediate landing point of a
# start -> mid -> goal path. `xy_dist` sums to the same total for every split --
# both TIE identically across the whole sweep. So whichever split minimises
# TOTAL cost is, by construction, whichever split minimises TOTAL ENERGY
# (e_inject + momentum_lost) -- and that number comes directly out of
# `_validate_and_cost`'s least-injection angle choice at each hop. If the
# sweep's minimum lands where A* actually put the intermediate hop, that is a
# direct proof the ~2 m spacing is the angle/energy choice at work, not a
# side effect of xy-distance ties or hop-count tie-breaking (both of which
# are already ruled out analytically, since they're constant here).

SPLIT_STEP = 0.05  # m


def binding_names(trace: list) -> tuple[str, str]:
    """Which constraint owns each end of the final alpha interval.

    Copied from `test/demo_takeoff_angle_range.py::binding_names` (not
    imported, to keep this script independent of another demo's top-level
    state): the last constraint to move a bound owns it, since every
    constraint is a running max/min and a later equal bound never displaces
    an earlier one.
    """
    lo_name = hi_name = "eq4"
    for rec in trace:
        if rec["cut_lo"] > 0.0:
            lo_name = rec["name"]
        if rec["cut_hi"] > 0.0:
            hi_name = rec["name"]
    return lo_name, hi_name


def explain_hop(planner, m, p0, p1, v_g_in, alpha_s: float) -> dict:
    """Where the FLOWN angle `alpha_s` actually sits, not just which
    constraint owns each end of the interval.

    `alpha_for_clearance` picks `clamp(alpha_star, alpha_c, alpha_max)` where
    `alpha_star` is the unconstrained minimum-energy angle (`min_energy_tan`).
    Reporting only "which constraint sets the interval floor" is misleading
    when the flown angle sits well above that floor, at the free minimum-
    energy point -- this classifies which of the three actually happened.
    """
    z0 = float(m.get_elevation(*p0))
    z1 = float(m.get_elevation(*p1))
    X = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    Z = z1 - z0
    normals = m.surface_normals()
    r0, c0 = m.world_to_grid(*p0)
    r1, c1 = m.world_to_grid(*p1)
    v_s_min = math.sqrt(planner.eta) * v_g_in
    trace: list = []
    iv = feasible_alpha_interval(
        X, Z, planner.V_max, planner.g,
        mu=planner.mu,
        n_s=tuple(normals[r0, c0]), n_g=tuple(normals[r1, c1]),
        theta=math.atan2(p1[1] - p0[1], p1[0] - p0[0]),
        v_s_min=v_s_min, e_inject_max=planner.e_inject_max,
        mass=planner.mass, min_apex=planner.min_apex, V_g_max=planner.V_g_max,
        trace=trace,
    )
    alpha_star = math.atan(min_energy_tan(X, Z))
    if iv is None:
        where = "infeasible"
    else:
        lo_name, hi_name = binding_names(trace)
        alpha_min, alpha_max = iv
        eps = 1e-3
        if abs(alpha_s - alpha_min) < eps:
            where = f"AT the interval floor -- set by [{lo_name}]"
        elif abs(alpha_s - alpha_max) < eps:
            where = f"AT the interval ceiling -- set by [{hi_name}]"
        elif abs(alpha_s - alpha_star) < math.radians(0.5):
            where = "FREE at the unconstrained min-energy angle (no bound is binding)"
        else:
            side = "above" if alpha_s > alpha_star else "below"
            where = (
                f"interior, {side} the min-energy angle by "
                f"{abs(math.degrees(alpha_s - alpha_star)):.1f} deg -- neither "
                f"the interval bound nor the free min-energy point; the "
                f"clearance gate (`alpha_for_clearance`) picked this one off "
                f"the terrain profile, between floor {math.degrees(alpha_min):.1f} "
                f"and ceiling {math.degrees(alpha_max):.1f} deg"
            )
    return {"X": X, "Z": Z, "v_s_min": v_s_min, "interval": iv,
            "alpha_star_deg": math.degrees(alpha_star), "where": where}


def split_cost_sweep(planner, m, start, goal) -> list[dict]:
    """Sweep the intermediate landing point of a fixed-count 2-hop path."""
    total_X = math.hypot(goal[0] - start[0], goal[1] - start[1])
    dx = (goal[0] - start[0]) / total_X
    dy = (goal[1] - start[1]) / total_X
    start_cell = m.world_to_grid(*start)
    start_z = float(m.grid[start_cell[0], start_cell[1]])
    goal_cell = m.world_to_grid(*goal)
    v_g_initial = planner.v_g_initial

    rows = []
    x1 = SPLIT_STEP
    while x1 <= total_X - SPLIT_STEP:
        mid = (start[0] + x1 * dx, start[1] + x1 * dy)
        mid_cell = m.world_to_grid(*mid)
        mid_z = float(m.grid[mid_cell[0], mid_cell[1]])

        edge1 = planner._validate_and_cost(start_cell, start_z, mid_cell, v_g_initial)
        if edge1 is not None:
            cost1, hop1 = edge1
            # BINNED, not exact: the real A* graph transitions through
            # (cell, speed_bin) states, so the second hop's search neighbors
            # are generated from `speed_bin * SPEED_BIN`, not the exact `v_g`
            # hop 1 produced. Chaining the exact value here would score a
            # split A* could never actually have reached this way -- the
            # same pitfall CLAUDE.md flags for `diagnose_path`.
            v_g_mid_binned = planner._speed_bin(hop1["v_g"]) * planner.speed_bin
            edge2 = planner._validate_and_cost(mid_cell, mid_z, goal_cell, v_g_mid_binned)
            if edge2 is not None:
                cost2, hop2 = edge2
                rows.append({
                    "X1": x1, "X2": total_X - x1, "feasible": True,
                    "total_cost": cost1 + cost2,
                    "e_inject1": hop1["e_inject"], "e_inject2": hop2["e_inject"],
                    "v_g_mid": v_g_mid_binned, "alpha1": hop1["alpha_s"],
                    "alpha2": hop2["alpha_s"],
                })
                x1 += SPLIT_STEP
                continue
        rows.append({"X1": x1, "X2": total_X - x1, "feasible": False})
        x1 += SPLIT_STEP
    return rows


def print_split_cost_report(
    planner, m, start, goal, rows: list[dict], actual_X1: float, label: str,
) -> None:
    print(f"\n--- Test 3: split-cost decomposition -- {label} ---")
    ok = [r for r in rows if r["feasible"]]
    best = min(ok, key=lambda r: r["total_cost"])
    print(f"xy_dist ties across every split here (fixed "
          f"2-hop count, fixed total distance) -- so this minimum IS the "
          f"minimum of e_inject + momentum_lost alone.")
    print(f"Sweep minimum:  X1={best['X1']:.2f} m  (X2={best['X2']:.2f} m)  "
          f"total_cost={best['total_cost']:.3f}")
    print(f"A* actually chose:  X1={actual_X1:.2f} m")
    if abs(best["X1"] - actual_X1) <= SPLIT_STEP * 1.5:
        print("Verdict: MATCH -- A*'s split sits at the sweep's cost minimum.")
    else:
        print("Verdict: MISMATCH -- A*'s split does NOT sit at this sweep's "
              "minimum; something else is also in play.")

    # Snap every point through world_to_grid -> grid_to_world before
    # re-deriving the interval: `_validate_and_cost` always computes X/Z/theta
    # off the CELL CENTER (`grid_to_world(cell)`), not the raw continuous
    # point, and START/GOAL (0.5, 2.5) are themselves not cell centers at
    # 0.1 m resolution (centers sit at 0.05, 0.15, ...). Recomputing off the
    # unsnapped point silently re-derives a slightly different interval than
    # the one that actually gated this hop.
    def _snap(p):
        return m.grid_to_world(*m.world_to_grid(*p))

    total_X = math.hypot(goal[0] - start[0], goal[1] - start[1])
    dx = (goal[0] - start[0]) / total_X
    dy = (goal[1] - start[1]) / total_X
    mid = (start[0] + best["X1"] * dx, start[1] + best["X1"] * dy)
    start_s, mid_s, goal_s = _snap(start), _snap(mid), _snap(goal)
    e1 = explain_hop(planner, m, start_s, mid_s, planner.v_g_initial, best["alpha1"])
    e2 = explain_hop(planner, m, mid_s, goal_s, best["v_g_mid"], best["alpha2"])
    print(f"  hop 1 (X={e1['X']:.2f} m): alpha_s={math.degrees(best['alpha1']):.1f} deg "
          f"(min-energy angle={e1['alpha_star_deg']:.1f} deg) -- {e1['where']}, "
          f"e_inject={best['e_inject1']:.3f} J")
    print(f"  hop 2 (X={e2['X']:.2f} m): alpha_s={math.degrees(best['alpha2']):.1f} deg "
          f"(min-energy angle={e2['alpha_star_deg']:.1f} deg) -- {e2['where']}, "
          f"e_inject={best['e_inject2']:.3f} J")


# --------------------------------------------------------------------------- #
# Figures
# --------------------------------------------------------------------------- #

def _plot_path(ax, m, path, color) -> None:
    draw_topdown_compact(m, ax)
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    ax.plot(xs, ys, color=color, linewidth=2.2, zorder=8)
    ax.plot(xs, ys, "o", color=color, markersize=7, zorder=9)
    ax.plot(xs[0], ys[0], "s", color="black", markersize=9, zorder=10)
    ax.plot(xs[-1], ys[-1], "*", color="black", markersize=15, zorder=10)


def plot_topdown_comparison(
    scenario_data: list[tuple[str, "Map2D5", list, list]],
    r_before: float, r_after: float,
) -> None:
    """Before/after top-down paths: HOP_RADIUS=`r_before` vs `r_after`.

    `scenario_data` is `[(label, map, path_before, path_after), ...]`.
    """
    n = len(scenario_data)
    fig, axes = plt.subplots(n, 2, figsize=(TOPDOWN_FIGSIZE[0] * 1.9,
                                            TOPDOWN_FIGSIZE[1] * n * 0.55))
    if n == 1:
        axes = axes.reshape(1, 2)
    for row, (label, m, path_before, path_after) in enumerate(scenario_data):
        _plot_path(axes[row, 0], m, path_before, C_BALL)
        axes[row, 0].set_title(
            f"{label}: hop_radius={r_before} m "
            f"({len(path_before) - 1} hops, mean X="
            f"{(sum(math.hypot(path_before[i+1][0]-path_before[i][0], path_before[i+1][1]-path_before[i][1]) for i in range(len(path_before)-1)) / (len(path_before)-1)):.2f} m)",
            fontsize=LABEL_FS,
        )
        _plot_path(axes[row, 1], m, path_after, C_CHOSEN)
        axes[row, 1].set_title(
            f"{label}: hop_radius={r_after} m "
            f"({len(path_after) - 1} hops, mean X="
            f"{(sum(math.hypot(path_after[i+1][0]-path_after[i][0], path_after[i+1][1]-path_after[i][1]) for i in range(len(path_after)-1)) / (len(path_after)-1)):.2f} m)",
            fontsize=LABEL_FS,
        )
    fig.suptitle(
        "Same start/goal, only HOP_RADIUS changes -- fewer, longer hops once "
        "the ring stops truncating the physics",
        fontsize=TITLE_FS,
    )
    fig.text(0.5, 0.01, "■ start    ● hop landing    ★ goal",
             ha="center", fontsize=ANNOT_FS)
    fig.tight_layout(rect=(0, 0.03, 1, 0.94))
    save(fig, out_path("hop_radius_topdown.png"))
    plt.close(fig)


def plot_headroom_bars(headroom_by_scenario: dict[str, list[dict]]) -> None:
    """Per-hop X_taken vs physics_max_X, at the current HOP_RADIUS."""
    labels = list(headroom_by_scenario.keys())
    fig, axes = plt.subplots(1, len(labels), figsize=(6.5 * len(labels), 5.0))
    if len(labels) == 1:
        axes = [axes]
    for ax, label in zip(axes, labels):
        rows = headroom_by_scenario[label]
        idx = [r["hop"] for r in rows]
        width = 0.35
        ax.bar([i - width / 2 for i in idx], [r["X_taken"] for r in rows],
               width, color=C_BALL, label="X taken (actual hop)")
        ax.bar([i + width / 2 for i in idx], [r["physics_max_X"] for r in rows],
               width, color=C_ACCEPT, alpha=0.85,
               label="physics_max_X (farthest feasible)")
        ax.axhline(rows[0]["hop_radius"], color="black", linestyle="--",
                   linewidth=1.3, label=f"hop_radius = {rows[0]['hop_radius']} m")
        ax.set_ylim(0, rows[0]["hop_radius"] * 1.18)
        for i, r in enumerate(rows):
            ax.annotate(r["gate"], (i + width / 2, r["physics_max_X"]),
                       textcoords="offset points", xytext=(0, 6),
                       ha="center", fontsize=ANNOT_FS - 3, color="#555")
        ax.set_xticks(idx)
        ax.set_xlabel("hop index")
        ax.set_ylabel("distance (m)")
        ax.set_title(label, fontsize=LABEL_FS)
        ax.legend(fontsize=ANNOT_FS - 2, loc="lower right")
    fig.suptitle(
        f"Test 1: per-hop headroom past hop_radius (labels = stopping gate)",
        fontsize=TITLE_FS,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    save(fig, out_path("hop_radius_headroom_bars.png"))
    plt.close(fig)


def plot_split_cost(
    split_data: dict[str, tuple[list[dict], float]],
) -> None:
    """Total 2-hop cost vs. intermediate-landing distance X1.

    `split_data` is `{label: (rows, actual_X1)}`.
    """
    fig, axes = plt.subplots(1, len(split_data), figsize=(6.5 * len(split_data), 5.0))
    if len(split_data) == 1:
        axes = [axes]
    for ax, (label, (rows, actual_X1)) in zip(axes, split_data.items()):
        ok = [r for r in rows if r["feasible"]]
        xs = [r["X1"] for r in ok]
        ys = [r["total_cost"] for r in ok]
        best = min(ok, key=lambda r: r["total_cost"])
        ax.plot(xs, ys, "-", color=C_BALL, linewidth=2.0)
        ax.axvline(best["X1"], color=C_ACCEPT, linestyle="-", linewidth=2.0,
                   label=f"sweep minimum  X1={best['X1']:.2f} m")
        ax.axvline(actual_X1, color="black", linestyle="--", linewidth=1.5,
                   label=f"A*'s actual split  X1={actual_X1:.2f} m")
        ax.set_xlabel("X1 -- distance of the first hop (m)")
        ax.set_ylabel("total cost (both hops)")
        ax.set_title(label, fontsize=LABEL_FS)
        ax.legend(fontsize=ANNOT_FS - 1)
    fig.suptitle(
        "Test 3: total cost vs. where the split lands\n"
        "xy_dist ties here, so this curve IS the energy cost",
        fontsize=LABEL_FS - 1,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.84))
    save(fig, out_path("hop_radius_split_cost.png"))
    plt.close(fig)


def plot_sweep(sweep_by_scenario: dict[str, list[dict]]) -> None:
    """hop_radius vs mean hop length and hop count, one line per scenario."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.0, 5.0))
    colors = {"flat": C_BALL, "stairs": C_CHOSEN}
    for label, rows in sweep_by_scenario.items():
        ok = [r for r in rows if r["ok"]]
        radii = [r["hop_radius"] for r in ok]
        color = colors.get(label, None)
        ax1.plot(radii, [r["X_mean"] for r in ok], "o-", color=color, label=label)
        ax2.plot(radii, [r["n_hops"] for r in ok], "o-", color=color, label=label)
    ax1.set_xlabel("HOP_RADIUS (m)")
    ax1.set_ylabel("mean hop length X (m)")
    ax1.set_title("Mean hop length vs. ring size", fontsize=LABEL_FS)
    ax1.legend(fontsize=ANNOT_FS)
    ax2.set_xlabel("HOP_RADIUS (m)")
    ax2.set_ylabel("hop count")
    ax2.set_title("Hop count vs. ring size", fontsize=LABEL_FS)
    ax2.legend(fontsize=ANNOT_FS)
    fig.suptitle(
        "Test 2: radius sweep -- the plateau is where\n"
        "physics/cost, not the ring, is choosing the hop",
        fontsize=LABEL_FS,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.85))
    save(fig, out_path("hop_radius_sweep.png"))
    plt.close(fig)


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #

def main() -> None:
    scenarios = [
        ("flat", flat, (0.5, 2.5), (4.5, 2.5)),
        ("stairs", stairs, (0.5, 2.5), (4.5, 2.5)),
    ]
    r_before = 1.0  # the pre-change default, for the topdown before/after

    topdown_data = []
    headroom_by_scenario: dict[str, list[dict]] = {}
    sweep_by_scenario: dict[str, list[dict]] = {}
    split_data: dict[str, tuple[list[dict], float]] = {}

    for label, map_builder, start, goal in scenarios:
        print(f"\n{'=' * 70}\nScenario: {label}\n{'=' * 70}")

        m = map_builder.build()
        planner = make_planner(m, False, start, goal, hop_radius=config.HOP_RADIUS)
        path = planner.plan()
        if path is None:
            print(f"plan() returned None at HOP_RADIUS={config.HOP_RADIUS} -- "
                  f"skipping Test 1/3 for this scenario.")
        else:
            rows = headroom_probe(planner, m, path)
            print_headroom_report(rows, label)
            headroom_by_scenario[label] = rows

            m_before = map_builder.build()
            planner_before = make_planner(m_before, False, start, goal,
                                          hop_radius=r_before)
            path_before = planner_before.plan()
            if path_before is not None:
                topdown_data.append((label, m, path_before, path))

            if len(path) == 3:  # exactly 2 hops, the case Test 3 explains
                actual_X1 = planner.path_hops[0]["X"]
                split_rows = split_cost_sweep(planner, m, start, goal)
                print_split_cost_report(planner, m, start, goal, split_rows,
                                        actual_X1, label)
                split_data[label] = (split_rows, actual_X1)
            else:
                print(f"\n--- Test 3 skipped for {label}: A* used "
                      f"{len(path) - 1} hops, not 2 ---")

        sweep_by_scenario[label] = sweep_hop_radius(map_builder, start, goal, label)

    print(f"\n{'=' * 70}\nWriting figures\n{'=' * 70}")
    if topdown_data:
        plot_topdown_comparison(topdown_data, r_before, config.HOP_RADIUS)
    if headroom_by_scenario:
        plot_headroom_bars(headroom_by_scenario)
    if split_data:
        plot_split_cost(split_data)
    plot_sweep(sweep_by_scenario)


if __name__ == "__main__":
    main()

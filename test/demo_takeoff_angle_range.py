"""Instrument the takeoff-angle interval on flat, empty ground.

`feasible_alpha_interval` intersects eight constraints and returns only the
surviving `(alpha_min, alpha_max)`. That makes it impossible to tell from the
outside WHICH constraint set each end, or which takeoff angles each one
cancelled — so it is impossible to check that the interval is being built
correctly rather than merely producing plausible-looking numbers.

It plans a 2.5 m goal in +x and replays each hop through the interval with
`trace=` enabled, recording every constraint's own bound and what it actually
cut. Two maps are available; switch by flipping `SCENARIO` and the two map lines
at the top of `main()`.

  * `maps/flat.py`     — no obstacles, every bound has a hand-checkable closed
                         form. The control: this is where the interval math is
                         verified rather than merely observed.
  * `maps/low_wall.py` — a 0.4 m wall across the middle (the shipped default).
                         Flat ground never makes the clearance gate lift the
                         arc, so the flown angle sits on `alpha_min` and
                         injection stays at 0.00 J the whole way; the wall is
                         what forces the planner off the floor and switches
                         injection on.

Outputs (into `results/energy_aware_planning/`, or `$PLANNER_OUT_DIR`):

  * `takeoff_angle_range.csv`             one row per jump, every constraint
  * `takeoff_angle_range_topdown.png`     top view of the path
  * `takeoff_angle_range_sideview.png`    per hop: the arc family, and a bar
                                          chart of how the interval was built

Two things to expect ON THE FLAT MAP, both correct and both surprising the first
time (self-check group 4 asserts them there, and downgrades to informational
prints on any other map, where the expected answer is not hand-derivable):

1. **The hop split is 1.00 / 0.90 / 0.60 m, not 1.0 / 1.0 / 0.5.** Edge cost is
   XY distance, and the total XY distance is 2.5 m for *any*
   3-hop split — so every equal-count split ties exactly and A* tie-breaks on
   heap order.

2. **Only the energy floor (E1) ever binds here.** Injection is 0.00 J on all
   three hops and the drops (0.92 / 0.64 / 0.45 m) stay clear of `min_apex`
   = 0.30 m: the robot is coasting down its initial `H_INITIAL` surplus the
   whole way. `E3_min_apex` still reports its own bound in the CSV — it is just
   never the binding one. Watch `E3_min_apex_own_lo` climb toward
   `E1_energy_floor_own_lo` as the surplus decays; at a longer goal distance
   they cross and min-apex takes over.

Run:
    python test/demo_takeoff_angle_range.py
"""

import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import config
from demo_common import (
    C_ACCEPT, C_ANNOT, C_BALL, C_REJECT,
    ANNOT_FS, TOPDOWN_FIGSIZE, arcs_figsize, draw_topdown_compact,
    make_planner, out_path, param_caption, save,
)
from hopping_astar_planner import (
    _ALPHA_EPS,
    _TRACE_STEPS,
    _arc_z,
    feasible_alpha_interval,
    injection_energy,
    landing_speed,
    takeoff_speed,
)
from maps import flat, low_wall
from visualizer import Visualizer


# Start and goal: 2.5 m apart in +x, on the map's mid-line so the hop circles
# stay well clear of the edges (the ray-search discards out-of-bounds
# candidates, which would quietly bias the split near a boundary).
START = (0.5, 2.5)
GOAL = (3.0, 2.5)

G = config.G_ACCEL

#: Which map `main()` builds. Only `"flat"` has a hand-derivable expected
#: answer, so the scenario-shape self-checks (group 4) run as PASS/FAIL there
#: and as informational prints everywhere else — see `main()`.
SCENARIO = "low_wall"
_SCENARIO_LABEL = {
    "flat":     "flat ground",
    "low_wall": f"a {low_wall.WALL_HEIGHT:.1f} m wall at "
                f"x ∈ [{low_wall.WALL_XMIN}, {low_wall.WALL_XMAX}]",
}


# --------------------------------------------------------------------------- #
# Replay
# --------------------------------------------------------------------------- #

def replay_hop(planner, m, p0, p1, v_g_in) -> dict:
    """Rebuild one hop's takeoff-angle interval, recording every constraint.

    `v_g_in` must be the speed the PLANNER used, i.e. the quantised state speed
    from `path_hops[i]["v_g_in"]` — not the exact landing speed of the previous
    hop. Feeding the exact value drifts by up to `speed_bin/2` and silently
    reproduces a different interval than the one the planner gated on.
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
        n_s=tuple(normals[r0, c0]),
        n_g=tuple(normals[r1, c1]),
        theta=math.atan2(p1[1] - p0[1], p1[0] - p0[0]),
        v_s_min=v_s_min,
        e_inject_max=planner.e_inject_max,
        mass=planner.mass,
        min_apex=planner.min_apex,
        V_g_max=planner.V_g_max,
        trace=trace,
    )
    return {
        "X": X, "Z": Z, "z0": z0, "z1": z1,
        # Kept so the side view can sample the terrain the hop actually flies
        # over, rather than drawing a straight chord between the endpoints.
        "p0": p0, "p1": p1,
        "v_g_in": v_g_in,
        "v_s_min": v_s_min,
        # What the energy band alone allows, before V_max clips it. Reported
        # separately so a hop limited by the global backstop rather than by its
        # own parent is visible in the CSV.
        "v_s_max_energy": math.sqrt(
            v_s_min * v_s_min + 2.0 * planner.e_inject_max / planner.mass),
        "v_s_max_effective": math.sqrt(min(
            v_s_min * v_s_min + 2.0 * planner.e_inject_max / planner.mass,
            planner.V_max ** 2)),
        "interval": iv,
        "trace": trace,
    }


def binding_names(trace: list) -> tuple[str, str]:
    """Which constraint set each end of the final interval.

    The last one to move a bound owns it: every constraint is a max/min against
    the running value, so a later equal bound never displaces an earlier one.
    """
    lo_name = hi_name = "eq4"
    for rec in trace:
        if rec["cut_lo"] > 0.0:
            lo_name = rec["name"]
        if rec["cut_hi"] > 0.0:
            hi_name = rec["name"]
    return lo_name, hi_name


# --------------------------------------------------------------------------- #
# CSV
# --------------------------------------------------------------------------- #

def write_csv(rows: list[dict], path: str) -> None:
    """One row per jump. Angles in degrees, speeds m/s, energies J.

    Column groups, in order:
      hop / geometry / incoming energy state
      per constraint (in the order the planner applies them):
        <c>_own_lo, <c>_own_hi   the constraint's OWN interval
        <c>_cut_below,           how much it removed from each end (0 = it did
        <c>_cut_above            not bind on this hop)
        <c>_run_lo, <c>_run_hi   the running intersection AFTER applying it
      outcome: final interval, which constraint set each end, chosen angle
      cancelled totals, and the resulting flight physics
    """
    field_names = [
        "hop", "X", "Z", "v_g_in", "v_s_min", "v_s_max_energy", "v_s_max_effective",
    ]
    for key, _label in _TRACE_STEPS:
        field_names += [f"{key}_own_lo", f"{key}_own_hi",
                        f"{key}_cut_below", f"{key}_cut_above",
                        f"{key}_run_lo", f"{key}_run_hi"]
    field_names += [
        "alpha_min", "alpha_max", "binding_lo", "binding_hi", "alpha_chosen",
        "cancelled_below", "cancelled_above",
        "v_s", "v_g", "e_inject", "apex_drop",
    ]

    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=field_names)
        w.writeheader()
        for r in rows:
            w.writerow({k: ("" if v is None else v) for k, v in r.items()})
    print(f"Saved: {path}")


def csv_row(hop_i: int, rep: dict, chosen: dict) -> dict:
    """Flatten one replayed hop into the wide per-jump CSV row."""
    deg = lambda v: None if v is None else round(math.degrees(v), 4)
    alpha_min, alpha_max = rep["interval"]
    by_name = {rec["name"]: rec for rec in rep["trace"]}
    lo_name, hi_name = binding_names(rep["trace"])

    row = {
        "hop": hop_i + 1,
        "X": round(rep["X"], 4),
        "Z": round(rep["Z"], 4),
        "v_g_in": round(rep["v_g_in"], 4),
        "v_s_min": round(rep["v_s_min"], 4),
        "v_s_max_energy": round(rep["v_s_max_energy"], 4),
        "v_s_max_effective": round(rep["v_s_max_effective"], 4),
    }
    for key, _label in _TRACE_STEPS:
        rec = by_name.get(key)
        row[f"{key}_own_lo"] = deg(rec["own_lo"]) if rec else None
        row[f"{key}_own_hi"] = deg(rec["own_hi"]) if rec else None
        row[f"{key}_cut_below"] = deg(rec["cut_lo"]) if rec else None
        row[f"{key}_cut_above"] = deg(rec["cut_hi"]) if rec else None
        row[f"{key}_run_lo"] = deg(rec["run_lo"]) if rec else None
        row[f"{key}_run_hi"] = deg(rec["run_hi"]) if rec else None

    row.update({
        "alpha_min": deg(alpha_min),
        "alpha_max": deg(alpha_max),
        "binding_lo": lo_name,
        "binding_hi": hi_name,
        "alpha_chosen": deg(chosen["alpha_s"]),
        # Everything the gates threw away, as two spans: from straight-ahead up
        # to the floor, and from the ceiling up to vertical.
        "cancelled_below": deg(alpha_min),
        "cancelled_above": round(90.0 - math.degrees(alpha_max), 4),
        "v_s": round(chosen["v_s"], 4),
        "v_g": round(chosen["v_g"], 4),
        "e_inject": round(chosen["e_inject"], 4),
        "apex_drop": round(chosen["apex_drop"], 4),
    })
    return row


# --------------------------------------------------------------------------- #
# Figures
# --------------------------------------------------------------------------- #

def fig_topdown(m, path, planner) -> str:
    # TOPDOWN_FIGSIZE, not the 16:9 slide canvas: the map is square and drawn
    # with equal aspect, so a wide canvas is half dead margin.
    fig, ax = plt.subplots(figsize=TOPDOWN_FIGSIZE)
    vis = Visualizer.__new__(Visualizer)
    vis.map_env, vis.fig, vis.ax = m, fig, ax
    ax.set_xlim(0, config.MAP_SIZE_X)
    ax.set_ylim(0, config.MAP_SIZE_Y)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    # `draw_topdown_compact` rather than `Visualizer.draw_map`: the latter writes
    # the elevation into all 2500 cells, which on an all-zero map is 2500 copies
    # of "0.00" and no colourbar worth having.
    draw_topdown_compact(m, ax)
    vis.draw_hop_circles(path, planner.hop_radius)
    vis.draw_path(path)
    vis.draw_start_goal(START, GOAL)

    for i, h in enumerate(planner.path_hops):
        x0, y0 = path[i]
        x1, y1 = path[i + 1]
        ax.annotate(
            f"hop {i+1}\nX={h['X']:.2f} m\nα={math.degrees(h['alpha_s']):.1f}°",
            xy=(0.5 * (x0 + x1), y0), xytext=(0.5 * (x0 + x1), y0 - 0.55),
            ha="center", va="top", fontsize=9, color=C_ANNOT,
            arrowprops=dict(arrowstyle="-", color=C_ANNOT, lw=0.8),
        )

    dy = path[0][1] - path[-1][1]
    ax.set_title(
        f"{_SCENARIO_LABEL[SCENARIO].capitalize()}, "
        f"goal {GOAL[0] - START[0]:.1f} m in +x — "
        f"{len(planner.path_hops)} hops (Δy = {dy:+.3f} m)\n"
        + param_caption(planner), fontsize=10,
    )
    p = out_path("takeoff_angle_range_topdown.png")
    save(fig, p)
    plt.close(fig)
    return p


def _draw_arc(ax, X, z_s, Z, alpha, **kw):
    us = np.linspace(0.0, X, 160)
    ax.plot(us, _arc_z(us, X, Z, z_s, alpha), **kw)


#: Short row labels for the constraint chart. The full sentences live in
#: `_TRACE_STEPS`; nine of them stacked in a half-height panel collide at any
#: legible size, and they repeat identically across columns, so the rows are
#: keyed short and named once in the figure caption instead.
_SHORT_LABEL = {
    "eq4":               "Eq. 4",
    "E1_energy_floor":   "E1 floor",
    "E2_inject_ceiling": "E2 ceiling",
    "E3_min_apex":       "E3 min-apex",
    "c3_takeoff_speed":  "(3) v_s",
    "c4_landing_speed":  "(4) v_g",
    "c1_takeoff_cone":   "(1) cone_s",
    "c2_landing_cone":   "(2) cone_g",
}


def _terrain_along(m, p0, p1, us, X):
    """Ground elevation under a hop, sampled at the same `u` grid as the arcs.

    A straight chord between the two endpoint elevations is what this used to
    draw, and on flat ground the two agree exactly — but anything the hop flies
    OVER (the whole point of the wall scenario) is invisible in a chord. Uses
    the bilinear lookup so the drawn ground matches what the clearance gate
    sampled, not the nearest-cell staircase.
    """
    t = us / X if X > 0.0 else np.zeros_like(us)
    xs = p0[0] + t * (p1[0] - p0[0])
    ys = p0[1] + t * (p1[1] - p0[1])
    return np.asarray(m.sample_bilinear(xs, ys), dtype=float)


def fig_sideview(reps, planner, m) -> str:
    """Per hop: the surviving arc family (top), how it was built (bottom)."""
    n = len(reps)
    fig, axes = plt.subplots(
        2, n, figsize=arcs_figsize(n),
        gridspec_kw={"height_ratios": [1.0, 1.15]},
        constrained_layout=True,
    )
    if n == 1:
        axes = axes.reshape(2, 1)

    lo_plot, hi_plot = -5.0, 95.0

    for i, (rep, chosen) in enumerate(reps):
        X, Z = rep["X"], rep["Z"]
        alpha_min, alpha_max = rep["interval"]
        z_s = rep["z0"] + planner.leg_length

        # --- row 1: the arc family ---------------------------------------- #
        ax = axes[0, i]
        us = np.linspace(0.0, X, 160)
        z_lo = _arc_z(us, X, Z, z_s, alpha_min)
        z_hi = _arc_z(us, X, Z, z_s, alpha_max)
        z_terrain = _terrain_along(m, rep["p0"], rep["p1"], us, X)

        # Everything below alpha_min was cancelled: those arcs all lie under the
        # shallowest SURVIVING one, so the band between terrain and z_lo is
        # exactly the set of trajectories the gates removed.
        ax.fill_between(us, z_terrain, z_lo, color=C_REJECT, alpha=0.10,
                        label="cancelled (below α_min)")
        ax.fill_between(us, z_lo, z_hi, color=C_ACCEPT, alpha=0.22,
                        label="surviving α range")
        _draw_arc(ax, X, z_s, Z, alpha_max, color=C_ANNOT, lw=1.5, ls="--",
                  label="α_max")
        _draw_arc(ax, X, z_s, Z, chosen["alpha_s"], color=C_BALL, lw=3.0,
                  label="flown")
        # α_min drawn LAST, dotted, on top of the flown arc: the two coincide
        # whenever the energy floor is what the planner flew (which is every hop
        # here), and stacking them in this order is what makes that visible
        # rather than hiding one line completely under the other.
        _draw_arc(ax, X, z_s, Z, alpha_min, color=C_REJECT, lw=1.6, ls=":",
                  zorder=5, label="α_min")
        ax.fill_between(us, z_terrain.min() - 0.1, z_terrain,
                        color="#8d6e63", alpha=0.35, zorder=1)
        ax.plot(us, z_terrain, color="#8d6e63", lw=2.0, zorder=2)

        ax.set_title(
            f"hop {i+1}   X = {X:.2f} m\n"
            f"v_g_in = {rep['v_g_in']:.2f} m/s\n"
            f"α ∈ [{math.degrees(alpha_min):.1f}°, {math.degrees(alpha_max):.1f}°]"
            f" → flew {math.degrees(chosen['alpha_s']):.1f}°"
        )
        if i == 0:
            ax.set_ylabel("CoM height (m)")
        ax.grid(alpha=0.25)

        # --- row 2: how the interval was built ---------------------------- #
        ax = axes[1, i]
        for k, (key, _label) in enumerate(_TRACE_STEPS):
            rec = next((r for r in rep["trace"] if r["name"] == key), None)
            y = len(_TRACE_STEPS) - k
            if rec is None:
                continue
            own_lo = math.degrees(rec["own_lo"]) if rec["own_lo"] is not None else lo_plot
            own_hi = math.degrees(rec["own_hi"]) if rec["own_hi"] is not None else hi_plot
            lo_d, hi_d = max(own_lo, lo_plot), min(own_hi, hi_plot)
            binds = rec["cut_lo"] > 1e-12 or rec["cut_hi"] > 1e-12
            ax.barh(y, hi_d - lo_d, left=lo_d, height=0.55,
                    color=C_ACCEPT if binds else "#b0bec5",
                    alpha=0.80 if binds else 0.40, zorder=3)
            # Hatch what this constraint actually removed — the cancelled
            # parabolas, attributed to whichever gate is responsible for them.
            if rec["cut_lo"] > 1e-12:
                prev = math.degrees(rec["run_lo"] - rec["cut_lo"])
                ax.barh(y, math.degrees(rec["cut_lo"]), left=prev, height=0.55,
                        color=C_REJECT, alpha=0.35, hatch="///", zorder=4)
            if rec["cut_hi"] > 1e-12:
                ax.barh(y, math.degrees(rec["cut_hi"]),
                        left=math.degrees(rec["run_hi"]), height=0.55,
                        color=C_REJECT, alpha=0.35, hatch="///", zorder=4)

        alo, ahi = math.degrees(alpha_min), math.degrees(alpha_max)
        ax.barh(0, max(ahi - alo, 0.8), left=alo, height=0.55, color=C_BALL,
                alpha=0.95, zorder=5)
        ax.axvline(math.degrees(chosen["alpha_s"]), color=C_BALL, lw=1.6,
                   ls=":", zorder=6)

        ax.set_yticks(list(range(len(_TRACE_STEPS) + 1)))
        if i == 0:
            # Labelled once: the rows are identical across columns, and the
            # left margin is the only place they fit at a legible size.
            ax.set_yticklabels(
                ["FINAL"] + [_SHORT_LABEL[k] for k, _ in reversed(_TRACE_STEPS)],
                fontsize=ANNOT_FS - 3,
            )
        else:
            ax.set_yticklabels([])
        ax.set_xlim(lo_plot, hi_plot)
        ax.set_ylim(-0.7, len(_TRACE_STEPS) + 0.7)
        ax.grid(axis="x", alpha=0.25)
        lo_name, hi_name = binding_names(rep["trace"])
        ax.set_title(f"floor: {_SHORT_LABEL[lo_name]}   "
                     f"ceiling: {_SHORT_LABEL[hi_name]}")

    # Row labels go on the outer axes rather than one per panel: three copies of
    # a full-width axis label do not fit side by side and run into each other.
    # `supxlabel` would fight the outside legend below for the same strip.
    for i in range(n):
        axes[0, i].set_xlabel("u along hop (m)")
        axes[1, i].set_xlabel("takeoff angle α (deg)")

    handles, labels = axes[0, 0].get_legend_handles_labels()
    # "outside" placement so constrained_layout RESERVES the strip instead of
    # overlaying it on the axis labels.
    fig.legend(handles, labels, loc="outside lower center", ncol=5,
               frameon=False, fontsize=ANNOT_FS - 2)
    fig.suptitle(
        "How each hop's takeoff-angle interval was built\n"
        "green = this constraint binds · grey = slack · hatched = the angles it cancelled"
    )
    p = out_path("takeoff_angle_range_sideview.png")
    save(fig, p)
    plt.close(fig)
    return p


# --------------------------------------------------------------------------- #

def main() -> int:
    # Swap these two lines (and `SCENARIO` above) to go back to flat ground.
    # m = flat.build()
    m = low_wall.build()
    planner = make_planner(m, False, START, GOAL, hop_radius=config.HOP_RADIUS)
    path = planner.plan()
    if path is None:
        print(f"FAIL: no path across {_SCENARIO_LABEL[SCENARIO]}.")
        return 1

    print(f"== takeoff-angle interval on {_SCENARIO_LABEL[SCENARIO]} ==")
    print(f"   {param_caption(planner)}")
    print(f"   start={START} goal={GOAL}  ->  {len(planner.path_hops)} hops\n")

    reps, rows = [], []
    all_ok = True
    for i, hop in enumerate(planner.path_hops):
        rep = replay_hop(planner, m, path[i], path[i + 1], hop["v_g_in"])
        if rep["interval"] is None:
            print(f"FAIL: hop {i+1} replayed as infeasible but the planner flew it.")
            return 1
        reps.append((rep, hop))
        rows.append(csv_row(i, rep, hop))

        lo_name, hi_name = binding_names(rep["trace"])
        print(f"hop {i+1}: X={rep['X']:.2f} v_g_in={rep['v_g_in']:.2f} -> "
              f"[{math.degrees(rep['interval'][0]):.2f}, "
              f"{math.degrees(rep['interval'][1]):.2f}]°  "
              f"floor={lo_name} ceiling={hi_name}  "
              f"flown={math.degrees(hop['alpha_s']):.2f}°")

    # -- self-checks ---------------------------------------------------- #
    print("\n(1) trace reproduces the returned interval")
    # If this fails, a bound was changed without updating its trace record —
    # which is the whole risk of recording inline rather than through a helper.
    worst = 0.0
    for rep, _ in reps:
        last = rep["trace"][-1]
        worst = max(worst,
                    abs(last["run_lo"] + _ALPHA_EPS - rep["interval"][0]),
                    abs(last["run_hi"] - _ALPHA_EPS - rep["interval"][1]))
    ok = worst < 1e-12
    all_ok &= ok
    print(f"  [{'PASS' if ok else 'FAIL'}] final running interval == return value  "
          f"(worst {worst:.2e} rad)")

    print("\n(2) cut accounting closes")
    # Every radian removed is attributed to exactly one constraint.
    ok = True
    for rep, _ in reps:
        tr = rep["trace"]
        eq4 = tr[0]
        sum_lo = sum(r["cut_lo"] for r in tr)
        sum_hi = sum(r["cut_hi"] for r in tr)
        ok &= abs(sum_lo - (tr[-1]["run_lo"] - eq4["run_lo"])) < 1e-12
        ok &= abs(sum_hi - (eq4["run_hi"] - tr[-1]["run_hi"])) < 1e-12
    all_ok &= ok
    print(f"  [{'PASS' if ok else 'FAIL'}] sum of per-constraint cuts == total narrowing")

    print("\n(3) replay matches what the planner flew")
    # Guards the v_g_in quantisation: chaining the exact landing speed instead
    # of the binned state speed drifts enough to move the chosen angle.
    ok, worst = True, 0.0
    for rep, hop in reps:
        alpha_min, alpha_max = rep["interval"]
        ok &= alpha_min - 1e-9 <= hop["alpha_s"] <= alpha_max + 1e-9
        v_s = takeoff_speed(rep["X"], rep["Z"], hop["alpha_s"], G)
        worst = max(worst, abs(v_s - hop["v_s"]),
                    abs(landing_speed(v_s, rep["Z"], G) - hop["v_g"]),
                    abs(injection_energy(v_s, rep["v_s_min"], planner.mass)
                        - hop["e_inject"]))
    all_ok &= ok and worst < 1e-6
    print(f"  [{'PASS' if ok and worst < 1e-6 else 'FAIL'}] flown angle lies in the "
          f"replayed interval, speeds agree (worst {worst:.2e})")

    # Checks (1)-(3) hold on ANY map: they say the trace faithfully describes
    # whatever the planner did. This last group is different — it asserts a
    # specific ANSWER, and the only map whose answer is derivable by hand is the
    # flat one. On a terrain scenario the same quantities are still worth
    # printing (they are how you read the run), but as observations, not as a
    # verdict, so they do not touch the exit code.
    strict = SCENARIO == "flat"
    tag = (lambda ok: f"[{'PASS' if ok else 'FAIL'}]") if strict else (lambda ok: "[info]")
    print(f"\n(4) the scenario is the one we think it is"
          f"{'' if strict else '   (informational — not flat ground)'}")

    ys = [p[1] for p in path]
    straight = max(ys) - min(ys) < 1e-9
    all_ok &= straight or not strict
    print(f"  {tag(straight)} path is straight "
          f"(Δy = {max(ys) - min(ys):.3e} m)")

    floors = [binding_names(rep["trace"])[0] for rep, _ in reps]
    ok = all(f == "E1_energy_floor" for f in floors)
    all_ok &= ok or not strict
    print(f"  {tag(ok)} every hop's floor is the energy floor  "
          f"({', '.join(floors)})")

    # min-apex is present and slack on every hop — the fallback semantics.
    slack = [math.degrees(next(r["own_lo"] for r in rep["trace"]
                               if r["name"] == "E3_min_apex"))
             for rep, _ in reps]
    ok = all(s < math.degrees(rep["interval"][0]) for s, (rep, _) in zip(slack, reps))
    all_ok &= ok or not strict
    print(f"  {tag(ok)} min-apex bound is slack, not binding  "
          f"({', '.join(f'{s:.1f}°' for s in slack)} vs floor "
          f"{', '.join(f'{math.degrees(r[0]['interval'][0]):.1f}°' for r in reps)})")

    # The wall's payoff, and the reason the scenario exists: on flat ground the
    # clearance gate never has to lift the arc, so the flown angle sits exactly
    # on alpha_min and injection stays at 0.00 J on every hop.
    lifted = [i + 1 for i, (rep, hop) in enumerate(reps)
              if hop["alpha_s"] > rep["interval"][0] + 1e-9]
    injecting = [i + 1 for i, (_, hop) in enumerate(reps) if hop["e_inject"] > 1e-9]
    print(f"  [info] hops flown above α_min: {lifted or 'none'}   "
          f"hops injecting energy: {injecting or 'none'}")

    # -- artefacts ------------------------------------------------------- #
    print()
    write_csv(rows, out_path("takeoff_angle_range.csv"))
    fig_topdown(m, path, planner)
    fig_sideview(reps, planner, m)

    print("\n" + ("ALL PASSED" if all_ok else "SOME FAILED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

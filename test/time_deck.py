"""Timing harness for the demo-deck scenarios — before/after comparison.

Runs `plan()` on every scenario in the deck for both the ballistic planner and the
`disable_clearance=True` baseline, and records wall-clock time alongside the planner's
own instrumentation counters (`n_expansions`, `n_edge_checks`, `n_edges_accepted`).

This script is deliberately written against only the *default* interface of
`demo_common.make_planner(m, disable_clearance, start, goal)` and the free functions
`feasible_alpha_interval` / `_xdot`, so that the SAME file runs unmodified against both
the pre- and post-change planner.  That is the whole point: capture a baseline at HEAD
before touching anything, then re-run afterwards and diff the two tables.

    python test/time_deck.py                        # table to stdout
    python test/time_deck.py results/x/timings.md   # ...and write it there
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
import demo_common
from demo_common import make_planner, diagnose_path
from hopping_astar_planner import _xdot
from maps import (
    tall_wall, tall_narrow_wall, barely_jumpable_wall,
    tall_stairs, stairs_with_curb, slope_crest, stairs,
)


# (label, map builder, start, goal) — mirrors test/demo_overview.py's SCENARIOS,
# plus `stairs`, which has a map module but no demo of its own.
SCENARIOS = [
    ("tall_wall",            tall_wall.build,            (0.5, 2.4), (4.5, 2.4)),
    ("tall_narrow_wall",     tall_narrow_wall.build,     (0.5, 2.4), (4.5, 2.4)),
    ("barely_jumpable_wall", barely_jumpable_wall.build, (0.5, 2.4), (4.5, 2.4)),
    ("tall_stairs",          tall_stairs.build,          (0.5, 2.5), (4.0, 2.5)),
    ("stairs_with_curb",     stairs_with_curb.build,     (0.5, 2.5), (4.5, 2.5)),
    ("slope_crest",          slope_crest.build,          (0.5, 2.5), (4.5, 2.5)),
    ("stairs",               stairs.build,               (0.5, 2.4), (4.5, 2.4)),
]


def path_length(path) -> float:
    """Total XY length of a path in metres."""
    return sum(
        math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        for i in range(len(path) - 1)
    )


def leg_energy_fractions(planner, m, path) -> list[float]:
    """Per-hop `v_s / V_max` for every feasible hop on `path`.

    `v_s = xdot / cos(alpha_s)` is the takeoff speed the leg must produce; dividing by
    `V_max` gives the fraction of the leg's energy budget the hop consumes.  Reads
    `alpha_s` back from `diagnose_path`, so it automatically follows whatever
    takeoff-angle rule the planner currently implements.
    """
    fracs = []
    for d in diagnose_path(planner, m, path):
        if not d["feasible"] or d["alpha_s"] is None:
            continue
        try:
            v_s = _xdot(d["X"], d["Z"], d["alpha_s"], planner.g) / math.cos(d["alpha_s"])
        except (ValueError, ZeroDivisionError):
            continue
        fracs.append(v_s / planner.V_max)
    return fracs


def time_one(label, build, start, goal, disable_clearance) -> dict:
    m = build()
    planner = make_planner(m, disable_clearance, start, goal)

    t0 = time.perf_counter()
    path = planner.plan()
    elapsed = time.perf_counter() - t0

    row = {
        "scenario": label,
        "variant": "baseline" if disable_clearance else "ballistic",
        "seconds": elapsed,
        "expansions": planner.n_expansions,
        "edge_checks": planner.n_edge_checks,
        "edges_accepted": planner.n_edges_accepted,
        "found": path is not None,
        "hops": (len(path) - 1) if path else 0,
        "length_m": path_length(path) if path else float("nan"),
        "energy_mean": float("nan"),
        "energy_max": float("nan"),
    }
    if path:
        fracs = leg_energy_fractions(planner, m, path)
        if fracs:
            row["energy_mean"] = sum(fracs) / len(fracs)
            row["energy_max"] = max(fracs)
    return row


def render(rows) -> str:
    hdr = (
        f"# Deck planning times\n\n"
        f"`CELL_RESOLUTION={config.CELL_RESOLUTION}` · "
        f"`hop_radius=dynamic (per-state)` · "
        f"`V_MAX={demo_common.V_MAX:.3f}` · "
        f"`ROBOT_RADIUS={config.ROBOT_RADIUS}`\n\n"
        f"`v_s/V_max` is the fraction of the leg's energy budget a hop consumes.\n\n"
        "| scenario | variant | seconds | expansions | edge checks | accepted | hops | "
        "path (m) | mean v_s/V_max | max v_s/V_max |\n"
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|\n"
    )
    lines = []
    for r in rows:
        found = "" if r["found"] else " *(no path)*"
        lines.append(
            f"| {r['scenario']}{found} | {r['variant']} | {r['seconds']:.3f} | "
            f"{r['expansions']} | {r['edge_checks']} | {r['edges_accepted']} | "
            f"{r['hops']} | {r['length_m']:.2f} | "
            f"{r['energy_mean']:.3f} | {r['energy_max']:.3f} |"
        )
    total = sum(r["seconds"] for r in rows)
    return hdr + "\n".join(lines) + f"\n\n**Total: {total:.2f} s over {len(rows)} runs.**\n"


def main() -> int:
    rows = []
    for label, build, start, goal in SCENARIOS:
        for disable in (False, True):
            row = time_one(label, build, start, goal, disable)
            rows.append(row)
            print(
                f"{row['scenario']:22s} {row['variant']:9s} "
                f"{row['seconds']:8.3f}s  exp={row['expansions']:6d} "
                f"checks={row['edge_checks']:7d}  "
                f"{'path' if row['found'] else 'NO PATH'}",
                flush=True,
            )

    table = render(rows)
    print("\n" + table)

    if len(sys.argv) > 1:
        dest = sys.argv[1]
        if not os.path.isabs(dest):
            dest = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))), dest
            )
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        with open(dest, "w") as fh:
            fh.write(table)
        print(f"Wrote: {dest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

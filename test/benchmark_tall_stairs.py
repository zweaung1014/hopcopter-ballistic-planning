"""Timing benchmark for the tall-stairs scenario.

Runs the ballistic planner (clearance ON) and the baseline planner (clearance
OFF) several times each on the same tall_stairs map/start/goal, timing only
the `plan()` call with `time.perf_counter()`, and reports wall-clock
statistics plus internal counters (node expansions, edge/clearance checks)
so the numbers can be checked against the algorithm's expected O(N*A*R*S)
scaling rather than trusted on wall-clock alone.

Note what the A/B measures. Clearance is a pure feasibility gate — it no longer
shapes edge cost — so `disable_clearance=True` differs by exactly which
candidate landings exist, not by how they are priced. The gap between the two
runs is therefore the cost of *pruning*, and the difference in expansions is
how much of the search space the gate removes.

Run:
    python test/benchmark_tall_stairs.py
"""

import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from hopping_astar_planner import HoppingAStarPlanner
from map2d5 import Map2D5
from maps.tall_stairs import build as build_map

# Mirrors the scenario in test/demo_tall_stairs.py.
START = (0.5, 2.5)
GOAL = (4.0, 2.5)
HOP_RADIUS = 1.5
N_ANGLES = 16
V_MAX = config.V_MAX

# A plan() call is ~2 s at the shipped 0.1 m resolution (it was ~0.3 s at 0.2 m),
# so the trial count is sized to keep the whole benchmark around a minute.
N_TRIALS = 5
N_WARMUP = 1


def make_planner(m: Map2D5, disable_clearance: bool) -> HoppingAStarPlanner:
    return HoppingAStarPlanner(
        map_env=m,
        start=START,
        goal=GOAL,
        hop_radius=HOP_RADIUS,
        n_angles=N_ANGLES,
        max_jump_height=config.MAX_JUMP_HEIGHT,
        w_energy=config.W_ENERGY,
        g=config.G_ACCEL,
        V_max=V_MAX,
        mass=config.ROBOT_MASS,
        eta=config.ETA_HOP,
        e_inject_max=config.E_INJECT_MAX,
        min_apex=config.MIN_APEX_HEIGHT,
        h_initial=config.H_INITIAL,
        V_g_max=config.V_G_MAX,
        speed_bin=config.SPEED_BIN,
        mu=config.MU,
        robot_radius=config.ROBOT_RADIUS,
        leg_radius=config.LEG_CYLINDER_RADIUS,
        foot_radius=config.FOOT_TIP_RADIUS,
        leg_length=config.LEG_LENGTH,
        min_clearance_gate=config.MIN_CLEARANCE,
        arc_max_step=config.ARC_SAMPLE_MAX_STEP,
        n_lateral=config.ARC_LATERAL_SAMPLES,
        obstacle_wall_extra=config.OBSTACLE_WALL_EXTRA,
        leg_clearance_start_frac=config.LEG_CLEARANCE_START_FRAC,
        hop_scan_step=config.HOP_SCAN_STEP,
        disable_clearance=disable_clearance,
    )


def time_planner(
    disable_clearance: bool,
    n_trials: int = N_TRIALS,
    warmup: int = N_WARMUP,
) -> dict:
    """Build+plan `warmup + n_trials` times; time only the `plan()` call.

    Returns per-trial timings plus counters/path from the last trial (the
    map/start/goal are fixed, so every trial is identical up to OS noise).
    """
    times: list[float] = []
    planner = None
    path = None

    for i in range(warmup + n_trials):
        m = build_map()
        planner = make_planner(m, disable_clearance)

        t0 = time.perf_counter()
        path = planner.plan()
        t1 = time.perf_counter()

        if i >= warmup:
            times.append(t1 - t0)

    return {
        "times": times,
        "path": path,
        "n_expansions": planner.n_expansions,
        "n_edge_checks": planner.n_edge_checks,
        "n_edges_accepted": planner.n_edges_accepted,
    }


def report(name: str, result: dict) -> None:
    times = result["times"]
    times_ms = [t * 1000.0 for t in times]
    path = result["path"]

    print(f"{name}")
    print(f"  trials         : {len(times)}")
    print(f"  mean           : {statistics.mean(times_ms):.3f} ms")
    print(f"  median         : {statistics.median(times_ms):.3f} ms")
    if len(times_ms) > 1:
        print(f"  stdev          : {statistics.stdev(times_ms):.3f} ms")
    print(f"  min / max      : {min(times_ms):.3f} / {max(times_ms):.3f} ms")
    print(f"  path           : {'FOUND' if path else 'NONE'}"
          + (f" ({len(path)} waypoints)" if path else ""))
    print(f"  node expansions: {result['n_expansions']}")
    print(f"  edge checks    : {result['n_edge_checks']}")
    print(f"  edges accepted : {result['n_edges_accepted']}")


def main() -> int:
    ballistic = time_planner(disable_clearance=False)
    baseline = time_planner(disable_clearance=True)

    print("=" * 60)
    report("Ballistic (clearance ON)", ballistic)
    print("-" * 60)
    report("Baseline (clearance OFF)", baseline)
    print("=" * 60)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

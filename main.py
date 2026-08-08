"""Entry point for the A* 2.5D map planning simulation."""

import importlib
import math
import sys

import config
from hopping_astar_planner import HoppingAStarPlanner
from visualizer import Visualizer

DEFAULT_MAP = "stairs"


def load_map(name: str):
    try:
        module = importlib.import_module(f"maps.{name}")
    except ModuleNotFoundError:
        import os
        available = [
            f[:-3] for f in os.listdir(os.path.join(os.path.dirname(__file__), "maps"))
            if f.endswith(".py") and f != "__init__.py"
        ]
        print(f"Unknown map: '{name}'. Available maps: {', '.join(sorted(available))}")
        sys.exit(1)
    return module.build()


def main():
    map_name = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_MAP
    env_map = load_map(map_name)

    # Plan path
    planner = HoppingAStarPlanner(
        map_env=env_map,
        start=config.START,
        goal=config.GOAL,
        hop_radius=config.HOP_RADIUS,
        n_angles=config.HOP_N_ANGLES,
        max_jump_height=config.MAX_JUMP_HEIGHT,
        alpha_uphill=config.ALPHA_UPHILL,
        alpha_downhill=config.ALPHA_DOWNHILL,
        g=config.G_ACCEL,
        V_max=config.V_MAX,
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
        hop_fixed_cost=config.HOP_FIXED_COST,
        hop_scan_step=config.HOP_SCAN_STEP,
    )
    path = planner.plan()

    if path is None:
        print("No path found.")
    else:
        print(f"Path found with {len(path)} waypoints.")
        # The energy chain is the whole point of this planner, so show it: each
        # hop's takeoff speed is bounded below by the previous hop's landing
        # speed and cannot be undone by replanning that hop alone.
        print(f"{'hop':>3}  {'X':>5} {'Z':>6}  {'alpha':>6}  "
              f"{'v_s':>5} {'v_g':>5}  {'drop':>5}  {'E_inj':>6}")
        for i, h in enumerate(planner.path_hops):
            print(f"{i + 1:>3}  {h['X']:5.2f} {h['Z']:+6.2f}  "
                  f"{math.degrees(h['alpha_s']):5.1f}°  "
                  f"{h['v_s']:5.2f} {h['v_g']:5.2f}  "
                  f"{h['apex_drop']:5.2f}  {h['e_inject']:6.2f} J")

    # Visualize
    vis = Visualizer(env_map)
    vis.draw_map()
    if path:
        vis.draw_hop_circles(path, config.HOP_RADIUS)
        vis.draw_path(path)
    vis.draw_start_goal(config.START, config.GOAL)
    vis.draw_robot_pose(config.START, config.ROBOT_RADIUS, config.MIN_CLEARANCE)
    vis.draw_robot_pose(config.GOAL, config.ROBOT_RADIUS, config.MIN_CLEARANCE)
    vis.show()


main()

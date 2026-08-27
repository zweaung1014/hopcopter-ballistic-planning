"""Entry point for the A* 2.5D map planning simulation."""

import importlib
import math
import os
import sys

import config
from hopping_astar_planner import HoppingAStarPlanner, max_hop_radius
from visualizer import Visualizer, draw_map_analysis

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
    return module, module.build()


def main():
    map_name = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_MAP
    map_module, env_map = load_map(map_name)
    start = map_module.START
    goal = map_module.GOAL

    # Plan path
    planner = HoppingAStarPlanner(
        map_env=env_map,
        start=start,
        goal=goal,
        max_jump_height=config.MAX_JUMP_HEIGHT,
        w_energy=config.W_ENERGY,
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
        leg_length=config.LEG_LENGTH,
        min_clearance_gate=config.MIN_CLEARANCE,
        steep_grade=config.STEEP_INFLATE_GRADE,
        arc_max_step=config.ARC_SAMPLE_MAX_STEP,
        hop_scan_step=config.HOP_SCAN_STEP,
        hop_scan_step_ref_radius=config.HOP_SCAN_STEP_REF_RADIUS,
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
    fig_analysis = draw_map_analysis(env_map, config.ROBOT_RADIUS, config.MIN_CLEARANCE, config.STEEP_INFLATE_GRADE)
    _out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", "demonstration_scenarios")
    os.makedirs(_out_dir, exist_ok=True)
    _save_path = os.path.join(_out_dir, f"{map_name}_map_analysis.png")
    fig_analysis.savefig(_save_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {_save_path}")
    vis = Visualizer(env_map)
    vis.draw_map()
    if path:
        # hop_radii[i] is the ring available when departing waypoint i: the
        # first hop's radius comes from the seeded start speed, every later
        # one from the previous hop's recorded radius (path has one more
        # point than path_hops).
        hop_radii = [max_hop_radius(
            planner.v_g_initial, config.ETA_HOP, config.E_INJECT_MAX,
            config.ROBOT_MASS, config.G_ACCEL, config.V_MAX,
        )] + [h["hop_radius"] for h in planner.path_hops]
        vis.draw_hop_circles(path, hop_radii)
        vis.draw_path(path)
    vis.draw_start_goal(start, goal)
    vis.draw_robot_pose(start, config.ROBOT_RADIUS, config.MIN_CLEARANCE)
    vis.draw_robot_pose(goal, config.ROBOT_RADIUS, config.MIN_CLEARANCE)
    vis.show()


main()

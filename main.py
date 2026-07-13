"""Entry point for the A* 2.5D map planning simulation."""

import config
from map2d5 import Map2D5
from hopping_astar_planner import HoppingAStarPlanner
from visualizer import Visualizer


def main():
    # Build map
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )

    # (Optional) Add obstacles here for testing, e.g.:
    # env_map.set_obstacle_region(2.0, 2.0, 3.0, 3.0)

    # Set elevation regions
    r_min, c_min = env_map.world_to_grid(2.0, 2.4)
    r_max, c_max = env_map.world_to_grid(3.0, 3.0)
    env_map.grid[r_min:r_max + 1, c_min:c_max + 1] = 0.4

    r_min, c_min = env_map.world_to_grid(2.0, 1.8)
    r_max, c_max = env_map.world_to_grid(3.0, 2.4)
    env_map.grid[r_min:r_max + 1, c_min:c_max + 1] = 0.2

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
    )
    path = planner.plan()

    if path is None:
        print("No path found.")
    else:
        print(f"Path found with {len(path)} waypoints.")

    # Visualize
    vis = Visualizer(env_map)
    vis.draw_map()
    if path:
        vis.draw_path(path)
    vis.draw_start_goal(config.START, config.GOAL)
    vis.show()


main()

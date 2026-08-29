"""A* path planning algorithm on a 2.5D grid map."""

import heapq
import numpy as np

from map2d5 import Map2D5


class AStarPlanner:
    """A* path planner on a 2.5D grid map.

    Plans an optimal path from start to goal using 8-connected grid neighbors,
    with edge costs that account for both distance and elevation changes.
    """

    # 8-connected neighbor offsets: (drow, dcol)
    NEIGHBORS = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),           (0, 1),
        (1, -1),  (1, 0),  (1, 1),
    ]

    def __init__(
        self,
        map_env: Map2D5,
        start: tuple[float, float],
        goal: tuple[float, float],
        max_jump_height: float = 0.5,
        alpha_uphill: float = 1.0,
        alpha_downhill: float = 0.5,
    ):
        self.map_env = map_env
        self.start_world = start
        self.goal_world = goal
        self.max_jump_height = max_jump_height
        self.alpha_uphill = alpha_uphill
        self.alpha_downhill = alpha_downhill

        # Convert start/goal to grid coordinates
        self.start_cell = map_env.world_to_grid(start[0], start[1])
        self.goal_cell = map_env.world_to_grid(goal[0], goal[1])

    def plan(self) -> list[tuple[float, float]] | None:
        """Run A* and return the path as a list of (x, y) world coords, or None."""
        start = self.start_cell
        goal = self.goal_cell

        # Priority queue: (f_cost, counter, (row, col))
        open_set = []
        counter = 0
        heapq.heappush(open_set, (0.0, counter, start))

        # g_cost[cell] = best known cost from start to cell
        g_cost = {start: 0.0}
        # came_from[cell] = parent cell
        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}

        while open_set:
            f, _, current = heapq.heappop(open_set)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            # Skip if we've already found a better path to this node
            if f > g_cost.get(current, float("inf")) + self._heuristic(current):
                continue

            current_z = self.map_env.grid[current[0], current[1]]

            for dr, dc in self.NEIGHBORS:
                neighbor = (current[0] + dr, current[1] + dc)

                # Bounds check
                if not (0 <= neighbor[0] < self.map_env.rows and
                        0 <= neighbor[1] < self.map_env.cols):
                    continue

                neighbor_z = self.map_env.grid[neighbor[0], neighbor[1]]

                # Obstacle check
                if neighbor_z == Map2D5.OBSTACLE:
                    continue

                # Hard jump height constraint
                dz = neighbor_z - current_z
                if abs(dz) > self.max_jump_height:
                    continue

                # Edge cost: distance + elevation penalty
                edge_cost = self._edge_cost(dr, dc, dz)
                tentative_g = g_cost[current] + edge_cost

                if tentative_g < g_cost.get(neighbor, float("inf")):
                    g_cost[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f_cost = tentative_g + self._heuristic(neighbor)
                    counter += 1
                    heapq.heappush(open_set, (f_cost, counter, neighbor))

        return None  # No path found

    def _edge_cost(self, dr: int, dc: int, dz: float) -> float:
        """Compute cost of moving to an adjacent cell.

        Cost = xy_distance + elevation_penalty.
        Diagonal moves have distance sqrt(2) * resolution, cardinal moves have resolution.
        """
        if dr != 0 and dc != 0:
            xy_dist = self.map_env.resolution * np.sqrt(2)
        else:
            xy_dist = self.map_env.resolution

        if dz > 0:
            elevation_penalty = self.alpha_uphill * dz
        elif dz < 0:
            elevation_penalty = self.alpha_downhill * abs(dz)
        else:
            elevation_penalty = 0.0

        return xy_dist + elevation_penalty

    def _heuristic(self, cell: tuple[int, int]) -> float:
        """Admissible heuristic: Euclidean distance in world coordinates to goal.

        Admissible because actual cost >= Euclidean distance (elevation only adds cost).
        """
        x1, y1 = self.map_env.grid_to_world(cell[0], cell[1])
        x2, y2 = self.map_env.grid_to_world(self.goal_cell[0], self.goal_cell[1])
        return np.hypot(x1 - x2, y1 - y2)

    def _reconstruct_path(
        self,
        came_from: dict[tuple[int, int], tuple[int, int] | None],
        current: tuple[int, int],
    ) -> list[tuple[float, float]]:
        """Trace back from goal to start, return path in world coordinates."""
        path_cells = []
        while current is not None:
            path_cells.append(current)
            current = came_from[current]
        path_cells.reverse()

        path = []
        for row, col in path_cells:
            x, y = self.map_env.grid_to_world(row, col)
            path.append((x, y))
        return path

"""Visualization of the 2.5D map and planned path."""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
import matplotlib.cm as cm
import numpy as np

from map2d5 import Map2D5


class Visualizer:
    """Renders the map and final path using matplotlib."""

    def __init__(self, map_env: Map2D5):
        self.map_env = map_env
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlim(0, map_env.size_x)
        self.ax.set_ylim(0, map_env.size_y)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("A* Path Planning on 2.5D Map")

    def draw_map(self):
        """Draw the map grid with cell boundaries and height-based coloring."""
        res = self.map_env.resolution
        rows = self.map_env.rows
        cols = self.map_env.cols

        # Compute elevation range (excluding obstacles) for colormap normalization
        non_obstacle = self.map_env.grid[self.map_env.grid != Map2D5.OBSTACLE]
        z_min = float(non_obstacle.min()) if non_obstacle.size else 0.0
        z_max = float(non_obstacle.max()) if non_obstacle.size else 1.0
        if z_min == z_max:
            z_max = z_min + 1.0  # avoid degenerate normalization
        norm = mcolors.Normalize(vmin=z_min, vmax=z_max)
        cmap = cm.get_cmap("YlOrBr")

        # Draw grid lines
        for i in range(rows + 1):
            self.ax.axhline(i * res, color="gray", linewidth=0.3, alpha=0.5)
        for j in range(cols + 1):
            self.ax.axvline(j * res, color="gray", linewidth=0.3, alpha=0.5)

        # Draw cells: color by elevation, obstacles black
        font_size = max(3, min(7, int(200 / max(rows, cols))))
        for r in range(rows):
            for c in range(cols):
                z = self.map_env.grid[r, c]
                cx = (c + 0.5) * res
                cy = (r + 0.5) * res
                if z == Map2D5.OBSTACLE:
                    facecolor = "black"
                    alpha = 0.85
                else:
                    facecolor = cmap(norm(z))
                    alpha = 0.75
                rect = plt.Rectangle(
                    (c * res, r * res), res, res,
                    facecolor=facecolor, alpha=alpha, linewidth=0,
                )
                self.ax.add_patch(rect)
                if z != Map2D5.OBSTACLE:
                    self.ax.text(
                        cx, cy, f"{z:.1f}",
                        ha="center", va="center",
                        fontsize=font_size, color="black", alpha=0.9,
                    )

        # Add colorbar legend
        sm = cm.ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])
        cbar = self.fig.colorbar(sm, ax=self.ax, fraction=0.03, pad=0.02)
        cbar.set_label("Elevation (m)", fontsize=9)

        # Add obstacle legend patch
        self._obstacle_patch = mpatches.Patch(facecolor="black", alpha=0.85, label="Obstacle")

    def draw_path(self, path: list[tuple[float, float]]):
        """Draw the final planned path."""
        if not path:
            return
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        self.ax.plot(xs, ys, color="red", linewidth=2.0, label="Path")
        # Draw waypoint markers
        self.ax.plot(xs, ys, "o", color="red", markersize=5, zorder=5)

    def draw_hop_circles(self, path: list[tuple[float, float]], hop_radius: float):
        """Draw the reachable ring at each waypoint to visualise hop coverage."""
        if not path:
            return
        for i, (x, y) in enumerate(path):
            circle = mpatches.Circle(
                (x, y),
                radius=hop_radius,
                fill=False,
                edgecolor="deepskyblue",
                linewidth=0.8,
                linestyle="--",
                alpha=0.6,
                label="Hop radius" if i == 0 else None,
            )
            self.ax.add_patch(circle)

    def draw_start_goal(self, start: tuple[float, float], goal: tuple[float, float]):
        """Draw start and goal markers."""
        self.ax.plot(start[0], start[1], "go", markersize=10, label="Start")
        self.ax.plot(goal[0], goal[1], "r*", markersize=15, label="Goal")

    def show(self):
        """Display the plot."""
        handles, labels = self.ax.get_legend_handles_labels()
        if hasattr(self, "_obstacle_patch"):
            handles.append(self._obstacle_patch)
            labels.append("Obstacle")
        self.ax.legend(handles, labels, loc="upper left")
        plt.tight_layout()
        plt.show()

"""Visualization of the 2.5D map and planned path."""

import math

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
import matplotlib.cm as cm
import numpy as np

from map2d5 import Map2D5


def draw_arc_side_view(
    ax,
    c_s: tuple[float, float, float],
    c_g: tuple[float, float, float],
    alpha_s: float,
    height_map: Map2D5,
    robot_radius: float,
    leg_length: float,
    obstacle_fill: float,
    max_step: float,
    label: str | None = None,
    *,
    min_clearance_gate: float = 0.15,
    n_lateral: int = 3,
) -> float:
    """Plot the side view (u vs z) of a ballistic hop over the terrain profile.

    Mirrors the sampling semantics of `hopping_astar_planner.terrain_profile`
    so what you see matches what the planner decided. Returns the same
    clearance value the planner would have computed (for numerical assertions
    in demo scripts).

    `c_s`/`c_g` carry *terrain* heights; the plotted arc is the CoM
    trajectory, running between `terrain + leg_length` at each end.

    Drawing:
      * terrain profile filled in brown along `u in [0, X]` — this is the
        *swept-corridor* profile (max across the body's width), which is what
        the gate actually tests, so a ridge beside the centreline shows up;
      * parabolic arc (Campana Eq. 2) as a line, green when the whole arc
        clears the gate, red when any sample falls below it;
      * the body's underside as a dashed line `robot_radius` below the arc —
        clearance is the gap between that and the terrain;
      * a marker at the minimum-clearance sample plus an annotation of the
        clearance and `alpha_s`.
    """
    x_s, y_s, t_s = c_s
    x_g, y_g, t_g = c_g

    dx = x_g - x_s
    dy = y_g - y_s
    X = math.hypot(dx, dy)
    if X < 1e-9:
        return math.inf
    cos_t = dx / X
    sin_t = dy / X

    # Delayed import to avoid a circular dependency at module load time.
    from hopping_astar_planner import (
        _arc_z, clearance_for_alpha, terrain_profile,
    )

    z_s = t_s + leg_length
    Z = t_g - t_s

    # Dense resampling for a smooth curve; the numeric verdict comes from the
    # planner's own profile below, so the two need not share a sample count.
    step = min(max_step, height_map.resolution / 3.0)
    n = max(64, int(math.ceil(X / step)) + 1)
    us = np.linspace(0.0, X, n)
    z_arc = _arc_z(us, X, Z, z_s, alpha_s)

    if n_lateral <= 1:
        offsets = np.zeros(1)
    else:
        offsets = np.linspace(-robot_radius, robot_radius, n_lateral)
    cx = x_s + us * cos_t
    cy = y_s + us * sin_t
    px = cx[:, None] - offsets[None, :] * sin_t
    py = cy[:, None] + offsets[None, :] * cos_t
    z_terr = height_map.sample_bilinear(
        px, py, obstacle_fill=obstacle_fill
    ).max(axis=1)
    # Off-map samples read as blocked (the planner rejects such hops outright).
    off_map = (cx < 0.0) | (cx >= height_map.size_x) | \
              (cy < 0.0) | (cy >= height_map.size_y)
    z_terr = np.where(off_map, obstacle_fill, z_terr)

    clearance = z_arc - z_terr - robot_radius

    # Authoritative value from the planner's own function, so the displayed
    # number matches exactly what the A* gate would evaluate.
    profile = terrain_profile(
        c_s, c_g, height_map, robot_radius, leg_length,
        max_step, obstacle_fill, n_lateral,
    )
    min_c = math.inf if profile is None else clearance_for_alpha(profile, alpha_s)

    min_idx = int(np.argmin(clearance))
    rejected = min_c < min_clearance_gate
    arc_colour = "#c62828" if rejected else "#2e7d32"

    # --- draw ---
    ax.fill_between(
        us, z_terr, min(z_terr.min(), (z_arc - robot_radius).min()) - 0.1,
        color="#8d6e63", alpha=0.55, linewidth=0, label="Terrain (swept)",
    )
    ax.plot(
        us, z_arc,
        color=arc_colour, linewidth=2.0,
        label=(label or ("Arc (rejected)" if rejected else "Arc (clears)")),
    )
    # Underside of the body — the surface the gate is actually measuring.
    ax.plot(
        us, z_arc - robot_radius,
        color=arc_colour, linewidth=1.0, linestyle="--", alpha=0.6,
        label=f"Body underside (r={robot_radius} m)",
    )
    # Clearance marker at the tightest sample.
    ax.plot([us[min_idx], us[min_idx]],
            [z_arc[min_idx] - robot_radius, z_terr[min_idx]],
            color=arc_colour, linewidth=1.2, linestyle=":")
    ax.plot(us[min_idx], z_arc[min_idx] - robot_radius, "o",
            color=arc_colour, markersize=5)

    verdict = "REJECT" if rejected else "ACCEPT"
    ax.set_title(
        f"α={math.degrees(alpha_s):.1f}°   clearance={min_c:+.3f} m   "
        f"[{verdict} @ {min_clearance_gate:.2f} m]",
        fontsize=9,
    )
    ax.set_xlabel("u (m along XY segment)")
    ax.set_ylabel("z (m)")
    ax.set_xlim(0, X)
    return min_c


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

        # On a fine grid the per-cell chrome stops being legible and starts
        # costing real time (2500 rectangles + 2500 text objects), so drop it.
        detailed = rows * cols <= 900

        # Draw grid lines
        if detailed:
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
                if detailed and z != Map2D5.OBSTACLE:
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

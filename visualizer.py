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
    g: float,
    robot_radius: float,
    obstacle_fill: float,
    epsilon: float,
    max_step: float,
    label: str | None = None,
) -> float:
    """Plot the side view (u vs z) of a ballistic hop over the terrain profile.

    Mirrors the sampling and body-clearance semantics of
    `hopping_astar_planner.min_clearance` so what you see matches what the
    planner decided. Returns the same `min_clearance` value the planner
    would have computed (for numerical assertions in demo scripts).

    Drawing:
      * terrain profile filled in brown along `u in [0, X]`;
      * parabolic arc (Campana Eq. 2) as a line, coloured green when every
        interior sample clears, red when any interior sample collides;
      * faded arc segment near the endpoints where the arc has not yet
        risen `robot_radius` above the takeoff/landing ground plane (these
        samples are excluded from the clearance calc, matching the planner);
      * a vertical marker at the minimum-clearance sample plus an
        annotation of `mc` and `alpha_s`.
    """
    x_s, y_s, z_s = c_s
    x_g, y_g, z_g = c_g

    dx = x_g - x_s
    dy = y_g - y_s
    X = math.hypot(dx, dy)
    if X < 1e-9:
        return math.inf
    cos_t = dx / X
    sin_t = dy / X

    Z = z_g - z_s
    # xdot from Campana Eq. 4. Caller is expected to have chosen alpha_s
    # inside the feasible interval, so denom > 0.
    xdot_sq = g * X * X / (2.0 * (X * math.tan(alpha_s) - Z))
    tan_a = math.tan(alpha_s)

    step = min(max_step, height_map.resolution / 3.0)
    n = max(64, int(math.ceil(X / step)) + 1)
    us = np.linspace(0.0, X, n)

    z_arc = z_s + us * tan_a - g * us * us / (2.0 * xdot_sq)
    z_terr = np.empty_like(us)
    for i, u in enumerate(us):
        sx = x_s + u * cos_t
        sy = y_s + u * sin_t
        if height_map.is_within_bounds(sx, sy):
            z_terr[i] = height_map.get_elevation_bilinear(
                sx, sy, obstacle_fill=obstacle_fill
            )
        else:
            z_terr[i] = obstacle_fill  # off-map is treated as blocked

    # Endpoint body-clear mask matches min_clearance()'s per-sample filter.
    body_clear = (z_arc - z_s >= robot_radius) & (z_arc - z_g >= robot_radius)
    # Also honour the horizontal `epsilon` trim.
    interior = (us > epsilon) & (us < X - epsilon)
    counted = body_clear & interior

    clearance = z_arc - z_terr - robot_radius

    # Authoritative min_clearance from the planner's own function, so the
    # displayed number matches exactly what the A* gate would evaluate.
    # Delayed import to avoid a circular dependency at module load time.
    from hopping_astar_planner import min_clearance as _mc
    min_c = _mc(
        c_s, c_g, alpha_s, height_map, g, robot_radius,
        epsilon, max_step, obstacle_fill,
    )

    # Approximate marker location: argmin of the helper's dense sampling
    # restricted to counted samples. Purely for the on-plot dot; the
    # numeric verdict comes from `min_c` above.
    if counted.any():
        min_idx = int(np.where(counted)[0][np.argmin(clearance[counted])])
    else:
        min_idx = -1

    collides = min_c < 0.0
    arc_colour = "#c62828" if collides else "#2e7d32"

    # --- draw ---
    ax.fill_between(
        us, z_terr, z_terr.min() - 0.1,
        color="#8d6e63", alpha=0.55, linewidth=0, label="Terrain",
    )
    # Faded arc where samples are excluded from the clearance decision.
    ax.plot(
        us[~counted], z_arc[~counted],
        color=arc_colour, linewidth=1.2, alpha=0.35,
    )
    # Full-strength arc over counted samples.
    ax.plot(
        us[counted], z_arc[counted],
        color=arc_colour, linewidth=2.0,
        label=(label or ("Arc (collides)" if collides else "Arc (clears)")),
    )
    # Robot-radius footprint at the min-clearance sample.
    if min_idx >= 0:
        ax.plot([us[min_idx], us[min_idx]],
                [z_arc[min_idx] - robot_radius, z_arc[min_idx]],
                color=arc_colour, linewidth=1.0, linestyle=":")
        ax.plot(us[min_idx], z_arc[min_idx], "o",
                color=arc_colour, markersize=5)

    verdict = "REJECT" if collides else "ACCEPT"
    ax.set_title(
        f"α={math.degrees(alpha_s):.1f}°   mc={min_c:+.3f} m   [{verdict}]",
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

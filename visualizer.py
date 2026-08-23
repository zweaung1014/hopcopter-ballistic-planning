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
    max_step: float,
    label: str | None = None,
    *,
    min_clearance_gate: float = 0.15,
    steep_grade: float = 1.7320508075688767,  # tan(60 deg); see config
) -> float:
    """Plot the side view (u vs z) of a ballistic hop over the terrain profile.

    Reads the same inflated height field the planner's gate reads, so what you
    see is what the planner decided. Returns the same clearance value the
    planner would have computed (for numerical assertions in demo scripts).

    `c_s`/`c_g` carry *terrain* heights; the plotted arc is the CoM
    trajectory, running between `terrain + leg_length` at each end. The foot
    tip trajectory (`arc - leg_length`) and the clearance envelope
    (`arc - leg_length - min_clearance_gate`) are also drawn — the envelope is
    the surface the gate is really testing against.

    Drawing:
      * INFLATED terrain filled in brown along `u in [0, X]` — the field the
        gate actually reads, i.e. real terrain already widened by the body's
        lateral reach, so the plot and the verdict cannot disagree;
      * CoM arc as a line, green when the whole body clears the gate,
        red when any sample falls below it;
      * foot line = `arc - leg_length` (the cylinder's flat bottom);
      * clearance envelope = `arc - leg_length - min_clearance_gate` (dashed):
        the inflated terrain must stay below this to satisfy the gate;
      * marker at the min-clearance sample and an α / clearance annotation.
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
    from hopping_astar_planner import _arc_z, arc_clearance

    z_s = t_s + leg_length
    Z = t_g - t_s

    # Dense resampling for a smooth curve; the numeric verdict comes from the
    # planner's own profile below, so the two need not share a sample count.
    step = min(max_step, height_map.resolution / 3.0)
    n = max(64, int(math.ceil(X / step)) + 1)
    us = np.linspace(0.0, X, n)
    z_arc = _arc_z(us, X, Z, z_s, alpha_s)

    # The gate's own view of the terrain: dilated sideways by the body's full
    # lateral reach, read along the centreline with a nearest-cell lookup.
    # `inflated_field` memoises, so this IS the planner's array.
    inflated = height_map.inflated_field(
        robot_radius + min_clearance_gate, steep_grade,
    )
    cx = x_s + us * cos_t
    cy = y_s + us * sin_t
    res = height_map.resolution
    ci = np.clip((cx / res).astype(np.int64), 0, height_map.cols - 1)
    ri = np.clip((cy / res).astype(np.int64), 0, height_map.rows - 1)
    z_terr = inflated[ri, ci]
    # Off-map samples read as blocked (the planner rejects such hops outright),
    # and OBSTACLE columns inflate to +inf, which no axis can scale. Both are
    # clamped to something tall but finite for drawing only.
    off_map = (cx < 0.0) | (cx >= height_map.size_x) | \
              (cy < 0.0) | (cy >= height_map.size_y)
    finite = z_terr[np.isfinite(z_terr)]
    wall_z = (float(finite.max()) if finite.size else 0.0) + 10.0
    z_terr = np.where(off_map | ~np.isfinite(z_terr), wall_z, z_terr)

    # The cylinder has a flat bottom at the foot and square edges, so the gate
    # under it is exactly `min_clearance_gate` — the body radius is already in
    # the inflated field, laterally, and contributes nothing downward.
    z_foot = z_arc - leg_length
    z_bottom_env = z_foot - min_clearance_gate

    # Authoritative value from the planner's own function, so the displayed
    # number matches exactly what the A* gate would evaluate.
    min_c = arc_clearance(
        c_s, c_g, height_map, inflated, leg_length, max_step, alpha_s,
    )

    # Where the envelope comes closest to the inflated terrain in the side
    # view. Same quantity `arc_clearance` minimises, at drawing density.
    display_clear = z_bottom_env - z_terr
    if display_clear.size >= 2:
        display_clear = display_clear.copy()
        display_clear[0] = np.inf
        display_clear[-1] = np.inf
    min_idx = int(np.argmin(display_clear))
    rejected = min_c < min_clearance_gate
    arc_colour = "#c62828" if rejected else "#2e7d32"

    # --- draw ---
    ax.fill_between(
        us, z_terr, min(z_terr.min(), z_bottom_env.min()) - 0.1,
        color="#8d6e63", alpha=0.55, linewidth=0, label="Terrain (inflated by body)",
    )
    ax.plot(
        us, z_arc,
        color=arc_colour, linewidth=2.0,
        label=(label or ("Arc (rejected)" if rejected else "Arc (clears)")),
    )
    # Foot tip: the axis-segment bottom of the capsule.
    ax.plot(
        us, z_foot,
        color=arc_colour, linewidth=1.0, linestyle="-", alpha=0.6,
        label=f"Foot tip (arc − L, L={leg_length} m)",
    )
    # Bottom-cap envelope: the surface the gate needs the terrain to stay below.
    ax.plot(
        us, z_bottom_env,
        color=arc_colour, linewidth=1.0, linestyle="--", alpha=0.6,
        label=f"Bottom-cap envelope (foot − gate, gate={min_clearance_gate} m)",
    )
    # Clearance marker at the tightest sample.
    ax.plot([us[min_idx], us[min_idx]],
            [z_bottom_env[min_idx], z_terr[min_idx]],
            color=arc_colour, linewidth=1.2, linestyle=":")
    ax.plot(us[min_idx], z_bottom_env[min_idx], "o",
            color=arc_colour, markersize=5)

    verdict = "REJECT" if rejected else "ACCEPT"
    # Deliberately terse: these titles sit above panels that can be under 2.5 in
    # wide in a multi-hop strip. The gate value is stated once in the figure's
    # suptitle, so repeating it per panel is what overflows the layout.
    ax.set_title(f"{verdict}   {min_c:+.3f} m\nα = {math.degrees(alpha_s):.1f}°")
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
        cbar.set_label("Elevation (m)")

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

    def draw_hop_circles(self, path: list[tuple[float, float]], hop_radii: list[float]):
        """Draw the reachable ring at each waypoint to visualise hop coverage.

        `hop_radii[i]` is the ring available when departing waypoint `i` — it
        varies along the path since the ring is derived from the robot's
        speed at that state, not a fixed constant.
        """
        if not path:
            return
        for i, ((x, y), r) in enumerate(zip(path, hop_radii)):
            circle = mpatches.Circle(
                (x, y),
                radius=r,
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

    def draw_robot_pose(
        self,
        pose: tuple[float, float],
        robot_radius: float,
        min_clearance: float,
    ):
        """Overlay top-down body and clearance envelope at a pose.

        Two concentric rings: the inner one is the body (`robot_radius`), the
        outer one is the safety envelope the leg-cylinder-sides check widens
        to (`robot_radius + min_clearance`). Placed at start and goal to make
        the scale of the collision volume visible relative to the map.
        """
        x, y = pose
        body = mpatches.Circle(
            (x, y), radius=robot_radius,
            fill=False, edgecolor="#1565c0", linewidth=1.4, alpha=0.9,
            label=f"Robot body (r={robot_radius} m)",
        )
        env = mpatches.Circle(
            (x, y), radius=robot_radius + min_clearance,
            fill=False, edgecolor="#1565c0", linewidth=0.8,
            linestyle="--", alpha=0.6,
            label=f"Clearance envelope (+{min_clearance} m)",
        )
        # Only label once so the legend doesn't accumulate duplicates when the
        # method is called for start AND goal.
        if getattr(self, "_pose_drawn", False):
            body.set_label(None)
            env.set_label(None)
        self.ax.add_patch(body)
        self.ax.add_patch(env)
        self._pose_drawn = True

    def show(self):
        """Display the plot."""
        handles, labels = self.ax.get_legend_handles_labels()
        if hasattr(self, "_obstacle_patch"):
            handles.append(self._obstacle_patch)
            labels.append("Obstacle")
        self.ax.legend(handles, labels, loc="upper left")
        plt.tight_layout()
        plt.show()

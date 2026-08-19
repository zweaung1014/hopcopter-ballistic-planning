"""Visual demo: prove the A* planner actively rejects colliding arcs.

Runs the ballistic HoppingAStarPlanner on a map with a tall pillar blocking
the direct route between start and goal. For the takeoff cell that is
most affected by the pillar (chosen as the last waypoint of the resulting
path whose ring of candidates overlaps the pillar), we then re-evaluate
every one of that node's ring candidates and colour-code them:

  * green (light)     -- accepted (arc clears the pillar)
  * red               -- rejected (arc would collide with the pillar, or
                        no feasible takeoff angle exists)
  * green (bold)      -- accepted AND the one the planner actually chose

This directly demonstrates the "move the takeoff back" behaviour: the
planner considered a straight-shot landing that goes right over the
pillar, evaluated its arc, saw a negative clearance, and picked a
different ring candidate instead.

Two figures are saved:
  * test/planner_reroute_topdown.png -- top-down of the map with the
    chosen path and every ring candidate at the highlighted takeoff cell.
  * test/planner_reroute_sideview.png -- side view (u vs z) of the arc
    for each ring candidate at that takeoff cell, in a grid of panels.

Run:
    python test/demo_planner_reroute.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

import config
from demo_common import (
    out_path,
    planner_alpha_interval,
    planner_angle_and_clearance,
)
from hopping_astar_planner import (
    HoppingAStarPlanner,
    feasible_alpha_interval,
    max_hop_radius,
)
from map2d5 import Map2D5
from visualizer import Visualizer, draw_arc_side_view


# --- Scenario -------------------------------------------------------------- #
MAP_X = 6.0
MAP_Y = 5.0
RES = config.CELL_RESOLUTION

# Wall-like pillar blocking the direct XY route. Painted as an OBSTACLE region,
# so its stored height is irrelevant — `Map2D5.inflated_field` gives OBSTACLE
# cells `+inf`, which no arc can clear at any takeoff angle.
PILLAR_XMIN, PILLAR_XMAX = 2.8, 3.2
PILLAR_YMIN, PILLAR_YMAX = 1.0, 3.0

#: Height to draw the OBSTACLE pillar at. It is infinitely tall to the
#: clearance check; figures need a finite number.
PILLAR_DISPLAY_H = 1.5

START = (1.0, 2.0)
GOAL = (5.0, 2.0)
N_ANGLES = 16
V_MAX = config.V_MAX


def build_map() -> Map2D5:
    m = Map2D5(size_x=MAP_X, size_y=MAP_Y, resolution=RES)
    # Represent the pillar as an OBSTACLE region: landing on it is illegal,
    # and the arc-clearance check reads these cells as infinitely tall, so no
    # arc clears them. That forces the planner to route around rather than
    # land on top of the pillar.
    m.set_obstacle_region(PILLAR_XMIN, PILLAR_YMIN, PILLAR_XMAX, PILLAR_YMAX)
    return m


def make_planner(m: Map2D5) -> HoppingAStarPlanner:
    return HoppingAStarPlanner(
        map_env=m,
        start=START,
        goal=GOAL,
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
        leg_length=config.LEG_LENGTH,
        min_clearance_gate=config.MIN_CLEARANCE,
        arc_max_step=config.ARC_SAMPLE_MAX_STEP,
        hop_scan_step=config.HOP_SCAN_STEP,
        hop_scan_step_ref_radius=config.HOP_SCAN_STEP_REF_RADIUS,
    )


def enumerate_ring_candidates(
    planner: HoppingAStarPlanner, parent_cell, n_angles: int = N_ANGLES,
):
    """Regenerate a single ring of `n_angles` evenly spaced candidates from
    `parent_cell`, and classify each as ACCEPTED / REJECTED, with the
    parameters used to make that decision.

    Deliberately simpler than the planner's own scanline disk fill
    (`HoppingAStarPlanner._generate_hop_neighbors`) — one ring, not an area
    fill — so the figure stays readable.

    Returns a list of dicts with keys:
        cell, c_s, c_g, X, Z, alpha_s, mc, accepted, reason
    """
    m = planner.map_env
    px, py = m.grid_to_world(*parent_cell)
    pz = float(m.grid[parent_cell[0], parent_cell[1]])
    r = max_hop_radius(
        planner.v_g_initial, planner.eta, planner.e_inject_max, planner.mass,
        planner.g, planner.V_max,
    )

    seen: set = {parent_cell}
    out: list = []
    for i in range(n_angles):
        a = 2.0 * math.pi * i / n_angles
        dx, dy = math.cos(a), math.sin(a)
        tx = px + r * dx
        ty = py + r * dy
        if not m.is_within_bounds(tx, ty):
            continue
        cell = m.world_to_grid(tx, ty)
        if cell in seen:
            continue
        seen.add(cell)

        nx, ny = m.grid_to_world(*cell)
        nz = float(m.grid[cell[0], cell[1]])
        c_s = (px, py, pz)
        c_g = (nx, ny, nz)
        X = math.hypot(nx - px, ny - py)
        Z = nz - pz

        entry: dict = {
            "cell": cell, "c_s": c_s, "c_g": c_g,
            "X": X, "Z": Z, "alpha_s": None, "mc": None,
            "accepted": False, "reason": "",
        }
        if nz == Map2D5.OBSTACLE:
            entry["reason"] = "landing is OBSTACLE"
            out.append(entry)
            continue
        iv = planner_alpha_interval(planner, m, (px, py), (nx, ny), X, Z)
        if iv is None:
            entry["reason"] = "no feasible alpha (friction cone / leg energy / geometry)"
            out.append(entry)
            continue
        picked = planner_angle_and_clearance(planner, c_s, c_g, iv[0], iv[1])
        if picked is None:
            entry["reason"] = "arc leaves the map"
            out.append(entry)
            continue
        a, mc = picked
        entry["alpha_s"] = a
        entry["mc"] = mc
        if mc < planner.min_clearance_gate:
            entry["reason"] = f"arc collides (mc={mc:+.3f} m)"
        else:
            entry["accepted"] = True
            entry["reason"] = f"clears (mc={mc:+.3f} m)"
        out.append(entry)
    return out


def choose_takeoff_cell(planner: HoppingAStarPlanner, path_cells):
    """Pick the path cell whose ring of candidates is most affected by the
    pillar (i.e., has the largest number of REJECTED candidates whose
    landing lies on the pillar's far side)."""
    best = path_cells[0]
    best_score = -1
    for cell in path_cells[:-1]:  # skip the goal (no outgoing hops)
        cands = enumerate_ring_candidates(planner, cell)
        rejected = sum(1 for c in cands if not c["accepted"] and c["mc"] is not None)
        # Prefer cells where at least one candidate lands past the pillar
        # (roughly x > PILLAR_XMAX from a cell with x < PILLAR_XMIN).
        px, _ = planner.map_env.grid_to_world(*cell)
        crosses = px < PILLAR_XMIN and any(
            planner.map_env.grid_to_world(*c["cell"])[0] > PILLAR_XMAX
            for c in cands
        )
        score = rejected * (2 if crosses else 1)
        if score > best_score:
            best_score = score
            best = cell
    return best


def path_cells_from(planner: HoppingAStarPlanner, path):
    return [planner.map_env.world_to_grid(x, y) for (x, y) in path]


def draw_topdown(fig, ax, planner, path, chosen_cell, candidates):
    m = planner.map_env
    vis = Visualizer.__new__(Visualizer)
    vis.map_env = m
    vis.fig = fig
    vis.ax = ax
    ax.set_xlim(0, MAP_X)
    ax.set_ylim(0, MAP_Y)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    vis.draw_map()

    # Full chosen path.
    xs = [p[0] for p in path]; ys = [p[1] for p in path]
    ax.plot(xs, ys, color="red", linewidth=2.0, zorder=6, label="chosen path")
    ax.plot(xs, ys, "o", color="red", markersize=4, zorder=7)

    # Highlight the takeoff cell whose ring we're visualising.
    px, py = m.grid_to_world(*chosen_cell)
    ring_r = max_hop_radius(
        planner.v_g_initial, planner.eta, planner.e_inject_max, planner.mass,
        planner.g, planner.V_max,
    )
    ax.add_patch(mpatches.Circle(
        (px, py), radius=ring_r, fill=False,
        edgecolor="deepskyblue", linewidth=1.2, linestyle="--",
        alpha=0.9, zorder=5, label="hop ring",
    ))
    ax.plot(px, py, "s", color="deepskyblue", markersize=9, zorder=8,
            label="takeoff under study")

    # Chosen next waypoint = the actual path node right after the chosen
    # takeoff, so we can bold-highlight it.
    path_cells = path_cells_from(planner, path)
    try:
        idx = path_cells.index(chosen_cell)
        chosen_next = path_cells[idx + 1] if idx + 1 < len(path_cells) else None
    except ValueError:
        chosen_next = None

    # Draw each ring candidate as an arrow from parent to landing point,
    # coloured by accepted/rejected. Bold the actually-chosen one.
    for cand in candidates:
        lx, ly = m.grid_to_world(*cand["cell"])
        if cand["cell"] == chosen_next:
            colour = "#1b5e20"; lw = 3.0; z = 9
        elif cand["accepted"]:
            colour = "#66bb6a"; lw = 1.0; z = 4
        else:
            colour = "#c62828"; lw = 1.4; z = 4
        ax.annotate(
            "",
            xy=(lx, ly), xytext=(px, py),
            arrowprops=dict(arrowstyle="->", color=colour, lw=lw, alpha=0.9),
            zorder=z,
        )

    ax.plot(START[0], START[1], "go", markersize=10, label="start", zorder=10)
    ax.plot(GOAL[0], GOAL[1], "r*", markersize=14, label="goal", zorder=10)
    handles, labels = ax.get_legend_handles_labels()
    handles += [
        mpatches.Patch(color="#66bb6a", label="candidate: ACCEPT"),
        mpatches.Patch(color="#c62828", label="candidate: REJECT"),
        mpatches.Patch(color="#1b5e20", label="candidate: CHOSEN"),
    ]
    labels += ["candidate: ACCEPT", "candidate: REJECT", "candidate: CHOSEN"]
    ax.legend(handles, labels, loc="upper left")
    ax.set_title(
        f"Chosen path (red) + all {len(candidates)} ring candidates from"
        f" takeoff cell {chosen_cell}", wrap=True
    )


def draw_candidate_sideviews(fig, planner, candidates, chosen_cell, chosen_next):
    m = planner.map_env
    n = len(candidates)
    ncols = 4
    nrows = int(math.ceil(n / ncols))
    axes = fig.subplots(nrows, ncols)
    axes = np.array(axes).reshape(-1)
    for i, cand in enumerate(candidates):
        ax = axes[i]
        if cand["alpha_s"] is None:
            # No feasible alpha: draw a stub telling the user why.
            ax.set_facecolor("#fbe9e7")
            ax.text(0.5, 0.5, f"INFEASIBLE\n{cand['reason']}",
                    ha="center", va="center",
                    transform=ax.transAxes)
            ax.set_xticks([]); ax.set_yticks([])
        else:
            draw_arc_side_view(
                ax, cand["c_s"], cand["c_g"], cand["alpha_s"],
                m, planner.robot_radius, planner.leg_length,
                planner.arc_max_step,
                min_clearance_gate=planner.min_clearance_gate,
            )
            ax.set_ylim(-0.05, PILLAR_DISPLAY_H + 0.4)
        marker = "  <-- CHOSEN" if cand["cell"] == chosen_next else ""
        ax.set_xlabel(f"cell {cand['cell']}{marker}")
    for j in range(len(candidates), len(axes)):
        axes[j].axis("off")


def main() -> int:
    m = build_map()

    planner = make_planner(m)
    path = planner.plan()
    if not path:
        print("No path found - the pillar geometry may fully block the corridor.")
        return 1

    print(f"Path with {len(path)} waypoints:")
    for p in path:
        print(f"  ({p[0]:.2f}, {p[1]:.2f})  z={m.get_elevation(*p):.2f}")

    path_cells = path_cells_from(planner, path)
    chosen_cell = choose_takeoff_cell(planner, path_cells)
    print(f"\nHighlighting takeoff cell: {chosen_cell} "
          f"(world = {m.grid_to_world(*chosen_cell)})")

    candidates = enumerate_ring_candidates(planner, chosen_cell)
    idx = path_cells.index(chosen_cell)
    chosen_next = path_cells[idx + 1] if idx + 1 < len(path_cells) else None
    n_acc = sum(1 for c in candidates if c["accepted"])
    n_rej = sum(1 for c in candidates if not c["accepted"])
    print(f"Ring candidates: total={len(candidates)}  ACCEPT={n_acc}  REJECT={n_rej}")
    for c in candidates:
        tag = "CHOSEN" if c["cell"] == chosen_next else (
            "ACCEPT" if c["accepted"] else "REJECT"
        )
        print(f"  cell {str(c['cell']):>10}  X={c['X']:.2f}  Z={c['Z']:+.2f}  {tag}"
              f"  ({c['reason']})")

    # --- top-down figure ---
    fig_top = plt.figure(figsize=(10, 7))
    ax_top = fig_top.add_subplot(1, 1, 1)
    draw_topdown(fig_top, ax_top, planner, path, chosen_cell, candidates)
    fig_top.tight_layout()
    top_path = out_path("planner_reroute_topdown.png")
    fig_top.savefig(top_path, dpi=110)
    print(f"\nSaved: {top_path}")

    # --- side-view grid ---
    ncols = 4
    nrows = int(math.ceil(len(candidates) / ncols))
    fig_side = plt.figure(figsize=(3.4 * ncols, 2.4 * nrows))
    draw_candidate_sideviews(fig_side, planner, candidates, chosen_cell, chosen_next)
    fig_side.suptitle(
        f"Ring candidates from takeoff cell {chosen_cell}: "
        f"per-candidate arc vs terrain profile", wrap=True
    )
    fig_side.tight_layout(rect=(0, 0, 1, 0.96))
    side_path = out_path("planner_reroute_sideview.png")
    fig_side.savefig(side_path, dpi=110)
    print(f"Saved: {side_path}")

    if matplotlib.get_backend().lower() != "agg":
        plt.show()

    # Fail loudly if the demo didn't actually show any rejections (the
    # whole point of the demo is to show the planner rejecting collisions).
    return 0 if n_rej > 0 else 2


if __name__ == "__main__":
    sys.exit(main())

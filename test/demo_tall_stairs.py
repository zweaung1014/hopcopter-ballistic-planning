"""Tall-stairs robustness demo: ballistic planner climbs to the top platform.

Goal is placed on the 1.8 m top platform.  The robot must ascend three
0.6 m risers.  At certain takeoff positions the arc would clip a riser face
(negative clearance); the ballistic planner rejects those candidates and
adjusts its takeoff position.  The baseline planner (clearance OFF) charges
through, leaving arcs that intersect the terrain.

Figures produced
----------------
  test/stairs_topdown.png          — top-down: baseline vs ballistic paths,
                                     ring candidates (accept/reject) at the
                                     most-constrained ballistic waypoint
  test/stairs_ring_candidates.png  — side-view panel grid for every
                                     ballistic waypoint that has ≥1 rejection
  test/stairs_arcs.png             — per-hop side-view strip (both planners)

Run:
    python test/demo_tall_stairs.py
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
from hopping_astar_planner import (
    HoppingAStarPlanner,
    feasible_alpha_interval,
    min_clearance,
)
from map2d5 import Map2D5
from maps.tall_stairs import (
    build as build_map,
    STEP1_Z, STEP2_Z, TOP_Z,
    STEP1_XMIN, STEP2_XMIN, TOP_XMIN,
)
from visualizer import Visualizer, draw_arc_side_view


# ---------------------------------------------------------------------------
# Scenario knobs
# ---------------------------------------------------------------------------
START      = (0.5, 2.5)
GOAL       = (4.0, 2.5)   # on top platform (z = 1.2 m)
HOP_RADIUS = 1.5
N_ANGLES   = 16
V_MAX      = 6.0

# Step-height reference lines used in arc plots
STEP_ZS = [STEP1_Z, STEP2_Z, TOP_Z]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_planner(m: Map2D5, disable_clearance: bool) -> HoppingAStarPlanner:
    return HoppingAStarPlanner(
        map_env=m,
        start=START,
        goal=GOAL,
        hop_radius=HOP_RADIUS,
        n_angles=N_ANGLES,
        max_jump_height=config.MAX_JUMP_HEIGHT,
        alpha_uphill=config.ALPHA_UPHILL,
        alpha_downhill=config.ALPHA_DOWNHILL,
        g=config.G_ACCEL,
        V_max=V_MAX,
        robot_radius=config.ROBOT_RADIUS,
        clearance_margin=config.CLEARANCE_MARGIN,
        clearance_weight=config.CLEARANCE_WEIGHT,
        arc_max_step=config.ARC_SAMPLE_MAX_STEP,
        arc_endpoint_epsilon=config.ARC_ENDPOINT_EPSILON,
        obstacle_wall_extra=config.OBSTACLE_WALL_EXTRA,
        disable_clearance=disable_clearance,
    )


def diagnose_edge(
    planner: HoppingAStarPlanner,
    m: Map2D5,
    p0: tuple[float, float],
    p1: tuple[float, float],
) -> dict:
    z0 = float(m.get_elevation(*p0))
    z1 = float(m.get_elevation(*p1))
    X  = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    Z  = z1 - z0
    iv = feasible_alpha_interval(X, Z, V_MAX, config.G_ACCEL)
    if iv is None:
        return {"feasible": False, "X": X, "Z": Z, "alpha_s": None,
                "mc": -math.inf, "z0": z0, "z1": z1}
    a  = 0.5 * (iv[0] + iv[1])
    mc = min_clearance(
        (p0[0], p0[1], z0), (p1[0], p1[1], z1), a,
        m, config.G_ACCEL, config.ROBOT_RADIUS,
        config.ARC_ENDPOINT_EPSILON, config.ARC_SAMPLE_MAX_STEP,
        planner._obstacle_fill,
    )
    return {"feasible": True, "X": X, "Z": Z, "alpha_s": a, "mc": mc,
            "z0": z0, "z1": z1}


def enumerate_ring_candidates(
    planner: HoppingAStarPlanner,
    cell: tuple[int, int],
) -> list[dict]:
    """All 16 full-radius ring candidates from `cell`, with accept/reject verdict.

    Scans only the full hop_radius (not inward), matching the original ring
    visualisation.  Out-of-bounds and physics-infeasible candidates are
    included so their red arrows appear in the top-down figure.
    """
    m   = planner.map_env
    px, py = m.grid_to_world(*cell)
    pz  = float(m.grid[cell[0], cell[1]])
    obs = planner._obstacle_fill

    seen: set = {cell}
    out: list[dict] = []
    for dx, dy in planner._hop_dirs:
        tx = px + planner.hop_radius * dx
        ty = py + planner.hop_radius * dy
        if not m.is_within_bounds(tx, ty):
            out.append({
                "cell": None, "c_s": (px, py, pz), "c_g": None,
                "X": None, "Z": None, "r": planner.hop_radius,
                "alpha_s": None, "mc": None,
                "accepted": False, "reason": "out of bounds",
            })
            continue

        nb = m.world_to_grid(tx, ty)
        if nb in seen:
            continue
        seen.add(nb)

        nx, ny = m.grid_to_world(*nb)
        nz = float(m.grid[nb[0], nb[1]])
        X  = math.hypot(nx - px, ny - py)
        Z  = nz - pz
        c_s = (px, py, pz)
        c_g = (nx, ny, nz)

        entry: dict = {
            "cell": nb, "c_s": c_s, "c_g": c_g,
            "X": X, "Z": Z, "r": planner.hop_radius,
            "alpha_s": None, "mc": None,
            "accepted": False, "reason": "",
        }
        if nz == Map2D5.OBSTACLE:
            entry["reason"] = "OBSTACLE landing"
            out.append(entry)
            continue
        iv = feasible_alpha_interval(X, Z, planner.V_max, planner.g)
        if iv is None:
            entry["reason"] = "infeasible (V_max / geometry)"
            out.append(entry)
            continue
        a  = 0.5 * (iv[0] + iv[1])
        mc = min_clearance(
            c_s, c_g, a, m, planner.g, planner.robot_radius,
            planner.arc_endpoint_epsilon, planner.arc_max_step, obs,
        )
        entry["alpha_s"] = a
        entry["mc"]      = mc
        if mc < 0:
            entry["reason"] = f"arc clips terrain  mc={mc:+.3f} m"
        else:
            entry["accepted"] = True
            entry["reason"]   = f"clears  mc={mc:+.3f} m"
        out.append(entry)

    return out


def _n_clearance_rejects(cands: list[dict]) -> int:
    """Count candidates rejected specifically due to arc-terrain clearance (mc < 0)."""
    return sum(1 for c in cands if c["mc"] is not None and c["mc"] < 0)


def find_interesting_cells(
    planner: HoppingAStarPlanner,
    path_cells: list[tuple[int, int]],
) -> list[tuple[tuple[int, int], list[dict]]]:
    """Return (cell, candidates) pairs where ≥1 candidate is rejected
    (any reason: out-of-bounds, infeasible physics, or mc < 0)."""
    results = []
    for cell in path_cells[:-1]:   # skip goal (no outgoing hops)
        cands = enumerate_ring_candidates(planner, cell)
        if any(not c["accepted"] for c in cands):
            results.append((cell, cands))
    return results


def path_cells_of(planner: HoppingAStarPlanner, path) -> list[tuple[int, int]]:
    return [planner.map_env.world_to_grid(x, y) for x, y in path]


# ---------------------------------------------------------------------------
# Figure 1: top-down path comparison + ring arrows at the most-constrained node
# ---------------------------------------------------------------------------

def draw_topdown(
    m: Map2D5,
    path_base: list,
    path_ball: list,
    diags_base: list,
    planner_ball: HoppingAStarPlanner,
    interesting: list,
    save_path: str,
) -> None:
    fig, ax = plt.subplots(figsize=(11, 9))
    vis = Visualizer.__new__(Visualizer)
    vis.map_env = m; vis.fig = fig; vis.ax = ax
    ax.set_xlim(0, m.size_x); ax.set_ylim(0, m.size_y)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    vis.draw_map()

    # Step boundary lines
    for x_bnd, label, col in [
        (STEP1_XMIN, f"riser 1\n(z=0→{STEP1_Z})", "#e65100"),
        (STEP2_XMIN, f"riser 2\n(z={STEP1_Z}→{STEP2_Z})", "#bf360c"),
        (TOP_XMIN,   f"riser 3\n(z={STEP2_Z}→{TOP_Z})", "#7f0000"),
    ]:
        ax.axvline(x_bnd, color=col, linewidth=1.5, linestyle="--",
                   alpha=0.75, zorder=4)
        ax.text(x_bnd + 0.03, 0.15, label, color=col, fontsize=7,
                va="bottom", zorder=5)

    # Baseline path (dashed orange)
    xs_b = [p[0] for p in path_base]
    ys_b = [p[1] for p in path_base]
    ax.plot(xs_b, ys_b, color="#e65100", linewidth=2.0,
            linestyle="--", zorder=6, label="Baseline (clearance OFF)")
    ax.plot(xs_b, ys_b, "x", color="#e65100", markersize=9, zorder=7)
    for i, (p, d) in enumerate(zip(path_base, [None] + list(diags_base))):
        z = m.get_elevation(*p)
        mc_str = f"\nmc={d['mc']:+.3f}" if d and d["feasible"] else ""
        ax.annotate(f"B{i}\nz={z:.1f}{mc_str}", p,
                    xytext=(-18, -16), textcoords="offset points",
                    fontsize=6, color="#e65100", zorder=8)

    # Highlight baseline hops below clearance margin (orange) or clipping (red)
    clearance_margin = config.CLEARANCE_MARGIN
    _low_margin_labelled = False
    for i, d in enumerate(diags_base):
        if not d["feasible"]:
            continue
        p0, p1 = path_base[i], path_base[i + 1]
        if d["mc"] < 0.0:
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]],
                    color="#d50000", linewidth=8, alpha=0.55,
                    zorder=6.5, solid_capstyle="round")
            ax.annotate(
                f"CLIPS\nmc={d['mc']:+.3f}m",
                (0.5*(p0[0]+p1[0]), 0.5*(p0[1]+p1[1])),
                xytext=(0, 26), textcoords="offset points",
                ha="center", fontsize=8, color="#b71c1c", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.2", fc="white",
                          ec="#b71c1c", lw=1.1), zorder=11,
            )
        elif d["mc"] < clearance_margin:
            penalty = config.CLEARANCE_WEIGHT * (clearance_margin - d["mc"])
            # Only add legend label once
            lbl = (f"mc < {clearance_margin} m → clearance penalty"
                   if not _low_margin_labelled else "_nolegend_")
            _low_margin_labelled = True
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]],
                    color="#ff6f00", linewidth=7, alpha=0.45,
                    zorder=6.4, solid_capstyle="round", label=lbl)
            ax.annotate(
                f"mc={d['mc']:+.3f}m\n(pen≈+{penalty:.2f})",
                (0.5*(p0[0]+p1[0]), 0.5*(p0[1]+p1[1])),
                xytext=(0, 22), textcoords="offset points",
                ha="center", fontsize=7, color="#e65100", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.2", fc="white",
                          ec="#ff6f00", lw=1.1), zorder=11,
            )

    # Ballistic path (teal solid)
    xs_a = [p[0] for p in path_ball]
    ys_a = [p[1] for p in path_ball]
    ax.plot(xs_a, ys_a, color="#00695c", linewidth=2.2,
            zorder=8, label="Ballistic (clearance ON)")
    ax.plot(xs_a, ys_a, "o", color="#00695c", markersize=7, zorder=9)
    for i, p in enumerate(path_ball):
        z = m.get_elevation(*p)
        ax.annotate(f"A{i}\nz={z:.1f}", p,
                    xytext=(5, 5), textcoords="offset points",
                    fontsize=6, color="#00695c", zorder=10)

    # Ring candidates at the most-constrained ballistic waypoint
    if interesting:
        # pick cell with the most total rejections (infeasible + OOB + mc<0)
        chosen_cell, cands = max(
            interesting,
            key=lambda t: sum(1 for c in t[1] if not c["accepted"]),
        )
        px, py = m.grid_to_world(*chosen_cell)

        # hop circle
        ax.add_patch(mpatches.Circle(
            (px, py), radius=HOP_RADIUS, fill=False,
            edgecolor="deepskyblue", linewidth=1.3, linestyle="--",
            alpha=0.9, zorder=5,
        ))
        ax.plot(px, py, "s", color="deepskyblue", markersize=10,
                zorder=8, label=f"highlighted takeoff  {chosen_cell}")

        # find the actual chosen next cell for this waypoint
        try:
            idx = path_cells_of(planner_ball, path_ball).index(chosen_cell)
            chosen_next = path_cells_of(planner_ball, path_ball)[idx + 1]
        except (ValueError, IndexError):
            chosen_next = None

        n_acc = n_rej = 0
        for cand in cands:
            if cand["cell"] is None:
                # OOB — no arrow, but count as reject
                n_rej += 1
                continue
            lx, ly = m.grid_to_world(*cand["cell"])
            if cand["cell"] == chosen_next:
                colour, lw, zo = "#1b5e20", 3.5, 9
            elif cand["accepted"]:
                colour, lw, zo = "#66bb6a", 1.1, 4
                n_acc += 1
            else:
                colour, lw, zo = "#c62828", 1.4, 4
                n_rej += 1
            ax.annotate(
                "", xy=(lx, ly), xytext=(px, py),
                arrowprops=dict(arrowstyle="->", color=colour,
                                lw=lw, alpha=0.9),
                zorder=zo,
            )

        # legend patches
        extra_h = [
            mpatches.Patch(color="#66bb6a", label=f"ACCEPT ({n_acc})"),
            mpatches.Patch(color="#c62828", label=f"REJECT ({n_rej}) OOB/infeasible/mc<0"),
            mpatches.Patch(color="#1b5e20", label="CHOSEN"),
            mpatches.Patch(color="#d50000", alpha=0.55,
                           label="baseline hop clips terrain (mc < 0)"),
        ]
    else:
        extra_h = []

    ax.plot(START[0], START[1], "go", markersize=11, zorder=10, label="start")
    ax.plot(GOAL[0], GOAL[1], "r*", markersize=15, zorder=10,
            label=f"goal  z={m.get_elevation(*GOAL):.1f} m")

    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles + extra_h, labels + [h.get_label() for h in extra_h],
              loc="upper left", fontsize=7)

    ax.set_title(
        f"tall_stairs: ballistic planner climbs to z={TOP_Z:.1f} m platform\n"
        f"Baseline's low-mc hops (orange) incur clearance penalty → ballistic chose safer step-by-step route",
        fontsize=10,
    )
    fig.tight_layout()
    fig.savefig(save_path, dpi=110)
    print(f"Saved: {save_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 2: ring-candidate side-view panels for every interesting ballistic node
# ---------------------------------------------------------------------------

def draw_ring_panels(
    m: Map2D5,
    planner_ball: HoppingAStarPlanner,
    path_ball: list,
    interesting: list,
    save_path: str,
) -> None:
    if not interesting:
        print("No rejection cells found — skipping ring-panels figure.")
        return

    path_cells = path_cells_of(planner_ball, path_ball)
    obs_fill   = planner_ball._obstacle_fill
    NCOLS      = 4

    # Build a list of (header_label, cell, cands, chosen_next) rows
    rows_data: list[tuple[str, tuple, list, tuple | None]] = []
    for cell, cands in interesting:
        try:
            idx         = path_cells.index(cell)
            chosen_next = path_cells[idx + 1] if idx + 1 < len(path_cells) else None
        except ValueError:
            chosen_next = None
        pz  = float(m.grid[cell[0], cell[1]])
        px, py = m.grid_to_world(*cell)
        n_acc = sum(1 for c in cands if c["accepted"])
        n_rej = sum(1 for c in cands if not c["accepted"])
        label = (f"Takeoff  ({px:.1f}, {py:.1f})  z={pz:.1f} m"
                 f"   {n_acc} ACCEPT / {n_rej} REJECT")
        rows_data.append((label, cell, cands, chosen_next))

    # One sub-block of panels per interesting cell, separated by thin hlines
    total_panels = sum(len(rd[2]) for rd in rows_data)
    nrows_total  = sum(math.ceil(len(rd[2]) / NCOLS) for rd in rows_data)
    ymax_arc     = TOP_Z + 0.5

    fig, all_axes = plt.subplots(
        nrows_total, NCOLS,
        figsize=(3.6 * NCOLS, 2.8 * nrows_total),
        squeeze=False,
    )

    panel_row = 0
    for row_label, cell, cands, chosen_next in rows_data:
        nrows_block = math.ceil(len(cands) / NCOLS)
        # Label the first panel of this block
        all_axes[panel_row, 0].set_title(row_label, fontsize=8,
                                         loc="left", pad=3,
                                         color="#1a237e", fontweight="bold")

        for k, cand in enumerate(cands):
            ax = all_axes[panel_row + k // NCOLS, k % NCOLS]

            if cand["alpha_s"] is None or cand["c_g"] is None:
                ax.set_facecolor("#fbe9e7")
                ax.text(0.5, 0.5,
                        f"INFEASIBLE\n{cand['reason']}",
                        ha="center", va="center", fontsize=7,
                        transform=ax.transAxes)
                ax.set_xticks([]); ax.set_yticks([])
            else:
                draw_arc_side_view(
                    ax, cand["c_s"], cand["c_g"], cand["alpha_s"],
                    m, planner_ball.g, planner_ball.robot_radius,
                    obs_fill, planner_ball.arc_endpoint_epsilon,
                    planner_ball.arc_max_step,
                )
                ax.set_ylim(-0.1, ymax_arc)
                # Step reference lines
                for sz, sc in [(STEP1_Z, "#e65100"), (STEP2_Z, "#bf360c"),
                               (TOP_Z, "#7f0000")]:
                    ax.axhline(sz, color=sc, linewidth=0.8,
                               linestyle=":", alpha=0.7, zorder=3)

            tag = ""
            r_str = f"  r={cand.get('r', '?'):.2f}" if cand.get("r") is not None else ""
            if cand["cell"] == chosen_next:
                tag = "  ← CHOSEN"
                ax.set_facecolor("#f1f8e9")
            ax.set_xlabel(
                f"cell {cand['cell']}{r_str}{tag}",
                fontsize=7,
                color=("#1b5e20" if cand["cell"] == chosen_next
                       else ("#c62828" if not cand["accepted"] else "black")),
            )

        # blank unused panels in this block
        block_used = len(cands)
        for k in range(block_used, nrows_block * NCOLS):
            all_axes[panel_row + k // NCOLS, k % NCOLS].axis("off")

        panel_row += nrows_block

    fig.suptitle(
        "Ring-candidate side views at every ballistic waypoint with ≥1 rejection\n"
        "Green arc = ACCEPT · Red arc = REJECT · Green background = CHOSEN · "
        "Dotted lines = step elevations",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(save_path, dpi=100)
    print(f"Saved: {save_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 3: per-hop side-view strip (both planners)
# ---------------------------------------------------------------------------

def draw_arc_strip(
    m: Map2D5,
    planner_ball: HoppingAStarPlanner,
    path_base: list,
    diags_base: list,
    path_ball: list,
    diags_ball: list,
    save_path: str,
) -> None:
    n_base = len(path_base) - 1
    n_ball = len(path_ball) - 1
    ncols  = max(n_base, n_ball)
    fig, axes = plt.subplots(2, ncols, figsize=(3.8 * ncols, 6.0), squeeze=False)

    obs_fill = planner_ball._obstacle_fill
    ymax     = TOP_Z + 0.45

    for row_idx, (row_label, path, diags) in enumerate([
        ("Baseline", path_base, diags_base),
        ("Ballistic", path_ball, diags_ball),
    ]):
        for i in range(ncols):
            ax = axes[row_idx, i]
            if i >= len(path) - 1:
                ax.axis("off")
                continue
            d = diags[i]
            if not d["feasible"]:
                ax.set_facecolor("#fbe9e7")
                ax.text(0.5, 0.5, "INFEASIBLE", ha="center", va="center",
                        fontsize=9, transform=ax.transAxes)
                ax.set_xticks([]); ax.set_yticks([])
            else:
                p0, p1 = path[i], path[i + 1]
                draw_arc_side_view(
                    ax,
                    (p0[0], p0[1], d["z0"]), (p1[0], p1[1], d["z1"]),
                    d["alpha_s"], m,
                    config.G_ACCEL, config.ROBOT_RADIUS, obs_fill,
                    config.ARC_ENDPOINT_EPSILON, config.ARC_SAMPLE_MAX_STEP,
                )
                ax.set_ylim(-0.1, ymax)
                # Step reference lines
                for sz, sc in [(STEP1_Z, "#e65100"), (STEP2_Z, "#bf360c"),
                               (TOP_Z, "#7f0000")]:
                    ax.axhline(sz, color=sc, linewidth=0.9,
                               linestyle=":", alpha=0.8, zorder=4)

            if i == 0:
                ax.set_ylabel(f"{row_label}\nz (m)", fontsize=10)
            ax.set_xlabel(
                f"hop {i}: ({path[i][0]:.1f},{path[i][1]:.1f})"
                f" → ({path[i+1][0]:.1f},{path[i+1][1]:.1f})",
                fontsize=8,
            )

    fig.suptitle(
        "Per-hop side view — baseline (top) vs ballistic (bottom)\n"
        f"Dotted lines = step elevations  {STEP1_Z} / {STEP2_Z} / {TOP_Z} m · "
        "Red arc = clips terrain (mc<0) · Green = clears",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.91))
    fig.savefig(save_path, dpi=110)
    print(f"Saved: {save_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    m = build_map()

    planner_base = make_planner(m, disable_clearance=True)
    planner_ball = make_planner(m, disable_clearance=False)

    path_base = planner_base.plan()
    path_ball = planner_ball.plan()

    if path_base is None or path_ball is None:
        print(f"Baseline path:  {'FOUND' if path_base else 'NONE'}")
        print(f"Ballistic path: {'FOUND' if path_ball else 'NONE'}")
        return 1

    diags_base = [
        diagnose_edge(planner_ball, m, path_base[i], path_base[i + 1])
        for i in range(len(path_base) - 1)
    ]
    diags_ball = [
        diagnose_edge(planner_ball, m, path_ball[i], path_ball[i + 1])
        for i in range(len(path_ball) - 1)
    ]

    # --- console summary ---
    n_base_bad = sum(1 for d in diags_base if not d["feasible"] or d["mc"] < 0)
    n_ball_bad = sum(1 for d in diags_ball if not d["feasible"] or d["mc"] < 0)

    print("=" * 72)
    print(f"BASELINE   ({len(path_base)} waypoints, {len(path_base)-1} hops)")
    for i, p in enumerate(path_base):
        z = m.get_elevation(*p)
        note = ""
        if i > 0:
            d = diags_base[i - 1]
            note = (f"   incoming X={d['X']:.2f} Z={d['Z']:+.2f}"
                    f" α={math.degrees(d['alpha_s']):.1f}°"
                    f" mc={d['mc']:+.3f}" if d["feasible"] else "   INFEASIBLE")
        print(f"  B{i}: ({p[0]:.2f},{p[1]:.2f})  z={z:.2f}{note}")
    print(f"  → {n_base_bad} hop(s) clip the terrain")

    print("-" * 72)
    print(f"BALLISTIC  ({len(path_ball)} waypoints, {len(path_ball)-1} hops)")
    for i, p in enumerate(path_ball):
        z = m.get_elevation(*p)
        note = ""
        if i > 0:
            d = diags_ball[i - 1]
            note = (f"   incoming X={d['X']:.2f} Z={d['Z']:+.2f}"
                    f" α={math.degrees(d['alpha_s']):.1f}°"
                    f" mc={d['mc']:+.3f}" if d["feasible"] else "   INFEASIBLE")
        print(f"  A{i}: ({p[0]:.2f},{p[1]:.2f})  z={z:.2f}{note}")
    print(f"  → {n_ball_bad} hop(s) clip the terrain")
    print("=" * 72)

    # --- find interesting cells for ring visualization ---
    interesting = find_interesting_cells(planner_ball, path_cells_of(planner_ball, path_ball))
    print(f"\nBallistic path nodes with ≥1 ring rejection: {len(interesting)}")
    for cell, cands in interesting:
        px, py = m.grid_to_world(*cell)
        pz = m.get_elevation(px, py)
        n_acc = sum(1 for c in cands if c["accepted"])
        n_rej = sum(1 for c in cands if not c["accepted"])
        n_clr = _n_clearance_rejects(cands)
        n_oob = sum(1 for c in cands if c["cell"] is None)
        n_inf = sum(1 for c in cands if not c["accepted"] and c["cell"] is not None
                    and c["mc"] is None and c["reason"].startswith("infeasible"))
        print(f"  ({px:.1f},{py:.1f}) z={pz:.1f}  ACCEPT={n_acc}  REJECT={n_rej}"
              f"  (mc<0:{n_clr}  infeasible:{n_inf}  OOB:{n_oob})")
        for c in cands:
            if not c["accepted"] and c["mc"] is not None and c["mc"] < 0:
                if c["cell"] is not None:
                    lx, ly = m.grid_to_world(*c["cell"])
                    print(f"    REJECT mc<0 → ({lx:.1f},{ly:.1f})  {c['reason']}")

    out_dir = os.path.dirname(os.path.abspath(__file__))

    draw_topdown(
        m, path_base, path_ball, diags_base, planner_ball, interesting,
        os.path.join(out_dir, "stairs_topdown.png"),
    )
    draw_ring_panels(
        m, planner_ball, path_ball, interesting,
        os.path.join(out_dir, "stairs_ring_candidates.png"),
    )
    draw_arc_strip(
        m, planner_ball,
        path_base, diags_base,
        path_ball, diags_ball,
        os.path.join(out_dir, "stairs_arcs.png"),
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

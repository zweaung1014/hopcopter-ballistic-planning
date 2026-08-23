"""Curbed-stairs demo: rejection driven by the *clearance* gate, not by physics.

`test/demo_tall_stairs.py` shows the planner climbing plain stairs, but nothing
there is rejected on *clearance*: a hop onto a plain riser passes well above the
terrain, so what prunes the search is the stance check and the physics gate.
This demo closes that gap.

Each tread here carries a curb standing `CURB_RISE` proud of the tread behind
it, so the curb is a genuine local maximum sitting under the descending part of
an incoming arc. A hop that approaches too flat catches it and is rejected on
clearance.

Every rejected ring candidate is labelled with the gate that stopped it —
`bounds`, `obstacle`, `stance`, `physics` or `clearance` — which is the whole
point of the figure: it lets you say which check did the work rather than
assuming.

Figures produced
----------------
  test/stairs_curb_topdown.png         — top-down A/B overlay
  test/stairs_curb_arcs.png            — per-hop side-view strip (both planners)
  test/stairs_curb_ring_candidates.png — ring candidates with per-gate verdicts

Run:
    python test/demo_stairs_curb.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import matplotlib.patches as mpatches

import config
from demo_common import (
    SLIDE_FIGSIZE, TOPDOWN_FIGSIZE, arcs_figsize,
    PRESENTATION_DPI, TITLE_FS, LABEL_FS, ANNOT_FS,
    C_BASE, C_BALL, C_COLLIDE, C_ACCEPT, C_REJECT, C_CHOSEN,
    make_planner, diagnose_path, n_bad_hops,
    draw_topdown_base, draw_ab_paths, add_collision_legend,
    enumerate_ring_candidates, find_interesting_cells, gate_counts,
    path_cells_of, print_ab_summary, param_caption, save, out_path,
)
from maps.stairs_with_curb import (
    build as build_map,
    TREAD1_Z, TREAD2_Z, TOP_Z,
    CURB1_Z, CURB2_Z, CURB3_Z, CURB_RISE,
    CURB1_XMIN, CURB2_XMIN, CURB3_XMIN,
)
from visualizer import draw_arc_side_view


START = (0.5, 2.5)
GOAL  = (4.5, 2.5)   # on the top platform (z = 1.20 m)

CURBS = [
    (CURB1_XMIN, CURB1_Z, TREAD1_Z, "#e65100"),
    (CURB2_XMIN, CURB2_Z, TREAD2_Z, "#bf360c"),
    (CURB3_XMIN, CURB3_Z, TOP_Z,    "#7f0000"),
]

# Human-readable names for the gate that rejected a candidate.
GATE_LABEL = {
    "bounds":    "REJECT · off map",
    "obstacle":  "REJECT · OBSTACLE cell",
    "stance":    "REJECT · body cannot stand",
    "physics":   "REJECT · physics (V_max)",
    "clearance": "REJECT · clearance gate",
    "":          "ACCEPT",
}


# ---------------------------------------------------------------------------
# Figure 1: top-down A/B overlay
# ---------------------------------------------------------------------------

def draw_topdown(m, path_base, path_ball, diags_base, save_path):
    fig, ax = plt.subplots(figsize=TOPDOWN_FIGSIZE)
    draw_topdown_base(m, fig, ax)

    # Stagger the three curb labels vertically: at presentation type size they
    # are wider than the 0.2 m gap between the curbs they annotate.
    for k, (x_bnd, cz, tz, col) in enumerate(CURBS):
        ax.axvline(x_bnd, color=col, linewidth=1.6, linestyle="--",
                   alpha=0.8, zorder=4)
        ax.text(x_bnd + 0.04, 0.15 + 0.42 * k, f"curb {cz:.2f}",
                color=col, va="bottom", zorder=5,
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.7))

    draw_ab_paths(ax, path_base, path_ball, diags_base)

    for i, p in enumerate(path_base):
        ax.annotate(f"B{i}\nz={m.get_elevation(*p):.2f}", p,
                    xytext=(-20, -20), textcoords="offset points", color=C_BASE, zorder=10)
    for i, p in enumerate(path_ball):
        ax.annotate(f"A{i}\nz={m.get_elevation(*p):.2f}", p,
                    xytext=(6, 8), textcoords="offset points", color=C_BALL, zorder=10)

    ax.plot(*START, "go", markersize=11, zorder=11, label="start  z=0.00")
    ax.plot(*GOAL, "r*", markersize=15, zorder=11,
            label=f"goal  z={m.get_elevation(*GOAL):.2f} (top)")
    add_collision_legend(ax)

    ax.set_title(
        f"stairs_with_curb — each tread edge stands {CURB_RISE:.2f} m proud\n"
        "curbs sit under the descending limb, so flat\n"
        "approaches are rejected on clearance", wrap=True
    )
    fig.tight_layout()
    save(fig, save_path)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 2: per-hop side-view strip
# ---------------------------------------------------------------------------

def draw_arc_strip(m, planner_ball, path_base, diags_base, path_ball, diags_ball,
                   save_path):
    ncols = max(len(path_base), len(path_ball)) - 1
    fig, axes = plt.subplots(2, ncols, figsize=arcs_figsize(ncols), squeeze=False)
    ymax = CURB3_Z + 0.45

    for row, (row_label, path, diags) in enumerate([
        ("Baseline", path_base, diags_base),
        ("Ballistic", path_ball, diags_ball),
    ]):
        for i in range(ncols):
            ax = axes[row, i]
            if i >= len(path) - 1:
                ax.axis("off")
                continue
            d = diags[i]
            if not d["feasible"]:
                ax.set_facecolor("#fbe9e7")
                ax.text(0.5, 0.5, "INFEASIBLE\n(physics gate)", ha="center",
                        va="center", transform=ax.transAxes)
                ax.set_xticks([]); ax.set_yticks([])
            else:
                p0, p1 = path[i], path[i + 1]
                draw_arc_side_view(
                    ax, (p0[0], p0[1], d["z0"]), (p1[0], p1[1], d["z1"]),
                    d["alpha_s"], m, config.ROBOT_RADIUS, config.LEG_LENGTH, config.ARC_SAMPLE_MAX_STEP,
                    min_clearance_gate=config.MIN_CLEARANCE,
                    steep_grade=config.STEEP_INFLATE_GRADE,
                )
                ax.set_ylim(-0.1, ymax)
                for _, cz, _, col in CURBS:
                    ax.axhline(cz, color=col, linewidth=0.9, linestyle=":",
                               alpha=0.8, zorder=4)

            if i == 0:
                ax.set_ylabel(f"{row_label}\nz (m)")
            # Terse: at 5 panels across a slide each is under 2.7 in wide.
            ax.set_xlabel(f"hop {i}   {path[i][0]:.1f}→{path[i+1][0]:.1f} m")
            ax.xaxis.set_major_locator(MaxNLocator(nbins=3))
            ax.yaxis.set_major_locator(MaxNLocator(nbins=4))

    fig.suptitle(
        "Per-hop side view — baseline (top) vs ballistic (bottom)\n"
        f"curbs at {CURB1_Z:.2f} / {CURB2_Z:.2f} / {CURB3_Z:.2f} m  ·  "
        f"red = below the {config.MIN_CLEARANCE} m gate", wrap=True
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    save(fig, save_path)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 3: ring candidates, labelled by the gate that rejected them
# ---------------------------------------------------------------------------

def draw_ring_panels(m, planner_ball, path_ball, interesting, save_path):
    if not interesting:
        print("No rejection cells found — skipping ring-panel figure.")
        return

    path_cells = path_cells_of(planner_ball, path_ball)
    NCOLS = 4

    rows_data = []
    for cell, cands in interesting:
        try:
            idx = path_cells.index(cell)
            chosen_next = path_cells[idx + 1] if idx + 1 < len(path_cells) else None
        except ValueError:
            chosen_next = None
        px, py = m.grid_to_world(*cell)
        pz = float(m.grid[cell[0], cell[1]])
        t = gate_counts(cands)
        label = (f"Takeoff ({px:.1f}, {py:.1f})  z={pz:.2f} m   "
                 f"ACCEPT {t['accept']} · clearance {t['clearance']} · "
                 f"physics {t['physics']} · off-map {t['bounds']}")
        rows_data.append((label, cell, cands, chosen_next))

    nrows_total = sum(math.ceil(len(rd[2]) / NCOLS) for rd in rows_data)
    fig, all_axes = plt.subplots(
        nrows_total, NCOLS,
        figsize=(3.7 * NCOLS, 2.9 * nrows_total), squeeze=False,
    )

    panel_row = 0
    for row_label, cell, cands, chosen_next in rows_data:
        nrows_block = math.ceil(len(cands) / NCOLS)
        # Place the block header above the row in axes coordinates rather than
        # via set_title(loc="left"): draw_arc_side_view sets its own centered
        # title, and matplotlib draws left- and center-aligned titles at the
        # same height, so the two would overlap.
        all_axes[panel_row, 0].text(
            0.0, 1.34, row_label, transform=all_axes[panel_row, 0].transAxes, color="#1a237e", fontweight="bold",
            va="bottom", ha="left",
        )

        for k, cand in enumerate(cands):
            ax = all_axes[panel_row + k // NCOLS, k % NCOLS]

            if cand["alpha_s"] is None or cand["c_g"] is None:
                ax.set_facecolor("#fbe9e7")
                ax.text(0.5, 0.5, f"{GATE_LABEL[cand['gate']]}\n{cand['reason']}",
                        ha="center", va="center",
                        transform=ax.transAxes)
                ax.set_xticks([]); ax.set_yticks([])
            else:
                draw_arc_side_view(
                    ax, cand["c_s"], cand["c_g"], cand["alpha_s"], m,
                    planner_ball.robot_radius, planner_ball.leg_length, planner_ball.arc_max_step,
                    min_clearance_gate=planner_ball.min_clearance_gate,
                )
                ax.set_ylim(-0.1, CURB3_Z + 0.45)
                for _, cz, _, col in CURBS:
                    ax.axhline(cz, color=col, linewidth=0.8, linestyle=":",
                               alpha=0.7, zorder=3)
                if cand["gate"] == "clearance":
                    ax.set_facecolor("#fff3e0")

            tag = ""
            if cand["cell"] == chosen_next:
                tag = "  ← CHOSEN"
                ax.set_facecolor("#f1f8e9")
            colour = (C_CHOSEN if cand["cell"] == chosen_next
                      else (C_REJECT if not cand["accepted"] else "black"))
            ax.set_xlabel(f"{GATE_LABEL[cand['gate']]}{tag}", color=colour)

        for k in range(len(cands), nrows_block * NCOLS):
            all_axes[panel_row + k // NCOLS, k % NCOLS].axis("off")
        panel_row += nrows_block

    fig.suptitle(
        "Ring candidates at each ballistic waypoint, labelled by the gate that "
        "rejected them\n"
        "Cream background = rejected on CLEARANCE · "
        "Pink = rejected before the arc was even computed · "
        "Green = the candidate A* took", wrap=True
    )
    fig.tight_layout(rect=(0, 0, 1, 0.955), h_pad=3.2)
    save(fig, save_path, dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    print(param_caption())
    m = build_map()

    planner_base = make_planner(m, True, START, GOAL)
    planner_ball = make_planner(m, False, START, GOAL)
    path_base = planner_base.plan()
    path_ball = planner_ball.plan()

    if path_base is None or path_ball is None:
        print(f"Baseline path:  {'FOUND' if path_base else 'NONE'}")
        print(f"Ballistic path: {'FOUND' if path_ball else 'NONE'}")
        return 1

    diags_base = diagnose_path(planner_ball, m, path_base)
    diags_ball = diagnose_path(planner_ball, m, path_ball)
    n_base_bad, n_ball_bad = print_ab_summary(
        m, path_base, diags_base, path_ball, diags_ball,
    )

    # Look for clearance rejections along BOTH paths, not just the ballistic
    # one. That is where they live: the ballistic planner's waypoints are the
    # ones it picked *because* their ring candidates clear, so nothing is
    # rejected there. The baseline parks on the tread edges, and from those
    # cells most of the ring dies on clearance — which is the comparison the
    # figure is meant to make.
    cells = path_cells_of(planner_ball, path_ball)
    for cell in path_cells_of(planner_ball, path_base):
        if cell not in cells:
            cells.append(cell)
    interesting = find_interesting_cells(planner_ball, cells, gate="clearance")

    # The headline claim of this demo: clearance, not physics, does the rejecting.
    total = {"accept": 0, "bounds": 0, "obstacle": 0, "stance": 0,
             "physics": 0, "clearance": 0}
    for _, cands in interesting:
        for key, val in gate_counts(cands).items():
            total[key] += val

    print(f"\nRing candidates across {len(interesting)} ballistic waypoint(s):")
    print(f"  ACCEPT                 {total['accept']}")
    print(f"  REJECT · clearance     {total['clearance']}   ← arc clips terrain")
    print(f"  REJECT · stance        {total['stance']}   ← body cannot rest there")
    print(f"  REJECT · physics       {total['physics']}")
    print(f"  REJECT · off map       {total['bounds']}")
    print(f"  REJECT · OBSTACLE      {total['obstacle']}")

    if total["clearance"] == 0:
        print("\nWARNING: no candidate was rejected on clearance — this demo's "
              "premise does not hold. Re-run test/calibrate_geometry.py.")

    draw_topdown(m, path_base, path_ball, diags_base,
                 out_path("stairs_curb_topdown.png"))
    draw_arc_strip(m, planner_ball, path_base, diags_base, path_ball, diags_ball,
                   out_path("stairs_curb_arcs.png"))
    draw_ring_panels(m, planner_ball, path_ball, interesting,
                     out_path("stairs_curb_ring_candidates.png"))

    same = path_base == path_ball
    if same:
        print("\nWARNING: both planners produced the same path.")
    # Demand a genuine clearance rejection, not merely a path difference.
    ok = (not same and n_base_bad > 0 and n_ball_bad == 0
          and total["clearance"] > 0)
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())

"""Curbed-stairs demo: rejection driven by the *clearance* gate, not by physics.

`test/demo_tall_stairs.py` shows the planner climbing plain stairs, but its
rejections come from the physics feasibility gate — on a plain riser the
body-guard inside `min_clearance` skips every interior sample and returns `+inf`
(see the note in `maps/tall_stairs.py`).  This demo closes that gap.

Each tread here carries a curb standing 0.10 m proud of the tread behind it, so
the curb is a local maximum and its samples survive the body-guard.  A hop that
approaches too flat catches the curb on the descending part of its arc and is
rejected with a finite negative `mc`.

Every rejected ring candidate is labelled with the gate that stopped it —
`bounds`, `obstacle`, `physics` or `clearance` — which is the whole point of the
figure: it lets you say which check did the work rather than assuming.

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
import matplotlib.patches as mpatches

import config
from demo_common import (
    HOP_RADIUS, PRESENTATION_DPI, TITLE_FS, LABEL_FS, ANNOT_FS,
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
    "physics":   "REJECT · physics (V_max)",
    "clearance": "REJECT · clearance (mc<0)",
    "":          "ACCEPT",
}


# ---------------------------------------------------------------------------
# Figure 1: top-down A/B overlay
# ---------------------------------------------------------------------------

def draw_topdown(m, path_base, path_ball, diags_base, save_path):
    fig, ax = plt.subplots(figsize=(11, 9))
    draw_topdown_base(m, fig, ax)

    for x_bnd, cz, tz, col in CURBS:
        ax.axvline(x_bnd, color=col, linewidth=1.6, linestyle="--",
                   alpha=0.8, zorder=4)
        ax.text(x_bnd + 0.03, 0.12, f"curb z={cz}\n(tread {tz})",
                color=col, fontsize=ANNOT_FS - 1, va="bottom", zorder=5)

    draw_ab_paths(ax, path_base, path_ball, diags_base)

    for i, p in enumerate(path_base):
        ax.annotate(f"B{i}\nz={m.get_elevation(*p):.2f}", p,
                    xytext=(-20, -20), textcoords="offset points",
                    fontsize=ANNOT_FS - 1, color=C_BASE, zorder=10)
    for i, p in enumerate(path_ball):
        ax.annotate(f"A{i}\nz={m.get_elevation(*p):.2f}", p,
                    xytext=(6, 8), textcoords="offset points",
                    fontsize=ANNOT_FS - 1, color=C_BALL, zorder=10)

    ax.plot(*START, "go", markersize=11, zorder=11, label="start  z=0.00")
    ax.plot(*GOAL, "r*", markersize=15, zorder=11,
            label=f"goal  z={m.get_elevation(*GOAL):.2f} (top)")
    add_collision_legend(ax)

    ax.set_title(
        f"stairs_with_curb: each tread edge stands {CURB_RISE:.2f} m proud\n"
        "Curbs are local maxima, so the clearance gate survives the body-guard "
        "and rejects flat approaches\n"
        f"{param_caption()}",
        fontsize=TITLE_FS - 2,
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
    fig, axes = plt.subplots(2, ncols, figsize=(3.9 * ncols, 6.0), squeeze=False)
    obs_fill = planner_ball._obstacle_fill
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
                        va="center", fontsize=9, transform=ax.transAxes)
                ax.set_xticks([]); ax.set_yticks([])
            else:
                p0, p1 = path[i], path[i + 1]
                draw_arc_side_view(
                    ax, (p0[0], p0[1], d["z0"]), (p1[0], p1[1], d["z1"]),
                    d["alpha_s"], m, config.G_ACCEL, config.ROBOT_RADIUS,
                    obs_fill, config.ARC_ENDPOINT_EPSILON,
                    config.ARC_SAMPLE_MAX_STEP,
                )
                ax.set_ylim(-0.1, ymax)
                for _, cz, _, col in CURBS:
                    ax.axhline(cz, color=col, linewidth=0.9, linestyle=":",
                               alpha=0.8, zorder=4)

            if i == 0:
                ax.set_ylabel(f"{row_label}\nz (m)", fontsize=LABEL_FS)
            ax.set_xlabel(
                f"hop {i}: ({path[i][0]:.1f},{path[i][1]:.1f})"
                f" → ({path[i+1][0]:.1f},{path[i+1][1]:.1f})",
                fontsize=ANNOT_FS,
            )

    fig.suptitle(
        "Per-hop side view — baseline (top) vs ballistic (bottom)\n"
        f"Dotted: curb elevations {CURB1_Z} / {CURB2_Z} / {CURB3_Z} m · "
        "Red arc = clips a curb (mc<0) · Green = clears",
        fontsize=TITLE_FS - 2,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.90))
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
    obs_fill = planner_ball._obstacle_fill
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
            0.0, 1.34, row_label, transform=all_axes[panel_row, 0].transAxes,
            fontsize=ANNOT_FS + 1, color="#1a237e", fontweight="bold",
            va="bottom", ha="left",
        )

        for k, cand in enumerate(cands):
            ax = all_axes[panel_row + k // NCOLS, k % NCOLS]

            if cand["alpha_s"] is None or cand["c_g"] is None:
                ax.set_facecolor("#fbe9e7")
                ax.text(0.5, 0.5, f"{GATE_LABEL[cand['gate']]}\n{cand['reason']}",
                        ha="center", va="center", fontsize=ANNOT_FS,
                        transform=ax.transAxes)
                ax.set_xticks([]); ax.set_yticks([])
            else:
                draw_arc_side_view(
                    ax, cand["c_s"], cand["c_g"], cand["alpha_s"], m,
                    planner_ball.g, planner_ball.robot_radius, obs_fill,
                    planner_ball.arc_endpoint_epsilon, planner_ball.arc_max_step,
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
            ax.set_xlabel(f"{GATE_LABEL[cand['gate']]}{tag}",
                          fontsize=ANNOT_FS, color=colour)

        for k in range(len(cands), nrows_block * NCOLS):
            all_axes[panel_row + k // NCOLS, k % NCOLS].axis("off")
        panel_row += nrows_block

    fig.suptitle(
        "Ring candidates at each ballistic waypoint, labelled by the gate that "
        "rejected them\n"
        "Cream background = rejected on CLEARANCE (finite mc < 0) · "
        "Pink = rejected before the arc was even computed · "
        "Green = the candidate A* took",
        fontsize=TITLE_FS - 2,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.955), h_pad=3.2)
    save(fig, save_path, dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
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

    interesting = find_interesting_cells(planner_ball, path_cells_of(planner_ball, path_ball))

    # The headline claim of this demo: clearance, not physics, does the rejecting.
    total = {"accept": 0, "bounds": 0, "obstacle": 0, "physics": 0, "clearance": 0}
    for _, cands in interesting:
        for key, val in gate_counts(cands).items():
            total[key] += val

    print(f"\nRing candidates across {len(interesting)} ballistic waypoint(s):")
    print(f"  ACCEPT                 {total['accept']}")
    print(f"  REJECT · clearance     {total['clearance']}   ← finite mc < 0")
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

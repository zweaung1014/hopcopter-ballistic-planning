"""Tall-stairs robustness demo: ballistic planner climbs to the top platform.

Goal is placed on the 1.2 m top platform.  The robot must ascend three 0.4 m
risers (see `maps/tall_stairs.py` for the exact column layout).

What this demo does and does not show
-------------------------------------
It shows the planner ascending three risers to the goal.  It does NOT show
either rejection gate firing.  Sweeping all 625 cells at full hop radius with
HOP_RADIUS=1.5 m and V_MAX=6.0 m/s gives:

    accept 6518 · off-map 3482 · physics 0 · clearance 0

Two separate reasons, both worth knowing before you present this:

* Clearance never fires because `min_clearance` discards any arc sample that has
  not risen `robot_radius` above BOTH endpoints.  A hop climbing onto a riser
  rises only ~0.07-0.24 m above its landing, so nearly every interior sample is
  skipped and the function returns `+inf`.
* Physics never fires because 0.4 m risers are comfortably inside the leg's
  budget at these parameters — `feasible_alpha_interval` returns a valid interval
  for every in-bounds candidate.

So the baseline/ballistic path difference here comes from *cost shaping* — the
smooth clearance proximity penalty and the uphill elevation penalty — rather than
from any hop being ruled out.  That is a legitimate result, but state it as such.

For a genuine clearance *rejection* on stairs use `test/demo_stairs_curb.py`,
where the curbs are local maxima and finite negative `mc` values appear.
`test/demo_barely_jumpable.py` is the other clearance-driven case.

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

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import config
from demo_common import (
    N_ANGLES, V_MAX,
    PRESENTATION_DPI, TITLE_FS, LABEL_FS, ANNOT_FS,
    make_planner, diagnose_edge, n_bad_hops,
    enumerate_ring_candidates, find_interesting_cells, gate_counts,
    path_cells_of, param_caption, save, out_path, TIGHT_BAND, C_LOWMARG
)
from hopping_astar_planner import HoppingAStarPlanner
from map2d5 import Map2D5
from maps.tall_stairs import (
    build as build_map,
    STEP1_Z, STEP2_Z, TOP_Z,
    STEP1_XMIN, STEP2_XMIN, TOP_XMIN,
)
from visualizer import Visualizer, draw_arc_side_view


# ---------------------------------------------------------------------------
# Scenario knobs (HOP_RADIUS / N_ANGLES / V_MAX come from demo_common so every
# figure in the deck quotes the same physics)
# ---------------------------------------------------------------------------
START = (0.5, 2.5)
GOAL  = (4.0, 2.5)   # on top platform (z = 1.2 m)

# Step-height reference lines used in arc plots
STEP_ZS = [STEP1_Z, STEP2_Z, TOP_Z]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _n_clearance_rejects(cands: list[dict]) -> int:
    """Count candidates rejected specifically by the arc-terrain clearance gate."""
    return sum(1 for c in cands if c["mc"] is not None and c["mc"] < config.MIN_CLEARANCE)


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
        (STEP1_XMIN, f"riser 1\n(z=0→{STEP1_Z:.2f})", "#e65100"),
        (STEP2_XMIN, f"riser 2\n(z={STEP1_Z:.2f}→{STEP2_Z:.2f})", "#bf360c"),
        (TOP_XMIN,   f"riser 3\n(z={STEP2_Z:.2f}→{TOP_Z:.2f})", "#7f0000"),
    ]:
        ax.axvline(x_bnd, color=col, linewidth=1.5, linestyle="--",
                   alpha=0.75, zorder=4)
        ax.text(x_bnd + 0.03, 0.15, label, color=col,
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
                    xytext=(-18, -16), textcoords="offset points", color="#e65100", zorder=8)

    # Highlight baseline hops the clearance gate would have rejected (red), and
    # those it accepts only barely (orange). The gate is hard — clearance no
    # longer feeds into cost — so the orange band is a *presentation* nicety
    # marking hops with little margin, not a second cost regime.
    gate = config.MIN_CLEARANCE
    _tight_labelled = False
    for i, d in enumerate(diags_base):
        if not d["feasible"]:
            continue
        p0, p1 = path_base[i], path_base[i + 1]
        if d["mc"] < gate:
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]],
                    color="#d50000", linewidth=8, alpha=0.55,
                    zorder=6.5, solid_capstyle="round")
            ax.annotate(
                f"REJECTED\nmc={d['mc']:+.3f}m",
                (0.5*(p0[0]+p1[0]), 0.5*(p0[1]+p1[1])),
                xytext=(0, 26), textcoords="offset points",
                ha="center", color="#b71c1c", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.2", fc="white",
                          ec="#b71c1c", lw=1.1), zorder=11,
            )
        elif d["mc"] < gate + TIGHT_BAND:
            # Only add legend label once
            lbl = (f"within {TIGHT_BAND} m of the {gate} m gate"
                   if not _tight_labelled else "_nolegend_")
            _tight_labelled = True
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]],
                    color=C_LOWMARG, linewidth=7, alpha=0.45,
                    zorder=6.4, solid_capstyle="round", label=lbl)
            ax.annotate(
                f"mc={d['mc']:+.3f}m\n(tight)",
                (0.5*(p0[0]+p1[0]), 0.5*(p0[1]+p1[1])),
                xytext=(0, 22), textcoords="offset points",
                ha="center", color="#e65100", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.2", fc="white",
                          ec=C_LOWMARG, lw=1.1), zorder=11,
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
                    xytext=(5, 5), textcoords="offset points", color="#00695c", zorder=10)

    # Ring candidates at the most-constrained ballistic waypoint
    if interesting:
        # pick cell with the most total rejections (any gate)
        chosen_cell, cands = max(
            interesting,
            key=lambda t: sum(1 for c in t[1] if not c["accepted"]),
        )
        px, py = m.grid_to_world(*chosen_cell)

        # hop circle -- all candidates share the radius they were generated
        # with (`enumerate_ring_candidates` computes it once per call, from
        # this state's v_g_in)
        ax.add_patch(mpatches.Circle(
            (px, py), radius=cands[0]["r"], fill=False,
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
            mpatches.Patch(color="#c62828", label=f"REJECT ({n_rej}) OOB/stance/physics/clearance"),
            mpatches.Patch(color="#1b5e20", label="CHOSEN"),
            mpatches.Patch(color="#d50000", alpha=0.55,
                           label="baseline hop fails the clearance gate"),
        ]
    else:
        extra_h = []

    ax.plot(START[0], START[1], "go", markersize=11, zorder=10, label="start")
    ax.plot(GOAL[0], GOAL[1], "r*", markersize=15, zorder=10,
            label=f"goal  z={m.get_elevation(*GOAL):.1f} m")

    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles + extra_h, labels + [h.get_label() for h in extra_h],
              loc="upper left")

    tally = {"accept": 0, "bounds": 0, "obstacle": 0, "stance": 0,
             "physics": 0, "clearance": 0}
    for _, cands in interesting:
        for key, val in gate_counts(cands).items():
            tally[key] += val

    ax.set_title(
        f"tall_stairs: ballistic planner climbs to the z={TOP_Z:.1f} m platform\n"
        f"Ring rejections at these waypoints: clearance {tally['clearance']} · "
        f"stance {tally['stance']} · physics {tally['physics']} · "
        f"off-map {tally['bounds']}\n"
        f"The risers are well inside the leg's budget, so what prunes the search "
        f"is the robot's body: it cannot stand within "
        f"{config.ROBOT_RADIUS + config.MIN_CLEARANCE:.2f} m of a riser "
        f"(sphere at CoM + leg cylinder sides), and flat approach arcs clip the "
        f"step edge.", wrap=True
    )
    fig.tight_layout()
    save(fig, save_path)
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
        # Label the first panel of this block. Using a text artist rather than
        # set_title(loc="left"): draw_arc_side_view sets its own centered title,
        # and matplotlib draws left- and center-aligned titles at the same
        # height, so the two would overlap.
        all_axes[panel_row, 0].text(
            0.0, 1.34, row_label, transform=all_axes[panel_row, 0].transAxes, color="#1a237e", fontweight="bold",
            va="bottom", ha="left",
        )

        for k, cand in enumerate(cands):
            ax = all_axes[panel_row + k // NCOLS, k % NCOLS]

            if cand["alpha_s"] is None or cand["c_g"] is None:
                ax.set_facecolor("#fbe9e7")
                ax.text(0.5, 0.5,
                        f"INFEASIBLE\n{cand['reason']}",
                        ha="center", va="center",
                        transform=ax.transAxes)
                ax.set_xticks([]); ax.set_yticks([])
            else:
                draw_arc_side_view(
                    ax, cand["c_s"], cand["c_g"], cand["alpha_s"],
                    m, planner_ball.robot_radius, planner_ball.leg_length, planner_ball.arc_max_step,
                    min_clearance_gate=planner_ball.min_clearance_gate,
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
        "Dotted lines = step elevations", wrap=True
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96), h_pad=3.2)
    save(fig, save_path, dpi=110)
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
                ax.text(0.5, 0.5, "INFEASIBLE", ha="center", va="center", transform=ax.transAxes)
                ax.set_xticks([]); ax.set_yticks([])
            else:
                p0, p1 = path[i], path[i + 1]
                draw_arc_side_view(
                    ax,
                    (p0[0], p0[1], d["z0"]), (p1[0], p1[1], d["z1"]),
                    d["alpha_s"], m,
                    config.ROBOT_RADIUS, config.LEG_LENGTH,
                    config.ARC_SAMPLE_MAX_STEP,
                    min_clearance_gate=config.MIN_CLEARANCE,
                    steep_grade=config.STEEP_INFLATE_GRADE,
                )
                ax.set_ylim(-0.1, ymax)
                # Step reference lines
                for sz, sc in [(STEP1_Z, "#e65100"), (STEP2_Z, "#bf360c"),
                               (TOP_Z, "#7f0000")]:
                    ax.axhline(sz, color=sc, linewidth=0.9,
                               linestyle=":", alpha=0.8, zorder=4)

            if i == 0:
                ax.set_ylabel(f"{row_label}\nz (m)")
            ax.set_xlabel(
                f"hop {i}: ({path[i][0]:.1f},{path[i][1]:.1f})"
                f" → ({path[i+1][0]:.1f},{path[i+1][1]:.1f})",
            )

    fig.suptitle(
        "Per-hop side view — baseline (top) vs ballistic (bottom)\n"
        f"Dotted lines = step elevations  {STEP1_Z:.2f} / {STEP2_Z:.2f} / {TOP_Z:.2f} m · "
        f"Red arc = below the {config.MIN_CLEARANCE} m clearance gate · Green = clears", wrap=True
    )
    fig.tight_layout(rect=(0, 0, 1, 0.91))
    save(fig, save_path)
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

    diags_base = [
        diagnose_edge(planner_ball, m, path_base[i], path_base[i + 1])
        for i in range(len(path_base) - 1)
    ]
    diags_ball = [
        diagnose_edge(planner_ball, m, path_ball[i], path_ball[i + 1])
        for i in range(len(path_ball) - 1)
    ]

    # --- console summary ---
    n_base_bad = n_bad_hops(diags_base)
    n_ball_bad = n_bad_hops(diags_ball)

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
              f"  (clearance:{n_clr}  infeasible:{n_inf}  OOB:{n_oob})")
        for c in cands:
            if not c["accepted"] and c["mc"] is not None and c["mc"] < config.MIN_CLEARANCE:
                if c["cell"] is not None:
                    lx, ly = m.grid_to_world(*c["cell"])
                    print(f"    REJECT clearance → ({lx:.1f},{ly:.1f})  {c['reason']}")

    draw_topdown(
        m, path_base, path_ball, diags_base, planner_ball, interesting,
        out_path("stairs_topdown.png"),
    )
    draw_ring_panels(
        m, planner_ball, path_ball, interesting,
        out_path("stairs_ring_candidates.png"),
    )
    draw_arc_strip(
        m, planner_ball,
        path_base, diags_base,
        path_ball, diags_ball,
        out_path("stairs_arcs.png"),
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

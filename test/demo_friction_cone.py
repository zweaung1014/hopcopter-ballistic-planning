"""What the friction cone (Campana BEAM constraints 1 and 2) actually changes.

Runs on `maps/cross_slope.py`, a constant-grade side-hill, and produces three
figures:

  1. `friction_cone_polar.png` — maximum feasible hop length as a function of
     heading, with and without the cone. The headline: the cone is the only gate
     in the planner that depends on which *direction* the robot hops, so on a
     slope the reachable distance stops being a circle.
  2. `friction_cone_paths.png` — A/B of the planned paths. Same route, more
     waypoints with the cone on, and the extra hops land exactly on the hillside.
     This is the effect Campana & Laumond report in their Table I.
  3. `friction_cone_geometry.png` — the five BEAM constraint intervals stacked
     for two hops of identical length from the same cell, one up the fall line
     and one across it, showing which constraint actually decides each.

Run:
    python test/demo_friction_cone.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

import config
from demo_common import (
    ANNOT_FS,
    C_ACCEPT,
    C_BALL,
    C_BASE,
    C_REJECT,
    LABEL_FS,
    SLIDE_FIGSIZE,
    TOPDOWN_FIGSIZE,
    draw_topdown_compact,
    make_planner,
    out_path,
    param_caption,
    save,
)
from hopping_astar_planner import (
    _landing_cone_alpha_s,
    _speed_tan_interval,
    feasible_alpha_interval,
    inplane_friction_cone,
)
from maps import cross_slope

G = config.G_ACCEL
V_MAX = config.V_MAX
MU = config.MU

# The cell the per-heading figures are computed at: mid-hillside, mid-map.
PROBE_XY = (2.0, 2.5)
# Hop lengths the polar sweep tests, coarse enough to stay fast and fine enough
# to resolve the fall-line/cross-slope gap (0.55 m vs 1.50 m at shipped values).
LENGTHS = np.arange(0.10, 1.55, 0.05)


def _probe_normal(m):
    r, c = m.world_to_grid(*PROBE_XY)
    return tuple(m.surface_normals()[r, c])


def max_hop_by_heading(n_probe, mu, n_headings: int = 145):
    """Longest feasible hop from the probe cell, for each heading.

    `Z` follows the local plane: hopping a horizontal distance `X` on a grade
    `s` in a direction `theta` off the fall line lands `s * X * cos(theta)`
    higher. Both contacts sit on the same plane, so both normals are `n_probe`.
    """
    grade = math.hypot(n_probe[0], n_probe[1]) / n_probe[2]
    thetas = np.linspace(0.0, 2.0 * math.pi, n_headings)
    out = np.zeros_like(thetas)
    for i, th in enumerate(thetas):
        best = 0.0
        for X in LENGTHS:
            Z = grade * X * math.cos(th)
            if feasible_alpha_interval(X, Z, V_MAX, G, mu=mu,
                                       n_s=n_probe, n_g=n_probe, theta=th):
                best = float(X)
        out[i] = best
    return thetas, out


# --------------------------------------------------------------------------- #
# Figure 1 — reachable hop length vs heading
# --------------------------------------------------------------------------- #

def fig_polar(n_probe) -> str:
    thetas, with_cone = max_hop_by_heading(n_probe, MU)
    _, no_cone = max_hop_by_heading(n_probe, None)

    fig = plt.figure(figsize=SLIDE_FIGSIZE)
    # Leave the right third for the caption, and headroom above for the title:
    # polar annotations placed outside the data radius are otherwise clipped.
    ax = fig.add_axes((0.02, 0.06, 0.58, 0.80), projection="polar")
    ax.plot(thetas, no_cone, color=C_BASE, linewidth=2.4, linestyle="--",
            label="no friction cone (leg energy only)")
    ax.fill(thetas, with_cone, color=C_BALL, alpha=0.18)
    ax.plot(thetas, with_cone, color=C_BALL, linewidth=2.8,
            label=f"friction cone, $\\mu$ = {MU}")

    ax.set_theta_zero_location("E")
    ax.set_rlabel_position(112.5)
    # Head-room inside the axes so the callouts sit on the canvas, not past it.
    ax.set_ylim(0, float(LENGTHS[-1]) * 1.45)
    ax.set_rticks([0.5, 1.0, 1.5])
    ax.tick_params(labelsize=ANNOT_FS - 2)
    # In the right-hand column, above the caption: a legend under the polar
    # axes gets clipped by the canvas edge.
    ax.legend(loc="upper left", bbox_to_anchor=(1.06, 1.02), frameon=False,
              fontsize=ANNOT_FS - 1)

    up = float(with_cone[0])
    across = float(with_cone[len(thetas) // 4])
    down = float(with_cone[len(thetas) // 2])
    # 0 deg is +x, straight up the fall line on this map; 90 deg is a pure
    # cross-slope traverse; 180 deg is straight down it.
    for ang, val, text, ha in (
        (0.0, up, f"up the fall line\n{up:.2f} m", "center"),
        (math.pi / 2, across, f"across the slope\n{across:.2f} m", "center"),
        (math.pi, down, f"down the fall line\n{down:.2f} m", "center"),
    ):
        ax.annotate(
            text, xy=(ang, val), xytext=(ang, val + 0.55),
            ha=ha, va="center", color="#1b5e20", fontweight="bold",
            fontsize=ANNOT_FS - 1,
            bbox=dict(boxstyle="round,pad=0.28", fc="white", ec=C_BALL, lw=1.4),
            arrowprops=dict(arrowstyle="->", color=C_BALL, lw=1.6),
        )

    fig.text(
        0.30, 0.94,
        "Longest feasible hop vs heading, mid-hillside "
        f"(grade {math.hypot(n_probe[0], n_probe[1]) / n_probe[2]:.1f})",
        ha="center", fontsize=LABEL_FS, fontweight="bold",
    )
    fig.text(
        0.635, 0.68,
        "Leg energy alone already costs\n"
        "reach uphill (dashed oval): climbing\n"
        "spends budget on height.\n\n"
        "The friction cone takes a further\n"
        f"{100 * (1 - up / float(no_cone[0])):.0f}% off BOTH fall-line\n"
        "directions while leaving the\n"
        "cross-slope traverse untouched.\n\n"
        "It is the only gate in the planner\n"
        "that depends on heading at all —\n"
        "the others see only X and Z, and\n"
        "cannot tell one direction from\n"
        "another. Only the part of the\n"
        "surface normal lying in the hop\n"
        "plane enters the cone, and that\n"
        "is what varies with heading.",
        ha="left", va="top", fontsize=ANNOT_FS - 1, color="#37474f",
    )
    p = out_path("friction_cone_polar.png")
    save(fig, p)
    return p


# --------------------------------------------------------------------------- #
# Figure 2 — A/B of the planned paths
# --------------------------------------------------------------------------- #

def fig_paths(m) -> tuple[str, list, list]:
    print("\nPlanning with the friction cone ON ...")
    p_cone = make_planner(m, False, cross_slope.START, cross_slope.GOAL, mu=MU)
    path_cone = p_cone.plan()
    print("Planning with the friction cone OFF (baseline) ...")
    p_bare = make_planner(m, False, cross_slope.START, cross_slope.GOAL, mu=None)
    path_bare = p_bare.plan()

    fig, (ax, ax2) = plt.subplots(
        1, 2, figsize=SLIDE_FIGSIZE, gridspec_kw={"width_ratios": [1.0, 0.85]},
    )
    draw_topdown_compact(m, ax, colorbar_on=fig)

    # The hillside band. Drawn as edge lines rather than a shaded span: the
    # elevation colormap already fills the axes, so a translucent overlay is
    # invisible against it.
    for x in (cross_slope.BENCH_X1, cross_slope.SLOPE_X1):
        ax.axvline(x, color="#263238", linestyle=":", linewidth=2.0, zorder=5)
    ax.annotate(
        f"side-hill, grade {cross_slope.GRADE}",
        (0.5 * (cross_slope.BENCH_X1 + cross_slope.SLOPE_X1), 0.22),
        ha="center", color="#263238", fontweight="bold", zorder=10,
        fontsize=ANNOT_FS - 1,
        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#263238", lw=1.0),
    )

    # Both planners pick the same diagonal route, so the two polylines coincide
    # exactly. Drawing one under the other would hide the baseline entirely —
    # what differs is only where the waypoints fall, so draw the shared route
    # once and then the two marker sets at different sizes.
    xs = [q[0] for q in path_cone]
    ys = [q[1] for q in path_cone]
    ax.plot(xs, ys, color="#546e7a", linewidth=2.0, zorder=7,
            label="route (identical for both)")
    ax.plot([q[0] for q in path_bare], [q[1] for q in path_bare], "o",
            color=C_BASE, markersize=15, markerfacecolor="none",
            markeredgewidth=2.5, zorder=8,
            label=f"no cone — {len(path_bare)} waypoints")
    ax.plot(xs, ys, "o", color=C_BALL, markersize=7, zorder=9,
            label=f"friction cone, $\\mu$ = {MU} — {len(path_cone)} waypoints")

    ax.set_title("Same route, different stride")
    ax.legend(loc="upper left", fontsize=ANNOT_FS - 2)

    # Per-hop stride length, which is where the effect actually lives.
    def _hops(path):
        return [math.hypot(path[i + 1][0] - path[i][0],
                           path[i + 1][1] - path[i][1])
                for i in range(len(path) - 1)]

    def _on_hill(path, i):
        return cross_slope.BENCH_X1 <= path[i][0] < cross_slope.SLOPE_X1

    hops_bare, hops_cone = _hops(path_bare), _hops(path_cone)
    for offset, hops, path, colour, label in (
        (-0.2, hops_bare, path_bare, C_BASE, "no cone"),
        (0.2, hops_cone, path_cone, C_BALL, f"friction cone, $\\mu$ = {MU}"),
    ):
        idx = np.arange(len(hops)) + offset
        ax2.bar(idx, hops, width=0.38, color=colour, alpha=0.85, label=label)
        for j, (x, h) in enumerate(zip(idx, hops)):
            if _on_hill(path, j):
                ax2.plot(x, h + 0.06, marker="v", color="#263238",
                         markersize=8, zorder=5)

    ax2.plot([], [], "v", color="#263238", markersize=8,
             label="launched from the hillside")
    ax2.set_xlabel("hop index")
    ax2.set_ylabel("hop length X (m)")
    ax2.set_title("The cone costs stride, and only on the grade")
    ax2.set_xticks(range(max(len(hops_bare), len(hops_cone))))
    ax2.legend(loc="upper right", fontsize=ANNOT_FS - 2)
    ax2.grid(axis="y", alpha=0.3)

    fig.suptitle("Narrower cone, more waypoints (cf. Campana & Laumond, Table I)")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    p = out_path("friction_cone_paths.png")
    save(fig, p)
    return p, path_bare, path_cone


# --------------------------------------------------------------------------- #
# Figure 3 — which constraint decides, per heading
# --------------------------------------------------------------------------- #

def _constraint_rows(X, Z, n, theta, mu):
    """The five BEAM intervals for one candidate hop, before intersection.

    Mirrors `feasible_alpha_interval` term by term so the figure shows the same
    numbers the planner gates on. Unbounded ends are clipped to the plot range.
    """
    rows = []
    rows.append(("Eq. 4 validity\n(aim above the chord)",
                 math.atan2(Z, X), 0.5 * math.pi))

    t3 = _speed_tan_interval(X, Z, V_MAX * V_MAX, G)
    rows.append(("(3) takeoff speed\n$v_s \\leq V_{max}$",
                 math.atan(t3[0]) if t3 else None,
                 math.atan(t3[1]) if t3 else None))

    t4 = _speed_tan_interval(X, Z, V_MAX * V_MAX + 2.0 * G * Z, G)
    rows.append(("(4) landing speed\n$v_g \\leq V_{max}$",
                 math.atan(t4[0]) if t4 else None,
                 math.atan(t4[1]) if t4 else None))

    cone = inplane_friction_cone(n, theta, mu)
    rows.append(("(1) takeoff friction\n$\\gamma_s \\pm \\delta_s$",
                 cone[0] - cone[1] if cone else None,
                 cone[0] + cone[1] if cone else None))

    land = _landing_cone_alpha_s(X, Z, cone[0], cone[1]) if cone else None
    rows.append(("(2) landing friction\n(mapped through the arc)",
                 land[0] if land else None, land[1] if land else None))
    return rows


def fig_geometry(n_probe) -> str:
    grade = math.hypot(n_probe[0], n_probe[1]) / n_probe[2]
    X = 1.0
    cases = [
        ("Up the fall line", 0.0, grade * X),
        ("Across the slope", math.pi / 2, 0.0),
    ]

    fig, axes = plt.subplots(1, 2, figsize=SLIDE_FIGSIZE, sharey=True)
    lo_plot, hi_plot = -20.0, 100.0

    for ax, (title, theta, Z) in zip(axes, cases):
        rows = _constraint_rows(X, Z, n_probe, theta, MU)
        final = feasible_alpha_interval(X, Z, V_MAX, G, mu=MU,
                                        n_s=n_probe, n_g=n_probe, theta=theta)

        for i, (label, lo, hi) in enumerate(rows):
            y = len(rows) - i
            if lo is None or hi is None:
                ax.text(0.5 * (lo_plot + hi_plot), y, "unsatisfiable",
                        ha="center", va="center", color=C_REJECT,
                        fontweight="bold")
                continue
            lo_d = max(math.degrees(lo), lo_plot)
            hi_d = min(math.degrees(hi), hi_plot)
            ax.barh(y, hi_d - lo_d, left=lo_d, height=0.52,
                    color="#90a4ae", alpha=0.55, zorder=3)
            ax.plot([lo_d, lo_d], [y - 0.28, y + 0.28], color="#37474f", lw=2,
                    zorder=4)
            ax.plot([hi_d, hi_d], [y - 0.28, y + 0.28], color="#37474f", lw=2,
                    zorder=4)

        # The surviving intersection.
        y0 = 0.3
        if final is None:
            # Name the binding pair rather than just reporting failure: the
            # whole point of the figure is which constraint decided.
            usable = [(lab, lo, hi) for lab, lo, hi in rows
                      if lo is not None and hi is not None]
            tightest_lo = max(usable, key=lambda r: r[1])
            tightest_hi = min(usable, key=lambda r: r[2])
            ax.text(0.5 * (lo_plot + hi_plot), y0 + 0.55, "NO FEASIBLE ANGLE",
                    ha="center", va="center", color=C_REJECT,
                    fontweight="bold", fontsize=LABEL_FS)
            ax.text(
                0.5 * (lo_plot + hi_plot), y0 - 0.35,
                f"{tightest_lo[0].splitlines()[0]} needs "
                f"$\\alpha_s \\geq$ {math.degrees(tightest_lo[1]):.1f}°,\n"
                f"but {tightest_hi[0].splitlines()[0]} caps it at "
                f"{math.degrees(tightest_hi[2]):.1f}°.\n"
                "Too steep to push without slipping; too shallow to stay in "
                "budget.",
                ha="center", va="center", color="#37474f",
                fontsize=ANNOT_FS - 2,
            )
        else:
            lo_d, hi_d = math.degrees(final[0]), math.degrees(final[1])
            ax.barh(y0, hi_d - lo_d, left=lo_d, height=0.52,
                    color=C_ACCEPT, alpha=0.9, zorder=5)
            ax.annotate(f"{lo_d:.1f}° – {hi_d:.1f}°",
                        (0.5 * (lo_d + hi_d), y0), xytext=(0, -30),
                        textcoords="offset points", ha="center",
                        color="#1b5e20", fontweight="bold")
            ax.axvspan(lo_d, hi_d, color=C_ACCEPT, alpha=0.10, zorder=0)

        ax.set_yticks([len(rows) - i for i in range(len(rows))] + [y0])
        ax.set_yticklabels([r[0] for r in rows] + ["INTERSECTION"])
        ax.set_xlim(lo_plot, hi_plot)
        ax.set_ylim(-0.6, len(rows) + 0.7)
        ax.set_xlabel("takeoff angle $\\alpha_s$ (deg)")
        ax.set_title(f"{title}   (X = {X:.1f} m, Z = {Z:+.2f} m)")
        ax.grid(axis="x", alpha=0.3)

    fig.suptitle(
        f"Which BEAM constraint decides — same hop length, two headings, "
        f"grade {grade:.1f}, $\\mu$ = {MU}"
    )
    fig.tight_layout(rect=(0, 0.02, 1, 0.94))
    p = out_path("friction_cone_geometry.png")
    save(fig, p)
    return p


def main() -> int:
    print("== friction cone demo (maps/cross_slope.py) ==")
    print(param_caption())
    print(f"   mu={MU}  beta=atan(mu)={math.degrees(math.atan(MU)):.2f}°")

    m = cross_slope.build()
    n_probe = _probe_normal(m)
    grade = math.hypot(n_probe[0], n_probe[1]) / n_probe[2]
    print(f"   probe cell {PROBE_XY}: grade {grade:.3f}, "
          f"normal {tuple(round(v, 4) for v in n_probe)}")

    cone_fall = inplane_friction_cone(n_probe, 0.0, MU)
    cone_cross = inplane_friction_cone(n_probe, math.pi / 2, MU)
    print(f"   fall line : gamma={math.degrees(cone_fall[0]):6.2f}°  "
          f"delta={math.degrees(cone_fall[1]):5.2f}°  "
          f"-> alpha floor {math.degrees(cone_fall[0] - cone_fall[1]):.2f}°")
    print(f"   cross     : gamma={math.degrees(cone_cross[0]):6.2f}°  "
          f"delta={math.degrees(cone_cross[1]):5.2f}°  "
          f"-> alpha floor {math.degrees(cone_cross[0] - cone_cross[1]):.2f}°")

    p1 = fig_polar(n_probe)
    p3 = fig_geometry(n_probe)
    p2, path_bare, path_cone = fig_paths(m)

    print("\n-- summary --")
    if path_bare is None or path_cone is None:
        print("  a planner returned no path; figures may be incomplete")
        return 2
    for label, path in (("no cone", path_bare), (f"cone mu={MU}", path_cone)):
        length = sum(math.hypot(path[i + 1][0] - path[i][0],
                                path[i + 1][1] - path[i][1])
                     for i in range(len(path) - 1))
        on_hill = sum(
            1 for i in range(len(path) - 1)
            if cross_slope.BENCH_X1 <= path[i][0] < cross_slope.SLOPE_X1
        )
        print(f"  {label:14s}: {len(path):2d} waypoints, xy length {length:.2f} m, "
              f"{on_hill} hops launched from the hillside")
    print(f"\n  figures: {os.path.basename(p1)}, {os.path.basename(p2)}, "
          f"{os.path.basename(p3)}")

    if matplotlib.get_backend().lower() != "agg":
        plt.show()

    # The demo is only informative if the cone changed something.
    return 0 if len(path_cone) > len(path_bare) else 2


if __name__ == "__main__":
    sys.exit(main())

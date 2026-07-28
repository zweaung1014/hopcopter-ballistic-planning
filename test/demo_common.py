"""Shared scaffolding for the `test/demo_*.py` presentation scripts.

Every demo builds a scenario, runs the ballistic planner against a baseline with
`disable_clearance=True`, and saves annotated PNGs into `test/`.  The planner
construction, per-edge diagnosis and ring-candidate enumeration were previously
copy-pasted byte-identically across five demos; they live here instead.

The scenario constants below are the *deck-wide* physics parameters.  They
deliberately override `config.HOP_RADIUS` (1.0) and `config.V_MAX` (4.5): a
1.5 m hop needs `V_max > sqrt(g * 1.5) = 3.84 m/s` with margin, and the whole
demo suite must quote one set of numbers or the "one planner, many behaviours"
story falls apart.  Print them on every figure via `param_caption()`.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

import config
from hopping_astar_planner import (
    HoppingAStarPlanner,
    feasible_alpha_interval,
    min_clearance,
)
from map2d5 import Map2D5
from visualizer import Visualizer


# ---------------------------------------------------------------------------
# Deck-wide scenario constants
# ---------------------------------------------------------------------------
HOP_RADIUS = 1.5
N_ANGLES   = 16
V_MAX      = 6.0

# Figure styling, sized for projection rather than for reading on a laptop.
PRESENTATION_DPI = 140
TITLE_FS = 13
LABEL_FS = 11
ANNOT_FS = 8

# Colour vocabulary — consistent across every figure in the deck.
C_BASE    = "#e65100"   # baseline path (clearance OFF)
C_BALL    = "#00695c"   # ballistic path (clearance ON)
C_COLLIDE = "#d50000"   # baseline hop that clips terrain
C_LOWMARG = "#ff6f00"   # hop below clearance_margin (penalised, not rejected)
C_ACCEPT  = "#66bb6a"   # accepted ring candidate
C_REJECT  = "#c62828"   # rejected ring candidate
C_CHOSEN  = "#1b5e20"   # the candidate A* actually took
C_ANNOT   = "#0277bd"   # measurement annotations (Δz arrows etc.)


def param_caption() -> str:
    """One-line physics summary to stamp on every figure."""
    return (
        f"hop_radius={HOP_RADIUS} m · V_max={V_MAX} m/s · "
        f"robot_radius={config.ROBOT_RADIUS} m · "
        f"α_uphill={config.ALPHA_UPHILL} · α_downhill={config.ALPHA_DOWNHILL} · "
        f"clearance_margin={config.CLEARANCE_MARGIN} m"
    )


# ---------------------------------------------------------------------------
# Planner construction and per-edge diagnosis
# ---------------------------------------------------------------------------

def make_planner(
    m: Map2D5,
    disable_clearance: bool,
    start: tuple[float, float],
    goal: tuple[float, float],
    hop_radius: float = HOP_RADIUS,
    n_angles: int = N_ANGLES,
    V_max: float = V_MAX,
    **overrides,
) -> HoppingAStarPlanner:
    """Build a planner with every `config.*` field threaded in by name.

    `overrides` accepts any remaining `HoppingAStarPlanner` keyword (e.g.
    `alpha_uphill=...`) so a demo can vary one parameter without restating
    the rest.
    """
    kwargs = dict(
        map_env=m,
        start=start,
        goal=goal,
        hop_radius=hop_radius,
        n_angles=n_angles,
        max_jump_height=config.MAX_JUMP_HEIGHT,
        alpha_uphill=config.ALPHA_UPHILL,
        alpha_downhill=config.ALPHA_DOWNHILL,
        g=config.G_ACCEL,
        V_max=V_max,
        robot_radius=config.ROBOT_RADIUS,
        clearance_margin=config.CLEARANCE_MARGIN,
        clearance_weight=config.CLEARANCE_WEIGHT,
        arc_max_step=config.ARC_SAMPLE_MAX_STEP,
        arc_endpoint_epsilon=config.ARC_ENDPOINT_EPSILON,
        obstacle_wall_extra=config.OBSTACLE_WALL_EXTRA,
        disable_clearance=disable_clearance,
    )
    kwargs.update(overrides)
    return HoppingAStarPlanner(**kwargs)


def diagnose_edge(
    planner: HoppingAStarPlanner,
    m: Map2D5,
    p0: tuple[float, float],
    p1: tuple[float, float],
) -> dict:
    """Re-score one hop with the ballistic criterion, whichever planner produced it.

    Pass the *clearance-ON* planner even when diagnosing the baseline path —
    that is what makes the A/B comparison honest: both paths are judged by the
    same gate.  All physics parameters are read off the planner, so the
    diagnosis can never drift from what the planner itself evaluated.
    """
    z0 = float(m.get_elevation(*p0))
    z1 = float(m.get_elevation(*p1))
    X  = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    Z  = z1 - z0
    iv = feasible_alpha_interval(X, Z, planner.V_max, planner.g)
    if iv is None:
        return {"feasible": False, "X": X, "Z": Z, "alpha_s": None,
                "mc": -math.inf, "z0": z0, "z1": z1}
    a  = 0.5 * (iv[0] + iv[1])
    mc = min_clearance(
        (p0[0], p0[1], z0), (p1[0], p1[1], z1), a,
        m, planner.g, planner.robot_radius,
        planner.arc_endpoint_epsilon, planner.arc_max_step,
        planner._obstacle_fill,
    )
    return {"feasible": True, "X": X, "Z": Z, "alpha_s": a, "mc": mc,
            "z0": z0, "z1": z1}


def diagnose_path(planner: HoppingAStarPlanner, m: Map2D5, path: list) -> list[dict]:
    """`diagnose_edge` over every consecutive pair in `path`."""
    return [diagnose_edge(planner, m, path[i], path[i + 1])
            for i in range(len(path) - 1)]


def n_bad_hops(diags: list[dict]) -> int:
    """Hops that are infeasible or clip the terrain."""
    return sum(1 for d in diags if not d["feasible"] or d["mc"] < 0.0)


# ---------------------------------------------------------------------------
# Arc geometry helpers
# ---------------------------------------------------------------------------

def arc_z_at(
    c_s: tuple[float, float, float],
    Z: float,
    X: float,
    alpha_s: float,
    u: float,
    g: float,
) -> float:
    """Height of the parabola (Campana Eq. 2) at horizontal distance `u`.

    `X`/`Z` are the hop's horizontal span and rise, matching `diagnose_edge`.
    """
    tan_a = math.tan(alpha_s)
    # Campana Eq. 4 rearranged: g*X^2 / (2*xdot^2) == X*tan(alpha) - Z
    k = (X * tan_a - Z) / (X * X)
    return c_s[2] + u * tan_a - k * u * u


def u_of_x(p0: tuple[float, float], p1: tuple[float, float], target_x: float) -> float | None:
    """Horizontal distance `u` along a hop at which world-x reaches `target_x`.

    Returns None for hops with no x-component (u is undefined there).
    """
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    X  = math.hypot(dx, dy)
    if X < 1e-9 or abs(dx) < 1e-9:
        return None
    return (target_x - p0[0]) * X / dx


# ---------------------------------------------------------------------------
# Ring-candidate enumeration
# ---------------------------------------------------------------------------

def enumerate_ring_candidates(
    planner: HoppingAStarPlanner,
    cell: tuple[int, int],
) -> list[dict]:
    """All `n_angles` full-radius ring candidates from `cell`, with a verdict each.

    Scans only the full `hop_radius`, not the planner's inward ray-search, so the
    figure stays readable.  Out-of-bounds and physics-infeasible candidates are
    kept so they show up as rejections.

    Each entry carries a `gate` field naming *which* check rejected it —
    `"bounds"`, `"obstacle"`, `"physics"`, `"clearance"`, or `""` when accepted.
    Distinguishing physics from clearance matters: on steep uphill hops the
    body-guard in `min_clearance` skips nearly every interior sample and returns
    `+inf`, so a demo that claims "clearance rejected this" must actually check.
    """
    m = planner.map_env
    px, py = m.grid_to_world(*cell)
    pz = float(m.grid[cell[0], cell[1]])
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
                "accepted": False, "gate": "bounds", "reason": "out of bounds",
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
            "accepted": False, "gate": "", "reason": "",
        }
        if nz == Map2D5.OBSTACLE:
            entry["gate"] = "obstacle"
            entry["reason"] = "OBSTACLE landing"
            out.append(entry)
            continue

        iv = feasible_alpha_interval(X, Z, planner.V_max, planner.g)
        if iv is None:
            entry["gate"] = "physics"
            entry["reason"] = "infeasible (V_max / geometry)"
            out.append(entry)
            continue

        a = 0.5 * (iv[0] + iv[1])
        mc = min_clearance(
            c_s, c_g, a, m, planner.g, planner.robot_radius,
            planner.arc_endpoint_epsilon, planner.arc_max_step, obs,
        )
        entry["alpha_s"] = a
        entry["mc"] = mc
        if mc < 0:
            entry["gate"] = "clearance"
            entry["reason"] = f"arc clips terrain  mc={mc:+.3f} m"
        else:
            entry["accepted"] = True
            entry["reason"] = f"clears  mc={mc:+.3f} m"
        out.append(entry)

    return out


def gate_counts(cands: list[dict]) -> dict[str, int]:
    """Tally accepted candidates and rejections by which gate stopped them."""
    tally = {"accept": 0, "bounds": 0, "obstacle": 0, "physics": 0, "clearance": 0}
    for c in cands:
        tally["accept" if c["accepted"] else c["gate"]] += 1
    return tally


def find_interesting_cells(
    planner: HoppingAStarPlanner,
    path_cells: list[tuple[int, int]],
) -> list[tuple[tuple[int, int], list[dict]]]:
    """(cell, candidates) for each path node with at least one rejected candidate."""
    results = []
    for cell in path_cells[:-1]:   # goal has no outgoing hops
        cands = enumerate_ring_candidates(planner, cell)
        if any(not c["accepted"] for c in cands):
            results.append((cell, cands))
    return results


def path_cells_of(planner: HoppingAStarPlanner, path: list) -> list[tuple[int, int]]:
    return [planner.map_env.world_to_grid(x, y) for x, y in path]


# ---------------------------------------------------------------------------
# Figure scaffolding
# ---------------------------------------------------------------------------

def draw_topdown_base(m: Map2D5, fig, ax) -> None:
    """Render the elevation grid onto a caller-supplied axis.

    `Visualizer.__init__` creates its own figure, so demos that need to compose
    several artists onto one axis borrow `draw_map()` via `__new__` instead.
    """
    vis = Visualizer.__new__(Visualizer)
    vis.map_env = m
    vis.fig = fig
    vis.ax = ax
    ax.set_xlim(0, m.size_x)
    ax.set_ylim(0, m.size_y)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)", fontsize=LABEL_FS)
    ax.set_ylabel("Y (m)", fontsize=LABEL_FS)
    vis.draw_map()


def draw_topdown_compact(m: Map2D5, ax, colorbar_on=None) -> None:
    """Elevation grid without per-cell text — for multi-panel figures.

    `Visualizer.draw_map` writes the elevation into every cell, which is helpful
    on a single full-page figure and illegible on a 4-across contact sheet.  This
    renders the same YlOrBr scale via `imshow` instead.
    """
    import matplotlib.colors as mcolors
    import numpy as np

    grid = m.grid
    non_obs = grid[grid != Map2D5.OBSTACLE]
    z_min = float(non_obs.min()) if non_obs.size else 0.0
    z_max = float(non_obs.max()) if non_obs.size else 1.0
    if z_min == z_max:
        z_max = z_min + 1.0

    shown = np.where(grid == Map2D5.OBSTACLE, np.nan, grid)
    cmap = plt.get_cmap("YlOrBr").copy()
    cmap.set_bad("black")           # OBSTACLE sentinel renders black
    im = ax.imshow(
        shown, origin="lower", extent=(0, m.size_x, 0, m.size_y),
        cmap=cmap, norm=mcolors.Normalize(vmin=z_min, vmax=z_max),
        interpolation="nearest", alpha=0.85, zorder=0,
    )
    ax.set_xlim(0, m.size_x)
    ax.set_ylim(0, m.size_y)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)", fontsize=LABEL_FS)
    ax.set_ylabel("Y (m)", fontsize=LABEL_FS)
    if colorbar_on is not None:
        cbar = colorbar_on.colorbar(im, ax=ax, fraction=0.036, pad=0.02)
        cbar.set_label("Elevation (m)", fontsize=ANNOT_FS)
    return im


def segment_crosses_region(
    p0: tuple[float, float],
    p1: tuple[float, float],
    xmin: float, xmax: float,
    ymin: float, ymax: float,
    n: int = 60,
) -> bool:
    """Does the XY segment actually pass through the given rectangle?

    Comparing x-ranges alone is not enough: a detour hop can share a wall's
    x-range while passing well outside it in y, which would otherwise be
    misreported as a wall crossing.
    """
    for i in range(n + 1):
        t = i / n
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        if xmin <= x <= xmax and ymin <= y <= ymax:
            return True
    return False


def draw_ab_paths(
    ax,
    path_base: list,
    path_ball: list,
    diags_base: list,
    annotate: bool = True,
) -> None:
    """Overlay the baseline and ballistic paths, flagging baseline collisions."""
    xs_b = [p[0] for p in path_base]
    ys_b = [p[1] for p in path_base]
    ax.plot(xs_b, ys_b, color=C_BASE, linewidth=2.0, linestyle="--",
            zorder=6, label="Baseline (clearance OFF)")
    ax.plot(xs_b, ys_b, "x", color=C_BASE, markersize=10, zorder=7)

    for i, d in enumerate(diags_base):
        if d["feasible"] and d["mc"] < 0.0:
            p0, p1 = path_base[i], path_base[i + 1]
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]],
                    color=C_COLLIDE, linewidth=9.0, alpha=0.65, zorder=6.5,
                    solid_capstyle="round")
            if annotate:
                ax.annotate(
                    f"COLLIDES\nmc={d['mc']:+.3f} m",
                    (0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1])),
                    xytext=(0, 24), textcoords="offset points",
                    ha="center", fontsize=ANNOT_FS + 1, color="#b71c1c",
                    fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.25", fc="white",
                              ec="#b71c1c", lw=1.2), zorder=11,
                )

    xs_a = [p[0] for p in path_ball]
    ys_a = [p[1] for p in path_ball]
    ax.plot(xs_a, ys_a, color=C_BALL, linewidth=2.2, zorder=8,
            label="Ballistic (clearance ON)")
    ax.plot(xs_a, ys_a, "o", color=C_BALL, markersize=7, zorder=9)


def add_collision_legend(ax, extra: list | None = None) -> None:
    """Append the shared collision patch (and any extras) to the axis legend."""
    handles, labels = ax.get_legend_handles_labels()
    patches = [mpatches.Patch(color=C_COLLIDE, alpha=0.65,
                              label="baseline hop clips terrain (mc < 0)")]
    if extra:
        patches.extend(extra)
    ax.legend(handles + patches,
              labels + [p.get_label() for p in patches],
              loc="upper left", fontsize=ANNOT_FS)


def save(fig, save_path: str, dpi: int = PRESENTATION_DPI) -> None:
    fig.savefig(save_path, dpi=dpi)
    print(f"Saved: {save_path}")


def out_path(name: str) -> str:
    """Absolute path to `name` inside the `test/` directory."""
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), name)


# ---------------------------------------------------------------------------
# Console reporting
# ---------------------------------------------------------------------------

def print_ab_summary(
    m: Map2D5,
    path_base: list,
    diags_base: list,
    path_ball: list,
    diags_ball: list,
    extra_col=None,
) -> tuple[int, int]:
    """Print the standard B{i}/A{i} waypoint table.  Returns (n_base_bad, n_ball_bad).

    `extra_col` is an optional `f(point) -> str` appended to each waypoint line.
    """
    n_base_bad = n_bad_hops(diags_base)
    n_ball_bad = n_bad_hops(diags_ball)

    for tag, label, path, diags, bad in [
        ("B", "BASELINE ", path_base, diags_base, n_base_bad),
        ("A", "BALLISTIC", path_ball, diags_ball, n_ball_bad),
    ]:
        print("=" * 76 if tag == "B" else "-" * 76)
        print(f"{label}  ({len(path)} waypoints, {len(path) - 1} hops)")
        for i, p in enumerate(path):
            z = m.get_elevation(*p)
            note = ""
            if i > 0:
                d = diags[i - 1]
                if d["feasible"]:
                    note = (f"   X={d['X']:.2f} Z={d['Z']:+.2f}"
                            f" α={math.degrees(d['alpha_s']):.1f}°"
                            f" mc={d['mc']:+.3f}")
                else:
                    note = "   INFEASIBLE (physics gate)"
            suffix = f"  {extra_col(p)}" if extra_col else ""
            print(f"  {tag}{i}: ({p[0]:.2f},{p[1]:.2f})  z={z:.2f}{suffix}{note}")
        print(f"  → {bad} hop(s) infeasible or clipping terrain")
    print("=" * 76)

    return n_base_bad, n_ball_bad

"""Shared scaffolding for the `test/demo_*.py` presentation scripts.

Every demo builds a scenario, runs the ballistic planner against a baseline with
`disable_clearance=True`, and saves annotated PNGs via `out_path()` (see there
for where they land).  The planner construction, per-edge diagnosis and
ring-candidate enumeration were previously copy-pasted byte-identically across
five demos; they live here instead.

The scenario constants below are the *deck-wide* physics parameters. There is
no deck-wide `HOP_RADIUS` any more: the ring the planner scans is derived
per-state from the robot's own energy (`hopping_astar_planner.max_hop_radius`),
not a tuned constant, so `make_planner` no longer takes a `hop_radius`
argument at all. `V_MAX` still comes from `config.py`, since the robot's
capability is derived from `MAX_APEX_HEIGHT` rather than tuned per demo.
`param_caption()` renders the rest for the console.

Text size for every figure in the deck is set once here — see the styling block
below before adding any `fontsize=` argument to a demo.
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
    alpha_for_clearance,
    feasible_alpha_interval,
    injection_energy,
    landing_speed,
    max_hop_radius,
    min_clearance,
    takeoff_speed,
    terrain_profile,
)
from map2d5 import Map2D5
from visualizer import Visualizer


# ---------------------------------------------------------------------------
# Deck-wide scenario constants
# ---------------------------------------------------------------------------
N_ANGLES   = 16
V_MAX      = config.V_MAX   # derived from MAX_APEX_HEIGHT; 4.85 m/s

# ---------------------------------------------------------------------------
# Figure styling — sized to be read off a projected slide, not a laptop screen.
#
# These three constants plus the rcParams block below are the ONLY place text
# size is set. Every demo imports this module, so this is the single lever.
#
# If you add a `fontsize=` argument anywhere in a demo it OVERRIDES rcParams and
# silently opts that element out of this lever — which is exactly how the deck
# ended up with 66 hardcoded sizes that no longer agreed with each other. Only
# pass a size when the element genuinely needs to differ from the hierarchy.
#
# Sizes assume a canvas around 13.3 x 7.5 in (a full 16:9 slide). The smallest
# text, ANNOT_FS, is ~2.6% of that canvas height, which stays legible when
# projected.
# ---------------------------------------------------------------------------
PRESENTATION_DPI = 200
TITLE_FS = 22    # suptitles and the single-axes figure titles
LABEL_FS = 17    # axis labels, per-panel titles
ANNOT_FS = 14    # in-plot annotations, legends, tick labels
PANEL_TITLE_FS = 15   # titles above the small panels of a multi-hop strip, where
                      # a full-size axes title would collide with its neighbour

plt.rcParams.update({
    "font.size":        ANNOT_FS,
    "axes.titlesize":   PANEL_TITLE_FS,
    "axes.labelsize":   LABEL_FS,
    "xtick.labelsize":  ANNOT_FS,   # never styled before this — used to stay at 10 pt
    "ytick.labelsize":  ANNOT_FS,
    "legend.fontsize":  ANNOT_FS,
    "figure.titlesize": TITLE_FS,
})

# Canvas that drops onto a 16:9 slide at 100% — no downscaling, so the text
# lands at the size it was rendered.
SLIDE_FIGSIZE = (13.33, 7.5)

# Top-down maps are drawn with equal aspect on a square 5 x 5 m grid, so a 16:9
# canvas would only add dead margin either side. This fits a 7.5 in slide height
# at ~94% with room for the title and colourbar.
TOPDOWN_FIGSIZE = (10.0, 8.0)


def arcs_figsize(ncols: int) -> tuple[float, float]:
    """Canvas for a 2-row per-hop arc strip with `ncols` hops.

    Fixed at the slide canvas so the figure needs no downscaling in PowerPoint —
    which is what used to shrink the text. The strips previously grew their
    width with the panel count (`3.8 * ncols`), so a five-hop path produced a
    19.5 in canvas that PowerPoint then squashed to 68%, undoing any font
    increase.

    Panel count is data-dependent and has already swung between 3 and 5 between
    runs, so past ~5 panels the canvas does have to widen or the panels become
    unreadably narrow; the slide then takes a downscale, which is the lesser
    evil.
    """
    # 2.95 rather than a tight 2.7: panel titles are centred on their axes and
    # overhang it slightly, so the last column needs margin to overhang into or
    # it gets clipped at the canvas edge.
    return (max(SLIDE_FIGSIZE[0], 2.95 * ncols), SLIDE_FIGSIZE[1])

# Colour vocabulary — consistent across every figure in the deck.
C_BASE    = "#e65100"   # baseline path (clearance OFF)
C_BALL    = "#00695c"   # ballistic path (clearance ON)
C_COLLIDE = "#d50000"   # baseline hop that clips terrain
C_LOWMARG = "#ff6f00"   # accepted, but within TIGHT_BAND of the clearance gate
C_ACCEPT  = "#66bb6a"   # accepted ring candidate
C_REJECT  = "#c62828"   # rejected ring candidate
C_CHOSEN  = "#1b5e20"   # the candidate A* actually took
C_ANNOT   = "#0277bd"   # measurement annotations (Δz arrows etc.)

# How close to `MIN_CLEARANCE` an accepted hop has to be before it is worth
# flagging as tight.  Purely a presentation threshold — the planner itself sees
# a hard accept/reject boundary with nothing graded about it.
TIGHT_BAND = 0.05


def param_caption(planner: HoppingAStarPlanner | None = None) -> str:
    """One-line physics summary of the run.

    Printed to stdout by each demo rather than stamped on the figures: at
    presentation type size this is ~150 characters of parameter soup that
    crowds out the actual title, and nobody reads eight parameters off a
    projected slide. The values are also recorded in the results README.

    **Pass the planner** whenever the demo overrode a parameter. Without it this
    reports this module's deck defaults. Given a planner, the values are read
    off the object that actually produced the result, and the energy chain is
    reported too. `hop_radius` is state-dependent now (`max_hop_radius`), not a
    single planner-level scalar, so it is omitted here — see the per-hop table
    or `enumerate_ring_candidates` for the radius at a specific state.
    """
    if planner is None:
        return (
            f"V_max={V_MAX:.2f} m/s "
            f"(apex {config.MAX_APEX_HEIGHT} m) · leg={config.LEG_LENGTH} m · "
            f"robot_radius={config.ROBOT_RADIUS} m · "
            f"min_clearance={config.MIN_CLEARANCE} m · res={config.CELL_RESOLUTION} m · "
            f"W_energy={config.W_ENERGY} m/J"
        )
    return (
        f"leg={planner.leg_length} m · "
        f"robot_radius={planner.robot_radius} m · "
        f"min_clearance={planner.min_clearance_gate} m · "
        f"res={planner.map_env.resolution} m · μ={planner.mu}\n"
        f"energy chain: η={planner.eta} · m={planner.mass} kg · "
        f"E_inject_max={planner.e_inject_max:.2f} J · "
        f"min_apex={planner.min_apex} m · V_g_max={planner.V_g_max:.2f} m/s · "
        f"V_max={planner.V_max:.2f} m/s · speed_bin={planner.speed_bin} m/s\n"
        f"cost: xy_dist + {planner.w_energy} m/J × (E_inject + momentum lost)"
    )


# ---------------------------------------------------------------------------
# Planner construction and per-edge diagnosis
# ---------------------------------------------------------------------------

def make_planner(
    m: Map2D5,
    disable_clearance: bool,
    start: tuple[float, float],
    goal: tuple[float, float],
    V_max: float = V_MAX,
    **overrides,
) -> HoppingAStarPlanner:
    """Build a planner with every `config.*` field threaded in by name.

    `overrides` accepts any remaining `HoppingAStarPlanner` keyword (e.g.
    `w_energy=...`) so a demo can vary one parameter without restating
    the rest.
    """
    kwargs = dict(
        map_env=m,
        start=start,
        goal=goal,
        max_jump_height=config.MAX_JUMP_HEIGHT,
        w_energy=config.W_ENERGY,
        g=config.G_ACCEL,
        V_max=V_max,
        mu=config.MU,
        robot_radius=config.ROBOT_RADIUS,
        mass=config.ROBOT_MASS,
        eta=config.ETA_HOP,
        e_inject_max=config.E_INJECT_MAX,
        min_apex=config.MIN_APEX_HEIGHT,
        h_initial=config.H_INITIAL,
        V_g_max=config.V_G_MAX,
        speed_bin=config.SPEED_BIN,
        leg_length=config.LEG_LENGTH,
        min_clearance_gate=config.MIN_CLEARANCE,
        arc_max_step=config.ARC_SAMPLE_MAX_STEP,
        n_lateral=config.ARC_LATERAL_SAMPLES,
        obstacle_wall_extra=config.OBSTACLE_WALL_EXTRA,
        hop_scan_step=config.HOP_SCAN_STEP,
        hop_scan_step_ref_radius=config.HOP_SCAN_STEP_REF_RADIUS,
        disable_clearance=disable_clearance,
    )
    kwargs.update(overrides)
    return HoppingAStarPlanner(**kwargs)


def planner_alpha_interval(
    planner: HoppingAStarPlanner,
    m: Map2D5,
    p0: tuple[float, float],
    p1: tuple[float, float],
    X: float,
    Z: float,
    v_g_in: float | None = None,
) -> tuple[float, float] | None:
    """`feasible_alpha_interval` fed the planner's own inputs.

    Two things are invisible in `X` and `Z` alone, and a diagnostic that guesses
    at either will silently disagree with the planner it is meant to explain:

      * the friction cone needs the surface normals at both contacts plus the
        hop heading. The bare function falls back to level ground, so on sloped
        terrain the numbers diverge. `Map2D5.surface_normals` is memoised, so
        looking them up is cheap.
      * the energy band needs the speed the robot ARRIVED at. Defaults to the
        planner's start-of-chain speed, which is right only for the first hop
        — `diagnose_path` threads the real value through for the rest.
    """
    if v_g_in is None:
        v_g_in = planner.v_g_initial
    normals = m.surface_normals()
    r0, c0 = m.world_to_grid(p0[0], p0[1])
    r1, c1 = m.world_to_grid(p1[0], p1[1])
    return feasible_alpha_interval(
        X, Z, planner.V_max, planner.g,
        mu=planner.mu,
        n_s=tuple(normals[r0, c0]),
        n_g=tuple(normals[r1, c1]),
        theta=math.atan2(p1[1] - p0[1], p1[0] - p0[0]),
        v_s_min=math.sqrt(planner.eta) * v_g_in,
        e_inject_max=planner.e_inject_max,
        mass=planner.mass,
        min_apex=planner.min_apex,
        V_g_max=planner.V_g_max,
    )


def diagnose_edge(
    planner: HoppingAStarPlanner,
    m: Map2D5,
    p0: tuple[float, float],
    p1: tuple[float, float],
    v_g_in: float | None = None,
) -> dict:
    """Re-score one hop with the ballistic criterion, whichever planner produced it.

    Pass the *clearance-ON* planner even when diagnosing the baseline path —
    that is what makes the A/B comparison honest: both paths are judged by the
    same gate.  All physics parameters are read off the planner, so the
    diagnosis can never drift from what the planner itself evaluated.

    `v_g_in` is the speed the robot arrived at `p0` with; it sets the energy
    band and therefore the takeoff angle. Use `diagnose_path` rather than
    calling this in a loop, or every hop after the first will be scored against
    the start-of-chain energy.

    The returned dict adds `v_s`, `v_g`, `e_inject` and `apex_drop` on feasible
    hops, so callers can chain and can report the energy story.
    """
    z0 = float(m.get_elevation(*p0))
    z1 = float(m.get_elevation(*p1))
    X  = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    Z  = z1 - z0
    if v_g_in is None:
        v_g_in = planner.v_g_initial
    # Can the robot's body actually rest at the landing cell? A hop can be a
    # perfectly good parabola and still be invalid because the robot would be
    # standing inside a wall when it got there — the baseline planner skips
    # this check, so it is exactly where baseline paths tend to go wrong.
    landing = m.world_to_grid(p1[0], p1[1])
    standable = bool(planner._standable[landing[0], landing[1]])
    infeasible = {"feasible": False, "standable": standable,
                  "X": X, "Z": Z, "alpha_s": None, "mc": -math.inf,
                  "z0": z0, "z1": z1, "v_g_in": v_g_in,
                  "v_s": None, "v_g": None, "e_inject": None,
                  "apex_drop": None}

    iv = planner_alpha_interval(planner, m, p0, p1, X, Z, v_g_in)
    if iv is None:
        return infeasible
    profile = terrain_profile(
        (p0[0], p0[1], z0), (p1[0], p1[1], z1),
        m, planner.robot_radius, planner.leg_length,
        planner.arc_max_step, planner._obstacle_fill, planner.n_lateral,
        min_clearance_gate=planner.min_clearance_gate,
    )
    if profile is None:
        return infeasible
    # Follows the planner's own angle rule, escalation included, so the
    # reported alpha is the one the planner would actually have flown.
    a, mc = alpha_for_clearance(
        profile, iv[0], iv[1], planner.min_clearance_gate,
    )
    v_s = takeoff_speed(X, Z, a, planner.g)
    T = math.tan(a)
    return {"feasible": True, "standable": standable,
            "X": X, "Z": Z, "alpha_s": a, "mc": mc,
            "z0": z0, "z1": z1, "v_g_in": v_g_in,
            "v_s": v_s,
            "v_g": landing_speed(v_s, Z, planner.g),
            "e_inject": injection_energy(
                v_s, math.sqrt(planner.eta) * v_g_in, planner.mass,
            ),
            "apex_drop": (X * T - 2.0 * Z) ** 2 / (4.0 * (X * T - Z))}


def diagnose_path(planner: HoppingAStarPlanner, m: Map2D5, path: list) -> list[dict]:
    """`diagnose_edge` over every consecutive pair in `path`, CHAINED and BINNED.

    Chained, not mapped: the robot cannot shed energy, so each hop's takeoff
    speed is floored by the previous hop's landing speed. Diagnosing the hops
    independently would score every one of them against the start-of-chain
    energy and report angles the robot could not actually have flown.

    Binned for the same reason, one level down: the planner's search state
    carries a QUANTISED landing speed, so that — not the exact `v_g` — is what
    set the takeoff angle it chose. Chaining the exact value instead drifts by
    up to `speed_bin/2` per hop, which is enough to move the angle and hence the
    reported clearance across the gate. It shows up as this function accusing
    the ballistic planner's own path of containing a hop the planner accepted.

    Once a hop is infeasible the chain has no landing speed to continue from,
    so the remaining hops fall back to the start energy. They are already
    flagged infeasible-downstream by that point; `n_bad_hops` counts them.
    """
    diags = []
    v_g = planner.v_g_initial
    for i in range(len(path) - 1):
        binned = planner._speed_bin(v_g) * planner.speed_bin
        d = diagnose_edge(planner, m, path[i], path[i + 1], binned)
        diags.append(d)
        if d["v_g"] is not None:
            v_g = d["v_g"]
    return diags


def n_bad_hops(diags: list[dict], gate: float = config.MIN_CLEARANCE) -> int:
    """Hops the ballistic planner would refuse, for any reason.

    Three ways to be refused: no takeoff angle within the leg's budget, no
    angle that keeps the arc clear of the terrain, or a landing cell where the
    body cannot rest without overlapping terrain.
    """
    return sum(1 for d in diags
               if not d["feasible"] or d["mc"] < gate or not d.get("standable", True))


# ---------------------------------------------------------------------------
# Arc geometry helpers
# ---------------------------------------------------------------------------

def arc_z_at(
    c_s: tuple[float, float, float],
    Z: float,
    X: float,
    alpha_s: float,
    u: float,
    g: float = config.G_ACCEL,
    leg_length: float = config.LEG_LENGTH,
) -> float:
    """Height of the CoM parabola (Campana Eq. 2) at horizontal distance `u`.

    `X`/`Z` are the hop's horizontal span and rise, matching `diagnose_edge`.
    `c_s[2]` is the *terrain* height at takeoff; the arc runs `leg_length`
    above it, so this returns a CoM height, not a ground clearance.  `g` is
    accepted for call-site compatibility but is not needed by the closed form.
    """
    tan_a = math.tan(alpha_s)
    # Campana Eq. 4 rearranged: g*X^2 / (2*xdot^2) == X*tan(alpha) - Z
    k = (X * tan_a - Z) / (X * X)
    return c_s[2] + leg_length + u * tan_a - k * u * u


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
    v_g_in: float | None = None,
    n_angles: int = N_ANGLES,
) -> list[dict]:
    """All `n_angles` full-radius ring candidates from `cell`, with a verdict each.

    Scans only a single full-radius ring, not the planner's scanline disk
    fill (`HoppingAStarPlanner._generate_hop_neighbors`), so the figure stays
    readable.  Out-of-bounds and physics-infeasible candidates are kept so
    they show up as rejections.

    `v_g_in` is the speed the robot arrived at `cell` with, which sets the
    energy band and so decides which candidates are reachable at all — and,
    via `max_hop_radius`, the ring radius itself. It defaults to the
    start-of-chain speed; pass the value from `path_hops` or `diagnose_path`
    when illustrating a cell partway along a path, or the panel will show a
    different robot's options than the one in the figure.

    Each entry carries a `gate` field naming *which* check rejected it —
    `"bounds"`, `"obstacle"`, `"stance"`, `"physics"`, `"clearance"`, or `""`
    when accepted.  Distinguishing them matters: a demo that claims "clearance
    rejected this" must actually check, since a candidate can equally well die
    on physics (no takeoff angle within the leg's budget) or on stance (the body
    cannot rest at the landing cell without overlapping terrain).
    """
    m = planner.map_env
    px, py = m.grid_to_world(*cell)
    pz = float(m.grid[cell[0], cell[1]])
    obs = planner._obstacle_fill
    if v_g_in is None:
        v_g_in = planner.v_g_initial
    r = max_hop_radius(
        v_g_in, planner.eta, planner.e_inject_max, planner.mass, planner.g,
        planner.V_max,
    )

    seen: set = {cell}
    out: list[dict] = []
    for i in range(n_angles):
        a = 2.0 * math.pi * i / n_angles
        dx, dy = math.cos(a), math.sin(a)
        tx = px + r * dx
        ty = py + r * dy
        if not m.is_within_bounds(tx, ty):
            out.append({
                "cell": None, "c_s": (px, py, pz), "c_g": None,
                "X": None, "Z": None, "r": r,
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
            "X": X, "Z": Z, "r": r,
            "alpha_s": None, "mc": None,
            "accepted": False, "gate": "", "reason": "",
        }
        if nz == Map2D5.OBSTACLE:
            entry["gate"] = "obstacle"
            entry["reason"] = "OBSTACLE landing"
            out.append(entry)
            continue

        if not planner._standable[nb[0], nb[1]]:
            entry["gate"] = "stance"
            entry["reason"] = "body cannot rest here"
            out.append(entry)
            continue

        iv = planner_alpha_interval(planner, m, (px, py), (nx, ny), X, Z, v_g_in)
        if iv is None:
            entry["gate"] = "physics"
            entry["reason"] = "infeasible (energy band / min-apex / cone / geometry)"
            out.append(entry)
            continue

        profile = terrain_profile(
            c_s, c_g, m, planner.robot_radius, planner.leg_length,
            planner.arc_max_step, obs, planner.n_lateral,
        )
        gate = planner.min_clearance_gate
        a, mc = alpha_for_clearance(profile, iv[0], iv[1], gate)
        entry["alpha_s"] = a
        entry["mc"] = mc
        if mc < gate:
            entry["gate"] = "clearance"
            entry["reason"] = f"arc clips terrain  mc={mc:+.3f} m"
        else:
            entry["accepted"] = True
            entry["reason"] = f"clears  mc={mc:+.3f} m"
        out.append(entry)

    return out


def gate_counts(cands: list[dict]) -> dict[str, int]:
    """Tally accepted candidates and rejections by which gate stopped them."""
    tally = {"accept": 0, "bounds": 0, "obstacle": 0, "stance": 0,
             "physics": 0, "clearance": 0}
    for c in cands:
        tally["accept" if c["accepted"] else c["gate"]] += 1
    return tally


def find_interesting_cells(
    planner: HoppingAStarPlanner,
    path_cells: list[tuple[int, int]],
    gate: str | None = None,
) -> list[tuple[tuple[int, int], list[dict]]]:
    """(cell, candidates) for each path node with at least one rejected candidate.

    Pass `gate` (e.g. `"clearance"`) to keep only nodes where that *specific*
    check did the rejecting.  A demo whose whole point is the clearance gate
    should ask for clearance rejections rather than settling for whichever
    node happens to have an off-map candidate.
    """
    results = []
    for cell in path_cells[:-1]:   # goal has no outgoing hops
        cands = enumerate_ring_candidates(planner, cell)
        if gate is None:
            hit = any(not c["accepted"] for c in cands)
        else:
            hit = any(c["gate"] == gate for c in cands)
        if hit:
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
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
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
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    if colorbar_on is not None:
        cbar = colorbar_on.colorbar(im, ax=ax, fraction=0.036, pad=0.02)
        cbar.set_label("Elevation (m)")
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
        if d["feasible"] and d["mc"] < config.MIN_CLEARANCE:
            p0, p1 = path_base[i], path_base[i + 1]
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]],
                    color=C_COLLIDE, linewidth=9.0, alpha=0.65, zorder=6.5,
                    solid_capstyle="round")
            if annotate:
                ax.annotate(
                    f"COLLIDES\nmc={d['mc']:+.3f} m",
                    (0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1])),
                    xytext=(0, 24), textcoords="offset points",
                    ha="center", color="#b71c1c",
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
                              label="baseline hop fails the clearance gate")]
    if extra:
        patches.extend(extra)
    ax.legend(handles + patches,
              labels + [p.get_label() for p in patches],
              loc="upper left")


def save(fig, save_path: str, dpi: int = PRESENTATION_DPI) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
    fig.savefig(save_path, dpi=dpi)
    print(f"Saved: {save_path}")


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DEFAULT_OUT_DIR = os.path.join("results", "energy_aware_planning")


def out_dir() -> str:
    """Directory demo artefacts are written to, created if missing.

    Defaults to `results/energy_aware_planning/` so a plain
    `python test/demo_*.py` lands in the current results set.
    Override with `$PLANNER_OUT_DIR`;
    relative overrides resolve against the repo root, not the working
    directory, because the demos are documented as runnable from anywhere.
    """
    d = os.environ.get("PLANNER_OUT_DIR", _DEFAULT_OUT_DIR)
    if not os.path.isabs(d):
        d = os.path.join(_REPO_ROOT, d)
    os.makedirs(d, exist_ok=True)
    return d


def out_path(name: str) -> str:
    """Absolute path to `name` inside `out_dir()`."""
    return os.path.join(out_dir(), name)


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

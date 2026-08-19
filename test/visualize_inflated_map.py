"""Draw the inflated height field so it can be checked by eye.

`Map2D5.inflated_field` bakes the robot's size into the terrain, which is what
lets the planner treat the robot as a POINT. That machinery is otherwise only
verifiable through the numeric assertions in `test/test_inflated_field.py`; this
script renders it.

The robot's body is a CYLINDER of `ROBOT_RADIUS` with a FLAT BOTTOM and SQUARE
EDGES, standing on the foot, carrying a uniform `MIN_CLEARANCE` safety margin.
There is ONE field, and the whole model is that field plus one constant:

  * **terrain max** (`inflated_field(ROBOT_RADIUS + MIN_CLEARANCE)`) — "how tall
    is the tallest terrain within reach of me?" Nothing is added: flat ground
    reads 0.00, a wall reads its true height. A flat-topped plateau with sharp
    edges, widened sideways by the body's reach.

  * **clearance bound** — that field plus `MIN_CLEARANCE`, i.e. how high the
    FOOT must be. Drawn as a second line so the margin is visible, but it is not
    a second array: it is the same numbers shifted up by a constant.

There is deliberately NO TAPER. A tapered (rounded) field is the shape of a
SPHERE rolled over the terrain, and it charges the body radius as vertical
clearance underneath the foot, where a flat-bottomed cylinder has no extent at
all — demanding `body + margin` of headroom over flat ground when only `margin`
is called for. The margin around a square-edged body is square-edged too, which
is why it reduces to a constant.

Defaults to `maps/low_wall.py` (one 0.4 m ridge, 0.2 m thick, spanning the map in
y) because every number in the figure can then be checked by hand.

Run:
    python test/visualize_inflated_map.py              # low_wall
    python test/visualize_inflated_map.py tall_stairs  # any module in maps/
"""

import importlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
matplotlib.use("Agg")   # never try to open a window

import matplotlib.pyplot as plt
import numpy as np

import config
import demo_common          # imported for its deck-wide rcParams styling + save()
from map2d5 import Map2D5


#: Results land here rather than in `demo_common.out_dir()`, which defaults to
#: `results/energy_aware_planning/` — an unrelated results set.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DEFAULT_OUT_DIR = os.path.join("results", "inflated_map")

BODY = "#1b6ca8"     # the clearance-bound / safety colour
DETECT = "#d95f02"   # the terrain-max / detector colour


def out_path(name: str) -> str:
    """Absolute path to `name` in the output dir, honouring `$PLANNER_OUT_DIR`.

    Relative overrides resolve against the repo root, not the working directory,
    matching `demo_common.out_dir()` — this script is runnable from anywhere.
    """
    d = os.environ.get("PLANNER_OUT_DIR", _DEFAULT_OUT_DIR)
    if not os.path.isabs(d):
        d = os.path.join(_REPO_ROOT, d)
    return os.path.join(d, name)


def load_map(name: str) -> Map2D5:
    try:
        return importlib.import_module(f"maps.{name}").build()
    except ModuleNotFoundError:
        maps_dir = os.path.join(_REPO_ROOT, "maps")
        avail = sorted(
            f[:-3] for f in os.listdir(maps_dir)
            if f.endswith(".py") and not f.startswith("_")
        )
        print(f"Unknown map {name!r}. Available: {', '.join(avail)}")
        raise SystemExit(1)


def swept_outline(cx: float, foot: float, body_r: float, margin: float,
                  height: float) -> np.ndarray:
    """The swept safety shape: a SQUARE-EDGED box around the body.

    Vertical sides at `cx +- (body_r + margin)` and a flat bottom `margin` below
    the foot, meeting at right angles — the exact boundary the clearance field
    encodes. No corner rounding: the margin is measured vertically and laterally
    on its own axis, not as a straight-line distance around the corner, which is
    what lets it collapse to a constant. Returned as an (N, 2) polyline, drawn
    open at the top.
    """
    total = body_r + margin
    return np.asarray([
        (cx - total, foot + height),
        (cx - total, foot - margin),
        (cx + total, foot - margin),
        (cx + total, foot + height),
    ])


def main(argv: list[str]) -> int:
    map_name = argv[1] if len(argv) > 1 else "low_wall"
    m = load_map(map_name)

    # The exact radius the planner inflates by: `hopping_astar_planner` builds
    # `_inflated` with this same argument and `inflated_field` memoises, so this
    # is literally the planner's array.
    body_r = config.ROBOT_RADIUS
    margin = config.MIN_CLEARANCE
    R = body_r + margin
    terrain_max = m.inflated_field(R)
    clearance = terrain_max + margin   # the same numbers, shifted by the gate

    row = m.rows // 2
    xs = (np.arange(m.cols) + 0.5) * m.resolution
    terr_line = m.grid[row]
    clr_line = clearance[row]
    max_line = terrain_max[row]

    # ---- console report: the figure's claims, checkable without opening it ----
    flat_cols = np.isclose(terr_line, terr_line.min())
    print(f"map                    : {map_name}  ({m.cols}x{m.rows} cells @ "
          f"{m.resolution} m)")
    print(f"body / margin          : {body_r} m cylinder + {margin} m margin "
          f"= {R} m lateral reach")
    print(f"clearance over flat    : {clr_line[flat_cols].min():.3f} m"
          f"   (expect MIN_CLEARANCE = {margin}, NOT {R})")
    print(f"clearance max          : {clr_line.max():.3f} m"
          f"   (expect terrain max {terr_line.max():.2f} + {margin})")
    print(f"terrain-max over flat  : {max_line[flat_cols].min():.3f} m"
          f"   (expect 0.00 — nothing added)")
    print(f"terrain-max peak       : {max_line.max():.3f} m"
          f"   (expect terrain max {terr_line.max():.2f} exactly)")
    raised = max_line > terr_line.min() + 1e-9
    if raised.any():
        print(f"detected band width    : {raised.sum() * m.resolution:.2f} m "
              f"(terrain footprint widened by the {R} m reach)")

    # ------------------------------- figure -------------------------------- #
    fig = plt.figure(figsize=demo_common.SLIDE_FIGSIZE)
    gs = fig.add_gridspec(2, 2, height_ratios=[1.05, 1.0], hspace=0.38,
                          wspace=0.18, bottom=0.17)
    ax_terr = fig.add_subplot(gs[0, 0])
    ax_infl = fig.add_subplot(gs[0, 1])
    ax_side = fig.add_subplot(gs[1, :])

    extent = [0.0, m.size_x, 0.0, m.size_y]
    # OBSTACLE cells inflate to +inf, which would blow out the colour scale and
    # render everything else flat. No shipped map has any, but `set_obstacle_region`
    # exists, so scale off the finite values and let obstacles saturate.
    finite = clearance[np.isfinite(clearance)]
    vmax = float(max(finite.max() if finite.size else 0.0, m.grid.max()))
    vmin = float(min(terrain_max.min(), m.grid.min()))

    for ax, data, title in (
        (ax_terr, m.grid, "Terrain (raw)"),
        (ax_infl, clearance,
         f"Clearance bound (body {body_r} + margin {margin} m)"),
    ):
        im = ax.imshow(data, origin="lower", extent=extent, cmap="viridis",
                       vmin=vmin, vmax=vmax, interpolation="nearest")
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal")
        ax.axhline((row + 0.5) * m.resolution, color="white", ls=":", lw=1.5)

    raised_cols = np.where(m.grid[row] > m.grid.min() + 1e-9)[0]
    if raised_cols.size:
        x0 = raised_cols[0] * m.resolution
        x1 = (raised_cols[-1] + 1) * m.resolution
        for ax in (ax_terr, ax_infl):
            for x in (x0, x1):
                ax.axvline(x, color="red", ls="--", lw=1.6)

    cbar = fig.colorbar(im, ax=[ax_terr, ax_infl], shrink=0.85, pad=0.02)
    cbar.set_label("height (m)")

    # ----------------------------- side view ------------------------------- #
    # Cropped to the terrain feature and drawn at EQUAL ASPECT, so the body
    # glyph below is not distorted — its proportions are the whole point.
    if raised_cols.size:
        feat_lo = raised_cols[0] * m.resolution
        feat_hi = (raised_cols[-1] + 1) * m.resolution
        pad_x = max(2.0 * R, 1.0)
        x_lo, x_hi = max(0.0, feat_lo - pad_x), min(m.size_x, feat_hi + pad_x)
    else:
        x_lo, x_hi = 0.0, m.size_x

    ax_side.fill_between(xs, terr_line.min(), terr_line, step="mid",
                         color="#8d6e4a", label="terrain")
    ax_side.step(xs, max_line, where="mid", color=DETECT, lw=2.2,
                 label="terrain max in reach — no margin (the detector)")
    ax_side.plot(xs, clr_line, color=BODY, lw=2.6,
                 label="clearance bound — required FOOT height")
    ax_side.axhline(terr_line.min() + margin, color=BODY, ls=":", lw=1.6)

    # Measured bar rather than a text label: the label is wider than the flat
    # run it has to sit in, so it always collided with the rising curve.
    x_bar = x_lo + 0.16
    ax_side.annotate("", xy=(x_bar, terr_line.min()),
                     xytext=(x_bar, terr_line.min() + margin),
                     arrowprops=dict(arrowstyle="<->", color=BODY, lw=1.6))
    ax_side.text(x_bar + 0.05, terr_line.min() + margin / 2,
                 f"{margin:.2f} m", color=BODY, va="center")

    if raised_cols.size:
        # The body + margin, placed so its right flank meets the wall's corner:
        # the clearance curve is the locus of this shape's foot, which is why
        # the curve's shoulder is round rather than square.
        cx = feat_lo - body_r
        ci = int(np.clip(np.searchsorted(xs, cx), 0, m.cols - 1))
        foot = float(clr_line[ci])
        h_body = config.LEG_LENGTH + body_r
        ax_side.add_patch(plt.Rectangle((xs[ci] - body_r, foot),
                                        2 * body_r, h_body,
                                        facecolor=BODY, alpha=0.22,
                                        edgecolor=BODY, lw=1.4))
        out = swept_outline(xs[ci], foot, body_r, margin, h_body)
        ax_side.plot(out[:, 0], out[:, 1], color=BODY, lw=1.5, ls="--",
                     alpha=0.9)
        ax_side.plot([xs[ci]], [foot], "o", color=BODY, ms=6)
        ax_side.annotate("body + margin",
                         xy=(xs[ci] - body_r - margin, foot), xytext=(-14, 0),
                         textcoords="offset points", ha="right", va="center",
                         color=BODY,
                         arrowprops=dict(arrowstyle="->", color=BODY))
        ax_side.annotate(f"{clr_line.max():.2f} m",
                         xy=(xs[int(np.argmax(clr_line))], clr_line.max()),
                         xytext=(0, 8), textcoords="offset points",
                         ha="center", color=BODY)

    ax_side.set_title(f"Side view — cross-section at y = "
                      f"{(row + 0.5) * m.resolution:.2f} m")
    ax_side.set_xlabel("x (m)")
    ax_side.set_ylabel("height (m)")
    ax_side.set_xlim(x_lo, x_hi)
    ax_side.set_ylim(terr_line.min() - margin - 0.06, clr_line.max() + 0.30)
    ax_side.set_aspect("equal", adjustable="box")
    ax_side.grid(alpha=0.25)
    # Anchored to the FIGURE, not the axes: equal aspect shrinks the axes box to
    # an unpredictable height, so an axes-relative offset drops the legend off
    # the bottom of the canvas.
    fig.legend(*ax_side.get_legend_handles_labels(), loc="lower center",
               bbox_to_anchor=(0.5, 0.012), ncol=3, framealpha=0.92)

    fig.suptitle(f"Inflated height field — {map_name}   "
                 f"(cylinder r = {body_r} m + margin {margin} m)")

    demo_common.save(fig, out_path(f"inflated_map_{map_name}.png"))
    plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

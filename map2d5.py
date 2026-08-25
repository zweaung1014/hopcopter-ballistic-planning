"""2.5D Map representation: a 2D grid with elevation (z) values per cell."""

import math

import numpy as np


class Map2D5:
    """A 2.5D map stored as a 2D grid of elevation values.

    Each cell holds a z-value (elevation in meters). A value of -1 indicates
    an obstacle. The map origin (0, 0) is at the bottom-left corner.
    """

    OBSTACLE = -1.0

    def __init__(self, size_x: float, size_y: float, resolution: float, default_z: float = 0.0):
        self.size_x = size_x
        self.size_y = size_y
        self.resolution = resolution
        self.cols = int(round(size_x / resolution))
        self.rows = int(round(size_y / resolution))
        self.grid = np.full((self.rows, self.cols), default_z, dtype=np.float64)
        # Single-entry memo for the obstacle-substituted grid, keyed on the fill
        # value (see `_filled_grid`). Maps are built once and then only read, so
        # caching is safe; `paint_region`/`set_obstacle*` invalidate it anyway.
        self._fill_cache_key: float | None = None
        self._fill_cache_grid: np.ndarray | None = None
        # Memo for the per-cell surface normals (see `surface_normals`).
        self._normal_cache: np.ndarray | None = None
        # Memo for the inflated height fields, keyed on their arguments (see
        # `inflated_field`). The planner builds one and reads it every edge.
        self._inflated_cache: dict[tuple[float, float, float], np.ndarray] = {}
        # Memo for the steep-edge masks, keyed on (grade threshold, finite_only)
        # (see `steep_mask`). `inflated_field` asks for one every time it builds.
        self._steep_cache: dict[tuple[float, bool], np.ndarray] = {}

    def _invalidate_caches(self) -> None:
        """Drop every grid-derived memo. Call after any write to `self.grid`."""
        self._fill_cache_key = None
        self._fill_cache_grid = None
        self._normal_cache = None
        self._inflated_cache = {}
        self._steep_cache = {}

    def _filled_grid(self, obstacle_fill: float | None) -> np.ndarray:
        """`self.grid` with OBSTACLE cells replaced by `obstacle_fill`.

        Memoised on the fill value: the clearance check calls this once per
        sampled arc, always with the same fill, so recomputing the `np.where`
        every time would dominate the lookup cost.
        """
        if obstacle_fill is None:
            return self.grid
        if self._fill_cache_key != obstacle_fill or self._fill_cache_grid is None:
            self._fill_cache_grid = np.where(
                self.grid == self.OBSTACLE, obstacle_fill, self.grid
            )
            self._fill_cache_key = obstacle_fill
        return self._fill_cache_grid

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world coordinates (meters) to grid indices (row, col)."""
        col = int(x / self.resolution)
        row = int(y / self.resolution)
        col = max(0, min(col, self.cols - 1))
        row = max(0, min(row, self.rows - 1))
        return row, col

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        """Convert grid indices to world coordinates (center of cell)."""
        x = (col + 0.5) * self.resolution
        y = (row + 0.5) * self.resolution
        return x, y

    def is_obstacle(self, x: float, y: float) -> bool:
        """Check if the world position (x, y) is an obstacle."""
        if x < 0 or x >= self.size_x or y < 0 or y >= self.size_y:
            return True
        row, col = self.world_to_grid(x, y)
        return self.grid[row, col] == self.OBSTACLE

    def get_elevation(self, x: float, y: float) -> float:
        """Get the elevation (z value) at world position (x, y)."""
        row, col = self.world_to_grid(x, y)
        return self.grid[row, col]

    def sample_bilinear(
        self,
        xs,
        ys,
        obstacle_fill: float | None = None,
    ) -> np.ndarray:
        """Vectorized bilinear elevation lookup at continuous world coords.

        `xs`/`ys` are broadcast-compatible array-likes; the return has their
        broadcast shape. `grid_to_world(row, col)` places cell centers at
        `((col+0.5)*res, (row+0.5)*res)`, so the continuous grid coordinate is:
            cx = x / resolution - 0.5   (column axis)
            cy = y / resolution - 0.5   (row axis)
        Neighbour indices are clamped to the map, so queries near the border
        degrade to nearest-edge rather than raising.

        `obstacle_fill` (if given) is substituted for OBSTACLE cells before
        mixing, letting the ballistic clearance check treat them as tall walls
        without mutating the grid. The substituted grid is memoised, so the
        per-call cost is just the gather + mix.

        The clearance check samples a whole arc at once, so batching here is
        what keeps the per-edge cost to a single numpy round-trip instead of
        ~130 scalar calls.
        """
        g = self._filled_grid(obstacle_fill)
        res = self.resolution

        cx = np.asarray(xs, dtype=np.float64) / res - 0.5
        cy = np.asarray(ys, dtype=np.float64) / res - 0.5

        c0 = np.floor(cx)
        r0 = np.floor(cy)

        # Fractional offsets, clamped in case (x, y) fell outside the grid.
        fx = np.clip(cx - c0, 0.0, 1.0)
        fy = np.clip(cy - r0, 0.0, 1.0)

        c0i = c0.astype(np.int64)
        r0i = r0.astype(np.int64)
        c0_c = np.clip(c0i, 0, self.cols - 1)
        c1_c = np.clip(c0i + 1, 0, self.cols - 1)
        r0_c = np.clip(r0i, 0, self.rows - 1)
        r1_c = np.clip(r0i + 1, 0, self.rows - 1)

        z00 = g[r0_c, c0_c]
        z10 = g[r0_c, c1_c]
        z01 = g[r1_c, c0_c]
        z11 = g[r1_c, c1_c]

        z0 = (1.0 - fx) * z00 + fx * z10  # bottom edge (row r0)
        z1 = (1.0 - fx) * z01 + fx * z11  # top edge    (row r1)
        return (1.0 - fy) * z0 + fy * z1

    def get_elevation_bilinear(
        self,
        x: float,
        y: float,
        obstacle_fill: float | None = None,
    ) -> float:
        """Scalar bilinear elevation lookup — thin wrapper over `sample_bilinear`."""
        return float(self.sample_bilinear(x, y, obstacle_fill))

    def paint_region(
        self,
        z: float,
        x_min: float | None = None,
        x_max: float | None = None,
        y_min: float | None = None,
        y_max: float | None = None,
    ) -> None:
        """Set every cell whose *center* lies in [x_min, x_max) x [y_min, y_max) to `z`.

        Resolution-invariant: the painted physical extent depends only on the
        world bounds, never on `self.resolution`. `None` means unbounded on
        that side.

        Prefer this over `grid[:, slice(a, b)] = z` (which hard-codes a cell
        count) and over `world_to_grid(...)` + an inclusive `+1` (which
        overshoots by up to one cell, so the footprint shrinks as the grid gets
        finer). Both idioms silently rescale the terrain when the resolution
        changes.
        """
        col_centers = (np.arange(self.cols) + 0.5) * self.resolution
        row_centers = (np.arange(self.rows) + 0.5) * self.resolution

        eps = 1e-9
        cmask = np.ones(self.cols, dtype=bool)
        if x_min is not None:
            cmask &= col_centers >= x_min - eps
        if x_max is not None:
            cmask &= col_centers < x_max - eps

        rmask = np.ones(self.rows, dtype=bool)
        if y_min is not None:
            rmask &= row_centers >= y_min - eps
        if y_max is not None:
            rmask &= row_centers < y_max - eps

        self.grid[np.ix_(rmask, cmask)] = z
        self._invalidate_caches()

    def standable_mask(
        self,
        radius: float,
        clearance: float,
        steep_grade: float,
    ) -> np.ndarray:
        """Boolean grid of cells where the robot can stand without clipping terrain.

        Standable iff the cell is NOT within `radius + clearance` of an
        OBSTACLE column AND NOT within `radius + clearance` of a finite-height
        steep edge (a real riser or wall, as opposed to an infinitely tall
        OBSTACLE column) — two independent boolean dilations, via
        `_dilate_bool`, of two independent source sets. There is no height
        comparison here at all, unlike `inflated_field`/`clearance_floor_alpha`:
        an OBSTACLE's height is irrelevant once it's known to be an OBSTACLE
        (its column is infinitely tall by construction), and a finite steep
        edge is treated as unconditionally blocking within reach rather than
        weighed against how tall the robot's own body is — the "smear a few
        cells thick" model the terrain categories call for, not a clearance
        calculation.

        Because only terrain EDGES feed `steep_mask` (see its docstring), the
        max standable constant grade is exactly `steep_grade` — a ramp below
        the threshold has no edges, so `finite_only` dilation has no source
        cells there and every cell on it is standable however steep. A ramp
        AT or above the threshold has every adjacent pair exceeding it, so
        every cell on it becomes a dilation source and the whole ramp goes
        non-standable at once — a step discontinuity, not a smooth limit.

        Dilating from both sides of a standable strip narrower than
        `2 * (radius + clearance)` closes it off entirely — intended: a gap
        too narrow for the body plus its margin on both sides isn't standable
        anywhere in it, and this falls out of plain dilation with no extra
        "is the interior big enough" logic needed.

        Computed once per planner. Screening landing cells against this is far
        cheaper than discovering the same collision by marching an arc.
        """
        reach = radius + clearance
        obstacle_blocked = self._dilate_bool(self.grid == self.OBSTACLE, reach)
        edge_blocked = self._dilate_bool(
            self.steep_mask(steep_grade, finite_only=True), reach
        )
        return ~obstacle_blocked & ~edge_blocked

    def _dilate_bool(self, source: np.ndarray, reach: float) -> np.ndarray:
        """Boolean sideways dilation: True within `reach` of a True source cell.

        Same offset-enumeration idiom as `inflated_field`'s loop (shift a
        padded array over every integer cell offset within `reach`, skip
        anything the circle doesn't actually reach), specialised to booleans:
        OR instead of max, False-padding instead of -inf-padding. No
        `lookup_pad` here — unlike `inflated_field`, this isn't read by a
        nearest-cell lookup at an arbitrary continuous query point, it's a
        plain "is this grid cell within `reach` of a source cell" test used to
        classify other grid cells, so there's no query-point-vs-cell-center
        slop to guard against.
        """
        r_cells = int(math.ceil(reach / self.resolution))
        pad = max(r_cells, 1)
        padded = np.zeros((self.rows + 2 * pad, self.cols + 2 * pad), dtype=bool)
        padded[pad:pad + self.rows, pad:pad + self.cols] = source

        out = np.zeros(self.grid.shape, dtype=bool)
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                if math.hypot(dr, dc) * self.resolution >= reach:
                    continue
                out |= padded[pad + dr:pad + dr + self.rows,
                              pad + dc:pad + dc + self.cols]
        return out

    def steep_mask(self, min_grade: float, finite_only: bool = False) -> np.ndarray:
        """Boolean grid: which cells are part of a terrain EDGE.

        A cell is steep iff some 8-neighbour's elevation differs from its own by
        more than `min_grade` times the distance to that neighbour — orthogonal
        neighbours are `resolution` away, diagonal ones `resolution * sqrt(2)`,
        so the test is an ANGLE and stays meaningful when the grid is refined.
        At `min_grade = tan(60 deg)` and 0.1 m cells that is a 0.173 m step
        orthogonally, 0.245 m diagonally.

        This is the source set for `inflated_field`, and it is the whole reason
        that field no longer smears uphill terrain across graded ground: a ramp
        below the threshold contains no edges at all, so it inflates to itself.

        **Both cells of a steep pair are marked**, not just the taller one. The
        low side's own (low) elevation is dominated by the `max` against local
        terrain that `inflated_field` applies anyway, at every cell it can
        reach, so marking it costs nothing — and it saves a "which side of the
        riser owns the edge" special case that has no good answer at a corner.

        OBSTACLE cells are steep unconditionally (their column is infinitely
        tall), and so is any cell adjacent to one: `-1.0` is a sentinel, not an
        elevation, so it cannot be differenced, and the neighbour of an
        infinitely tall column is an edge cell by inspection.

        **`finite_only=True`** drops all of that: OBSTACLE cells and any edge
        caused merely by obstacle adjacency are excluded, leaving only genuine
        height discontinuities between two real (non-obstacle) elevations —
        e.g. a stair riser or a finite wall, as opposed to an infinitely tall
        column. `Map2D5.standable_mask` uses this to keep "near an obstacle"
        and "near a finite steep edge" as two independent source sets, each
        dilated on its own.

        Memoised on `(min_grade, finite_only)`, dropped by `_invalidate_caches`,
        like `surface_normals` and `inflated_field`.
        """
        key = (float(min_grade), bool(finite_only))
        cached = self._steep_cache.get(key)
        if cached is not None:
            return cached

        z = self.grid
        obs = z == self.OBSTACLE
        mask = np.zeros_like(obs) if finite_only else obs.copy()

        res = self.resolution
        diag = res * math.sqrt(2.0)
        # Four offsets cover all eight neighbours: each pair is tested once and
        # marks both ends, so (0, -1) is (0, +1) seen from the other side. Slice
        # pairs rather than per-cell loops, the same idiom as `_min_abs_slope`.
        full = slice(None)
        lo, hi = slice(0, -1), slice(1, None)
        for a_sl, b_sl, dist in (
            ((full, lo), (full, hi), res),    # east
            ((lo, full), (hi, full), res),    # north
            ((lo, lo), (hi, hi), diag),       # north-east
            ((lo, hi), (hi, lo), diag),       # north-west
        ):
            steep = np.abs(z[b_sl] - z[a_sl]) > min_grade * dist
            if finite_only:
                # A pair touching OBSTACLE's `-1.0` sentinel differenced no real
                # elevation, so it cannot count as a genuine height edge.
                steep &= ~(obs[a_sl] | obs[b_sl])
            else:
                steep |= obs[a_sl] | obs[b_sl]
            mask[a_sl] |= steep
            mask[b_sl] |= steep

        if finite_only:
            mask &= ~obs

        self._steep_cache[key] = mask
        return mask

    def inflated_field(
        self,
        radius: float,
        steep_grade: float,
        lookup_pad: float | None = None,
    ) -> np.ndarray:
        """Terrain EDGES dilated sideways by `radius`, plus local ground height.

        Per cell:

            field[c] = max( grid[c],
                            max{ grid[n] : steep(n), dist(n, c) < radius } )

        — the tallest EDGE terrain within `radius`, but never below the cell's
        own ground. Nothing is added on top, so flat ground reads back as flat
        ground and a wall reads its own true height.

        **Only edges are sources, and that is the point.** `steep_grade` selects
        them via `steep_mask`: a cell counts only if some 8-neighbour differs
        from it by more than that grade. Dilation exists to keep the body off
        the RIM of a wall or step, so terrain with no rim contributes nothing —
        a ramp below the threshold inflates to itself exactly. Dilating every
        cell instead (the earlier behaviour) read `z + 0.32*g` at every cell of
        a grade-`g` ramp, which `clearance_floor_alpha`'s
        `field > max(t_s, t_g)` test then charged as a takeoff-angle floor on
        hops that came nowhere near it.

        The `max(grid[c], ...)` floor is load-bearing, not tidiness: with every
        neighbour silenced the dilation alone yields `-inf`, and `standable_mask`
        compares `field <= grid + leg_length`. A cell must always bound the
        ground it is standing on.

        **What this gives up.** In flight this field is the only representation
        of the body's width — the arc check itself samples a bare centreline —
        so terrain below the threshold is now laterally invisible, by up to
        `radius * steep_grade` of height. See the note in `config.py` under
        STEEP_INFLATE_ANGLE_DEG. In stance it makes the max standable constant
        grade exactly `steep_grade`, as a step discontinuity.

        This is the configuration-space form of a SHARP-EDGED body. The robot
        is a cylinder with a flat bottom and square edges, and the safety margin
        is the same shape grown outward, so it is square-edged too: sharp shape
        in, sharp result out. A caller checks clearance by comparing against
        this field plus its margin as a constant —

            foot_height >= inflated_field(body_radius + margin, grade) + margin

        — which keeps the margin an explicit number at the comparison rather
        than baking a shape into the terrain.

        **There is deliberately no taper.** A tapered (rounded) field belongs to
        a body whose edges are rounded — a sphere, or a cylinder whose bottom
        edge has been filleted by measuring the margin as a straight-line
        distance around the corner. This robot's body is neither. An earlier
        version inflated by a sphere of `body + margin`, which additionally
        charged the body radius as vertical clearance underneath the foot,
        where a cylinder has no extent at all.

        Because this is a `max` and never an average, it can never under-report,
        which is why one-cell-thick obstacles are safe here (unlike bilinear
        sampling, which halves them).

        `lookup_pad` (default `resolution * sqrt(2) / 2`) is what makes a
        NEAREST-CELL lookup of the result safe, and it is not optional. A query
        point can sit up to half a cell diagonal from the centre of the cell it
        lands in, so without the pad the field would be blind to terrain in a
        thin annulus just inside `radius` — which shows up as the field
        ACCEPTING hops the reference check rejects (see
        `test/test_inflated_field.py`). The pad widens the search by that
        distance, so the guarantee becomes: for any query point `P`, the nearest
        cell's value covers every terrain point within `radius` of `P`.
        Correctness only ever costs conservatism here, never permissiveness.

        OBSTACLE columns are infinitely tall and off-map neighbours impose no
        constraint, matching `standable_mask`.

        Memoised on `(radius, lookup_pad, steep_grade)` and dropped by
        `_invalidate_caches`, like `surface_normals`.

        Note `standable_mask` is a thin wrapper over this method, evaluated at
        standing height; `test/test_inflated_field.py` pins the two together so
        that reintroducing bespoke geometry there shows up as a failure.
        """
        if lookup_pad is None:
            lookup_pad = self.resolution * math.sqrt(2.0) / 2.0

        key = (float(radius), float(lookup_pad), float(steep_grade))
        cached = self._inflated_cache.get(key)
        if cached is not None:
            return cached

        # Search wider than the body by the lookup pad — see the docstring.
        reach = radius + lookup_pad
        r_cells = int(math.ceil(reach / self.resolution))

        filled = np.where(self.grid == self.OBSTACLE, np.inf, self.grid) # turn obstacles from -1 into infinity
        # Only edge cells get to shout their height outward. Silencing the rest
        # with -inf is all it takes: -inf is the identity for `max`, so the
        # dilation loop below needs no change at all.
        # So later, when we take the max, those cells labeled -infinity will not impact
        # their neighboring cells' inflations even though their grids may have some elevation.
        src = np.where(self.steep_mask(steep_grade), filled, -np.inf)
        pad = max(r_cells, 1)
        padded = np.full((self.rows + 2 * pad, self.cols + 2 * pad), -np.inf) # now you have inf array
        padded[pad:pad + self.rows, pad:pad + self.cols] = src # fill in your grid in the middle

        # Loop over the ~50 neighbour OFFSETS, shifting the whole grid each
        # time, rather than over the 2500 cells and their neighbours: same
        # arithmetic, but vectorised into a handful of numpy ops.
        out = np.full(self.grid.shape, -np.inf)
        for dr in range(-r_cells, r_cells + 1): # +1 is just Python range behavior
            for dc in range(-r_cells, r_cells + 1):
                if math.hypot(dr, dc) * self.resolution >= reach:
                    continue  # too far to matter, however tall. For regions of square outside of radius
                out = np.maximum(out, padded[pad + dr:pad + dr + self.rows,
                                             pad + dc:pad + dc + self.cols])

        # A cell always bounds its own ground, even with every neighbour
        # silenced — see the docstring. Also restores +inf on OBSTACLE cells.
        out = np.maximum(out, filled)

        self._inflated_cache[key] = out
        return out

    def _min_abs_slope(self, axis: int) -> np.ndarray:
        """Per-cell terrain slope along one grid axis, in the min-|.| sense.

        `axis=1` differences along columns (the world x direction), `axis=0`
        along rows (world y). See `surface_normals` for why the smaller-
        magnitude one-sided difference is the right estimate.

        OBSTACLE cells participate in no difference at all (their `-1.0` is a
        sentinel, not an elevation), so a cell beside an obstacle falls back to
        its other neighbour, and a cell with no valid neighbour on either side
        reads as flat.
        """
        z = self.grid
        ok = z != self.OBSTACLE
        res = self.resolution

        fwd = np.full(z.shape, np.nan)
        bwd = np.full(z.shape, np.nan)

        if axis == 1:
            d = (z[:, 1:] - z[:, :-1]) / res # compute all adjacent-pair slopes along x-axis
            pair_ok = ok[:, 1:] & ok[:, :-1]
            d = np.where(pair_ok, d, np.nan)
            fwd[:, :-1] = d   # slope looking forward from column c
            bwd[:, 1:] = d    # ...is the same slope looking back from c+1
        else:
            d = (z[1:, :] - z[:-1, :]) / res
            pair_ok = ok[1:, :] & ok[:-1, :]
            d = np.where(pair_ok, d, np.nan)
            fwd[:-1, :] = d
            bwd[1:, :] = d

        # Keep whichever one-sided difference is smaller in magnitude. A missing
        # neighbour reads as `inf` magnitude so it always loses to a real one;
        # when both are missing the `np.where` yields NaN, which becomes 0.
        mag_f = np.where(np.isnan(fwd), np.inf, np.abs(fwd))
        mag_b = np.where(np.isnan(bwd), np.inf, np.abs(bwd))
        return np.nan_to_num(np.where(mag_f <= mag_b, fwd, bwd), nan=0.0)

    def surface_normals(self) -> np.ndarray:
        """Per-cell outward unit surface normals, shape `(rows, cols, 3)`.

        A 2.5D grid stores no normals, so they have to be inferred by
        differencing neighbouring elevations. The estimator matters: a
        **central** difference is wrong at every terrain discontinuity, because
        it averages the flat ground and the vertical face into one fictitious
        ramp. On `maps/tall_stairs.py` (0.4 m riser, 0.1 m cells) the cell at
        `x in [1.9, 2.0)` — flat ground at the foot of the step — reads a grade
        of `0.4 / 0.2 = 2.0`, a 63 degree surface, when the foot is in fact
        resting on level ground.

        That matters here because the friction cone keys off this normal: at
        grade 2.0 the cone-plane intersection is degenerate (`cos(beta)/A > 1`)
        and *no* takeoff is possible from precisely the cell the planner most
        needs.

        So each axis takes the **smaller-magnitude of the two one-sided
        differences** instead:

          * at the foot of a riser, forward reads 4.0 and backward reads 0, so
            0 wins — the foot is on a flat tread, and the riser belongs to the
            neighbouring cell's contact plane, not this one;
          * on a uniform grade the two one-sided differences agree, so the
            grade is recovered exactly (`maps/slope_crest.py`'s ramp reads
            0.35, not an averaged value).

        The normal of the surface `z = f(x, y)` is `(-df/dx, -df/dy, 1)`,
        normalised. `n_z > 0` always, which the friction-cone code relies on
        (it puts the in-plane cone axis `gamma` in `(0, pi)`).

        OBSTACLE cells get `(0, 0, 1)`. The value is inert: obstacle cells are
        never standable and are rejected as landing cells outright, so no cone
        is ever built on one.

        Memoised — the planner reads this once per construction, like
        `standable_mask`.
        """
        if self._normal_cache is not None:
            return self._normal_cache

        s_x = self._min_abs_slope(axis=1) # contains slope map of slopes along x-axis
        s_y = self._min_abs_slope(axis=0) # contains slope map of slopes along y-axis

        n = np.stack([-s_x, -s_y, np.ones_like(s_x)], axis=-1) # build the unnormalized normal vector
        n /= np.linalg.norm(n, axis=-1, keepdims=True) # normalize to unit length
        n[self.grid == self.OBSTACLE] = (0.0, 0.0, 1.0)

        self._normal_cache = n
        return n

    def set_obstacle(self, x: float, y: float):
        """Mark the cell at world position (x, y) as an obstacle."""
        row, col = self.world_to_grid(x, y)
        self.grid[row, col] = self.OBSTACLE
        self._invalidate_caches()

    def set_obstacle_region(self, x_min: float, y_min: float, x_max: float, y_max: float):
        """Mark a rectangular region as obstacles."""
        r_min, c_min = self.world_to_grid(x_min, y_min)
        r_max, c_max = self.world_to_grid(x_max, y_max)
        self.grid[r_min:r_max + 1, c_min:c_max + 1] = self.OBSTACLE
        self._invalidate_caches()

    def is_within_bounds(self, x: float, y: float) -> bool:
        """Check if (x, y) is within map boundaries."""
        return 0 <= x < self.size_x and 0 <= y < self.size_y

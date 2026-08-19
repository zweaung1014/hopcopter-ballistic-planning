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
        # `inflated_field`). A planner builds two and reads them every edge.
        self._inflated_cache: dict[tuple[float, bool, float], np.ndarray] = {}

    def _invalidate_caches(self) -> None:
        """Drop every grid-derived memo. Call after any write to `self.grid`."""
        self._fill_cache_key = None
        self._fill_cache_grid = None
        self._normal_cache = None
        self._inflated_cache = {}

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
        leg_length: float,
    ) -> np.ndarray:
        """Boolean grid of cells where the robot can stand without clipping terrain.

        The robot's single collision cylinder (radius `radius`, foot to top of
        body) is standable at a cell iff its bottom (the foot, at the cell's own
        terrain height) clears every nearby terrain column within `radius +
        clearance` — exactly the same test `clearance_floor_alpha` runs during
        flight, evaluated at standing height. So this is just
        `inflated_field` read at `grid + leg_length`, not a bespoke geometry
        calculation: `inflated_field` memoises on `(radius, lookup_pad)`, so
        calling it with the same `radius + clearance` the planner already built
        for flight clearance hits that cache rather than recomputing anything.

        Computed once per planner. Screening landing cells against this is far
        cheaper than discovering the same collision by marching an arc.
        """
        field = self.inflated_field(radius + clearance)
        return (field <= self.grid + leg_length) & (self.grid != self.OBSTACLE)

    def inflated_field(
        self,
        radius: float,
        lookup_pad: float | None = None,
    ) -> np.ndarray:
        """Terrain dilated sideways by `radius`: the tallest terrain in reach.

        Per cell, the height of the tallest terrain within `radius` of it. That
        is all — nothing is added, so flat ground reads back as flat ground and
        a wall reads its own true height.

        This is the configuration-space form of a SHARP-EDGED body. The robot
        is a cylinder with a flat bottom and square edges, and the safety margin
        is the same shape grown outward, so it is square-edged too: sharp shape
        in, sharp result out. A caller checks clearance by comparing against
        this field plus its margin as a constant —

            foot_height >= inflated_field(body_radius + margin) + margin

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

        Memoised on `(radius, lookup_pad)` and dropped by `_invalidate_caches`,
        like `surface_normals`.

        Note `standable_mask` is a thin wrapper over this method, evaluated at
        standing height; `test/test_inflated_field.py` pins the two together so
        that reintroducing bespoke geometry there shows up as a failure.
        """
        if lookup_pad is None:
            lookup_pad = self.resolution * math.sqrt(2.0) / 2.0

        key = (float(radius), float(lookup_pad))
        cached = self._inflated_cache.get(key)
        if cached is not None:
            return cached

        # Search wider than the body by the lookup pad — see the docstring.
        reach = radius + lookup_pad
        r_cells = int(math.ceil(reach / self.resolution))

        filled = np.where(self.grid == self.OBSTACLE, np.inf, self.grid)
        pad = max(r_cells, 1)
        padded = np.full((self.rows + 2 * pad, self.cols + 2 * pad), -np.inf)
        padded[pad:pad + self.rows, pad:pad + self.cols] = filled

        # Loop over the ~50 neighbour OFFSETS, shifting the whole grid each
        # time, rather than over the 2500 cells and their neighbours: same
        # arithmetic, but vectorised into a handful of numpy ops.
        out = np.full(self.grid.shape, -np.inf)
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                if math.hypot(dr, dc) * self.resolution >= reach:
                    continue  # too far to matter, however tall
                out = np.maximum(out, padded[pad + dr:pad + dr + self.rows,
                                             pad + dc:pad + dc + self.cols])

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
            d = (z[:, 1:] - z[:, :-1]) / res
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

        s_x = self._min_abs_slope(axis=1)
        s_y = self._min_abs_slope(axis=0)

        n = np.stack([-s_x, -s_y, np.ones_like(s_x)], axis=-1)
        n /= np.linalg.norm(n, axis=-1, keepdims=True)
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

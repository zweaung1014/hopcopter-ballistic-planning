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

    def _invalidate_fill_cache(self) -> None:
        self._fill_cache_key = None
        self._fill_cache_grid = None

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
        self._invalidate_fill_cache()

    def standable_mask(
        self,
        radius: float,
        clearance: float,
        leg_length: float,
        leg_clearance_start_frac: float = 1.0 / 3.0,
    ) -> np.ndarray:
        """Boolean grid of cells where the robot can stand without clipping terrain.

        The body has two collision components at stance:

          * a sphere of `radius` centred at `center_z = grid[r, c] + leg_length`
            (the CoM the ballistic arc tracks);
          * the SIDES of the leg cylinder from `foot_z + leg_length * frac` up
            to `center_z`, radius `radius`, where `foot_z = grid[r, c]` and
            `frac = leg_clearance_start_frac`. The BOTTOM `frac` of the leg is
            deliberately exempt from the check — the foot is on the ground by
            definition, and without an exempt slab any rigid-vertical-leg model
            would condemn every graded slope. No bottom cap (no cap either at
            the exempt-zone top): this is `open` cylinder sides, so terrain
            below the exempt threshold does not fire the leg constraint.

        A cell is standable iff every terrain point around it stays at least
        `clearance` from BOTH components.

        Sphere distance (unchanged from the point-mass calc): for a terrain
        column at horizontal distance `d` with top at height `h`,

            d                       when h >= center_z
            hypot(d, center_z - h)  otherwise

        Leg-cylinder-sides distance: only fires when the column top rises into
        the checked zone (`h >= foot_z + leg_length * frac`). When it does, the
        column has points at horizontal distance `d` inside the cylinder's
        z-range, so the distance to the leg axis is exactly `d`; below the
        exempt threshold the leg check is skipped (return `+inf`).

        Combined body distance = min(sphere, leg). Cell is standable iff
        `min_dist - radius >= clearance` at every relevant offset.

        Max standable constant grade with `frac=1/3` at shipped geometry
        (L=0.4, R=0.2, MIN_CLEARANCE=0.15): `(L * frac) / (R + clearance)`
        ≈ 0.133 / 0.35 ≈ 0.38. Anything steeper is un-standable everywhere
        under the rigid-vertical-leg model.

        Only offsets closer than `radius + clearance` can ever fail (beyond
        that even an infinitely tall column is far enough away), so the
        neighbourhood is bounded by that. OBSTACLE columns are treated as
        infinitely tall. Off-map neighbours are ignored.

        Computed once per planner. Screening landing cells against this is far
        cheaper than discovering the same collision by marching an arc.
        """
        reach = radius + clearance
        r_cells = int(math.ceil(reach / self.resolution))

        filled = np.where(self.grid == self.OBSTACLE, np.inf, self.grid)

        pad = max(r_cells, 1)
        padded = np.full((self.rows + 2 * pad, self.cols + 2 * pad), -np.inf)
        padded[pad:pad + self.rows, pad:pad + self.cols] = filled

        center_z = self.grid + leg_length
        # Top of the exempt slab — any neighbour column whose top rises above
        # this triggers the leg-cylinder-sides check.
        leg_exempt_top = self.grid + leg_length * leg_clearance_start_frac
        min_dist = np.full(self.grid.shape, np.inf)

        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                d = math.hypot(dr, dc) * self.resolution
                if d >= reach:
                    continue  # too far to matter, however tall
                h = padded[pad + dr:pad + dr + self.rows,
                           pad + dc:pad + dc + self.cols]
                # Sphere at CoM.
                sphere_dist = np.where(
                    h >= center_z,
                    d,
                    np.hypot(d, center_z - np.where(np.isneginf(h), center_z, h)),
                )
                # Leg cylinder sides (open, upper `1 - frac` of leg). Only fires
                # when the column top rises into the checked zone; otherwise
                # `+inf` so it never becomes the minimum.
                leg_dist = np.where(h >= leg_exempt_top, d, np.inf)
                dist = np.minimum(sphere_dist, leg_dist)
                # Off-map neighbours (-inf) impose no constraint.
                dist = np.where(np.isneginf(h), np.inf, dist)
                min_dist = np.minimum(min_dist, dist)

        return (min_dist - radius >= clearance) & (self.grid != self.OBSTACLE)

    def set_obstacle(self, x: float, y: float):
        """Mark the cell at world position (x, y) as an obstacle."""
        row, col = self.world_to_grid(x, y)
        self.grid[row, col] = self.OBSTACLE
        self._invalidate_fill_cache()

    def set_obstacle_region(self, x_min: float, y_min: float, x_max: float, y_max: float):
        """Mark a rectangular region as obstacles."""
        r_min, c_min = self.world_to_grid(x_min, y_min)
        r_max, c_max = self.world_to_grid(x_max, y_max)
        self.grid[r_min:r_max + 1, c_min:c_max + 1] = self.OBSTACLE
        self._invalidate_fill_cache()

    def is_within_bounds(self, x: float, y: float) -> bool:
        """Check if (x, y) is within map boundaries."""
        return 0 <= x < self.size_x and 0 <= y < self.size_y

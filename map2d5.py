"""2.5D Map representation: a 2D grid with elevation (z) values per cell."""

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

    def get_elevation_bilinear(
        self,
        x: float,
        y: float,
        obstacle_fill: float | None = None,
    ) -> float:
        """Bilinearly interpolate the elevation at continuous world (x, y).

        `grid_to_world(row, col)` places cell centers at `((col+0.5)*res,
        (row+0.5)*res)`, so the continuous grid coordinate for (x, y) is:
            cx = x / resolution - 0.5   (column axis)
            cy = y / resolution - 0.5   (row axis)
        Neighbour cell indices are clamped to the map, so queries near the
        border degrade to nearest-edge rather than raising.

        `obstacle_fill` (if given) is substituted for any of the four corner
        cells whose stored value equals `OBSTACLE` before mixing. This lets
        the ballistic clearance check treat obstacle cells as tall walls
        without mutating the underlying grid.
        """
        res = self.resolution
        cx = x / res - 0.5
        cy = y / res - 0.5

        c0 = int(np.floor(cx))
        r0 = int(np.floor(cy))
        c1 = c0 + 1
        r1 = r0 + 1

        # Clamp to valid grid range (edge samples degrade gracefully).
        c0_c = max(0, min(c0, self.cols - 1))
        c1_c = max(0, min(c1, self.cols - 1))
        r0_c = max(0, min(r0, self.rows - 1))
        r1_c = max(0, min(r1, self.rows - 1))

        z00 = self.grid[r0_c, c0_c]
        z10 = self.grid[r0_c, c1_c]
        z01 = self.grid[r1_c, c0_c]
        z11 = self.grid[r1_c, c1_c]

        if obstacle_fill is not None:
            if z00 == self.OBSTACLE: z00 = obstacle_fill
            if z10 == self.OBSTACLE: z10 = obstacle_fill
            if z01 == self.OBSTACLE: z01 = obstacle_fill
            if z11 == self.OBSTACLE: z11 = obstacle_fill

        # Fractional offsets, clamped in case (x, y) fell outside the grid.
        fx = min(1.0, max(0.0, cx - c0))
        fy = min(1.0, max(0.0, cy - r0))

        z0 = (1.0 - fx) * z00 + fx * z10  # bottom edge (row r0)
        z1 = (1.0 - fx) * z01 + fx * z11  # top edge    (row r1)
        return float((1.0 - fy) * z0 + fy * z1)

    def set_obstacle(self, x: float, y: float):
        """Mark the cell at world position (x, y) as an obstacle."""
        row, col = self.world_to_grid(x, y)
        self.grid[row, col] = self.OBSTACLE

    def set_obstacle_region(self, x_min: float, y_min: float, x_max: float, y_max: float):
        """Mark a rectangular region as obstacles."""
        r_min, c_min = self.world_to_grid(x_min, y_min)
        r_max, c_max = self.world_to_grid(x_max, y_max)
        self.grid[r_min:r_max + 1, c_min:c_max + 1] = self.OBSTACLE

    def is_within_bounds(self, x: float, y: float) -> bool:
        """Check if (x, y) is within map boundaries."""
        return 0 <= x < self.size_x and 0 <= y < self.size_y

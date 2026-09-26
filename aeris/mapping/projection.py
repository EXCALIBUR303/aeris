"""2D altitude-band projection for planning/exploration/frontiers (spec §21.1).

The vehicle explores within a fixed altitude band ``[z_lo, z_hi]``. A 2D
cell is occupied if any voxel in the band is occupied, free if every voxel
in the band is *observed* free, and unknown otherwise -- exactly spec's
own wording. Inflation (vehicle radius + margin) is a separate,
post-projection dilation step so the *uninflated* grid stays available for
map-accuracy evaluation (inflating before comparing to ground truth would
manufacture false positives around every real obstacle).
"""

from __future__ import annotations

import math
from collections.abc import Iterator
from dataclasses import dataclass
from enum import StrEnum, unique

from aeris.core.frames.vector import Vec3
from aeris.mapping.voxel import VoxelMap

Cell2D = tuple[int, int]

_NEIGHBOR_OFFSETS_8: tuple[Cell2D, ...] = (
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
)


@unique
class CellState(StrEnum):
    FREE = "free"
    OCCUPIED = "occupied"
    UNKNOWN = "unknown"


@dataclass(frozen=True, slots=True)
class BandGrid:
    """A 2D cell-state grid over an explicit, finite index window."""

    resolution_m: float
    min_cell: Cell2D  # inclusive
    max_cell: Cell2D  # inclusive
    occupied: frozenset[Cell2D]
    free: frozenset[Cell2D]

    def state_at(self, cell: Cell2D) -> CellState:
        if cell in self.occupied:
            return CellState.OCCUPIED
        if cell in self.free:
            return CellState.FREE
        return CellState.UNKNOWN

    def in_bounds(self, cell: Cell2D) -> bool:
        (min_ix, min_iy), (max_ix, max_iy) = self.min_cell, self.max_cell
        return min_ix <= cell[0] <= max_ix and min_iy <= cell[1] <= max_iy

    def cells_in_bounds(self) -> Iterator[Cell2D]:
        (min_ix, min_iy), (max_ix, max_iy) = self.min_cell, self.max_cell
        for ix in range(min_ix, max_ix + 1):
            for iy in range(min_iy, max_iy + 1):
                yield (ix, iy)

    def cell_center_xy(self, cell: Cell2D) -> tuple[float, float]:
        ix, iy = cell
        r = self.resolution_m
        return ((ix + 0.5) * r, (iy + 0.5) * r)


def _dilate(cells: frozenset[Cell2D], radius_cells: int) -> frozenset[Cell2D]:
    if radius_cells <= 0:
        return cells
    dilated: set[Cell2D] = set(cells)
    frontier = set(cells)
    for _ in range(radius_cells):
        next_frontier: set[Cell2D] = set()
        for cx, cy in frontier:
            for dx, dy in _NEIGHBOR_OFFSETS_8:
                n = (cx + dx, cy + dy)
                if n not in dilated:
                    dilated.add(n)
                    next_frontier.add(n)
        frontier = next_frontier
    return frozenset(dilated)


def project_band(
    voxel_map: VoxelMap,
    *,
    z_lo_m: float,
    z_hi_m: float,
    x_range_m: tuple[float, float],
    y_range_m: tuple[float, float],
    inflation_m: float = 0.0,
) -> BandGrid:
    """Project ``voxel_map`` onto the 2D altitude band ``[z_lo_m, z_hi_m]``,
    over the finite window ``x_range_m`` x ``y_range_m`` (a caller-supplied
    bound -- an unbounded sparse map has no natural finite extent to
    iterate on its own).

    ``inflation_m`` (vehicle radius + margin, spec §21.1) dilates the
    occupied set by that many cells *after* classification, defaulting to
    0 (no inflation) -- a planner passes a real margin; map-accuracy
    evaluation must not, since inflating before comparing to exact GT
    geometry would count the safety margin itself as false positives.
    """
    res = voxel_map.config.resolution_m
    min_ix = math.floor(x_range_m[0] / res)
    max_ix = math.floor(x_range_m[1] / res)
    min_iy = math.floor(y_range_m[0] / res)
    max_iy = math.floor(y_range_m[1] / res)
    min_iz = math.floor(z_lo_m / res)
    max_iz = math.floor(z_hi_m / res)

    occupied: set[Cell2D] = set()
    free: set[Cell2D] = set()
    for ix in range(min_ix, max_ix + 1):
        for iy in range(min_iy, max_iy + 1):
            any_occupied = False
            all_observed = True
            for iz in range(min_iz, max_iz + 1):
                state = voxel_map.is_occupied_at_index((ix, iy, iz))
                if state is None:
                    all_observed = False
                elif state:
                    any_occupied = True
                    break
            if any_occupied:
                occupied.add((ix, iy))
            elif all_observed:
                free.add((ix, iy))
            # else: unknown -- neither set gets it

    radius_cells = math.ceil(inflation_m / res) if inflation_m > 0 else 0
    occupied_frozen = frozenset(occupied)
    if radius_cells > 0:
        occupied_frozen = _dilate(occupied_frozen, radius_cells)
        free = {c for c in free if c not in occupied_frozen}

    return BandGrid(
        resolution_m=res,
        min_cell=(min_ix, min_iy),
        max_cell=(max_ix, max_iy),
        occupied=occupied_frozen,
        free=frozenset(free),
    )


def cell_center_world(grid: BandGrid, cell: Cell2D, *, z_m: float) -> Vec3:
    x, y = grid.cell_center_xy(cell)
    return Vec3(x, y, z_m)

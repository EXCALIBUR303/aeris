"""Frontier extraction on a 2D :class:`~aeris.mapping.projection.BandGrid`
(spec §21.2's pipeline: "frontier extraction"). Consumed by Phase 12's
frontier exploration strategies -- this phase only builds and unit-tests
detection itself, on synthetic grids (spec's own testing line).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from aeris.mapping.projection import BandGrid, Cell2D, CellState

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


@dataclass(frozen=True, slots=True)
class FrontierCluster:
    cells: frozenset[Cell2D]
    centroid_xy: tuple[float, float]
    size: int


def _is_frontier_cell(grid: BandGrid, cell: Cell2D) -> bool:
    """A FREE cell, strictly inside the queried window (every 8-neighbor
    in-bounds -- a cell at the window's own edge is never called a
    frontier purely because the *window* ends there, only because
    genuinely unobserved space begins there), with >=1 UNKNOWN neighbor."""
    if grid.state_at(cell) != CellState.FREE:
        return False
    cx, cy = cell
    has_unknown_neighbor = False
    for dx, dy in _NEIGHBOR_OFFSETS_8:
        n = (cx + dx, cy + dy)
        if not grid.in_bounds(n):
            return False  # touches the window's own edge -- not a real frontier
        if grid.state_at(n) == CellState.UNKNOWN:
            has_unknown_neighbor = True
    return has_unknown_neighbor


def detect_frontiers(grid: BandGrid, *, min_cluster_size: int = 1) -> list[FrontierCluster]:
    """8-connected clusters of frontier cells, largest first."""
    frontier_cells = {cell for cell in grid.cells_in_bounds() if _is_frontier_cell(grid, cell)}

    clusters: list[FrontierCluster] = []
    visited: set[Cell2D] = set()
    for start in frontier_cells:
        if start in visited:
            continue
        component: set[Cell2D] = {start}
        visited.add(start)
        queue: deque[Cell2D] = deque([start])
        while queue:
            cx, cy = queue.popleft()
            for dx, dy in _NEIGHBOR_OFFSETS_8:
                n = (cx + dx, cy + dy)
                if n in frontier_cells and n not in visited:
                    visited.add(n)
                    component.add(n)
                    queue.append(n)
        if len(component) < min_cluster_size:
            continue
        xs = [grid.cell_center_xy(c)[0] for c in component]
        ys = [grid.cell_center_xy(c)[1] for c in component]
        centroid = (sum(xs) / len(xs), sum(ys) / len(ys))
        clusters.append(FrontierCluster(frozenset(component), centroid, len(component)))

    clusters.sort(key=lambda c: c.size, reverse=True)
    return clusters

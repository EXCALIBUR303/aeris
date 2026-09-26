"""Unit tests for :mod:`aeris.mapping.frontier` -- frontier detection on
synthetic grids (spec's own testing line).

All grids use a generously sized window (-10..10) so that test blobs sit
deep in the interior -- keeping "borders the window's own edge" (an
explicit exclusion, tested separately) from ever contaminating the
"borders genuinely unknown space" cases the other tests check.
"""

from __future__ import annotations

from aeris.mapping.frontier import detect_frontiers
from aeris.mapping.projection import BandGrid, CellState

_RES = 0.2
_WINDOW = ((-10, -10), (10, 10))


def _grid(*, occupied: set, free: set) -> BandGrid:
    return BandGrid(
        resolution_m=_RES,
        min_cell=_WINDOW[0],
        max_cell=_WINDOW[1],
        occupied=frozenset(occupied),
        free=frozenset(free),
    )


def test_free_cell_adjacent_to_unknown_is_a_frontier() -> None:
    # A 6x7 free block from x in [0,5], y in [0,6]; x=6 column is unknown.
    free = {(x, y) for x in range(0, 6) for y in range(0, 7)}
    grid = _grid(occupied=set(), free=free)

    clusters = detect_frontiers(grid)
    frontier_cells = {c for cluster in clusters for c in cluster.cells}
    # x=5 borders the unknown column x=6 -- its interior rows are frontiers.
    assert all((5, y) in frontier_cells for y in range(1, 6))
    # x=2, deep inside the free block, has only free neighbors on every side -- not a frontier.
    assert not any((2, y) in frontier_cells for y in range(1, 6))


def test_free_cell_with_no_unknown_neighbor_is_not_a_frontier() -> None:
    # The entire window is free -- no cell borders unknown space, and every
    # window-edge cell is excluded by the separate edge-of-window rule.
    xs = range(_WINDOW[0][0], _WINDOW[1][0] + 1)
    ys = range(_WINDOW[0][1], _WINDOW[1][1] + 1)
    free = {(x, y) for x in xs for y in ys}
    grid = _grid(occupied=set(), free=free)
    assert detect_frontiers(grid) == []


def test_free_cell_at_the_window_edge_is_never_a_frontier() -> None:
    """A free cell whose 8-neighbor falls outside the queried window
    entirely must not be treated the same as one bordering genuinely
    unknown space -- excluded even though `state_at` reports UNKNOWN for
    both an out-of-window query and a genuinely never-observed cell."""
    edge_cell = _WINDOW[1]  # the window's own top-right corner
    free = {edge_cell}
    grid = _grid(occupied=set(), free=free)
    assert detect_frontiers(grid) == []


def test_occupied_cell_is_never_a_frontier_even_next_to_unknown() -> None:
    occupied = {(0, 0)}
    free = {(1, 0), (-1, 0), (0, 1), (0, -1)}
    grid = _grid(occupied=occupied, free=free)
    frontier_cells = {c for cluster in detect_frontiers(grid) for c in cluster.cells}
    assert (0, 0) not in frontier_cells


def test_clusters_are_8_connected_and_sorted_largest_first() -> None:
    # Two separate frontier blobs of different sizes, each bordering an
    # unknown cell, both deep in the window's interior.
    free = {(0, y) for y in range(-1, 2)} | {(5, 5)}
    grid = _grid(occupied=set(), free=free)

    clusters = detect_frontiers(grid)
    assert len(clusters) == 2
    assert clusters[0].size >= clusters[1].size
    assert clusters[0].size == 3


def test_min_cluster_size_filters_small_clusters() -> None:
    free = {(0, 0)} | {(5, y) for y in range(-2, 2)}
    grid = _grid(occupied=set(), free=free)

    clusters = detect_frontiers(grid, min_cluster_size=2)
    assert len(clusters) == 1
    assert clusters[0].size == 4


def test_centroid_is_the_world_frame_average_of_cluster_cells() -> None:
    free = {(0, 0), (0, 1)}
    grid = _grid(occupied=set(), free=free)
    clusters = detect_frontiers(grid)
    assert len(clusters) == 1
    cx, cy = clusters[0].centroid_xy
    assert cx == 0.5 * _RES
    assert cy == 0.5 * (0.5 * _RES + 1.5 * _RES)


def test_state_at_of_a_frontier_cell_is_free_not_something_else() -> None:
    free = {(0, y) for y in range(-2, 3)}
    grid = _grid(occupied=set(), free=free)
    clusters = detect_frontiers(grid)
    for cluster in clusters:
        for cell in cluster.cells:
            assert grid.state_at(cell) == CellState.FREE

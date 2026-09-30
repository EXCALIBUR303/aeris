"""Unit tests for :mod:`aeris.autonomy.planning.astar` -- hand-verified
optimality on known grids (spec's own testing line for Phase 12)."""

from __future__ import annotations

import math

import pytest

from aeris.autonomy.planning.astar import astar
from aeris.mapping.projection import BandGrid

_RES = 1.0


def _grid(*, occupied: set = frozenset(), free: set = frozenset(), bound: int = 10) -> BandGrid:
    return BandGrid(
        resolution_m=_RES,
        min_cell=(-bound, -bound),
        max_cell=(bound, bound),
        occupied=frozenset(occupied),
        free=frozenset(free),
    )


def test_straight_line_path_costs_exactly_the_manhattan_distance() -> None:
    grid = _grid(free={(x, 0) for x in range(0, 6)})
    result = astar(grid, (0, 0), (5, 0))
    assert result.reason == "ok"
    assert result.path_length_m == pytest.approx(5.0)
    assert len(result.path_cells) == 6


def test_diagonal_path_costs_exactly_the_octile_distance() -> None:
    grid = _grid(free={(x, x) for x in range(0, 6)})
    result = astar(grid, (0, 0), (5, 5))
    assert result.reason == "ok"
    assert result.path_length_m == pytest.approx(5 * math.sqrt(2))


def test_start_equals_goal_is_a_zero_length_path() -> None:
    grid = _grid(free={(0, 0)})
    result = astar(grid, (0, 0), (0, 0))
    assert result.reason == "ok"
    assert result.path_length_m == 0.0
    assert result.path_cells == ((0, 0),)


def test_a_wall_forces_a_detour_that_is_longer_than_the_blocked_straight_line() -> None:
    """A single-row wall spanning the whole corridor width forces a detour
    around one end -- the optimal path must be strictly longer than the
    (now-blocked) straight-line distance, and A* must still find it."""
    free = {(x, y) for x in range(-2, 8) for y in range(-2, 3)}
    occupied = {(3, y) for y in range(-2, 2)}  # wall at x=3, leaves a gap at y=2
    grid = _grid(free=free - occupied, occupied=occupied)
    result = astar(grid, (0, 0), (6, 0))
    assert result.reason == "ok"
    straight_line = 6.0
    assert result.path_length_m > straight_line
    # every occupied cell must be strictly absent from the returned path
    assert not (set(result.path_cells) & occupied)


def test_no_path_when_goal_is_fully_enclosed() -> None:
    free = {(x, y) for x in range(-2, 3) for y in range(-2, 3)}
    enclosed = (0, 0)
    ring = {
        (dx, dy)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        if (dx, dy) != enclosed and max(abs(dx), abs(dy)) == 1
    }
    grid = _grid(free=(free - ring - {enclosed}), occupied=ring)
    result = astar(grid, (-2, -2), enclosed)
    assert result.reason == "no_path"
    assert result.path_cells == ()


def test_start_blocked_and_goal_blocked_are_reported_distinctly() -> None:
    grid = _grid(free={(1, 0)}, occupied={(0, 0), (2, 0)})
    assert astar(grid, (0, 0), (1, 0)).reason == "start_blocked"
    assert astar(grid, (1, 0), (2, 0)).reason == "goal_blocked"


def test_unknown_cost_multiplier_makes_a_known_free_detour_preferred() -> None:
    """With a high unknown-cost multiplier, A* prefers a longer all-FREE
    detour over a shorter path through UNKNOWN cells; with multiplier 1.0
    (the default), the shorter unknown-crossing path wins."""
    # A direct line of UNKNOWN cells (5 steps) vs a FREE detour around it (8 steps).
    free = {(x, -1) for x in range(0, 6)} | {(0, 0), (5, 0)}
    grid = _grid(free=free)  # the (x, 0) for x in 1..4 cells are simply never added -> UNKNOWN

    cheap = astar(grid, (0, 0), (5, 0), unknown_cost_multiplier=1.0)
    assert cheap.reason == "ok"
    assert cheap.path_length_m == pytest.approx(5.0)  # straight through unknown

    expensive = astar(grid, (0, 0), (5, 0), unknown_cost_multiplier=3.0)
    assert expensive.reason == "ok"
    assert expensive.path_length_m > cheap.path_length_m  # took the known-free detour

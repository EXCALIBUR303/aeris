"""Unit tests for :mod:`aeris.mapping.deltas`."""

from __future__ import annotations

from aeris.mapping.deltas import DeltaTracker
from aeris.mapping.projection import BandGrid, CellState


def _grid(occupied: set, free: set) -> BandGrid:
    return BandGrid(
        resolution_m=0.2,
        min_cell=(0, 0),
        max_cell=(2, 2),
        occupied=frozenset(occupied),
        free=frozenset(free),
    )


def test_first_diff_reports_every_non_unknown_cell_as_changed() -> None:
    tracker = DeltaTracker()
    grid = _grid(occupied={(0, 0)}, free={(1, 1)})
    delta = tracker.diff(1.0, grid)

    changed = dict(delta.changed)
    assert changed[(0, 0)] == CellState.OCCUPIED
    assert changed[(1, 1)] == CellState.FREE
    assert (2, 2) not in changed  # stayed UNKNOWN -- no change to report


def test_second_diff_with_identical_grid_reports_nothing() -> None:
    tracker = DeltaTracker()
    grid = _grid(occupied={(0, 0)}, free={(1, 1)})
    tracker.diff(1.0, grid)
    delta = tracker.diff(2.0, grid)
    assert delta.changed == ()


def test_diff_reports_only_cells_whose_state_actually_changed() -> None:
    tracker = DeltaTracker()
    tracker.diff(1.0, _grid(occupied={(0, 0)}, free=set()))
    delta = tracker.diff(2.0, _grid(occupied={(0, 0)}, free={(1, 1)}))

    changed = dict(delta.changed)
    assert changed == {(1, 1): CellState.FREE}


def test_a_cell_flipping_from_free_to_occupied_is_reported() -> None:
    tracker = DeltaTracker()
    tracker.diff(1.0, _grid(occupied=set(), free={(0, 0)}))
    delta = tracker.diff(2.0, _grid(occupied={(0, 0)}, free=set()))

    changed = dict(delta.changed)
    assert changed == {(0, 0): CellState.OCCUPIED}


def test_delta_carries_the_given_t_sim_s() -> None:
    tracker = DeltaTracker()
    delta = tracker.diff(42.5, _grid(occupied=set(), free=set()))
    assert delta.t_sim_s == 42.5

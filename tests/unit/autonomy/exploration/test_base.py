"""Unit tests for :mod:`aeris.autonomy.exploration.base`."""

from __future__ import annotations

import math

from aeris.autonomy.exploration.base import (
    N_BEARINGS,
    RANGE_OPTIONS_M,
    AgentPose,
    egocentric_candidates,
    nearest_cell_to_point,
    snap_to_nearest_free_cell,
    world_to_cell,
)
from aeris.core.frames.vector import Vec3
from aeris.mapping.projection import BandGrid

_RES = 0.5


def _grid(*, occupied: set = frozenset(), free: set = frozenset()) -> BandGrid:
    return BandGrid(
        resolution_m=_RES,
        min_cell=(-20, -20),
        max_cell=(20, 20),
        occupied=frozenset(occupied),
        free=frozenset(free),
    )


def test_world_to_cell_floors_toward_the_containing_cell() -> None:
    grid = _grid()
    assert world_to_cell(grid, Vec3(1.2, -0.1, 0.0)) == (2, -1)


def test_egocentric_candidates_produces_bearings_times_ranges_points() -> None:
    pose = AgentPose(position_m=Vec3(0.0, 0.0, 1.0), yaw_rad=0.0)
    candidates = egocentric_candidates(pose)
    assert len(candidates) == N_BEARINGS * len(RANGE_OPTIONS_M)
    # the yaw=0, bearing=0 candidate at the short range sits due +x
    short_range = min(RANGE_OPTIONS_M)
    assert any(
        math.isclose(c.x, short_range, abs_tol=1e-9) and math.isclose(c.y, 0.0, abs_tol=1e-9)
        for c in candidates
    )


def test_snap_to_nearest_free_cell_returns_the_true_nearest_among_ties() -> None:
    # Two FREE cells equidistant in Chebyshev terms but not in Euclidean terms.
    grid = _grid(free={(2, 0), (0, 2)})
    nearest = snap_to_nearest_free_cell(grid, Vec3(0.9 * _RES, 0.1 * _RES, 0.0))
    assert nearest == (2, 0)


def test_snap_to_nearest_free_cell_returns_none_when_nothing_is_close() -> None:
    grid = _grid(free={(100, 100)})
    assert snap_to_nearest_free_cell(grid, Vec3(0.0, 0.0, 0.0), max_search_radius_cells=2) is None


def test_nearest_cell_to_point_picks_the_closest_cell_in_the_set() -> None:
    grid = _grid()
    cells = frozenset({(0, 0), (10, 10)})
    assert nearest_cell_to_point(cells, (0.3, 0.3), grid) == (0, 0)

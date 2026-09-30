"""Unit tests for the pure-logic helpers in
:mod:`aeris.evaluation.exploration_episode` -- the live control loop
itself is exercised by ``tests/integration/test_exploration_live.py``."""

from __future__ import annotations

import math

import pytest

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import Vec3
from aeris.evaluation.exploration_episode import _path_to_waypoints, _yaw_from_orientation
from aeris.mapping.projection import BandGrid


def test_yaw_from_orientation_recovers_a_known_yaw() -> None:
    q = Quaternion.from_yaw(math.pi / 2.0)
    assert _yaw_from_orientation(q) == pytest.approx(math.pi / 2.0)


def test_yaw_from_orientation_is_zero_for_identity() -> None:
    assert _yaw_from_orientation(Quaternion.identity()) == pytest.approx(0.0)


def test_path_to_waypoints_converts_cells_to_world_centers_at_the_given_altitude() -> None:
    grid = BandGrid(
        resolution_m=1.0, min_cell=(-5, -5), max_cell=(5, 5), occupied=frozenset(), free=frozenset()
    )
    waypoints = _path_to_waypoints(grid, ((0, 0), (1, 0), (2, 0)), z_m=1.5)
    assert waypoints == [Vec3(0.5, 0.5, 1.5), Vec3(1.5, 0.5, 1.5), Vec3(2.5, 0.5, 1.5)]


def test_path_to_waypoints_is_empty_for_an_empty_path() -> None:
    grid = BandGrid(
        resolution_m=1.0, min_cell=(-5, -5), max_cell=(5, 5), occupied=frozenset(), free=frozenset()
    )
    assert _path_to_waypoints(grid, (), z_m=1.5) == []

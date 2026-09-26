"""Unit tests for :class:`PathFollower` (spec §51 Phase 10)."""

from __future__ import annotations

import pytest

from aeris.autonomy.navigation.follower import PathFollower
from aeris.core.frames.vector import Vec3


def test_far_from_goal_flies_at_cruise_speed_toward_it() -> None:
    follower = PathFollower(cruise_speed_mps=2.0, slowdown_radius_m=1.5)
    v = follower.compute_velocity(Vec3(0, 0, 1), Vec3(10, 0, 1))
    assert v.x == pytest.approx(2.0)
    assert v.y == pytest.approx(0.0)
    assert v.z == pytest.approx(0.0)


def test_direction_points_exactly_at_goal() -> None:
    follower = PathFollower(cruise_speed_mps=1.0, slowdown_radius_m=0.1)
    v = follower.compute_velocity(Vec3(0, 0, 0), Vec3(3, 4, 0))
    assert v.x == pytest.approx(0.6)  # 3/5
    assert v.y == pytest.approx(0.8)  # 4/5


def test_slows_down_within_slowdown_radius() -> None:
    follower = PathFollower(cruise_speed_mps=2.0, slowdown_radius_m=2.0, arrival_radius_m=0.1)
    v = follower.compute_velocity(Vec3(0, 0, 0), Vec3(1, 0, 0))  # 1m out, half the slowdown radius
    assert v.norm() == pytest.approx(1.0)  # 2.0 * (1/2)


def test_within_arrival_radius_returns_zero() -> None:
    follower = PathFollower(arrival_radius_m=0.3)
    v = follower.compute_velocity(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0))
    assert v == Vec3(0.0, 0.0, 0.0)


def test_at_exact_goal_returns_zero() -> None:
    follower = PathFollower()
    v = follower.compute_velocity(Vec3(5, 5, 1), Vec3(5, 5, 1))
    assert v == Vec3(0.0, 0.0, 0.0)

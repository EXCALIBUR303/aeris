"""Unit tests for :class:`ReactiveAvoidance` (spec §51 Phase 10, VFH-style)."""

from __future__ import annotations

import math

import pytest

from aeris.autonomy.navigation.reactive import ReactiveAvoidance
from aeris.core.frames.vector import Vec3
from aeris.safety.shield import N_SECTORS, sector_index


def _all_clear(clearance_m: float = 15.0) -> list[float]:
    return [clearance_m] * N_SECTORS


def test_picks_heading_toward_goal_when_all_sectors_clear() -> None:
    reactive = ReactiveAvoidance(cruise_speed_mps=2.0, slowdown_radius_m=1.5)
    v = reactive.compute_velocity(Vec3(0, 0, 1), Vec3(10, 0, 1), _all_clear())
    assert v.x == pytest.approx(2.0, abs=0.05)
    assert v.y == pytest.approx(0.0, abs=0.1)


def test_avoids_a_blocked_sector_directly_toward_the_goal() -> None:
    reactive = ReactiveAvoidance(
        cruise_speed_mps=2.0, min_sector_clearance_m=0.6, slowdown_radius_m=1.5
    )
    clearances = _all_clear()
    # Block the sector directly toward the goal (angle 0) and its neighbors.
    blocked = sector_index(Vec3(1.0, 0.0, 0.0))
    for offset in (-1, 0, 1):
        clearances[(blocked + offset) % N_SECTORS] = 0.1

    v = reactive.compute_velocity(Vec3(0, 0, 1), Vec3(10, 0, 1), clearances)
    # Should pick a nearby but different, clear sector rather than the blocked one.
    heading = math.atan2(v.y, v.x)
    assert abs(heading) > math.radians(5)  # not exactly toward the (blocked) goal direction


def test_no_usable_heading_holds_horizontally() -> None:
    reactive = ReactiveAvoidance(min_sector_clearance_m=0.6)
    v = reactive.compute_velocity(Vec3(0, 0, 1), Vec3(10, 0, 1), [0.1] * N_SECTORS)
    assert v.x == pytest.approx(0.0)
    assert v.y == pytest.approx(0.0)


def test_within_arrival_radius_returns_zero_horizontal() -> None:
    reactive = ReactiveAvoidance(arrival_radius_m=0.3)
    v = reactive.compute_velocity(Vec3(0, 0, 1), Vec3(0.1, 0.1, 1), _all_clear())
    assert v.x == pytest.approx(0.0)
    assert v.y == pytest.approx(0.0)


def test_vertical_component_seeks_goal_altitude_independent_of_sectors() -> None:
    reactive = ReactiveAvoidance(cruise_speed_mps=2.0)
    v = reactive.compute_velocity(Vec3(0, 0, 0), Vec3(10, 0, 1.5), _all_clear())
    assert v.z == pytest.approx(1.5)  # clipped to at most cruise_speed_mps, here under it
    v2 = reactive.compute_velocity(Vec3(0, 0, 0), Vec3(10, 0, 5.0), _all_clear())
    assert v2.z == pytest.approx(2.0)  # clipped to cruise_speed_mps

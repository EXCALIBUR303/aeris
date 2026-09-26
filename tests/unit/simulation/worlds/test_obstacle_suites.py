"""Unit tests for the Phase 10 obstacle suites (spec §51 Phase 9 task list)."""

from __future__ import annotations

import pytest

from aeris.simulation.worlds.obstacle_suites import ALL_SUITES
from aeris.simulation.worlds.occupancy import check_reachability


@pytest.mark.parametrize("name", sorted(ALL_SUITES))
def test_each_suite_spawn_is_reachable(name: str) -> None:
    spec = ALL_SUITES[name]()
    assert check_reachability(spec, resolution_m=0.3, min_free_fraction=0.1)


@pytest.mark.parametrize("name", sorted(ALL_SUITES))
def test_each_suite_has_exactly_one_spawn_pose(name: str) -> None:
    spec = ALL_SUITES[name]()
    assert len(spec.spawn_poses) == 1


def test_dead_end_has_two_side_walls_and_one_end_wall() -> None:
    from aeris.simulation.worlds.obstacle_suites import dead_end

    spec = dead_end(length_m=8.0)
    assert len(spec.boxes) == 3


def test_narrow_gap_has_a_passable_opening() -> None:
    from aeris.simulation.worlds.obstacle_suites import narrow_gap

    spec = narrow_gap(gap_width_m=0.9)
    assert check_reachability(spec, resolution_m=0.2, min_free_fraction=0.2)


def test_overhang_leaves_room_below_within_band() -> None:
    from aeris.simulation.worlds.obstacle_suites import overhang_within_band

    spec = overhang_within_band(overhang_z=1.8, altitude_band_m=(0.3, 3.0))
    assert check_reachability(spec, resolution_m=0.3, min_free_fraction=0.1)

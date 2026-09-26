"""Unit tests for :mod:`aeris.evaluation.metrics.map` -- hand-computed cases."""

from __future__ import annotations

import pytest

from aeris.core.frames.vector import Vec3
from aeris.evaluation.metrics.map import evaluate_map
from aeris.mapping.projection import BandGrid
from aeris.simulation.worlds.spec import Bounds, Box, SpawnPose, WorldFamily, WorldSpec, WorldSplit

_RES = 0.2


def _one_box_world() -> WorldSpec:
    """A single box wall at x in [0.8, 1.2], spanning the whole altitude
    band -- deliberately wide (not just one cell thick) so the queried
    cells' centers land robustly *inside* the box, not exactly on its
    boundary (a boundary-exact query point is a floating-point coin flip
    for `point_in_box`'s own `<=` comparison, unrelated to what this test
    means to check -- caught live via this exact fixture)."""
    return WorldSpec(
        name="test_map_metrics_world",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TEST_ID,
        seed=0,
        bounds=Bounds(min_x=-2.0, min_y=-2.0, max_x=2.0, max_y=2.0, max_z=5.0),
        altitude_band_m=(0.3, 3.0),
        boxes=(Box(x=1.0, y=0.0, z=1.5, size_x=0.4, size_y=2.0, size_z=3.0),),
        spawn_poses=(SpawnPose(x=-1.0, y=0.0, z=0.1),),
    )


def _grid(*, occupied: set, free: set) -> BandGrid:
    return BandGrid(
        resolution_m=_RES,
        min_cell=(-10, -10),
        max_cell=(10, 10),
        occupied=frozenset(occupied),
        free=frozenset(free),
    )


def test_perfect_agreement_gives_precision_and_recall_of_one() -> None:
    world = _one_box_world()
    wall_cell = (round(1.0 / _RES), 0)
    free_cell = (round(-1.0 / _RES), 0)
    grid = _grid(occupied={wall_cell}, free={free_cell})

    result = evaluate_map(grid, world, z_lo_m=0.3, z_hi_m=3.0)
    assert result.occupied_precision == pytest.approx(1.0)
    assert result.occupied_recall == pytest.approx(1.0)
    assert result.free_false_occupied_rate == pytest.approx(0.0)


def test_false_positive_occupied_cell_hurts_precision_not_recall() -> None:
    world = _one_box_world()
    wall_cell = (round(1.0 / _RES), 0)
    phantom_cell = (round(-1.0 / _RES), 0)  # agent thinks this is occupied; GT says it's clear
    grid = _grid(occupied={wall_cell, phantom_cell}, free=set())

    result = evaluate_map(grid, world, z_lo_m=0.3, z_hi_m=3.0)
    assert result.occupied_precision == pytest.approx(0.5)  # 1 TP, 1 FP
    assert result.occupied_recall == pytest.approx(1.0)  # the real wall was still found
    assert result.free_false_occupied_rate == pytest.approx(1.0)  # 1/1 truly-free cell mismarked


def test_missed_wall_hurts_recall_not_precision() -> None:
    """Recall is scoped to *observed* cells (spec: "within observed
    regions") -- a wall cell the agent never touched at all (still
    UNKNOWN) isn't a miss by this metric's own definition; the case that
    actually hurts recall is the agent *wrongly observing* the wall's
    cell as free."""
    world = _one_box_world()
    wall_cell = (round(1.0 / _RES), 0)
    free_cell = (round(-1.0 / _RES), 0)
    grid = _grid(occupied=set(), free={wall_cell, free_cell})  # wall wrongly classified free

    result = evaluate_map(grid, world, z_lo_m=0.3, z_hi_m=3.0)
    assert result.occupied_recall == pytest.approx(0.0)
    assert result.occupied_precision == pytest.approx(1.0)  # no positive predictions to be wrong


def test_unknown_cells_are_excluded_from_precision_recall_but_lower_coverage() -> None:
    world = _one_box_world()
    wall_cell = (round(1.0 / _RES), 0)
    grid = _grid(occupied={wall_cell}, free=set())  # everything else stays UNKNOWN

    result = evaluate_map(grid, world, z_lo_m=0.3, z_hi_m=3.0)
    assert result.n_observed_cells == 1
    assert result.n_total_cells == 21 * 21
    assert result.map_coverage == pytest.approx(1 / (21 * 21))


def test_spawn_world_converts_m_frame_cells_to_world_before_gt_check() -> None:
    """Regression test for a real Phase 11 bug: ``grid`` is in frame
    ``M``/``O`` (spawn-relative, per Phase 10's live finding), while
    ``spec``'s geometry is world-frame. Without the ``spawn_world``
    conversion, every M-frame cell gets checked against the wrong world
    location -- for a non-origin spawn, this made every real detection a
    false positive (0% precision) across every obstacle-suite world,
    since none of them spawn at/near world (0, 0, 0)."""
    world = _one_box_world()  # wall at world x=1.0
    spawn_world = Vec3(5.0, 3.0, 0.0)
    # The wall's cell, as the agent would report it in M-frame (world - spawn).
    wall_cell_m = (round((1.0 - spawn_world.x) / _RES), round((0.0 - spawn_world.y) / _RES))
    grid = BandGrid(
        resolution_m=_RES,
        min_cell=(-30, -30),
        max_cell=(30, 30),
        occupied=frozenset({wall_cell_m}),
        free=frozenset(),
    )

    without_conversion = evaluate_map(grid, world, z_lo_m=0.3, z_hi_m=3.0)
    assert without_conversion.occupied_precision == pytest.approx(0.0)

    with_conversion = evaluate_map(grid, world, z_lo_m=0.3, z_hi_m=3.0, spawn_world=spawn_world)
    assert with_conversion.occupied_precision == pytest.approx(1.0)
    assert with_conversion.occupied_recall == pytest.approx(1.0)


def test_map_coverage_is_zero_when_nothing_was_observed() -> None:
    world = _one_box_world()
    grid = _grid(occupied=set(), free=set())
    result = evaluate_map(grid, world, z_lo_m=0.3, z_hi_m=3.0)
    assert result.map_coverage == 0.0
    assert result.occupied_precision == pytest.approx(1.0)
    assert result.occupied_recall == pytest.approx(1.0)

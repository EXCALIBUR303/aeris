"""Unit tests for :mod:`aeris.evaluation.metrics.coverage` -- hand-computed
cases, synthetic worlds and trajectories (spec's own testing line for
Phase 12's coverage metric)."""

from __future__ import annotations

import math

import pytest

from aeris.core.frames.vector import Vec3
from aeris.evaluation.metrics.coverage import (
    coverage,
    explored_cells_from_gt_pose,
    reachable_free_cells_gt,
    running_coverage_trace,
    time_to_coverage_threshold,
)
from aeris.simulation.worlds.spec import Bounds, Box, SpawnPose, WorldFamily, WorldSpec, WorldSplit

_RES = 0.5


def _open_world() -> WorldSpec:
    """A fully open room -- no obstacles -- so every in-band cell within
    bounds is reachable free space."""
    return WorldSpec(
        name="test_coverage_open_world",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.VAL,
        seed=10_000,
        bounds=Bounds(min_x=-3.0, min_y=-3.0, max_x=3.0, max_y=3.0, max_z=5.0),
        altitude_band_m=(0.3, 2.0),
        boxes=(),
        spawn_poses=(SpawnPose(x=0.0, y=0.0, z=0.1),),
    )


def _walled_world() -> WorldSpec:
    """A wide wall at x in [0.75, 1.25], splitting the room into a near
    half (reachable from the spawn) and a far half (not, since the wall
    spans the whole altitude band and the room's own y-extent)."""
    return WorldSpec(
        name="test_coverage_walled_world",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.VAL,
        seed=10_001,
        bounds=Bounds(min_x=-3.0, min_y=-3.0, max_x=3.0, max_y=3.0, max_z=5.0),
        altitude_band_m=(0.3, 2.0),
        boxes=(Box(x=1.0, y=0.0, z=1.0, size_x=0.5, size_y=6.0, size_z=5.0),),
        spawn_poses=(SpawnPose(x=-1.0, y=0.0, z=0.1),),
    )


def test_reachable_free_cells_gt_is_empty_where_a_wall_seals_off_the_far_side() -> None:
    world = _walled_world()
    reachable = reachable_free_cells_gt(world, resolution_m=_RES)
    far_side_cell = (round(2.0 / _RES), 0)
    near_side_cell = (round(-1.0 / _RES), 0)
    assert near_side_cell in reachable
    assert far_side_cell not in reachable


def test_reachable_free_cells_gt_uses_the_same_cell_indexing_as_the_gt_raycaster() -> None:
    """Regression test: bounds whose min_x/min_y are deliberately *not*
    exact multiples of resolution_m -- if reachable_free_cells_gt still
    used OccupancyGrid's own bounds-relative cell indices (rather than
    re-deriving the plain floor(world / res) index every other cell in
    this module uses), a cell known to be reachable by construction
    would land at the wrong index and this intersection would come back
    empty even though the two sets genuinely overlap in world space."""
    world = WorldSpec(
        name="test_coverage_misaligned_bounds_world",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.VAL,
        seed=10_002,
        bounds=Bounds(min_x=-3.15, min_y=-3.15, max_x=3.15, max_y=3.15, max_z=5.0),
        altitude_band_m=(0.3, 2.0),
        boxes=(),
        spawn_poses=(SpawnPose(x=0.0, y=0.0, z=0.1),),
    )
    reachable = reachable_free_cells_gt(world, resolution_m=_RES)
    spawn_cell = (math.floor(0.0 / _RES), math.floor(0.0 / _RES))
    explored_at_spawn = explored_cells_from_gt_pose(
        world, (0.0, 0.0), 0.0, z_lo_m=0.3, z_hi_m=2.0, resolution_m=_RES, max_range_m=1.0, n_rays=4
    )
    assert spawn_cell in reachable
    assert coverage(explored_at_spawn, reachable) > 0.0


def test_reachable_free_cells_gt_covers_the_whole_open_room() -> None:
    world = _open_world()
    reachable = reachable_free_cells_gt(world, resolution_m=_RES)
    assert (round(2.0 / _RES), round(2.0 / _RES)) in reachable
    assert (round(-2.0 / _RES), round(-2.0 / _RES)) in reachable


def test_explored_cells_from_gt_pose_stops_at_the_wall() -> None:
    world = _walled_world()
    seen = explored_cells_from_gt_pose(
        world,
        (-1.0, 0.0),
        0.0,  # yaw=0, facing +x, straight at the wall
        z_lo_m=0.3,
        z_hi_m=2.0,
        resolution_m=_RES,
        max_range_m=10.0,
        fov_rad=0.2,  # a narrow beam straight ahead
        n_rays=3,
    )
    far_side_cell = (round(2.0 / _RES), 0)
    wall_face_cell = (round(0.9 / _RES), 0)
    assert far_side_cell not in seen
    assert wall_face_cell in seen


def test_coverage_hand_computed_ratio() -> None:
    reachable = frozenset({(0, 0), (1, 0), (2, 0), (3, 0)})
    explored = frozenset({(0, 0), (1, 0)})
    assert coverage(explored, reachable) == pytest.approx(0.5)


def test_coverage_is_vacuously_one_when_nothing_is_reachable() -> None:
    assert coverage(frozenset(), frozenset()) == 1.0


def test_time_to_coverage_threshold_returns_the_first_crossing() -> None:
    trace = [(0.0, 0.0), (1.0, 0.4), (2.0, 0.6), (3.0, 0.9)]
    assert time_to_coverage_threshold(trace, 0.5) == pytest.approx(2.0)


def test_time_to_coverage_threshold_is_none_when_censored() -> None:
    trace = [(0.0, 0.0), (1.0, 0.4), (2.0, 0.6)]
    assert time_to_coverage_threshold(trace, 0.9) is None


def test_running_coverage_trace_is_monotonically_non_decreasing() -> None:
    world = _open_world()
    reachable = reachable_free_cells_gt(world, resolution_m=_RES)
    poses = [
        (0.0, Vec3(0.0, 0.0, 1.0), 0.0),
        (1.0, Vec3(1.0, 0.0, 1.0), 0.0),
        (2.0, Vec3(1.0, 1.0, 1.0), 1.57),
    ]
    trace = running_coverage_trace(
        world, poses, reachable_free_cells=reachable, z_lo_m=0.3, z_hi_m=2.0, resolution_m=_RES
    )
    assert len(trace) == 3
    values = [c for _t, c in trace]
    assert values == sorted(values)
    assert values[-1] > 0.0

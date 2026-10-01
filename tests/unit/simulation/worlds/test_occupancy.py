"""Unit tests for :mod:`aeris.simulation.worlds.occupancy` (spec §9.3)."""

from __future__ import annotations

import math
import random

from aeris.core.frames.vector import Vec3
from aeris.simulation.worlds.occupancy import (
    _box_transform,
    _point_in_box_local,
    check_reachability,
    is_occupied,
    point_in_box,
    point_in_cylinder,
    voxelize,
)
from aeris.simulation.worlds.spec import (
    Bounds,
    Box,
    Cylinder,
    SpawnPose,
    WorldFamily,
    WorldSpec,
    WorldSplit,
)


def test_point_in_box_axis_aligned() -> None:
    box = Box(x=0, y=0, z=0, size_x=2, size_y=2, size_z=2)
    assert point_in_box(Vec3(0.9, 0, 0), box)
    assert not point_in_box(Vec3(1.1, 0, 0), box)


def test_point_in_box_rotated_45_degrees() -> None:
    # A 2x2 box (half-extent 1) rotated 45deg about z: a point at distance
    # 1.0 along the original x-axis is now *outside* the rotated box's
    # local x half-extent (its diagonal reaches ~1.41, but straight along
    # world-x it's now testing a different local axis) -- check a point on
    # the rotated box's own local +x axis instead, which stays at the same
    # local coordinate regardless of yaw.
    box = Box(x=0, y=0, z=0, size_x=2, size_y=2, size_z=2, yaw_rad=math.pi / 4)
    on_axis = Vec3(0.9 * math.cos(math.pi / 4), 0.9 * math.sin(math.pi / 4), 0)
    assert point_in_box(on_axis, box)
    outside = Vec3(1.5 * math.cos(math.pi / 4), 1.5 * math.sin(math.pi / 4), 0)
    assert not point_in_box(outside, box)


def test_point_in_cylinder() -> None:
    cyl = Cylinder(x=0, y=0, z=1, radius_m=1.0, height_m=2.0)
    assert point_in_cylinder(Vec3(0.5, 0, 1), cyl)
    assert not point_in_cylinder(Vec3(1.5, 0, 1), cyl)
    assert not point_in_cylinder(Vec3(0, 0, 2.5), cyl)  # radially inside, above the top


def test_is_occupied_checks_all_primitives() -> None:
    spec = WorldSpec(
        name="w",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TRAIN,
        seed=0,
        bounds=Bounds(min_x=-10, min_y=-10, max_x=10, max_y=10),
        boxes=(Box(x=5, y=0, z=0.5, size_x=1, size_y=1, size_z=1),),
        cylinders=(Cylinder(x=-5, y=0, z=0.5, radius_m=0.5, height_m=1.0),),
        spawn_poses=(SpawnPose(x=0, y=0, z=0.1),),
    )
    assert is_occupied(spec, Vec3(5, 0, 0.5))
    assert is_occupied(spec, Vec3(-5, 0, 0.5))
    assert not is_occupied(spec, Vec3(0, 0, 0.5))


def test_voxelize_marks_a_known_box_occupied() -> None:
    spec = WorldSpec(
        name="w",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TRAIN,
        seed=0,
        bounds=Bounds(min_x=-2, min_y=-2, max_x=2, max_y=2, max_z=2),
        boxes=(Box(x=0, y=0, z=0.5, size_x=1, size_y=1, size_z=1),),
        spawn_poses=(SpawnPose(x=1.5, y=1.5, z=0.1),),
    )
    grid = voxelize(spec, resolution_m=0.5)
    center_cell = grid.world_to_cell(Vec3(0, 0, 0.5))
    assert grid.is_cell_occupied(center_cell)
    far_cell = grid.world_to_cell(Vec3(1.8, 1.8, 0.1))
    assert not grid.is_cell_occupied(far_cell)


def test_reachable_free_cells_flood_fills_open_space() -> None:
    spec = WorldSpec(
        name="w",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TRAIN,
        seed=0,
        bounds=Bounds(min_x=-5, min_y=-5, max_x=5, max_y=5, max_z=2),
        spawn_poses=(SpawnPose(x=0, y=0, z=0.5),),
    )
    grid = voxelize(spec, resolution_m=0.5)
    reachable = grid.reachable_free_cells(Vec3(0, 0, 0.5), bounds_cells=((0, 0, 0), (19, 19, 3)))
    assert len(reachable) > 100  # an empty world should have most cells reachable


def test_check_reachability_true_for_open_world() -> None:
    spec = WorldSpec(
        name="w",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TRAIN,
        seed=0,
        bounds=Bounds(min_x=-5, min_y=-5, max_x=5, max_y=5, max_z=3),
        spawn_poses=(SpawnPose(x=0, y=0, z=1.0),),
        altitude_band_m=(0.3, 2.5),
    )
    assert check_reachability(spec, resolution_m=0.5)


def test_check_reachability_false_when_spawn_is_sealed_in() -> None:
    # A spawn point boxed in on all 4 sides by walls taller than the
    # altitude band, with no gap -- unreachable beyond its own cell.
    walls = (
        Box(x=1.5, y=0, z=1.5, size_x=1.0, size_y=4.0, size_z=3.0),
        Box(x=-1.5, y=0, z=1.5, size_x=1.0, size_y=4.0, size_z=3.0),
        Box(x=0, y=1.5, z=1.5, size_x=4.0, size_y=1.0, size_z=3.0),
        Box(x=0, y=-1.5, z=1.5, size_x=4.0, size_y=1.0, size_z=3.0),
    )
    spec = WorldSpec(
        name="w",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TRAIN,
        seed=0,
        bounds=Bounds(min_x=-10, min_y=-10, max_x=10, max_y=10, max_z=3),
        boxes=walls,
        spawn_poses=(SpawnPose(x=0, y=0, z=1.0),),
        altitude_band_m=(0.3, 2.5),
    )
    assert not check_reachability(spec, resolution_m=0.5, min_free_fraction=0.3)


def test_point_in_box_fast_path_matches_reference_rotation_test() -> None:
    """The bounding-sphere early-out and memoized inverse (Phase 13 perf
    fix) must never change an answer: compare against the original
    rotate-then-test path on rotated boxes, including points exactly on
    faces/corners and just past them."""
    rng = random.Random(13)
    for _ in range(200):
        box = Box(
            x=rng.uniform(-5, 5),
            y=rng.uniform(-5, 5),
            z=rng.uniform(0, 3),
            size_x=rng.uniform(0.1, 3),
            size_y=rng.uniform(0.1, 3),
            size_z=rng.uniform(0.1, 3),
            roll_rad=rng.uniform(-0.5, 0.5),
            pitch_rad=rng.uniform(-0.5, 0.5),
            yaw_rad=rng.uniform(-3.1, 3.1),
        )
        t = _box_transform(box)
        h = box.half_extent
        interior = [Vec3(*(rng.uniform(-1.5, 1.5) * e for e in (h.x, h.y, h.z))) for _ in range(20)]
        corners = [
            Vec3(sx * h.x * k, sy * h.y * k, sz * h.z * k)
            for sx in (-1, 1)
            for sy in (-1, 1)
            for sz in (-1, 1)
            for k in (1.0, 1.0 + 1e-12, 1.001)
        ]
        local_pts = interior + corners
        for pl in local_pts:
            p = t.apply(pl)
            assert point_in_box(p, box) == _point_in_box_local(t.inverse().apply(p), h)

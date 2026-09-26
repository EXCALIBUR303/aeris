"""Unit tests for :mod:`aeris.simulation.worlds.spec` (spec §9.3, §33.1)."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from aeris.simulation.worlds.spec import Bounds, Box, WorldFamily, WorldSpec, WorldSplit


def _bounds() -> Bounds:
    return Bounds(min_x=-5, min_y=-5, max_x=5, max_y=5)


def _spawn() -> tuple:
    from aeris.simulation.worlds.spec import SpawnPose

    return (SpawnPose(x=0, y=0, z=0.1),)


def test_bounds_rejects_min_greater_than_max() -> None:
    with pytest.raises(ValidationError):
        Bounds(min_x=5, min_y=-5, max_x=-5, max_y=5)


def test_bounds_contains() -> None:
    b = _bounds()
    from aeris.core.frames.vector import Vec3

    assert b.contains(Vec3(0, 0, 1))
    assert not b.contains(Vec3(10, 0, 1))


def test_box_center_and_half_extent() -> None:
    box = Box(x=1, y=2, z=3, size_x=2, size_y=4, size_z=6)
    assert box.center.x == 1 and box.center.y == 2 and box.center.z == 3
    assert box.half_extent.x == 1 and box.half_extent.y == 2 and box.half_extent.z == 3


def test_box_rejects_non_positive_size() -> None:
    with pytest.raises(ValidationError):
        Box(x=0, y=0, z=0, size_x=0, size_y=1, size_z=1)


def test_f4_collapsed_must_be_test_ood() -> None:
    with pytest.raises(ValidationError, match="test_ood"):
        WorldSpec(
            name="bad",
            family=WorldFamily.COLLAPSED,
            split=WorldSplit.TRAIN,
            seed=0,
            bounds=_bounds(),
            spawn_poses=_spawn(),
        )


def test_test_ood_must_be_f4_collapsed() -> None:
    with pytest.raises(ValidationError, match="test_ood"):
        WorldSpec(
            name="bad",
            family=WorldFamily.OFFICE,
            split=WorldSplit.TEST_OOD,
            seed=0,
            bounds=_bounds(),
            spawn_poses=_spawn(),
        )


def test_valid_f4_test_ood_pairing() -> None:
    spec = WorldSpec(
        name="ok",
        family=WorldFamily.COLLAPSED,
        split=WorldSplit.TEST_OOD,
        seed=30000,
        bounds=_bounds(),
        spawn_poses=_spawn(),
    )
    assert spec.family is WorldFamily.COLLAPSED


def test_altitude_band_must_be_ordered() -> None:
    with pytest.raises(ValidationError):
        WorldSpec(
            name="bad",
            family=WorldFamily.OFFICE,
            split=WorldSplit.TRAIN,
            seed=0,
            bounds=_bounds(),
            spawn_poses=_spawn(),
            altitude_band_m=(3.0, 0.5),
        )


def test_requires_at_least_one_spawn_pose() -> None:
    with pytest.raises(ValidationError):
        WorldSpec(
            name="bad",
            family=WorldFamily.OFFICE,
            split=WorldSplit.TRAIN,
            seed=0,
            bounds=_bounds(),
            spawn_poses=(),
        )


def test_content_hash_is_deterministic_for_identical_content() -> None:
    spec_a = WorldSpec(
        name="w",
        family=WorldFamily.OFFICE,
        split=WorldSplit.TRAIN,
        seed=1,
        bounds=_bounds(),
        spawn_poses=_spawn(),
    )
    spec_b = WorldSpec(
        name="w",
        family=WorldFamily.OFFICE,
        split=WorldSplit.TRAIN,
        seed=1,
        bounds=_bounds(),
        spawn_poses=_spawn(),
    )
    assert spec_a.content_hash() == spec_b.content_hash()


def test_content_hash_differs_for_different_content() -> None:
    spec_a = WorldSpec(
        name="w",
        family=WorldFamily.OFFICE,
        split=WorldSplit.TRAIN,
        seed=1,
        bounds=_bounds(),
        spawn_poses=_spawn(),
    )
    spec_b = WorldSpec(
        name="w",
        family=WorldFamily.OFFICE,
        split=WorldSplit.TRAIN,
        seed=2,
        bounds=_bounds(),
        spawn_poses=_spawn(),
    )
    assert spec_a.content_hash() != spec_b.content_hash()

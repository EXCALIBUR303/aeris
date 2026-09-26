"""Unit tests for :mod:`aeris.mapping.voxel` -- hand-computed known-geometry
cases (spec's own testing line: "single ray, plane, corner"), plus clamping
and the unknown/free/occupied three-way semantics."""

from __future__ import annotations

import pytest

from aeris.core.frames.vector import Vec3
from aeris.mapping.voxel import MappingConfig, VoxelMap


def _cfg(**overrides: float) -> MappingConfig:
    return MappingConfig(**overrides)


def test_untouched_voxel_is_unknown() -> None:
    m = VoxelMap(config=_cfg())
    assert m.is_occupied(Vec3(3.0, 3.0, 3.0)) is None
    assert m.log_odds_at(Vec3(3.0, 3.0, 3.0)) == 0.0


def test_single_ray_hit_becomes_occupied_path_becomes_free() -> None:
    m = VoxelMap(config=_cfg())
    m.integrate_ray(Vec3(0.0, 0.0, 0.0), Vec3(2.0, 0.0, 0.0), is_hit=True)

    assert m.is_occupied(Vec3(2.0, 0.0, 0.0)) is True
    assert m.is_occupied(Vec3(0.1, 0.0, 0.0)) is False
    assert m.is_occupied(Vec3(1.0, 0.0, 0.0)) is False


def test_no_return_ray_never_occupied() -> None:
    m = VoxelMap(config=_cfg())
    m.integrate_ray(Vec3(0.0, 0.0, 0.0), Vec3(5.0, 0.0, 0.0), is_hit=False)

    assert m.is_occupied(Vec3(5.0, 0.0, 0.0)) is False
    assert m.is_occupied(Vec3(2.0, 0.0, 0.0)) is False


def test_plane_geometry_a_fan_of_rays_hitting_a_flat_wall() -> None:
    """A synthetic depth frame looking at a flat wall at x=3.0: a fan of
    rays from one shared origin, each hitting a different point on the
    wall -- every ray's endpoint becomes occupied and every point along
    its own path becomes free, the expected shape of a real depth-camera
    frame against a plane."""
    m = VoxelMap(config=_cfg())
    origin = Vec3(0.0, 0.0, 0.0)
    # Deliberately not exact multiples of resolution_m (0.2): a coordinate
    # sitting exactly on a voxel boundary is a floating-point-precision
    # coin flip for floor division (0.4 / 0.2 can evaluate to slightly
    # under 2.0), unrelated to raycasting correctness -- caught live via
    # this exact test before this comment existed, and fixed by choosing
    # robustly-interior test coordinates instead of chasing float epsilon.
    endpoints = [Vec3(3.0, y, 0.0) for y in (-0.41, -0.21, 0.01, 0.21, 0.41)]
    m.integrate_points(origin, endpoints)

    for endpoint in endpoints:
        assert m.is_occupied(endpoint) is True
        midpoint = Vec3(endpoint.x / 2.0, endpoint.y / 2.0, endpoint.z / 2.0)
        assert m.is_occupied(midpoint) is False
    # Off the plane's extent (no ray ever passed near here): still unknown.
    assert m.is_occupied(Vec3(3.0, 5.0, 0.0)) is None


def test_corner_geometry_two_rays_converging_share_a_free_voxel() -> None:
    """Two rays from the same origin, hitting perpendicular walls, share
    the near-origin voxel -- its free delta accumulates from both."""
    m = VoxelMap(config=_cfg())
    origin = Vec3(0.0, 0.0, 0.0)
    m.integrate_points(origin, [Vec3(3.0, 0.0, 0.0), Vec3(0.0, 3.0, 0.0)])

    shared = m.log_odds_at(Vec3(0.05, 0.05, 0.0))
    assert shared == pytest.approx(2 * m.config.l_free)
    assert m.is_occupied(Vec3(3.0, 0.0, 0.0)) is True
    assert m.is_occupied(Vec3(0.0, 3.0, 0.0)) is True


def test_repeated_hits_clamp_at_l_max() -> None:
    m = VoxelMap(config=_cfg(l_max=2.0, l_occ=0.85))
    target = Vec3(2.0, 0.0, 0.0)
    for _ in range(20):
        m.integrate_ray(Vec3(0.0, 0.0, 0.0), target, is_hit=True)
    assert m.log_odds_at(target) == pytest.approx(2.0)


def test_repeated_misses_clamp_at_l_min() -> None:
    m = VoxelMap(config=_cfg(l_min=-2.0, l_free=-0.4))
    target = Vec3(0.1, 0.0, 0.0)
    for _ in range(20):
        m.integrate_ray(Vec3(0.0, 0.0, 0.0), Vec3(2.0, 0.0, 0.0), is_hit=True)
    assert m.log_odds_at(target) == pytest.approx(-2.0)


def test_integrate_points_with_empty_list_is_a_no_op() -> None:
    m = VoxelMap(config=_cfg())
    m.integrate_points(Vec3(0.0, 0.0, 0.0), [])
    assert m.touched_block_count() == 0


def test_integrate_points_explicit_is_hit_mixes_hit_and_miss() -> None:
    m = VoxelMap(config=_cfg())
    points = [Vec3(3.0, 0.0, 0.0), Vec3(0.0, 3.0, 0.0)]
    m.integrate_points(Vec3(0.0, 0.0, 0.0), points, is_hit=[True, False])
    assert m.is_occupied(Vec3(3.0, 0.0, 0.0)) is True
    assert m.is_occupied(Vec3(0.0, 3.0, 0.0)) is False

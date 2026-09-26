"""Unit tests for depth back-projection against a synthetic frontal plane."""

from __future__ import annotations

import math
from array import array

import pytest

from aeris.perception.depth.projection import (
    CameraIntrinsics,
    camera_intrinsics_from_k,
    depth_to_points_camera,
)

_WIDTH, _HEIGHT = 8, 6
_INTRINSICS = CameraIntrinsics(width=_WIDTH, height=_HEIGHT, fx=100.0, fy=100.0, cx=3.5, cy=2.5)


def test_frontal_plane_all_points_at_constant_z() -> None:
    """A plane perpendicular to the optical axis: every pixel has z == D."""
    depth_m = 5.0
    depth = array("f", [depth_m] * (_WIDTH * _HEIGHT))

    points = depth_to_points_camera(depth, _INTRINSICS)

    assert len(points) == _WIDTH * _HEIGHT
    for p in points:
        assert p.z == pytest.approx(depth_m)


def test_principal_point_maps_to_optical_axis() -> None:
    """A camera whose principal point falls exactly on a pixel center back-projects to (0, 0, D) there."""
    depth_m = 3.0
    intrinsics = CameraIntrinsics(width=_WIDTH, height=_HEIGHT, fx=100.0, fy=100.0, cx=4.0, cy=3.0)
    depth = array("f", [depth_m] * (_WIDTH * _HEIGHT))

    points = depth_to_points_camera(depth, intrinsics)
    p = points[3 * _WIDTH + 4]

    assert p.x == pytest.approx(0.0, abs=1e-6)
    assert p.y == pytest.approx(0.0, abs=1e-6)
    assert p.z == pytest.approx(depth_m)


def test_known_off_axis_pixel() -> None:
    """A pixel offset from the principal point back-projects via the pinhole formula."""
    depth_m = 4.0
    depth = array("f", [depth_m] * (_WIDTH * _HEIGHT))
    col, row = 5, 4  # 1.5 px right, 1.5 px down of principal point

    points = depth_to_points_camera(depth, _INTRINSICS)
    p = points[row * _WIDTH + col]

    expected_x = (col - _INTRINSICS.cx) / _INTRINSICS.fx * depth_m
    expected_y = (row - _INTRINSICS.cy) / _INTRINSICS.fy * depth_m
    assert p.x == pytest.approx(expected_x)
    assert p.y == pytest.approx(expected_y)


def test_invalid_and_out_of_range_samples_are_dropped() -> None:
    values = [5.0] * (_WIDTH * _HEIGHT)
    values[0] = math.nan
    values[1] = math.inf
    values[2] = 0.01  # below min_depth_m default (0.05)
    values[3] = 200.0  # above max_depth_m default (100.0)
    depth = array("f", values)

    points = depth_to_points_camera(depth, _INTRINSICS)

    assert len(points) == _WIDTH * _HEIGHT - 4


def test_wrong_buffer_length_raises() -> None:
    depth = array("f", [1.0] * 4)
    with pytest.raises(ValueError, match="expected"):
        depth_to_points_camera(depth, _INTRINSICS)


def test_stride_samples_a_subset_of_pixels() -> None:
    depth_m = 4.0
    depth = array("f", [depth_m] * (_WIDTH * _HEIGHT))

    full = depth_to_points_camera(depth, _INTRINSICS, stride=1)
    strided = depth_to_points_camera(depth, _INTRINSICS, stride=2)

    # rows 0,2,4 x cols 0,2,4,6 = 3*4 = 12 points, vs the full 8*6=48.
    assert len(strided) == 12
    assert len(full) == _WIDTH * _HEIGHT
    for p in strided:
        assert p.z == pytest.approx(depth_m)


def test_stride_rejects_non_positive_value() -> None:
    depth = array("f", [1.0] * (_WIDTH * _HEIGHT))
    with pytest.raises(ValueError, match="stride"):
        depth_to_points_camera(depth, _INTRINSICS, stride=0)


def test_camera_intrinsics_from_k_matrix() -> None:
    k = (615.0, 0.0, 320.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0)
    intrinsics = camera_intrinsics_from_k(640, 480, k)
    assert intrinsics == CameraIntrinsics(
        width=640, height=480, fx=615.0, fy=615.0, cx=320.0, cy=240.0
    )


def test_camera_intrinsics_from_k_wrong_size_raises() -> None:
    with pytest.raises(ValueError, match="9-element"):
        camera_intrinsics_from_k(640, 480, (1.0, 2.0, 3.0))

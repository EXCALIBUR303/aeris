"""Unit tests for 2D LiDAR back-projection against a synthetic circle of ranges."""

from __future__ import annotations

import math

import pytest

from aeris.perception.lidar.projection import ScanGeometry, scan_to_points

_GEOMETRY = ScanGeometry(
    angle_min_rad=-math.pi / 2,
    angle_step_rad=math.pi / 180,  # 1 degree steps, 181 rays covering -90..+90
    range_min_m=0.1,
    range_max_m=30.0,
)
_N_RAYS = 181


def test_constant_range_forms_a_circle_of_known_radius() -> None:
    r = 2.5
    ranges = [r] * _N_RAYS

    points = scan_to_points(ranges, _GEOMETRY)

    assert len(points) == _N_RAYS
    for p in points:
        assert p.norm() == pytest.approx(r)
        assert p.z == 0.0


def test_zero_angle_ray_points_along_sensor_x() -> None:
    r = 3.0
    ranges = [r] * _N_RAYS
    zero_angle_index = round(-_GEOMETRY.angle_min_rad / _GEOMETRY.angle_step_rad)

    points = scan_to_points(ranges, _GEOMETRY)

    p = points[zero_angle_index]
    assert p.x == pytest.approx(r, abs=1e-6)
    assert p.y == pytest.approx(0.0, abs=1e-6)


def test_no_return_and_invalid_ranges_are_dropped() -> None:
    ranges = [5.0] * _N_RAYS
    ranges[0] = _GEOMETRY.range_max_m  # gz's no-hit convention
    ranges[1] = math.nan
    ranges[2] = 0.05  # below range_min_m

    points = scan_to_points(ranges, _GEOMETRY)

    assert len(points) == _N_RAYS - 3


def test_ninety_degree_ray_points_along_sensor_y() -> None:
    r = 1.0
    ranges = [r] * _N_RAYS
    last_index = _N_RAYS - 1  # angle_min + 180*step == +pi/2

    points = scan_to_points(ranges, _GEOMETRY)

    p = points[last_index]
    assert p.x == pytest.approx(0.0, abs=1e-6)
    assert p.y == pytest.approx(r, abs=1e-6)

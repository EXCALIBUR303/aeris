from __future__ import annotations

import pytest

from aeris.core.frames.vector import Vec3
from aeris.evaluation.metrics.nav import (
    control_smoothness,
    distance_travelled_m,
    flight_time_s,
    mean_waypoint_error_m,
    return_to_base_success,
)


def test_distance_travelled_hand_computed() -> None:
    # (0,0,0) -> (3,0,0) -> (3,4,0): legs of 3m and 4m = 7m total.
    positions = [Vec3(0, 0, 0), Vec3(3, 0, 0), Vec3(3, 4, 0)]
    assert distance_travelled_m(positions) == pytest.approx(7.0)


def test_distance_travelled_empty_or_single_point_is_zero() -> None:
    assert distance_travelled_m([]) == 0.0
    assert distance_travelled_m([Vec3(1, 2, 3)]) == 0.0


def test_flight_time_hand_computed() -> None:
    assert flight_time_s(10.0, 42.5) == pytest.approx(32.5)


def test_flight_time_rejects_land_before_takeoff() -> None:
    with pytest.raises(ValueError, match="before"):
        flight_time_s(10.0, 5.0)


def test_control_smoothness_hand_computed() -> None:
    # v: (0,0,0) -> (1,0,0) -> (2,0,0), dt=1s: accel = 1 m/s^2 each step,
    # squared = 1.0 each -> mean = 1.0.
    velocities = [Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(2, 0, 0)]
    assert control_smoothness(velocities, 1.0) == pytest.approx(1.0)


def test_control_smoothness_constant_velocity_is_zero() -> None:
    velocities = [Vec3(1, 1, 0), Vec3(1, 1, 0), Vec3(1, 1, 0)]
    assert control_smoothness(velocities, 0.5) == pytest.approx(0.0)


def test_control_smoothness_scales_with_dt() -> None:
    velocities = [Vec3(0, 0, 0), Vec3(1, 0, 0)]
    # accel = dv/dt: halving dt doubles accel, quadruples squared accel.
    at_dt_1 = control_smoothness(velocities, 1.0)
    at_dt_half = control_smoothness(velocities, 0.5)
    assert at_dt_half == pytest.approx(4.0 * at_dt_1)


def test_control_smoothness_needs_at_least_two_samples() -> None:
    assert control_smoothness([], 1.0) == 0.0
    assert control_smoothness([Vec3(0, 0, 0)], 1.0) == 0.0


def test_control_smoothness_rejects_non_positive_dt() -> None:
    with pytest.raises(ValueError):
        control_smoothness([Vec3(0, 0, 0), Vec3(1, 0, 0)], 0.0)


def test_mean_waypoint_error_hand_computed() -> None:
    assert mean_waypoint_error_m([0.1, 0.2, 0.3]) == pytest.approx(0.2)


def test_mean_waypoint_error_empty_is_zero() -> None:
    assert mean_waypoint_error_m([]) == 0.0


def test_return_to_base_success_true_case() -> None:
    assert return_to_base_success(
        landed=True, disarmed=True, land_position=Vec3(0.5, 0.5, 0), home_position=Vec3(0, 0, 0)
    )


def test_return_to_base_success_false_when_not_landed() -> None:
    assert not return_to_base_success(
        landed=False, disarmed=True, land_position=Vec3(0, 0, 0), home_position=Vec3(0, 0, 0)
    )


def test_return_to_base_success_false_when_not_disarmed() -> None:
    assert not return_to_base_success(
        landed=True, disarmed=False, land_position=Vec3(0, 0, 0), home_position=Vec3(0, 0, 0)
    )


def test_return_to_base_success_false_when_too_far() -> None:
    assert not return_to_base_success(
        landed=True,
        disarmed=True,
        land_position=Vec3(10, 0, 0),
        home_position=Vec3(0, 0, 0),
        r_home_m=1.5,
    )

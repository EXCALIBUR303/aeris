from __future__ import annotations

import math

import pytest
from _fakes import make_state

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import Vec3
from aeris.safety.envelope import Envelope
from aeris.safety.validator import CommandValidator
from aeris.vehicle.interface import PositionSetpoint, VelocitySetpoint


@pytest.fixture
def envelope() -> Envelope:
    return Envelope.model_validate(
        {
            "v_xy_max_mps": 3.0,
            "vz_max_mps": 1.5,
            "yaw_rate_max_radps": 1.0,
            "accel_max_mps2": 2.0,
            "altitude_floor_m": 0.3,
            "altitude_ceiling_m": 20.0,
            "geofence_radius_m": 30.0,
        }
    )


@pytest.fixture
def validator(envelope: Envelope) -> CommandValidator:
    return CommandValidator(envelope)


# --- NaN/Inf rejection ---------------------------------------------------------------


def test_rejects_nan_velocity(validator: CommandValidator) -> None:
    sp = VelocitySetpoint(velocity=Vec3(float("nan"), 0.0, 0.0))
    result = validator.validate(sp, state=make_state(pose_odom=Vec3(0, 0, 5)), now_s=0.0)
    assert not result.accepted
    assert "NaN/Inf" in result.reason


def test_rejects_inf_velocity(validator: CommandValidator) -> None:
    sp = VelocitySetpoint(velocity=Vec3(0.0, float("inf"), 0.0))
    result = validator.validate(sp, state=make_state(pose_odom=Vec3(0, 0, 5)), now_s=0.0)
    assert not result.accepted


def test_rejects_nan_yaw_rate(validator: CommandValidator) -> None:
    sp = VelocitySetpoint(velocity=Vec3(0.5, 0, 0), yaw_rate_radps=float("nan"))
    result = validator.validate(sp, state=make_state(pose_odom=Vec3(0, 0, 5)), now_s=0.0)
    assert not result.accepted


def test_rejects_nan_position(validator: CommandValidator) -> None:
    sp = PositionSetpoint(position_odom=Vec3(float("nan"), 0.0, 5.0))
    result = validator.validate(sp, state=make_state(), now_s=0.0)
    assert not result.accepted


# --- bounds ----------------------------------------------------------------------


def test_accepts_velocity_within_bounds(validator: CommandValidator) -> None:
    sp = VelocitySetpoint(velocity=Vec3(1.0, 0.0, 0.0))
    result = validator.validate(sp, state=make_state(pose_odom=Vec3(0, 0, 5)), now_s=0.0)
    assert result.accepted, result.reason


def test_rejects_v_xy_over_max(validator: CommandValidator, envelope: Envelope) -> None:
    sp = VelocitySetpoint(velocity=Vec3(envelope.v_xy_max_mps + 1.0, 0.0, 0.0))
    result = validator.validate(sp, state=make_state(pose_odom=Vec3(0, 0, 5)), now_s=0.0)
    assert not result.accepted
    assert "v_xy_max" in result.reason


def test_rejects_vz_over_max(validator: CommandValidator, envelope: Envelope) -> None:
    sp = VelocitySetpoint(velocity=Vec3(0.0, 0.0, envelope.vz_max_mps + 1.0))
    result = validator.validate(sp, state=make_state(pose_odom=Vec3(0, 0, 5)), now_s=0.0)
    assert not result.accepted
    assert "vz_max" in result.reason


def test_rejects_yaw_rate_over_max(validator: CommandValidator, envelope: Envelope) -> None:
    sp = VelocitySetpoint(
        velocity=Vec3(0.5, 0, 0), yaw_rate_radps=envelope.yaw_rate_max_radps + 0.5
    )
    result = validator.validate(sp, state=make_state(pose_odom=Vec3(0, 0, 5)), now_s=0.0)
    assert not result.accepted
    assert "yaw_rate_max" in result.reason


def test_body_frame_velocity_is_rotated_to_world_before_bounds_check(
    validator: CommandValidator, envelope: Envelope
) -> None:
    # 90deg yaw: body +x (forward) becomes world +y (North). A body-frame
    # command that's within bounds in body x should still be within bounds
    # after rotation (bounds are frame-agnostic magnitudes) -- this checks
    # the rotation actually happens (not a magnitude test on its own).
    q = Quaternion.from_yaw(math.pi / 2)
    sp = VelocitySetpoint(velocity=Vec3(1.0, 0.0, 0.0), frame="body")
    state = make_state(pose_odom=Vec3(0, 0, 5), orientation_odom=q)
    result = validator.validate(sp, state=state, now_s=0.0)
    assert result.accepted, result.reason


def test_body_frame_velocity_over_max_is_rejected_in_world_frame(
    validator: CommandValidator, envelope: Envelope
) -> None:
    q = Quaternion.identity()
    sp = VelocitySetpoint(velocity=Vec3(envelope.v_xy_max_mps + 1.0, 0.0, 0.0), frame="body")
    state = make_state(pose_odom=Vec3(0, 0, 5), orientation_odom=q)
    result = validator.validate(sp, state=state, now_s=0.0)
    assert not result.accepted


def test_unknown_velocity_frame_is_rejected(validator: CommandValidator) -> None:
    sp = VelocitySetpoint(velocity=Vec3(0.5, 0, 0), frame="nonsense")
    result = validator.validate(sp, state=make_state(pose_odom=Vec3(0, 0, 5)), now_s=0.0)
    assert not result.accepted


# --- acceleration / rate-of-change ------------------------------------------------


def test_rejects_acceleration_over_max(validator: CommandValidator, envelope: Envelope) -> None:
    state = make_state(pose_odom=Vec3(0, 0, 5))
    first = VelocitySetpoint(velocity=Vec3(0.0, 0.0, 0.0))
    assert validator.validate(first, state=state, now_s=0.0).accepted

    # A jump of v_xy_max in 0.1s implies accel = v_xy_max/0.1, comfortably
    # over accel_max_mps2 for these fixture numbers (3.0/0.1 = 30 >> 2.0).
    second = VelocitySetpoint(velocity=Vec3(envelope.v_xy_max_mps, 0.0, 0.0))
    result = validator.validate(second, state=state, now_s=0.1)
    assert not result.accepted
    assert "acceleration" in result.reason


def test_accepts_gradual_acceleration_within_limit(
    validator: CommandValidator, envelope: Envelope
) -> None:
    state = make_state(pose_odom=Vec3(0, 0, 5))
    assert validator.validate(
        VelocitySetpoint(velocity=Vec3(0.0, 0.0, 0.0)), state=state, now_s=0.0
    ).accepted
    # accel_max_mps2=2.0 over dt=1.0s allows up to a 2.0 m/s change.
    result = validator.validate(
        VelocitySetpoint(velocity=Vec3(1.0, 0.0, 0.0)), state=state, now_s=1.0
    )
    assert result.accepted, result.reason


def test_reset_rate_limit_history_forgets_the_last_setpoint(
    validator: CommandValidator, envelope: Envelope
) -> None:
    state = make_state(pose_odom=Vec3(0, 0, 5))
    validator.validate(VelocitySetpoint(velocity=Vec3(0.0, 0.0, 0.0)), state=state, now_s=0.0)
    validator.reset_rate_limit_history()
    # Without history, even a large jump is accepted (nothing to diff against).
    result = validator.validate(
        VelocitySetpoint(velocity=Vec3(envelope.v_xy_max_mps, 0.0, 0.0)), state=state, now_s=0.1
    )
    assert result.accepted, result.reason


# --- altitude floor/ceiling --------------------------------------------------------


def test_rejects_descent_at_altitude_floor(validator: CommandValidator, envelope: Envelope) -> None:
    state = make_state(pose_odom=Vec3(0, 0, envelope.altitude_floor_m))
    sp = VelocitySetpoint(velocity=Vec3(0.0, 0.0, -0.5))
    result = validator.validate(sp, state=state, now_s=0.0)
    assert not result.accepted
    assert "altitude_floor_m" in result.reason


def test_allows_climb_at_altitude_floor(validator: CommandValidator, envelope: Envelope) -> None:
    state = make_state(pose_odom=Vec3(0, 0, envelope.altitude_floor_m))
    sp = VelocitySetpoint(velocity=Vec3(0.0, 0.0, 0.5))
    result = validator.validate(sp, state=state, now_s=0.0)
    assert result.accepted, result.reason


def test_rejects_climb_at_altitude_ceiling(validator: CommandValidator, envelope: Envelope) -> None:
    state = make_state(pose_odom=Vec3(0, 0, envelope.altitude_ceiling_m))
    sp = VelocitySetpoint(velocity=Vec3(0.0, 0.0, 0.5))
    result = validator.validate(sp, state=state, now_s=0.0)
    assert not result.accepted
    assert "altitude_ceiling_m" in result.reason


def test_rejects_position_target_above_ceiling(
    validator: CommandValidator, envelope: Envelope
) -> None:
    sp = PositionSetpoint(position_odom=Vec3(0, 0, envelope.altitude_ceiling_m + 1.0))
    result = validator.validate(sp, state=make_state(), now_s=0.0)
    assert not result.accepted


def test_rejects_position_target_below_floor(
    validator: CommandValidator, envelope: Envelope
) -> None:
    sp = PositionSetpoint(position_odom=Vec3(0, 0, envelope.altitude_floor_m - 0.1))
    result = validator.validate(sp, state=make_state(), now_s=0.0)
    assert not result.accepted


# --- geofence ----------------------------------------------------------------------


def test_rejects_velocity_moving_further_outside_geofence(
    validator: CommandValidator, envelope: Envelope
) -> None:
    state = make_state(
        pose_odom=Vec3(envelope.geofence_radius_m + 1.0, 0.0, 5.0), home_odom=Vec3(0, 0, 0)
    )
    sp = VelocitySetpoint(velocity=Vec3(1.0, 0.0, 0.0))  # continuing outward (+x, home at origin)
    result = validator.validate(sp, state=state, now_s=0.0)
    assert not result.accepted
    assert "geofence_radius_m" in result.reason


def test_allows_velocity_recovering_back_inside_geofence(
    validator: CommandValidator, envelope: Envelope
) -> None:
    state = make_state(
        pose_odom=Vec3(envelope.geofence_radius_m + 1.0, 0.0, 5.0), home_odom=Vec3(0, 0, 0)
    )
    sp = VelocitySetpoint(velocity=Vec3(-1.0, 0.0, 0.0))  # heading back toward home
    result = validator.validate(sp, state=state, now_s=0.0)
    assert result.accepted, result.reason


def test_rejects_position_target_outside_geofence(
    validator: CommandValidator, envelope: Envelope
) -> None:
    sp = PositionSetpoint(position_odom=Vec3(envelope.geofence_radius_m + 5.0, 0.0, 5.0))
    result = validator.validate(sp, state=make_state(home_odom=Vec3(0, 0, 0)), now_s=0.0)
    assert not result.accepted


def test_geofence_skipped_when_home_unknown(
    validator: CommandValidator, envelope: Envelope
) -> None:
    state = make_state(pose_odom=Vec3(1000.0, 0.0, 5.0), home_odom=None)
    sp = VelocitySetpoint(velocity=Vec3(1.0, 0.0, 0.0))
    result = validator.validate(sp, state=state, now_s=0.0)
    assert result.accepted, result.reason

import math

import pytest

from aeris.core.frames.vector import Vec3
from aeris.vehicle.interface import PositionSetpoint, VelocitySetpoint
from aeris.vehicle.px4_mavlink.setpoints import (
    MAV_FRAME_BODY_OFFSET_NED,
    MAV_FRAME_LOCAL_NED,
    encode_setpoint,
)


def test_position_setpoint_converts_enu_to_ned():
    sp = PositionSetpoint(position_odom=Vec3(1.0, 2.0, 3.0))  # E=1,N=2,U=3
    enc = encode_setpoint(sp)
    assert enc.coordinate_frame == MAV_FRAME_LOCAL_NED
    assert enc.x == pytest.approx(2.0)  # N
    assert enc.y == pytest.approx(1.0)  # E
    assert enc.z == pytest.approx(-3.0)  # -U


def test_position_setpoint_without_yaw_ignores_yaw_bit():
    sp = PositionSetpoint(position_odom=Vec3(0, 0, 0), yaw_rad=None)
    enc = encode_setpoint(sp)
    IGNORE_YAW = 1 << 10
    assert enc.type_mask & IGNORE_YAW


def test_position_setpoint_with_yaw_converts_and_does_not_ignore():
    sp = PositionSetpoint(position_odom=Vec3(0, 0, 0), yaw_rad=0.0)  # facing East
    enc = encode_setpoint(sp)
    IGNORE_YAW = 1 << 10
    assert not (enc.type_mask & IGNORE_YAW)
    assert enc.yaw == pytest.approx(math.pi / 2)  # NED yaw for ENU east-facing


def test_position_setpoint_ignores_velocity_and_accel():
    sp = PositionSetpoint(position_odom=Vec3(0, 0, 0))
    enc = encode_setpoint(sp)
    for bit in range(3, 9):  # vx,vy,vz,afx,afy,afz
        assert enc.type_mask & (1 << bit)


def test_velocity_setpoint_odom_frame_converts_enu_to_ned():
    sp = VelocitySetpoint(velocity=Vec3(1.0, 0.0, 0.0), frame="odom")  # 1 m/s East
    enc = encode_setpoint(sp)
    assert enc.coordinate_frame == MAV_FRAME_LOCAL_NED
    assert enc.vx == pytest.approx(0.0)
    assert enc.vy == pytest.approx(1.0)
    assert enc.vz == pytest.approx(0.0)


def test_velocity_setpoint_body_frame_converts_flu_to_frd():
    sp = VelocitySetpoint(velocity=Vec3(1.0, 0.0, 0.0), frame="body")  # 1 m/s Forward
    enc = encode_setpoint(sp)
    assert enc.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED
    assert enc.vx == pytest.approx(1.0)
    assert enc.vy == pytest.approx(0.0)
    assert enc.vz == pytest.approx(0.0)


def test_velocity_setpoint_unknown_frame_raises():
    sp = VelocitySetpoint(velocity=Vec3(0, 0, 0), frame="bogus")
    with pytest.raises(ValueError):
        encode_setpoint(sp)


def test_velocity_setpoint_ignores_position_and_accel():
    sp = VelocitySetpoint(velocity=Vec3(0, 0, 0))
    enc = encode_setpoint(sp)
    for bit in (0, 1, 2, 6, 7, 8):  # x,y,z,afx,afy,afz
        assert enc.type_mask & (1 << bit)


def test_velocity_setpoint_without_yaw_rate_ignores_yaw_rate_bit():
    sp = VelocitySetpoint(velocity=Vec3(0, 0, 0), yaw_rate_radps=None)
    enc = encode_setpoint(sp)
    IGNORE_YAW_RATE = 1 << 11
    assert enc.type_mask & IGNORE_YAW_RATE


def test_velocity_setpoint_with_yaw_rate_negated_for_ned():
    sp = VelocitySetpoint(velocity=Vec3(0, 0, 0), yaw_rate_radps=0.5)
    enc = encode_setpoint(sp)
    IGNORE_YAW_RATE = 1 << 11
    assert not (enc.type_mask & IGNORE_YAW_RATE)
    assert enc.yaw_rate == pytest.approx(-0.5)

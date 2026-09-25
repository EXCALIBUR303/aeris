"""Encoding :class:`Setpoint` into MAVLink ``SET_POSITION_TARGET_LOCAL_NED``.

The ENU->NED / FLU->FRD conversion happens here — the one place it's
allowed outside this file being :mod:`aeris.core.frames.conventions`
itself (import-linter contract).
"""

from __future__ import annotations

from aeris.core.frames.conventions import enu_to_ned_vec, enu_yaw_to_ned_yaw, flu_to_frd_vec
from aeris.vehicle.interface import PositionSetpoint, VelocitySetpoint

# MAV_FRAME
MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_BODY_OFFSET_NED = 9

# POSITION_TARGET_TYPEMASK bits
_IGNORE_X = 1 << 0
_IGNORE_Y = 1 << 1
_IGNORE_Z = 1 << 2
_IGNORE_VX = 1 << 3
_IGNORE_VY = 1 << 4
_IGNORE_VZ = 1 << 5
_IGNORE_AFX = 1 << 6
_IGNORE_AFY = 1 << 7
_IGNORE_AFZ = 1 << 8
_IGNORE_YAW = 1 << 10
_IGNORE_YAW_RATE = 1 << 11

_IGNORE_ALL_VELOCITY = _IGNORE_VX | _IGNORE_VY | _IGNORE_VZ
_IGNORE_ALL_ACCEL = _IGNORE_AFX | _IGNORE_AFY | _IGNORE_AFZ
_IGNORE_ALL_POSITION = _IGNORE_X | _IGNORE_Y | _IGNORE_Z


class EncodedSetpoint:
    """The exact fields ``set_position_target_local_ned_send`` needs."""

    __slots__ = (
        "afx",
        "afy",
        "afz",
        "coordinate_frame",
        "type_mask",
        "vx",
        "vy",
        "vz",
        "x",
        "y",
        "yaw",
        "yaw_rate",
        "z",
    )

    def __init__(
        self,
        *,
        coordinate_frame: int,
        type_mask: int,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        yaw: float = 0.0,
        yaw_rate: float = 0.0,
    ) -> None:
        self.coordinate_frame = coordinate_frame
        self.type_mask = type_mask
        self.x, self.y, self.z = x, y, z
        self.vx, self.vy, self.vz = vx, vy, vz
        self.afx = self.afy = self.afz = 0.0
        self.yaw = yaw
        self.yaw_rate = yaw_rate


def encode_setpoint(sp: PositionSetpoint | VelocitySetpoint) -> EncodedSetpoint:
    """Convert an ENU/FLU :class:`Setpoint` into NED/FRD MAVLink fields."""
    if isinstance(sp, PositionSetpoint):
        pos_ned = enu_to_ned_vec(sp.position_odom)
        mask = _IGNORE_ALL_VELOCITY | _IGNORE_ALL_ACCEL | _IGNORE_YAW_RATE
        yaw_ned = 0.0
        if sp.yaw_rad is None:
            mask |= _IGNORE_YAW
        else:
            yaw_ned = enu_yaw_to_ned_yaw(sp.yaw_rad)
        return EncodedSetpoint(
            coordinate_frame=MAV_FRAME_LOCAL_NED,
            type_mask=mask,
            x=pos_ned.x,
            y=pos_ned.y,
            z=pos_ned.z,
            yaw=yaw_ned,
        )

    # VelocitySetpoint
    if sp.frame == "odom":
        vel_ned = enu_to_ned_vec(sp.velocity)
        frame = MAV_FRAME_LOCAL_NED
    elif sp.frame == "body":
        vel_ned = flu_to_frd_vec(sp.velocity)
        frame = MAV_FRAME_BODY_OFFSET_NED
    else:
        raise ValueError(
            f"unknown VelocitySetpoint.frame: {sp.frame!r} (expected 'odom' or 'body')"
        )

    mask = _IGNORE_ALL_POSITION | _IGNORE_ALL_ACCEL | _IGNORE_YAW
    yaw_rate_ned = 0.0
    if sp.yaw_rate_radps is None:
        mask |= _IGNORE_YAW_RATE
    else:
        # Yaw *rate* has no East/North asymmetry to fix (spec §19.3's yaw
        # formula is about the *offset*, psi_NED = pi/2 - psi_ENU) — only
        # its rotational sense flips, since ENU yaw is CCW-positive about
        # +Z (Up) and NED yaw is CW-positive about +Z (Down).
        yaw_rate_ned = -sp.yaw_rate_radps

    return EncodedSetpoint(
        coordinate_frame=frame,
        type_mask=mask,
        vx=vel_ned.x,
        vy=vel_ned.y,
        vz=vel_ned.z,
        yaw_rate=yaw_rate_ned,
    )

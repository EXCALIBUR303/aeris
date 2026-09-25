"""S1 — command validation (spec §16.2).

``CommandValidator`` is a pure gate: it never talks to the vehicle. It
either accepts a setpoint (returning ``ValidationResult(accepted=True)``)
or rejects it with a human-readable reason. ``aeris.safety.supervisor``
owns actually forwarding accepted setpoints to the ``CommandPort``.

Stateful only in the sense of remembering the last *accepted* velocity
setpoint and when it was accepted, to enforce the acceleration /
rate-of-change limit -- this is instance state on the validator the
supervisor owns, not module-global state (spec §14.2: "no global mutable
state").
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from aeris.core.frames.vector import Vec3
from aeris.safety.envelope import Envelope
from aeris.vehicle.interface import PositionSetpoint, Setpoint, VehicleState, VelocitySetpoint


@dataclass(frozen=True, slots=True)
class ValidationResult:
    accepted: bool
    reason: str = ""


def _is_finite_vec3(v: Vec3) -> bool:
    return math.isfinite(v.x) and math.isfinite(v.y) and math.isfinite(v.z)


def _to_world_velocity(sp: VelocitySetpoint, state: VehicleState) -> Vec3:
    """Resolve a possibly body-frame velocity setpoint into the ENU/odom frame.

    A ``"body"`` (FLU) setpoint is rotated by the vehicle's current
    estimated orientation -- exact for any attitude, not just the
    near-level case a naive "body z ~= world z" shortcut would assume.
    """
    if sp.frame == "odom":
        return sp.velocity
    if sp.frame == "body":
        return state.orientation_odom.rotate(sp.velocity)
    raise ValueError(f"unknown VelocitySetpoint.frame: {sp.frame!r} (expected 'odom' or 'body')")


class CommandValidator:
    """S1 command validation against a fixed :class:`Envelope`."""

    def __init__(self, envelope: Envelope) -> None:
        self._envelope = envelope
        self._last_velocity_world: Vec3 | None = None
        self._last_velocity_t_s: float | None = None

    @property
    def envelope(self) -> Envelope:
        return self._envelope

    def reset_rate_limit_history(self) -> None:
        """Forget the last accepted velocity -- call when (re)starting offboard."""
        self._last_velocity_world = None
        self._last_velocity_t_s = None

    def validate(self, sp: Setpoint, *, state: VehicleState, now_s: float) -> ValidationResult:
        if isinstance(sp, VelocitySetpoint):
            return self._validate_velocity(sp, state=state, now_s=now_s)
        if isinstance(sp, PositionSetpoint):
            return self._validate_position(sp, state=state)
        raise TypeError(f"unknown setpoint type: {type(sp).__name__}")

    def _validate_velocity(
        self, sp: VelocitySetpoint, *, state: VehicleState, now_s: float
    ) -> ValidationResult:
        if not _is_finite_vec3(sp.velocity):
            return ValidationResult(False, "velocity setpoint contains NaN/Inf")
        if sp.yaw_rate_radps is not None and not math.isfinite(sp.yaw_rate_radps):
            return ValidationResult(False, "yaw_rate_radps is NaN/Inf")

        try:
            v_world = _to_world_velocity(sp, state)
        except ValueError as exc:
            return ValidationResult(False, str(exc))

        v_xy_norm = math.hypot(v_world.x, v_world.y)
        env = self._envelope
        if v_xy_norm > env.v_xy_max_mps:
            return ValidationResult(
                False, f"||v_xy||={v_xy_norm:.2f} m/s exceeds v_xy_max={env.v_xy_max_mps} m/s"
            )
        if abs(v_world.z) > env.vz_max_mps:
            return ValidationResult(
                False, f"|vz|={abs(v_world.z):.2f} m/s exceeds vz_max={env.vz_max_mps} m/s"
            )
        if sp.yaw_rate_radps is not None and abs(sp.yaw_rate_radps) > env.yaw_rate_max_radps:
            return ValidationResult(
                False,
                f"|yaw_rate|={abs(sp.yaw_rate_radps):.2f} rad/s exceeds "
                f"yaw_rate_max={env.yaw_rate_max_radps} rad/s",
            )

        if self._last_velocity_world is not None and self._last_velocity_t_s is not None:
            dt = now_s - self._last_velocity_t_s
            if dt > 0:
                dv = (v_world - self._last_velocity_world).norm()
                accel = dv / dt
                if accel > env.accel_max_mps2:
                    return ValidationResult(
                        False,
                        f"implied acceleration={accel:.2f} m/s^2 exceeds "
                        f"accel_max={env.accel_max_mps2} m/s^2 (dt={dt:.3f}s)",
                    )

        z = state.pose_odom.z
        if z <= env.altitude_floor_m and v_world.z < 0:
            return ValidationResult(
                False, f"at/below altitude_floor_m={env.altitude_floor_m} and commanding descent"
            )
        if z >= env.altitude_ceiling_m and v_world.z > 0:
            return ValidationResult(
                False, f"at/above altitude_ceiling_m={env.altitude_ceiling_m} and commanding climb"
            )

        if state.home_odom is not None:
            dx = state.pose_odom.x - state.home_odom.x
            dy = state.pose_odom.y - state.home_odom.y
            horiz_dist = math.hypot(dx, dy)
            # Already at/outside the fence -- only reject a command that
            # would move it further outward (radially). A component
            # pointed back inward is allowed (recovery move).
            if horiz_dist >= env.geofence_radius_m and horiz_dist > 1e-9:
                outward_component = (v_world.x * dx + v_world.y * dy) / horiz_dist
                if outward_component > 0:
                    return ValidationResult(
                        False,
                        f"at/outside geofence_radius_m={env.geofence_radius_m} "
                        f"(dist={horiz_dist:.1f}m) and commanding further outward",
                    )

        self._last_velocity_world = v_world
        self._last_velocity_t_s = now_s
        return ValidationResult(True)

    def _validate_position(self, sp: PositionSetpoint, *, state: VehicleState) -> ValidationResult:
        if not _is_finite_vec3(sp.position_odom):
            return ValidationResult(False, "position setpoint contains NaN/Inf")
        if sp.yaw_rad is not None and not math.isfinite(sp.yaw_rad):
            return ValidationResult(False, "yaw_rad is NaN/Inf")

        env = self._envelope
        z = sp.position_odom.z
        if z < env.altitude_floor_m:
            return ValidationResult(
                False, f"target z={z:.2f}m below altitude_floor_m={env.altitude_floor_m}"
            )
        if z > env.altitude_ceiling_m:
            return ValidationResult(
                False, f"target z={z:.2f}m above altitude_ceiling_m={env.altitude_ceiling_m}"
            )

        if state.home_odom is not None:
            dx = sp.position_odom.x - state.home_odom.x
            dy = sp.position_odom.y - state.home_odom.y
            horiz_dist = math.hypot(dx, dy)
            if horiz_dist > env.geofence_radius_m:
                return ValidationResult(
                    False,
                    f"target horizontal distance={horiz_dist:.1f}m exceeds "
                    f"geofence_radius_m={env.geofence_radius_m}",
                )

        return ValidationResult(True)

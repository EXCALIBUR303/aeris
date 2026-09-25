"""The fixed AERIS <-> PX4 frame conversions (spec §19, ADR-0008).

**These functions are the *only* place ENU<->NED / FLU<->FRD conversion
may happen anywhere in AERIS** (import-linter enforces this — see
``pyproject.toml``: only ``aeris.vehicle.px4_*`` may import this module's
NED/FRD-facing functions). Every other AERIS package works exclusively in
ENU (world/odometry/map) and FLU (body), per ADR-0008.

All formulas are transcribed directly from spec §19.3 and verified against
the pinned PX4 checkout's own conventions (MAVLink ``LOCAL_POSITION_NED``,
``ATTITUDE_QUATERNION``) in Phase 4.
"""

from __future__ import annotations

import math

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import Vec3
from aeris.core.units import wrap_pi

# --- ENU <-> NED ---------------------------------------------------------------
#
# R_NED_ENU = [[0,1,0],[1,0,0],[0,0,-1]] (spec §19.3). An involution (its
# own inverse) and a proper rotation (det = +1): a 180 deg rotation about
# the axis bisecting the East and North axes, (1,1,0)/sqrt(2).

_ENU_NED_ROTATION = Quaternion.from_axis_angle(Vec3(1.0, 1.0, 0.0), math.pi)


def enu_to_ned_vec(v_enu: Vec3) -> Vec3:
    """``(x_n, y_n, z_n) = (y_e, x_e, -z_e)`` (spec §19.3)."""
    return Vec3(v_enu.y, v_enu.x, -v_enu.z)


def ned_to_enu_vec(v_ned: Vec3) -> Vec3:
    """The inverse of :func:`enu_to_ned_vec` — same formula (involution)."""
    return Vec3(v_ned.y, v_ned.x, -v_ned.z)


def enu_ned_rotation() -> Quaternion:
    """The fixed rotation quaternion ``R_NED_ENU`` (== ``R_ENU_NED``, involution)."""
    return _ENU_NED_ROTATION


# --- FLU <-> FRD -----------------------------------------------------------------
#
# R_FRD_FLU = diag(1, -1, -1) (spec §19.3). Also an involution and a proper
# rotation: a 180 deg rotation about the body +X (Forward) axis.

_FLU_FRD_ROTATION = Quaternion.from_axis_angle(Vec3(1.0, 0.0, 0.0), math.pi)


def flu_to_frd_vec(v_flu: Vec3) -> Vec3:
    """``(x, -y, -z)`` (spec §19.3)."""
    return Vec3(v_flu.x, -v_flu.y, -v_flu.z)


def frd_to_flu_vec(v_frd: Vec3) -> Vec3:
    """The inverse of :func:`flu_to_frd_vec` — same formula (involution)."""
    return Vec3(v_frd.x, -v_frd.y, -v_frd.z)


def flu_frd_rotation() -> Quaternion:
    """The fixed rotation quaternion ``R_FRD_FLU`` (== ``R_FLU_FRD``, involution)."""
    return _FLU_FRD_ROTATION


# --- Attitude (full orientation) --------------------------------------------------


def attitude_enu_body_to_ned_frd(q_O_B: Quaternion) -> Quaternion:
    """Convert a body orientation from AERIS (ENU/FLU) to PX4 (NED/FRD).

    Spec §19.3: ``R_L_Bfrd = R_NED_ENU * R_O_B * R_FLU_FRD``.
    """
    return _ENU_NED_ROTATION.compose(q_O_B).compose(_FLU_FRD_ROTATION)


def attitude_ned_frd_to_enu_body(q_L_Bfrd: Quaternion) -> Quaternion:
    """The inverse of :func:`attitude_enu_body_to_ned_frd`.

    Both fixed rotations are involutions (self-inverse), so
    ``R_O_B = R_NED_ENU * R_L_Bfrd * R_FLU_FRD`` — the identical
    sandwich, since ``R_NED_ENU^-1 == R_NED_ENU`` and
    ``R_FLU_FRD^-1 == R_FLU_FRD``.
    """
    return _ENU_NED_ROTATION.compose(q_L_Bfrd).compose(_FLU_FRD_ROTATION)


# --- Yaw (scalar heading) ---------------------------------------------------------
#
# PX4/NED yaw 0 = North, positive clockwise (from above) when following
# the right-hand rule about NED's +Z (Down) axis. AERIS/ENU yaw 0 = East,
# positive counter-clockwise about ENU's +Z (Up) axis (spec §19.2/§19.3).


def enu_yaw_to_ned_yaw(yaw_enu_rad: float) -> float:
    """``psi_NED = wrap(pi/2 - psi_ENU)`` (spec §19.3)."""
    return wrap_pi(math.pi / 2.0 - yaw_enu_rad)


def ned_yaw_to_enu_yaw(yaw_ned_rad: float) -> float:
    """The inverse of :func:`enu_yaw_to_ned_yaw` — same form, solved for psi_ENU."""
    return wrap_pi(math.pi / 2.0 - yaw_ned_rad)


# --- Camera optical <-> body -------------------------------------------------------
#
# R_B_Copt = [[0,0,1],[-1,0,0],[0,-1,0]] (spec §19.3): optical +z (forward
# along the lens boresight) -> body +x (Forward); optical +x (right in the
# image) -> body -y (i.e. body's Right, since FLU's +y is Left); optical +y
# (down in the image) -> body -z (Down, since FLU's +z is Up). REP-103
# optical convention: x right, y down, z forward.

_CAMERA_OPTICAL_TO_BODY_ROTATION = Quaternion.from_matrix_columns(
    col_x=Vec3(0.0, -1.0, 0.0),  # optical +x -> body -y
    col_y=Vec3(0.0, 0.0, -1.0),  # optical +y -> body -z
    col_z=Vec3(1.0, 0.0, 0.0),  # optical +z -> body +x
)


def camera_optical_to_body_rotation() -> Quaternion:
    """The fixed rotation ``R_B_Copt`` (spec §19.3) — mount rotation is separate.

    The full camera extrinsic ``T_B_C = T_mount * Transform.from_rotation(this)``
    also needs the specific sensor's mount pose from its SDF (spec §18.2) —
    this function only captures the fixed optical<->robotics axis
    convention, which is the same for every camera.
    """
    return _CAMERA_OPTICAL_TO_BODY_ROTATION


# --- AERIS map frame (M) <-> three.js scene frame (V) ------------------------------
#
# three.js is Y-up, right-handed: (x_V, y_V, z_V) = (x_M, z_M, -y_M) (spec
# §19.1/§19.3). Frontend-only; included here so the one conversion formula
# has a single, tested source of truth even though the frontend itself is
# TypeScript, not Python (Phase 27+ ports this exact formula).


def map_enu_to_threejs(v_m: Vec3) -> Vec3:
    """``(x_V, y_V, z_V) = (x_M, z_M, -y_M)`` (spec §19.3)."""
    return Vec3(v_m.x, v_m.z, -v_m.y)

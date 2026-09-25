"""Decoding cached MAVLink messages into a :class:`VehicleState` (spec §15).

Pure functions over plain message-like objects (anything with the right
attributes — real pymavlink messages or a test double), so this is testable
without a live connection. All NED/FRD->ENU/FLU conversion happens here.
"""

from __future__ import annotations

from typing import Any, Protocol

from aeris.core.frames import Quaternion, Vec3
from aeris.core.frames.conventions import (
    attitude_ned_frd_to_enu_body,
    frd_to_flu_vec,
    ned_to_enu_vec,
)
from aeris.vehicle.interface import (
    BatterySimState,
    EkfFlags,
    FlightMode,
    GpsFixType,
    GpsStatus,
    LandedState,
    LinkStatus,
    VehicleState,
)
from aeris.vehicle.px4_mavlink.modes import decode_px4_custom_mode

_ARMED_BIT = 128  # MAV_MODE_FLAG_SAFETY_ARMED

_GPS_FIX_TYPE_MAP: dict[int, GpsFixType] = {
    0: GpsFixType.NO_GPS,
    1: GpsFixType.NO_FIX,
    2: GpsFixType.FIX_2D,
    3: GpsFixType.FIX_3D,
    4: GpsFixType.DGPS,
    5: GpsFixType.RTK_FLOAT,
    6: GpsFixType.RTK_FIXED,
}

_LANDED_STATE_MAP: dict[int, LandedState] = {
    0: LandedState.UNKNOWN,
    1: LandedState.ON_GROUND,
    2: LandedState.IN_AIR,
    3: LandedState.TAKEOFF,
    4: LandedState.LANDING,
}


class RawMessageCache(Protocol):
    """What :func:`build_vehicle_state` needs — matches the adapter's real
    cache and is easy to fake in tests (a plain ``dict`` satisfies this)."""

    def get(self, message_type: str) -> Any: ...


def build_vehicle_state(
    cache: RawMessageCache,
    *,
    t_sim_s: float,
    last_heartbeat_age_s: float,
    connected: bool,
) -> VehicleState:
    """Build a :class:`VehicleState` from the latest cached MAVLink messages.

    Missing optional messages (e.g. no ``HOME_POSITION`` yet) degrade
    gracefully rather than raising — a fresh connection legitimately hasn't
    seen every message type yet.
    """
    heartbeat = cache.get("HEARTBEAT")
    armed = bool(heartbeat.base_mode & _ARMED_BIT) if heartbeat is not None else False
    flight_mode = (
        decode_px4_custom_mode(heartbeat.custom_mode)
        if heartbeat is not None
        else FlightMode.UNKNOWN
    )

    ext_state = cache.get("EXTENDED_SYS_STATE")
    landed_state = (
        _LANDED_STATE_MAP.get(ext_state.landed_state, LandedState.UNKNOWN)
        if ext_state is not None
        else LandedState.UNKNOWN
    )

    local_pos = cache.get("LOCAL_POSITION_NED")
    if local_pos is not None:
        pose_odom = ned_to_enu_vec(Vec3(local_pos.x, local_pos.y, local_pos.z))
        velocity_odom = ned_to_enu_vec(Vec3(local_pos.vx, local_pos.vy, local_pos.vz))
    else:
        pose_odom = Vec3(0.0, 0.0, 0.0)
        velocity_odom = Vec3(0.0, 0.0, 0.0)

    attitude_q = cache.get("ATTITUDE_QUATERNION")
    if attitude_q is not None:
        q_ned_frd = Quaternion(attitude_q.q1, attitude_q.q2, attitude_q.q3, attitude_q.q4)
        orientation_odom = attitude_ned_frd_to_enu_body(q_ned_frd)
    else:
        orientation_odom = Quaternion.identity()

    attitude = cache.get("ATTITUDE")
    if attitude is not None:
        angular_velocity_body = frd_to_flu_vec(
            Vec3(attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed)
        )
    else:
        angular_velocity_body = Vec3(0.0, 0.0, 0.0)

    home = cache.get("HOME_POSITION")
    home_odom = ned_to_enu_vec(Vec3(home.x, home.y, home.z)) if home is not None else None

    gps_raw = cache.get("GPS_RAW_INT")
    if gps_raw is not None:
        gps = GpsStatus(
            fix_type=_GPS_FIX_TYPE_MAP.get(gps_raw.fix_type, GpsFixType.NO_GPS),
            satellites_visible=gps_raw.satellites_visible,
            eph_m=gps_raw.eph / 100.0 if gps_raw.eph != 65535 else float("nan"),
            epv_m=gps_raw.epv / 100.0 if gps_raw.epv != 65535 else float("nan"),
        )
    else:
        gps = GpsStatus(GpsFixType.NO_GPS, 0, float("nan"), float("nan"))

    sys_status = cache.get("SYS_STATUS")
    battery = (
        BatterySimState(
            remaining_fraction=(
                sys_status.battery_remaining / 100.0 if sys_status.battery_remaining >= 0 else 1.0
            ),
            voltage_v=sys_status.voltage_battery / 1000.0,
        )
        if sys_status is not None
        else BatterySimState(remaining_fraction=1.0, voltage_v=0.0)
    )

    est_status = cache.get("ESTIMATOR_STATUS")
    ekf_flags = (
        EkfFlags(
            raw_flags=est_status.flags,
            pos_horiz_accuracy_m=est_status.pos_horiz_accuracy,
            pos_vert_accuracy_m=est_status.pos_vert_accuracy,
        )
        if est_status is not None
        else EkfFlags(
            raw_flags=0, pos_horiz_accuracy_m=float("nan"), pos_vert_accuracy_m=float("nan")
        )
    )

    return VehicleState(
        t_sim_s=t_sim_s,
        armed=armed,
        flight_mode=flight_mode,
        landed_state=landed_state,
        pose_odom=pose_odom,
        orientation_odom=orientation_odom,
        velocity_odom_mps=velocity_odom,
        angular_velocity_body_radps=angular_velocity_body,
        home_odom=home_odom,
        gps=gps,
        battery_sim=battery,
        ekf_flags=ekf_flags,
        link=LinkStatus(connected=connected, last_heartbeat_age_s=last_heartbeat_age_s),
    )


def t_sim_s_from_local_position(cache: RawMessageCache, fallback: float = 0.0) -> float:
    """Best-effort sim time from the most position-adjacent timestamp available.

    Spec §15.4: "timestamps ... derived from PX4's MAVLink
    ``time_boot_ms``/``time_usec``, which in lockstep SITL tracks
    simulation time." Prefers ``LOCAL_POSITION_NED.time_boot_ms`` (ms);
    falls back to the caller-supplied value if no position has arrived yet.
    """
    local_pos = cache.get("LOCAL_POSITION_NED")
    if local_pos is not None:
        return float(local_pos.time_boot_ms) / 1000.0
    return fallback

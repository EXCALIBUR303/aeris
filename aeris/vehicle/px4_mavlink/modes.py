"""PX4's ``custom_mode`` bitfield <-> :class:`FlightMode` (spec §15).

Transcribed directly from ``src/modules/commander/px4_custom_mode.h`` in
the pinned checkout (Phase 4) — not guessed from MAVLink documentation.
That header defines::

    union px4_custom_mode {
        struct { uint16_t reserved; uint8_t main_mode; uint8_t sub_mode; };
        uint32_t data;
        ...
    };

so on the wire (little-endian, as MAVLink always is): ``main_mode`` is
bits 16-23 of ``custom_mode``, ``sub_mode`` is bits 24-31.
"""

from __future__ import annotations

from aeris.vehicle.interface import FlightMode

# PX4_CUSTOM_MAIN_MODE
_MAIN_MANUAL = 1
_MAIN_ALTCTL = 2
_MAIN_POSCTL = 3
_MAIN_AUTO = 4
_MAIN_ACRO = 5
_MAIN_OFFBOARD = 6
_MAIN_STABILIZED = 7

# PX4_CUSTOM_SUB_MODE_AUTO
_SUB_AUTO_READY = 1
_SUB_AUTO_TAKEOFF = 2
_SUB_AUTO_LOITER = 3
_SUB_AUTO_MISSION = 4
_SUB_AUTO_RTL = 5
_SUB_AUTO_LAND = 6
_SUB_AUTO_PRECLAND = 9

_AUTO_SUB_MODE_MAP: dict[int, FlightMode] = {
    _SUB_AUTO_TAKEOFF: FlightMode.TAKEOFF,
    _SUB_AUTO_LOITER: FlightMode.HOLD,
    _SUB_AUTO_MISSION: FlightMode.MISSION,
    _SUB_AUTO_RTL: FlightMode.RETURN_TO_LAUNCH,
    _SUB_AUTO_LAND: FlightMode.LAND,
    _SUB_AUTO_PRECLAND: FlightMode.LAND,
    _SUB_AUTO_READY: FlightMode.UNKNOWN,
}

_MAIN_MODE_MAP: dict[int, FlightMode] = {
    _MAIN_MANUAL: FlightMode.MANUAL,
    _MAIN_ALTCTL: FlightMode.ALTITUDE,
    _MAIN_POSCTL: FlightMode.POSITION,
    _MAIN_ACRO: FlightMode.ACRO,
    _MAIN_OFFBOARD: FlightMode.OFFBOARD,
    _MAIN_STABILIZED: FlightMode.STABILIZED,
}


def decode_px4_custom_mode(custom_mode: int) -> FlightMode:
    """Decode a MAVLink ``HEARTBEAT.custom_mode`` value into a :class:`FlightMode`."""
    custom_mode &= 0xFFFFFFFF
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF

    if main_mode == _MAIN_AUTO:
        return _AUTO_SUB_MODE_MAP.get(sub_mode, FlightMode.UNKNOWN)
    return _MAIN_MODE_MAP.get(main_mode, FlightMode.UNKNOWN)

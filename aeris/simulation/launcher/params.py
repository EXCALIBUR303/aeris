"""PX4 parameter profiles: a simple text format + MAVLink application.

Spec §51 Phase 3 asks for ``configs/vehicle/px4_params/sitl_base.params``
and "parameter profile application." AERIS's own format is intentionally
simple — one ``NAME VALUE`` pair per line, ``#`` comments — rather than
reverse-engineering QGC's exact ``.params`` export layout, which is
undocumented and not worth matching until a real need to interoperate with
QGC-exported files appears.

Applied over MAVLink (``PARAM_SET``) once PX4 is connected, rather than by
pre-seeding PX4's binary parameter store before boot — simpler, and it's
exactly the mechanism :mod:`aeris.vehicle` will use for the same purpose
from Phase 4 onward, so there's only one code path to trust.
"""

from __future__ import annotations

import struct
import time
from collections.abc import Mapping
from pathlib import Path

from pymavlink import mavutil

from aeris.core.errors import ConfigCompositionError, SimulationError
from aeris.core.logging import get_logger

_logger = get_logger(component="simulation.params")

# MAV_PARAM_TYPE values PX4 actually uses for its own params (verified
# against the pinned checkout in Phase 5 — PX4's own params are declared
# via PARAM_DEFINE_INT32/PARAM_DEFINE_FLOAT almost exclusively).
_MAV_PARAM_TYPE_UINT32 = 5
_MAV_PARAM_TYPE_INT32 = 6
_MAV_PARAM_TYPE_REAL32 = 9


def parse_params_file(path: Path | str) -> dict[str, float]:
    """Parse an AERIS ``.params`` file into ``{PARAM_NAME: value}``.

    Format: one ``NAME VALUE`` pair per line (whitespace-separated), blank
    lines ignored, ``#`` starts a comment to end of line.
    """
    p = Path(path)
    if not p.is_file():
        raise ConfigCompositionError(f"params file not found: {p}")

    params: dict[str, float] = {}
    for lineno, raw_line in enumerate(p.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 2:
            raise ConfigCompositionError(f"{p}:{lineno}: expected 'NAME VALUE', got: {raw_line!r}")
        name, raw_value = parts
        if len(name) > 16:
            raise ConfigCompositionError(
                f"{p}:{lineno}: PX4 parameter names are at most 16 chars, got {len(name)}: {name!r}"
            )
        try:
            value = float(raw_value)
        except ValueError as exc:
            raise ConfigCompositionError(
                f"{p}:{lineno}: value for {name!r} is not a number: {raw_value!r}"
            ) from exc
        params[name] = value
    return params


def _encode_param_value(value: float, param_type: int) -> float:
    """MAVLink's parameter protocol: an integer-typed param's wire
    ``param_value`` (a float32 field) carries the RAW BIT PATTERN of the
    integer, reinterpreted as float32 — not a numeric cast. (Confirmed
    live in Phase 5 by reading back a param and finding e.g. int value 1
    encoded as the float ``1.4e-45``, exactly ``struct.pack("<i", 1)``
    reinterpreted as ``<f``.)
    """
    if param_type == _MAV_PARAM_TYPE_INT32:
        return float(struct.unpack("<f", struct.pack("<i", round(value)))[0])
    if param_type == _MAV_PARAM_TYPE_UINT32:
        return float(struct.unpack("<f", struct.pack("<I", round(value)))[0])
    return value


def _decode_param_value(wire_value: float, param_type: int) -> float:
    """Inverse of :func:`_encode_param_value`."""
    if param_type == _MAV_PARAM_TYPE_INT32:
        return float(struct.unpack("<i", struct.pack("<f", wire_value))[0])
    if param_type == _MAV_PARAM_TYPE_UINT32:
        return float(struct.unpack("<I", struct.pack("<f", wire_value))[0])
    return wire_value


def _discover_param_type(
    conn: mavutil.mavlink_connection, name: str, *, timeout_s: float, poll_timeout_s: float = 2.0
) -> int:
    """Read a parameter's current value to learn its real ``MAV_PARAM_TYPE``.

    PX4 silently drops any ``PARAM_SET`` whose wire type doesn't exactly
    match the parameter's own declared type (verified in Phase 5 by
    reading ``mavlink_parameters.cpp``'s ``PARAM_SET`` handler — a type
    mismatch logs ``PX4_ERR("param types mismatch ...")`` and never sends
    a ``PARAM_VALUE`` ack). PX4's parameter subsystem also doesn't respond
    to *any* parameter-protocol message until
    ``Mavlink::boot_complete()`` fires — set explicitly near the end of
    the SITL startup script, but observed live to sometimes only trigger
    via its own ~20s sim-time fallback timer, well after the
    heartbeat/EKF readiness ``SimulationLauncher.start()`` normally
    already has by the time it calls :func:`apply_params`. Retrying this
    read across the full ``timeout_s`` (not just one ``poll_timeout_s``
    attempt) absorbs that.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        conn.mav.param_request_read_send(
            conn.target_system, conn.target_component, name.encode("utf-8"), -1
        )
        msg = conn.recv_match(
            type="PARAM_VALUE",
            blocking=True,
            timeout=poll_timeout_s,
            condition=f"PARAM_VALUE.param_id=='{name}'",
        )
        if msg is not None:
            return int(msg.param_type)
    raise SimulationError(
        f"no PARAM_VALUE response reading current {name!r} within {timeout_s}s "
        f"(unknown parameter, or PX4's parameter subsystem never became ready)"
    )


def apply_params(
    conn: mavutil.mavlink_connection,
    params: Mapping[str, float],
    *,
    timeout_s: float = 5.0,
    discovery_timeout_s: float = 30.0,
    tolerance: float = 1e-4,
) -> None:
    """Set each parameter over MAVLink and verify PX4's ``PARAM_VALUE`` ack.

    For each parameter, first reads its current value (learning its real
    ``MAV_PARAM_TYPE`` from the response — see :func:`_discover_param_type`
    for why this step exists and can take a while) before setting it with
    the matching wire encoding (see :func:`_encode_param_value`).

    Raises :class:`SimulationError` naming the first parameter that can't
    be found/read, times out on the set itself, or comes back with a
    different value than requested (which PX4 does for e.g. an
    out-of-range value it silently clamped).
    """
    for name, value in params.items():
        param_type = _discover_param_type(conn, name, timeout_s=discovery_timeout_s)

        wire_value = _encode_param_value(value, param_type)
        conn.mav.param_set_send(
            conn.target_system,
            conn.target_component,
            name.encode("utf-8"),
            wire_value,
            param_type,
        )
        ack = conn.recv_match(
            type="PARAM_VALUE",
            blocking=True,
            timeout=timeout_s,
            condition=f"PARAM_VALUE.param_id=='{name}'",
        )
        if ack is None:
            raise SimulationError(f"no PARAM_VALUE ack for {name!r} within {timeout_s}s")
        acked_value = _decode_param_value(ack.param_value, int(ack.param_type))
        if abs(acked_value - value) > tolerance:
            raise SimulationError(
                f"PX4 did not accept {name}={value} (reports {acked_value}); "
                f"likely out of range or read-only"
            )
        _logger.info("param.applied", name=name, value=value, param_type=param_type)

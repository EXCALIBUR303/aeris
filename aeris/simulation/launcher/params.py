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

from collections.abc import Mapping
from pathlib import Path

from pymavlink import mavutil

from aeris.core.errors import ConfigCompositionError, SimulationError
from aeris.core.logging import get_logger

_logger = get_logger(component="simulation.params")

# All PX4 params are sent over the wire as float32 regardless of their true
# underlying type (int32/float) — PX4's MAVLink parameter server casts on
# receipt. This matches how QGC and MAVSDK both do it.
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


def apply_params(
    conn: mavutil.mavlink_connection,
    params: Mapping[str, float],
    *,
    timeout_s: float = 5.0,
    tolerance: float = 1e-4,
) -> None:
    """Set each parameter over MAVLink and verify PX4's ``PARAM_VALUE`` ack.

    Raises :class:`SimulationError` naming the first parameter that either
    times out or comes back with a different value than requested (which
    PX4 does for e.g. an out-of-range value it silently clamped).
    """
    for name, value in params.items():
        conn.mav.param_set_send(
            conn.target_system,
            conn.target_component,
            name.encode("utf-8"),
            value,
            _MAV_PARAM_TYPE_REAL32,
        )
        ack = conn.recv_match(
            type="PARAM_VALUE",
            blocking=True,
            timeout=timeout_s,
            condition=f"PARAM_VALUE.param_id=='{name}'",
        )
        if ack is None:
            raise SimulationError(f"no PARAM_VALUE ack for {name!r} within {timeout_s}s")
        if abs(ack.param_value - value) > tolerance:
            raise SimulationError(
                f"PX4 did not accept {name}={value} (reports {ack.param_value}); "
                f"likely out of range or read-only"
            )
        _logger.info("param.applied", name=name, value=value)

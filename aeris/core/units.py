"""Unit conventions for AERIS.

Spec §14.2: "Units are SI internally: m, s, rad, m/s. Field names carry
units when ambiguous (``yaw_rad``, ``t_sim_s``)." This module holds the
handful of conversion constants and helpers needed to keep that convention
consistent, so unit math never gets hand-rolled inline.

Zero dependencies on the rest of AERIS, including other ``aeris.core``
modules (import-linter contract: "core.errors and core.types are
dependency-free within aeris.core").
"""

from __future__ import annotations

import math

# --- Angle -------------------------------------------------------------------

DEG_PER_RAD: float = 180.0 / math.pi
RAD_PER_DEG: float = math.pi / 180.0


def deg_to_rad(deg: float) -> float:
    """Convert degrees to radians (for display-layer input only — internals stay rad)."""
    return deg * RAD_PER_DEG


def rad_to_deg(rad: float) -> float:
    """Convert radians to degrees (for display-layer output only — internals stay rad)."""
    return rad * DEG_PER_RAD


def wrap_pi(angle_rad: float) -> float:
    """Wrap an angle in radians to ``(-pi, pi]``.

    Used throughout AERIS for yaw/heading arithmetic (spec §19.2: "Yaw ψ is
    measured from the frame's x-axis ... wrap(...)").
    """
    wrapped = (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    # (angle + pi) % (2*pi) is in [0, 2*pi); the above maps that to
    # [-pi, pi). Nudge -pi up to +pi so the range matches the docstring.
    if wrapped <= -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


# --- Length / speed ------------------------------------------------------------

M_PER_KM: float = 1000.0


# --- Time ----------------------------------------------------------------------

MS_PER_S: float = 1000.0
US_PER_S: float = 1_000_000.0


def us_to_s(t_us: int | float) -> float:
    """Convert microseconds (PX4/MAVLink ``time_usec`` convention) to seconds."""
    return t_us / US_PER_S


def ms_to_s(t_ms: int | float) -> float:
    """Convert milliseconds (MAVLink ``time_boot_ms`` convention) to seconds."""
    return t_ms / MS_PER_S

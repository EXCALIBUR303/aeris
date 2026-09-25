"""The AERIS command envelope: bounds + geofence for the S1 validator.

Spec §16.2 S1 examples: "bounds (‖v_xy‖ <= v_max, |v_z| <= vz_max, |psi_dot|
<= psi_dot_max); acceleration / rate-of-change limits; altitude
floor/ceiling; AERIS geofence (strictly inside PX4's)". "Strictly inside
PX4's" means this envelope's numbers must be smaller than whatever
``configs/vehicle/px4_params/safety_v1.params`` sets ``GF_MAX_HOR_DIST``/
``GF_MAX_VER_DIST`` to (PX4's own S0 geofence, a backstop) --
:func:`aeris.safety.envelope.assert_strictly_inside_px4_geofence` checks
this relationship explicitly rather than leaving it as an unverified
comment.
"""

from __future__ import annotations

from pathlib import Path

from pydantic import BaseModel, Field

from aeris.core.config import compose_config
from aeris.core.errors import ConfigCompositionError


class Envelope(BaseModel):
    """AERIS's own (S1) flight envelope -- a config, so Pydantic per spec §14.2."""

    model_config = {"frozen": True}

    v_xy_max_mps: float = Field(gt=0.0)
    vz_max_mps: float = Field(gt=0.0)
    yaw_rate_max_radps: float = Field(gt=0.0)
    accel_max_mps2: float = Field(gt=0.0)
    altitude_floor_m: float
    altitude_ceiling_m: float = Field(gt=0.0)
    geofence_radius_m: float = Field(gt=0.0)

    def model_post_init(self, _context: object) -> None:
        if self.altitude_ceiling_m <= self.altitude_floor_m:
            raise ValueError(
                f"altitude_ceiling_m ({self.altitude_ceiling_m}) must be > "
                f"altitude_floor_m ({self.altitude_floor_m})"
            )


def load_envelope(path: Path | str) -> Envelope:
    """Load and validate an :class:`Envelope` from a YAML file (``extends:`` composed)."""
    resolved = compose_config(path)
    try:
        return Envelope.model_validate(resolved)
    except Exception as exc:  # pydantic.ValidationError, but keep this typed
        raise ConfigCompositionError(f"invalid safety envelope at {path}: {exc}") from exc


def assert_strictly_inside_px4_geofence(
    envelope: Envelope, *, px4_gf_max_hor_dist_m: float, px4_gf_max_ver_dist_m: float
) -> None:
    """Raise :class:`ValueError` if ``envelope`` isn't strictly inside PX4's S0 fence.

    Spec §16.2: AERIS's own geofence must be "strictly inside PX4's" -- PX4's
    fence is meant as a backstop that should never actually trip if AERIS's
    own S1 validator is doing its job. Call this once at startup with the
    same ``GF_MAX_HOR_DIST``/``GF_MAX_VER_DIST`` values applied via
    ``configs/vehicle/px4_params/safety_v1.params``.
    """
    if envelope.geofence_radius_m >= px4_gf_max_hor_dist_m:
        raise ValueError(
            f"AERIS geofence_radius_m ({envelope.geofence_radius_m}) must be < "
            f"PX4 GF_MAX_HOR_DIST ({px4_gf_max_hor_dist_m}) -- it must trip first"
        )
    if envelope.altitude_ceiling_m >= px4_gf_max_ver_dist_m:
        raise ValueError(
            f"AERIS altitude_ceiling_m ({envelope.altitude_ceiling_m}) must be < "
            f"PX4 GF_MAX_VER_DIST ({px4_gf_max_ver_dist_m}) -- it must trip first"
        )

from __future__ import annotations

from pathlib import Path

import pydantic
import pytest

from aeris.safety.envelope import Envelope, assert_strictly_inside_px4_geofence, load_envelope

_REPO_SAFETY_YAML = Path(__file__).resolve().parents[3] / "configs" / "vehicle" / "safety.yaml"


def _envelope(**overrides: object) -> Envelope:
    defaults: dict[str, object] = {
        "v_xy_max_mps": 3.0,
        "vz_max_mps": 1.5,
        "yaw_rate_max_radps": 1.0,
        "accel_max_mps2": 2.0,
        "altitude_floor_m": 0.3,
        "altitude_ceiling_m": 20.0,
        "geofence_radius_m": 30.0,
    }
    defaults.update(overrides)
    return Envelope.model_validate(defaults)


def test_loads_the_real_committed_safety_yaml() -> None:
    envelope = load_envelope(_REPO_SAFETY_YAML)
    assert envelope.v_xy_max_mps > 0
    assert envelope.altitude_ceiling_m > envelope.altitude_floor_m


def test_rejects_ceiling_at_or_below_floor() -> None:
    with pytest.raises(pydantic.ValidationError):
        _envelope(altitude_floor_m=5.0, altitude_ceiling_m=5.0)


def test_rejects_non_positive_bounds() -> None:
    with pytest.raises(pydantic.ValidationError):
        _envelope(v_xy_max_mps=0.0)
    with pytest.raises(pydantic.ValidationError):
        _envelope(geofence_radius_m=-1.0)


def test_envelope_is_frozen() -> None:
    envelope = _envelope()
    with pytest.raises(pydantic.ValidationError):
        envelope.v_xy_max_mps = 99.0  # type: ignore[misc]


def test_strictly_inside_check_passes_for_the_real_committed_configs() -> None:
    envelope = load_envelope(_REPO_SAFETY_YAML)
    # Matches configs/vehicle/px4_params/safety_v1.params exactly.
    assert_strictly_inside_px4_geofence(
        envelope, px4_gf_max_hor_dist_m=60.0, px4_gf_max_ver_dist_m=40.0
    )


def test_strictly_inside_check_fails_when_aeris_fence_is_not_smaller() -> None:
    envelope = _envelope(geofence_radius_m=60.0)
    with pytest.raises(ValueError, match="geofence_radius_m"):
        assert_strictly_inside_px4_geofence(
            envelope, px4_gf_max_hor_dist_m=60.0, px4_gf_max_ver_dist_m=40.0
        )


def test_strictly_inside_check_fails_when_aeris_ceiling_is_not_smaller() -> None:
    envelope = _envelope(altitude_ceiling_m=40.0)
    with pytest.raises(ValueError, match="altitude_ceiling_m"):
        assert_strictly_inside_px4_geofence(
            envelope, px4_gf_max_hor_dist_m=60.0, px4_gf_max_ver_dist_m=40.0
        )

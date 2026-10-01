"""Unit tests for :class:`AltitudeHold` (the explicit altitude hold every
exploration-loop command now carries)."""

from __future__ import annotations

import math

import pytest

from aeris.autonomy.navigation.altitude import AltitudeHold
from aeris.core.frames.vector import Vec3


def test_below_target_commands_a_proportional_climb() -> None:
    hold = AltitudeHold(target_z_m=1.0, kp_per_s=1.0, max_vz_mps=0.5)
    assert hold.vz(0.8) == pytest.approx(0.2)


def test_above_target_commands_a_proportional_descent() -> None:
    hold = AltitudeHold(target_z_m=1.0, kp_per_s=2.0, max_vz_mps=0.5)
    assert hold.vz(1.1) == pytest.approx(-0.2)


def test_at_target_commands_zero() -> None:
    assert AltitudeHold(target_z_m=1.0).vz(1.0) == 0.0


def test_large_errors_are_clipped_both_ways() -> None:
    hold = AltitudeHold(target_z_m=1.0, kp_per_s=1.0, max_vz_mps=0.5)
    assert hold.vz(-0.01) == pytest.approx(0.5)  # on the floor: f2_office's failure
    assert hold.vz(10.0) == pytest.approx(-0.5)


@pytest.mark.parametrize("z_m", [-5.0, -0.01, 0.0, 0.3, 0.5, 0.99, 1.0, 1.01, 2.0, 50.0])
@pytest.mark.parametrize("kp", [0.1, 1.0, 10.0])
def test_output_is_bounded_and_always_points_toward_the_target(z_m: float, kp: float) -> None:
    hold = AltitudeHold(target_z_m=1.0, kp_per_s=kp, max_vz_mps=0.5)
    vz = hold.vz(z_m)
    assert abs(vz) <= 0.5
    assert math.copysign(1.0, vz) == math.copysign(1.0, 1.0 - z_m) or vz == 0.0


def test_apply_replaces_only_the_vertical_component() -> None:
    hold = AltitudeHold(target_z_m=1.0, kp_per_s=1.0, max_vz_mps=0.5)
    out = hold.apply(Vec3(0.7, -0.3, 0.9), z_m=1.2)
    assert out.x == 0.7
    assert out.y == -0.3
    assert out.z == pytest.approx(-0.2)


def test_apply_to_a_rotate_scan_command_still_climbs() -> None:
    """The rotate-in-place scan commands v=(0, 0, 0); Phase 12 sent that
    unchanged, so a vehicle below hover altitude was never told to climb."""
    hold = AltitudeHold(target_z_m=1.0, kp_per_s=1.0, max_vz_mps=0.5)
    out = hold.apply(Vec3(0.0, 0.0, 0.0), z_m=0.5)
    assert out == Vec3(0.0, 0.0, 0.5)


def test_apply_overrides_a_path_followers_own_vertical_component() -> None:
    hold = AltitudeHold(target_z_m=1.0)
    assert hold.apply(Vec3(1.0, 0.0, -0.8), z_m=1.0) == Vec3(1.0, 0.0, 0.0)


@pytest.mark.parametrize("bad_z", [math.nan, math.inf, -math.inf])
def test_non_finite_altitude_estimate_is_rejected(bad_z: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        AltitudeHold(target_z_m=1.0).vz(bad_z)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"target_z_m": math.nan},
        {"target_z_m": 1.0, "kp_per_s": 0.0},
        {"target_z_m": 1.0, "kp_per_s": -1.0},
        {"target_z_m": 1.0, "max_vz_mps": 0.0},
    ],
)
def test_invalid_configuration_is_rejected(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        AltitudeHold(**kwargs)


def test_default_max_vz_fits_inside_the_s1_envelope() -> None:
    from pathlib import Path

    from aeris.safety.envelope import load_envelope

    repo = Path(__file__).resolve().parents[4]
    env = load_envelope(str(repo / "configs" / "vehicle" / "safety.yaml"))
    assert AltitudeHold(target_z_m=1.0).max_vz_mps < env.vz_max_mps

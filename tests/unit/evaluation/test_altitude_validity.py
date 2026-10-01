"""Unit tests for :mod:`aeris.evaluation.metrics.altitude` -- the GT-altitude
episode validity check and the per-pose coverage filter."""

from __future__ import annotations

import pytest

from aeris.core.frames.vector import Vec3
from aeris.evaluation.metrics.altitude import (
    AltitudeValidity,
    check_altitude_band,
    in_band_poses,
)

_LO, _HI = 0.3, 3.0


def _traj(samples: list[tuple[float, float]]) -> list[tuple[float, Vec3, float]]:
    """``[(t, z), ...]`` -> GT poses at a fixed xy/yaw."""
    return [(t, Vec3(5.8, 0.8, z), 0.0) for t, z in samples]


def _hover(t0: float, t1: float, z: float, dt: float = 0.4) -> list[tuple[float, float]]:
    n = round((t1 - t0) / dt)
    return [(t0 + i * dt, z) for i in range(n)]


def test_an_episode_entirely_inside_the_band_is_valid() -> None:
    v = check_altitude_band(_traj(_hover(0.0, 180.0, 1.0)), z_lo_m=_LO, z_hi_m=_HI)
    assert v.valid
    assert v.excursions == ()
    assert v.n_poses_out_of_band == 0
    assert v.longest_excursion_s == 0.0
    assert v.describe() == "ok"


def test_a_short_dip_below_the_band_is_tolerated_but_recorded() -> None:
    samples = [(0.0, 1.0), (0.4, 0.25), (0.8, 0.28), (1.2, 0.6), (1.6, 1.0)]
    v = check_altitude_band(_traj(samples), z_lo_m=_LO, z_hi_m=_HI, max_excursion_s=2.0)
    assert v.valid
    assert len(v.excursions) == 1
    exc = v.excursions[0]
    # From the first out-of-band sample to the first in-band one after it.
    assert (exc.t_start_s, exc.t_end_s) == (0.4, 1.2)
    assert exc.duration_s == pytest.approx(0.8)
    assert (exc.min_z_m, exc.max_z_m) == (0.25, 0.28)
    assert v.n_poses_out_of_band == 2


def test_a_mid_episode_grounding_like_f1_rubble_is_invalid() -> None:
    """f1_rubble_10000: hovering at ~1 m, on the floor for ~10 s, then back."""
    samples = _hover(0.0, 60.0, 0.97) + _hover(60.4, 71.2, -0.01) + _hover(71.2, 180.0, 1.0)
    v = check_altitude_band(_traj(samples), z_lo_m=_LO, z_hi_m=_HI)
    assert not v.valid
    assert v.longest_excursion_s == pytest.approx(10.8)
    assert "left band [0.30, 3.00]" in v.describe()


def test_an_episode_that_never_leaves_the_floor_like_f2_office_is_invalid() -> None:
    samples = [(24.6, 0.55), (25.0, 0.28), *_hover(25.4, 201.4, -0.01)]
    v = check_altitude_band(_traj(samples), z_lo_m=_LO, z_hi_m=_HI)
    assert not v.valid
    # Still out of band at the last sample: the excursion runs to it.
    assert v.excursions[-1].t_end_s == pytest.approx(samples[-1][0])
    assert v.longest_excursion_s > 170.0


def test_climbing_above_the_band_counts_too() -> None:
    samples = _hover(0.0, 10.0, 1.0) + _hover(10.0, 15.0, 3.5) + _hover(15.0, 20.0, 1.0)
    v = check_altitude_band(_traj(samples), z_lo_m=_LO, z_hi_m=_HI)
    assert not v.valid
    assert v.excursions[0].max_z_m == 3.5


def test_band_edges_are_inside() -> None:
    v = check_altitude_band(_traj([(0.0, _LO), (10.0, _HI)]), z_lo_m=_LO, z_hi_m=_HI)
    assert v.valid
    assert v.n_poses_out_of_band == 0


def test_several_short_excursions_are_each_judged_on_their_own() -> None:
    samples = [(0.0, 1.0), (1.0, 0.1), (2.0, 1.0), (3.0, 0.1), (4.0, 1.0)]
    v = check_altitude_band(_traj(samples), z_lo_m=_LO, z_hi_m=_HI, max_excursion_s=1.5)
    assert v.valid
    assert len(v.excursions) == 2


def test_an_empty_trajectory_is_not_valid() -> None:
    v = check_altitude_band([], z_lo_m=_LO, z_hi_m=_HI)
    assert not v.valid
    assert v.describe() == "no GT poses recorded"


def test_an_inverted_band_is_rejected() -> None:
    with pytest.raises(ValueError):
        check_altitude_band([], z_lo_m=2.0, z_hi_m=1.0)


def test_in_band_poses_drops_grounded_poses_from_coverage() -> None:
    traj = _traj([(0.0, 1.0), (0.4, -0.01), (0.8, 0.29), (1.2, 0.3), (1.6, 3.01)])
    kept = in_band_poses(traj, z_lo_m=_LO, z_hi_m=_HI)
    assert [p[0] for p in kept] == [0.0, 1.2]


def test_to_record_is_json_ready_and_carries_the_verdict() -> None:
    import json

    samples = _hover(0.0, 10.0, 1.0) + _hover(10.0, 20.0, 0.0)
    v: AltitudeValidity = check_altitude_band(_traj(samples), z_lo_m=_LO, z_hi_m=_HI)
    rec = json.loads(json.dumps(v.to_record()))
    assert rec["altitude_valid"] is False
    assert rec["altitude_band_m"] == [_LO, _HI]
    assert rec["n_gt_poses_out_of_band"] == len(_hover(10.0, 20.0, 0.0))
    assert len(rec["altitude_excursions"]) == 1

"""Unit tests for :mod:`aeris.localization.px4_ekf`."""

from __future__ import annotations

import pytest
from tests.fixtures.vehicle_fakes import make_state

from aeris.core.frames.vector import Vec3
from aeris.core.types import Provenance
from aeris.localization.px4_ekf import Px4EkfPose


def test_pose_at_before_any_update_is_none() -> None:
    src = Px4EkfPose()
    assert src.pose_at(0.0) is None


def test_pose_at_after_one_update_returns_that_pose() -> None:
    src = Px4EkfPose()
    src.update(make_state(t_sim_s=1.0, pose_odom=Vec3(2.0, 3.0, 1.0)))

    estimate = src.pose_at(1.0)
    assert estimate is not None
    assert estimate.pose_m_b.translation == Vec3(2.0, 3.0, 1.0)
    assert estimate.provenance is Provenance.ESTIMATE
    assert estimate.source_name == "px4_ekf"


def test_pose_at_interpolates_between_two_updates() -> None:
    # 0.1s apart -- within PoseInterpolationBuffer's own max_gap_s (spec
    # §18.3: "gap > 100 ms" is dropped), matching a realistic ~10Hz tick.
    src = Px4EkfPose()
    src.update(make_state(t_sim_s=0.0, pose_odom=Vec3(0.0, 0.0, 0.0)))
    src.update(make_state(t_sim_s=0.1, pose_odom=Vec3(2.0, 0.0, 0.0)))

    estimate = src.pose_at(0.05)
    assert estimate is not None
    assert estimate.pose_m_b.translation == pytest.approx(Vec3(1.0, 0.0, 0.0))


def test_covariance_diag_derived_from_real_ekf_accuracy_fields() -> None:
    src = Px4EkfPose()
    src.update(make_state(t_sim_s=0.0))  # make_state defaults: horiz=0.2, vert=0.3
    estimate = src.pose_at(0.0)
    assert estimate is not None
    sx, sy, sz, sroll, spitch, syaw = estimate.covariance_diag
    assert sx == pytest.approx(0.2**2)
    assert sy == pytest.approx(0.2**2)
    assert sz == pytest.approx(0.3**2)
    assert (sroll, spitch, syaw) == (0.0, 0.0, 0.0)


def test_pose_at_a_time_outside_the_buffered_range_is_none() -> None:
    src = Px4EkfPose()
    src.update(make_state(t_sim_s=0.0))
    assert src.pose_at(100.0) is None

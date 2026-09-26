"""Unit tests for :mod:`aeris.evaluation.metrics.localization` -- hand-computed cases."""

from __future__ import annotations

import pytest

from aeris.core.frames.vector import Vec3
from aeris.evaluation.metrics.localization import ate, compute_t_w_o, rpe


def test_compute_t_w_o_is_the_mean_residual_translation() -> None:
    # Every sample's residual (gt - odom) is exactly (10, 5, 0) -- an
    # unambiguous check that the least-squares translation estimate is
    # the mean residual, independent of odom itself varying per sample.
    samples = [
        (Vec3(0.0, 0.0, 0.0), Vec3(10.0, 5.0, 0.0)),
        (Vec3(1.0, 0.0, 0.0), Vec3(11.0, 5.0, 0.0)),
        (Vec3(0.0, 1.0, 0.0), Vec3(10.0, 6.0, 0.0)),
    ]
    t_w_o = compute_t_w_o(samples)
    assert t_w_o.translation == pytest.approx(Vec3(10.0, 5.0, 0.0))


def test_compute_t_w_o_requires_at_least_one_sample() -> None:
    with pytest.raises(ValueError, match="at least one"):
        compute_t_w_o([])


def test_ate_zero_for_identical_trajectories() -> None:
    traj = [Vec3(float(i), 0.0, 0.0) for i in range(5)]
    assert ate(traj, traj) == 0.0


def test_ate_hand_computed_constant_offset() -> None:
    gt = [Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)]
    est = [Vec3(0.0, 0.0, 1.0), Vec3(1.0, 0.0, 1.0)]  # constant 1m z offset at every sample
    assert ate(est, gt) == pytest.approx(1.0)


def test_ate_hand_computed_mixed_errors() -> None:
    gt = [Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)]
    est = [Vec3(3.0, 0.0, 0.0), Vec3(0.0, 4.0, 0.0)]  # errors of 3 and 4 -> RMS = sqrt((9+16)/2)
    assert ate(est, gt) == pytest.approx((25.0 / 2) ** 0.5)


def test_ate_requires_equal_length_sequences() -> None:
    with pytest.raises(ValueError, match="same length"):
        ate([Vec3(0.0, 0.0, 0.0)], [Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)])


def test_ate_requires_at_least_one_sample() -> None:
    with pytest.raises(ValueError, match="at least one"):
        ate([], [])


def test_rpe_zero_for_identical_trajectories() -> None:
    gt = [Vec3(float(i), 0.0, 0.0) for i in range(15)]  # 14m of travel, one 10m segment
    assert rpe(gt, gt, delta_m=10.0) == 0.0


def test_rpe_hand_computed_single_segment_offset() -> None:
    gt = [Vec3(float(i), 0.0, 0.0) for i in range(11)]  # exactly 10m travelled
    est = list(gt)
    est[-1] = est[-1] + Vec3(0.0, 1.0, 0.0)  # the segment's endpoint estimate is 1m off in y
    assert rpe(est, gt, delta_m=10.0) == pytest.approx(1.0)


def test_rpe_raises_when_trajectory_shorter_than_one_segment() -> None:
    gt = [Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)]
    with pytest.raises(ValueError, match="less than one"):
        rpe(gt, gt, delta_m=10.0)


def test_rpe_requires_equal_length_sequences() -> None:
    with pytest.raises(ValueError, match="same length"):
        rpe([Vec3(0.0, 0.0, 0.0)], [Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)])

"""Unit tests for :mod:`aeris.localization.noisy`."""

from __future__ import annotations

import random
from dataclasses import dataclass

import pytest

from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import ZERO, Vec3
from aeris.core.types import Provenance
from aeris.localization.noisy import NoisyPoseSource
from aeris.localization.pose_source import PoseEstimate


@dataclass(slots=True)
class _FixedSource:
    """A trivial base PoseSource returning the same pose every call."""

    pose: Vec3 = ZERO
    calls: int = 0

    def pose_at(self, t_sim_s: float) -> PoseEstimate | None:
        self.calls += 1
        return PoseEstimate(
            pose_m_b=Transform.from_translation(self.pose),
            covariance_diag=(0.01, 0.01, 0.02, 0.0, 0.0, 0.0),
            provenance=Provenance.ESTIMATE,
            source_name="fixed",
        )


class _NoneSource:
    def pose_at(self, t_sim_s: float) -> PoseEstimate | None:
        return None


def test_none_base_estimate_propagates_as_none() -> None:
    src = NoisyPoseSource(base=_NoneSource(), rng=random.Random(1))
    assert src.pose_at(0.0) is None


def test_deterministic_given_the_same_seed() -> None:
    a = NoisyPoseSource(base=_FixedSource(), rng=random.Random(42))
    b = NoisyPoseSource(base=_FixedSource(), rng=random.Random(42))

    for t in (0.0, 0.1, 0.2, 0.3):
        ea, eb = a.pose_at(t), b.pose_at(t)
        assert ea is not None and eb is not None
        assert ea.pose_m_b.translation == eb.pose_m_b.translation


def test_different_seeds_diverge() -> None:
    a = NoisyPoseSource(base=_FixedSource(), rng=random.Random(1))
    b = NoisyPoseSource(base=_FixedSource(), rng=random.Random(2))
    ea, eb = a.pose_at(0.0), b.pose_at(0.0)
    assert ea is not None and eb is not None
    assert ea.pose_m_b.translation != eb.pose_m_b.translation


def test_covariance_is_inflated_relative_to_the_base_source() -> None:
    src = NoisyPoseSource(base=_FixedSource(), rng=random.Random(7), noise_std_m=0.05)
    estimate = src.pose_at(0.0)
    assert estimate is not None
    assert estimate.covariance_diag[0] == pytest.approx(0.01 + 0.05**2)
    assert estimate.covariance_diag[2] == pytest.approx(0.02 + 0.05**2)


def test_source_name_records_the_wrapped_base() -> None:
    src = NoisyPoseSource(base=_FixedSource(), rng=random.Random(0))
    estimate = src.pose_at(0.0)
    assert estimate is not None
    assert estimate.source_name == "noisy(fixed)"


def test_drift_accumulates_growing_the_offset_from_the_base_pose_over_many_calls() -> None:
    base_pos = Vec3(0.0, 0.0, 0.0)
    src = NoisyPoseSource(
        base=_FixedSource(pose=base_pos), rng=random.Random(123), walk_step_m=0.05, noise_std_m=0.0
    )
    offsets = []
    for t in range(200):
        estimate = src.pose_at(float(t))
        assert estimate is not None
        offsets.append((estimate.pose_m_b.translation - base_pos).norm())
    # A random walk's expected |displacement| grows with sqrt(n) -- late
    # offsets should typically exceed early ones by a wide margin over 200 steps.
    assert offsets[-1] > offsets[5]

"""Unit tests for :class:`PoseInterpolationBuffer` (spec §18.3, Phase 8)."""

from __future__ import annotations

import pytest

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import Vec3
from aeris.perception.pose_buffer import PoseInterpolationBuffer


def _pose(x: float) -> Transform:
    return Transform(Quaternion.identity(), Vec3(x, 0.0, 0.0))


def test_interpolates_midpoint_between_two_samples() -> None:
    buf = PoseInterpolationBuffer()
    buf.push(0.0, _pose(0.0))
    buf.push(0.05, _pose(10.0))

    result = buf.interpolate(0.025)

    assert result is not None
    assert result.translation.x == pytest.approx(5.0)
    assert buf.dropped_count == 0


def test_exact_sample_time_returns_that_pose_without_dropping() -> None:
    buf = PoseInterpolationBuffer()
    buf.push(0.0, _pose(0.0))
    buf.push(0.05, _pose(10.0))

    result = buf.interpolate(0.05)

    assert result is not None
    assert result.translation.x == pytest.approx(10.0)
    assert buf.dropped_count == 0


def test_gap_wider_than_max_gap_is_dropped_and_counted() -> None:
    buf = PoseInterpolationBuffer(max_gap_s=0.1)
    buf.push(0.0, _pose(0.0))
    buf.push(0.5, _pose(10.0))  # 500ms gap, way over the 100ms threshold

    result = buf.interpolate(0.25)

    assert result is None
    assert buf.dropped_count == 1


def test_request_time_beyond_buffered_range_is_dropped() -> None:
    buf = PoseInterpolationBuffer()
    buf.push(0.0, _pose(0.0))
    buf.push(0.05, _pose(10.0))

    before_range = buf.interpolate(-1.0)
    after_range = buf.interpolate(1.0)

    assert before_range is None
    assert after_range is None
    assert buf.dropped_count == 2


def test_empty_buffer_always_drops() -> None:
    buf = PoseInterpolationBuffer()
    assert buf.interpolate(0.0) is None
    assert buf.dropped_count == 1


def test_old_samples_are_evicted_past_history_window() -> None:
    buf = PoseInterpolationBuffer(history_s=1.0)
    buf.push(0.0, _pose(0.0))
    buf.push(0.5, _pose(5.0))
    buf.push(2.0, _pose(20.0))  # evicts t=0.0 (2.0 - 1.0 = 1.0 cutoff)

    # The t=0.0 sample should be gone; interpolating near it now fails.
    result = buf.interpolate(0.1)
    assert result is None


def test_rotation_interpolates_via_nlerp_shortest_path() -> None:
    buf = PoseInterpolationBuffer()
    q0 = Quaternion.identity()
    q1 = Quaternion.from_yaw(1.0)
    buf.push(0.0, Transform(q0, Vec3(0.0, 0.0, 0.0)))
    buf.push(0.05, Transform(q1, Vec3(0.0, 0.0, 0.0)))

    result = buf.interpolate(0.025)

    assert result is not None
    # Halfway rotation should be close to a yaw of ~0.5 rad (nlerp approximation).
    v = result.rotation.rotate(Vec3(1.0, 0.0, 0.0))
    import math

    angle = math.atan2(v.y, v.x)
    assert angle == pytest.approx(0.5, abs=0.05)

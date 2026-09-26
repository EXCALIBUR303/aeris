"""Localization error metrics (spec §41) and the evaluator alignment
``T_W_O`` computation (spec §19.3) they need as their preprocessing step.

Evaluator-only: every function here takes ground truth as an explicit
argument (never fetches it itself), so this module has nothing to import
from ``aeris.simulation.groundtruth`` and needing GT data is the caller's
concern, not this module's -- it's placed under ``aeris.evaluation``
rather than ``aeris.core.frames`` precisely because comparing against
ground truth is an evaluator concept, not a frame-math one.
"""

from __future__ import annotations

import math
import statistics
from collections.abc import Sequence

from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import Vec3


def compute_t_w_o(hover_samples: Sequence[tuple[Vec3, Vec3]]) -> Transform:
    """``T_W_O`` (spec §19.3): "computed once per episode from the GT spawn
    pose and PX4's reported EKF origin, then refined by least squares over
    the first N seconds of hover."

    ``hover_samples`` are paired ``(odom_position, gt_position)`` samples
    from that hover window. Rotation between ``O`` and ``W`` is assumed
    identity (ADR-0008: both are ENU; Phase 10 confirmed live that the two
    frames differ only in origin, never in orientation), so the
    least-squares estimate of the remaining unknown -- a pure translation
    -- is exactly the mean residual.
    """
    if not hover_samples:
        raise ValueError("compute_t_w_o requires at least one hover sample")
    residuals = [gt - odom for odom, gt in hover_samples]
    mean_residual = Vec3(
        statistics.mean(r.x for r in residuals),
        statistics.mean(r.y for r in residuals),
        statistics.mean(r.z for r in residuals),
    )
    return Transform.from_translation(mean_residual)


def ate(estimated: Sequence[Vec3], ground_truth: Sequence[Vec3]) -> float:
    """Absolute Trajectory Error (spec §41): ``sqrt(mean(||p_hat - p||^2))``.

    Both sequences must already be in the same frame -- apply
    :func:`compute_t_w_o` to the estimated trajectory (or its inverse to
    ground truth) before calling this, per spec §19.3's "``T_W_O`` is used
    only to compare agent estimates to GT."
    """
    if len(estimated) != len(ground_truth):
        raise ValueError(
            f"estimated ({len(estimated)}) and ground_truth ({len(ground_truth)}) "
            "must have the same length"
        )
    if not estimated:
        raise ValueError("ate requires at least one sample")
    squared_errors = [(e - g).norm() ** 2 for e, g in zip(estimated, ground_truth, strict=True)]
    return math.sqrt(statistics.mean(squared_errors))


def rpe(estimated: Sequence[Vec3], ground_truth: Sequence[Vec3], *, delta_m: float = 10.0) -> float:
    """Relative Pose Error over ``delta_m`` segments (spec §41: "RPE over
    Delta = 10 m segments"), translation-only -- a documented
    simplification (spec's own §41 line gives no explicit formula, unlike
    ATE's; the standard RPE definition also compares relative *rotation*,
    which this V1 dataset has no independent way to attribute error to
    beyond what ATE already captures at the position level).

    Segments are defined by *ground-truth distance travelled* (not a
    fixed sample count or time delta) reaching ``delta_m`` -- matching
    spec's literal "10 m segments" wording.
    """
    if len(estimated) != len(ground_truth):
        raise ValueError(
            f"estimated ({len(estimated)}) and ground_truth ({len(ground_truth)}) "
            "must have the same length"
        )
    n = len(estimated)
    errors: list[float] = []
    start = 0
    cumulative = 0.0
    for i in range(1, n):
        cumulative += (ground_truth[i] - ground_truth[i - 1]).norm()
        if cumulative >= delta_m:
            gt_disp = ground_truth[i] - ground_truth[start]
            est_disp = estimated[i] - estimated[start]
            errors.append((est_disp - gt_disp).norm())
            start = i
            cumulative = 0.0
    if not errors:
        raise ValueError(f"trajectory covers less than one {delta_m}m segment -- rpe is undefined")
    return math.sqrt(statistics.mean(e**2 for e in errors))

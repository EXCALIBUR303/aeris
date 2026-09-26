"""``NoisyPoseSource`` (spec §22): wraps a base :class:`PoseSource`, adding
deterministic random-walk drift + i.i.d. measurement noise -- "labeled as
synthetic degradation," the step before true GPS-denied navigation
(Phase 23/34), used here to measure mapping's sensitivity to localization
error (the map-accuracy experiment's third condition).

Drift accumulates *per call* to :meth:`pose_at`, not per unit of simulated
time (no ``sqrt(dt)`` Brownian scaling) -- a deliberate simplification
appropriate for a roughly fixed-rate control loop and explicitly "not
meant to be physically calibrated" degradation, not a claim about real
sensor drift statistics.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import ZERO, Vec3
from aeris.core.types import Provenance
from aeris.localization.pose_source import PoseEstimate, PoseSource


@dataclass(slots=True)
class NoisyPoseSource:
    base: PoseSource
    rng: random.Random
    walk_step_m: float = 0.01
    noise_std_m: float = 0.05
    _drift: Vec3 = field(default_factory=lambda: ZERO)

    def pose_at(self, t_sim_s: float) -> PoseEstimate | None:
        base_estimate = self.base.pose_at(t_sim_s)
        if base_estimate is None:
            return None

        self._drift = self._drift + Vec3(
            self.rng.gauss(0.0, self.walk_step_m),
            self.rng.gauss(0.0, self.walk_step_m),
            self.rng.gauss(0.0, self.walk_step_m),
        )
        noise = Vec3(
            self.rng.gauss(0.0, self.noise_std_m),
            self.rng.gauss(0.0, self.noise_std_m),
            self.rng.gauss(0.0, self.noise_std_m),
        )
        noisy_translation = base_estimate.pose_m_b.translation + self._drift + noise
        noisy_pose = Transform(base_estimate.pose_m_b.rotation, noisy_translation)

        added_variance = self.noise_std_m**2
        cov = base_estimate.covariance_diag
        inflated_cov = (
            cov[0] + added_variance,
            cov[1] + added_variance,
            cov[2] + added_variance,
            cov[3],
            cov[4],
            cov[5],
        )
        return PoseEstimate(
            pose_m_b=noisy_pose,
            covariance_diag=inflated_cov,
            provenance=Provenance.ESTIMATE,
            source_name=f"noisy({base_estimate.source_name})",
        )

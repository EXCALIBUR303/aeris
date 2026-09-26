"""``PoseSource`` protocol (spec §22): ``pose_at(t_sim) -> PoseEstimate``.

Kept synchronous, matching :class:`aeris.perception.pose_buffer.PoseInterpolationBuffer`'s
own push/interpolate split -- a source never does I/O itself; the caller's
async control loop calls a source's own ``update()`` (not part of this
protocol; each concrete source defines its own) each tick with the latest
live data, then calls the synchronous ``pose_at()`` to read it back,
possibly interpolated to an exact timestamp.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from aeris.core.frames.transform import Transform
from aeris.core.types import Provenance


@dataclass(frozen=True, slots=True)
class PoseEstimate:
    """``T_M_B`` plus a covariance and where it came from (spec §22).

    ``covariance_diag`` is a simplified diagonal 6-vector (position x/y/z
    variance in m^2, then orientation roll/pitch/yaw variance in rad^2) --
    a full 6x6 covariance matrix isn't meaningfully derivable from what
    PX4's own telemetry exposes at this level (documented simplification,
    matching e.g. :class:`aeris.evaluation.metrics.collision.BoxObstacle`'s
    own AABB simplification from Phase 9).
    """

    pose_m_b: Transform
    covariance_diag: tuple[float, float, float, float, float, float]
    provenance: Provenance
    source_name: str


@runtime_checkable
class PoseSource(Protocol):
    def pose_at(self, t_sim_s: float) -> PoseEstimate | None:
        """The pose at ``t_sim_s``, or ``None`` if it can't be resolved
        (no data yet, or the requested time can't be bracketed)."""
        ...

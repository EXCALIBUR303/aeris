"""``Px4EkfPose`` -- L-EST-GPS (spec §22): PX4 EKF2's own pose estimate,
agent-legitimate and the default V1 :class:`~aeris.localization.pose_source.PoseSource`.

Reuses :class:`aeris.perception.pose_buffer.PoseInterpolationBuffer` (built
in Phase 8, not wired to a real consumer until now) to answer "the pose at
exactly this timestamp" rather than "whatever the latest snapshot happens
to be" -- the caller's control loop feeds it a new :class:`VehicleState`
each tick via :meth:`Px4EkfPose.update`, matching the buffer's own
push/interpolate split.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from aeris.core.frames.transform import Transform
from aeris.core.types import Provenance
from aeris.localization.pose_source import PoseEstimate
from aeris.perception.pose_buffer import PoseInterpolationBuffer
from aeris.vehicle.interface import EkfFlags, VehicleState


@dataclass(slots=True)
class Px4EkfPose:
    """``max_gap_s`` (spec §18.3's own default: 100ms, sized for a
    ~10-30Hz sensor/telemetry pipeline) may need widening by a caller
    whose own control loop runs slower than that -- e.g. Phase 11's
    mapping episode, whose per-tick GT-pose subprocess poll plus a real
    map integration measured a real ~500-700ms effective period. Too
    tight a gap there made `pose_at()` silently return `None` for a
    timestamp just one tick old, which isn't a "no data" situation, just
    a slow loop; too loose a gap would let :meth:`PoseInterpolationBuffer.interpolate`
    average across large real motion instead. Left at the buffer's own
    default here; callers with a slower loop pass a wider value
    explicitly (see ``run_mapping_episode``)."""

    max_gap_s: float = 0.1
    _buffer: PoseInterpolationBuffer = field(init=False)
    _latest_ekf_flags: EkfFlags | None = None

    def __post_init__(self) -> None:
        self._buffer = PoseInterpolationBuffer(max_gap_s=self.max_gap_s)

    def update(self, state: VehicleState) -> None:
        self._buffer.push(state.t_sim_s, Transform(state.orientation_odom, state.pose_odom))
        self._latest_ekf_flags = state.ekf_flags

    def pose_at(self, t_sim_s: float) -> PoseEstimate | None:
        pose = self._buffer.interpolate(t_sim_s)
        if pose is None or self._latest_ekf_flags is None:
            return None
        # Orientation variance isn't exposed by PX4's telemetry at this
        # level (only ESTIMATOR_STATUS's horizontal/vertical position
        # accuracy is) -- left at 0.0, a documented simplification (see
        # PoseEstimate's own docstring).
        sigma_h = self._latest_ekf_flags.pos_horiz_accuracy_m
        sigma_v = self._latest_ekf_flags.pos_vert_accuracy_m
        return PoseEstimate(
            pose_m_b=pose,
            covariance_diag=(sigma_h**2, sigma_h**2, sigma_v**2, 0.0, 0.0, 0.0),
            provenance=Provenance.ESTIMATE,
            source_name="px4_ekf",
        )

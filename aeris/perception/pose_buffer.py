"""Vehicle-pose interpolation buffer for time-aligning sensor frames (spec §18.3).

Each sensor frame's ``t_sim_s`` needs the vehicle pose *interpolated at that
exact timestamp* from a rolling telemetry history (>= 2 s), since sensor
capture and pose sampling aren't synchronized. Frames whose pose can't be
interpolated (gap > 100 ms between the straddling samples, or ``t_sim_s``
outside the buffered range entirely) are dropped and counted (spec §18.3) —
this buffer never extrapolates.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.transform import Transform

_DEFAULT_MAX_GAP_S = 0.1  # spec §18.3: "gap > 100 ms"


@dataclass(frozen=True, slots=True)
class PoseSample:
    t_sim_s: float
    pose: Transform  # T_M_B (or T_O_B; V1 has M == O, spec §19.1) at this instant


def _nlerp(q0: Quaternion, q1: Quaternion, alpha: float) -> Quaternion:
    """Normalized linear interpolation, shortest-path (negates q1 if needed).

    Not SLERP: at the 20-50 Hz telemetry rates spec §18.1 targets, the
    per-sample orientation delta is small enough that NLERP's error is
    negligible, and it stays consistent with this dependency-free
    ``Quaternion`` (no trig-heavy slerp implementation needed).
    """
    dot = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z
    if dot < 0.0:
        q1 = Quaternion(-q1.w, -q1.x, -q1.y, -q1.z)
    return Quaternion(
        q0.w + (q1.w - q0.w) * alpha,
        q0.x + (q1.x - q0.x) * alpha,
        q0.y + (q1.y - q0.y) * alpha,
        q0.z + (q1.z - q0.z) * alpha,
    ).normalized()


class PoseInterpolationBuffer:
    """A rolling window of ``(t_sim_s, pose)`` samples supporting linear interpolation.

    Keeps up to ``history_s`` seconds of samples (spec: "≥ 2 s"), evicting
    older ones as new samples arrive via :meth:`push`.
    """

    def __init__(self, *, history_s: float = 2.0, max_gap_s: float = _DEFAULT_MAX_GAP_S) -> None:
        self._history_s = history_s
        self._max_gap_s = max_gap_s
        self._samples: deque[PoseSample] = deque()
        self.dropped_count = 0

    def push(self, t_sim_s: float, pose: Transform) -> None:
        self._samples.append(PoseSample(t_sim_s, pose))
        cutoff = t_sim_s - self._history_s
        while self._samples and self._samples[0].t_sim_s < cutoff:
            self._samples.popleft()

    def interpolate(self, t_sim_s: float) -> Transform | None:
        """Return the pose at ``t_sim_s``, or ``None`` if it can't be bracketed.

        Increments :attr:`dropped_count` on every ``None`` return, so
        callers (spec §18.3: "dropped and counted") don't need their own
        bookkeeping.
        """
        before: PoseSample | None = None
        after: PoseSample | None = None
        for sample in self._samples:
            if sample.t_sim_s <= t_sim_s:
                before = sample
            else:
                after = sample
                break

        if before is not None and before.t_sim_s == t_sim_s:
            return before.pose
        if before is None or after is None:
            self.dropped_count += 1
            return None
        if (after.t_sim_s - before.t_sim_s) > self._max_gap_s:
            self.dropped_count += 1
            return None

        alpha = (t_sim_s - before.t_sim_s) / (after.t_sim_s - before.t_sim_s)
        translation = before.pose.translation + (
            after.pose.translation - before.pose.translation
        ).scale(alpha)
        rotation = _nlerp(before.pose.rotation, after.pose.rotation, alpha)
        return Transform(rotation, translation)

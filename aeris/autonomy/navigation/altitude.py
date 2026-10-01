"""``AltitudeHold`` -- explicit vertical-velocity regulation toward a fixed
hover altitude (Phase 13 follow-up to Phase 12's exploration loop).

Phase 12's exploration loop corrected altitude only *implicitly*: every
A* waypoint sits at the hover altitude, so :class:`~aeris.autonomy.navigation.follower.PathFollower`
pointing at one carried a vertical component. Anything that wasn't
path following -- rotate-in-place scans command ``v = (0, 0, 0)``, and
an episode whose every A* attempt failed did nothing *but* rotate --
sent ``vz = 0`` regardless of altitude, so a vehicle handed over below
the hover altitude (or sinking for any other reason) was never told to
climb back. Confirmed live: ``f2_office_10000`` drifted from the 0.5 m
takeoff handoff to the floor and spun there for the whole episode
(``results/fastsim/tierh_trace/f2_office_10000_nearest_frontier/INVALID.md``).

This replaces the command's vertical component outright -- on every
command, whatever produced its horizontal part -- with a clipped
proportional term on the altitude error, so altitude regulation no
longer depends on which branch of the control loop happens to be active.

The ``z_m`` it is fed must be the agent's own estimate (PX4's EKF), never
simulator ground truth (spec §17.4); that also means it cannot correct
an EKF that is itself wrong about altitude -- see ``docs/exploration.md``
for the live case where that happened and the GT-side validity check
that catches it instead.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from aeris.core.frames.vector import Vec3


@dataclass(frozen=True, slots=True)
class AltitudeHold:
    target_z_m: float
    kp_per_s: float = 1.0
    # Well inside the S1 envelope's vz_max_mps (1.5, configs/vehicle/safety.yaml),
    # and a step to it from rest stays under its 2.0 m/s^2 accel_max at the
    # loop's real (>=0.1 s) tick period.
    max_vz_mps: float = 0.5

    def __post_init__(self) -> None:
        if not math.isfinite(self.target_z_m):
            raise ValueError(f"target_z_m must be finite, got {self.target_z_m}")
        if not self.kp_per_s > 0.0:
            raise ValueError(f"kp_per_s must be > 0, got {self.kp_per_s}")
        if not self.max_vz_mps > 0.0:
            raise ValueError(f"max_vz_mps must be > 0, got {self.max_vz_mps}")

    def vz(self, z_m: float) -> float:
        """``clip(kp * (target - z), -max_vz, +max_vz)``, ENU (positive = climb)."""
        if not math.isfinite(z_m):
            raise ValueError(f"altitude estimate must be finite, got {z_m}")
        raw = self.kp_per_s * (self.target_z_m - z_m)
        return max(-self.max_vz_mps, min(self.max_vz_mps, raw))

    def apply(self, v: Vec3, z_m: float) -> Vec3:
        """``v`` with its vertical component replaced by :meth:`vz`; the
        horizontal components pass through unchanged."""
        return Vec3(v.x, v.y, self.vz(z_m))

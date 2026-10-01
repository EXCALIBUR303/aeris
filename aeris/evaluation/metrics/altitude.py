"""Episode validity from the GT altitude trace (Phase 13 follow-up to Phase 12).

Coverage ``C(t)`` (:mod:`aeris.evaluation.metrics.coverage`) raycasts from
each GT pose's *horizontal* position only, within the world's altitude
band -- it never looks at the pose's own z. So a vehicle sitting on the
floor and spinning in place still "covers" whatever is visible from its
xy, and Phase 12's harness reported that as exploration: a Phase 13
re-run of ``f2_office_10000`` scored ``C(180) = 0.985`` from a vehicle
that never left the ground
(``results/fastsim/tierh_trace/f2_office_10000_nearest_frontier/INVALID.md``).

:func:`check_altitude_band` turns the recorded GT z into an explicit
validity verdict: an episode whose GT altitude leaves the world's
altitude band for longer than ``max_excursion_s`` in one stretch is
invalid. :func:`in_band_poses` is the per-pose half of the same rule --
the evaluator drops out-of-band poses before computing coverage, so even
a tolerated short excursion contributes nothing to ``C(t)``.

Excursion durations are measured between GT samples, which the
exploration loop records at its own (slow, ~2-3 Hz) tick rate: an
excursion runs from its first out-of-band sample to the first in-band
sample after it, an upper bound on the real time spent outside the band.
An excursion still open at the end of the trajectory runs to the last
sample.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

from aeris.core.frames.vector import Vec3

GtPose = tuple[float, Vec3, float]  # (t_sim_s, position_world, yaw_rad)

DEFAULT_MAX_EXCURSION_S = 2.0


@dataclass(frozen=True, slots=True)
class AltitudeExcursion:
    t_start_s: float
    t_end_s: float
    min_z_m: float
    max_z_m: float

    @property
    def duration_s(self) -> float:
        return self.t_end_s - self.t_start_s


@dataclass(frozen=True, slots=True)
class AltitudeValidity:
    z_lo_m: float
    z_hi_m: float
    max_excursion_s: float
    n_poses: int
    n_poses_out_of_band: int
    excursions: tuple[AltitudeExcursion, ...]

    @property
    def longest_excursion_s(self) -> float:
        return max((e.duration_s for e in self.excursions), default=0.0)

    @property
    def valid(self) -> bool:
        """``False`` for an empty trajectory too -- no GT poses means no
        evidence the vehicle was ever in the band."""
        return self.n_poses > 0 and self.longest_excursion_s <= self.max_excursion_s

    def describe(self) -> str:
        if self.n_poses == 0:
            return "no GT poses recorded"
        if self.valid:
            return "ok"
        worst = max(self.excursions, key=lambda e: e.duration_s)
        return (
            f"GT altitude left band [{self.z_lo_m:.2f}, {self.z_hi_m:.2f}] m for "
            f"{worst.duration_s:.1f} s (> {self.max_excursion_s:.1f} s) starting at "
            f"t={worst.t_start_s:.1f} s, z range {worst.min_z_m:.2f}..{worst.max_z_m:.2f} m"
        )

    def to_record(self) -> dict[str, Any]:
        return {
            "altitude_valid": self.valid,
            "altitude_band_m": [self.z_lo_m, self.z_hi_m],
            "altitude_max_excursion_s": self.max_excursion_s,
            "altitude_longest_excursion_s": round(self.longest_excursion_s, 3),
            "n_gt_poses_out_of_band": self.n_poses_out_of_band,
            "altitude_excursions": [
                [
                    round(e.t_start_s, 3),
                    round(e.t_end_s, 3),
                    round(e.min_z_m, 3),
                    round(e.max_z_m, 3),
                ]
                for e in self.excursions
            ],
        }


def _in_band(z_m: float, z_lo_m: float, z_hi_m: float) -> bool:
    return z_lo_m <= z_m <= z_hi_m


def check_altitude_band(
    gt_poses: Sequence[GtPose],
    *,
    z_lo_m: float,
    z_hi_m: float,
    max_excursion_s: float = DEFAULT_MAX_EXCURSION_S,
) -> AltitudeValidity:
    if z_hi_m <= z_lo_m:
        raise ValueError(f"z_hi_m ({z_hi_m}) must be > z_lo_m ({z_lo_m})")
    excursions: list[AltitudeExcursion] = []
    n_out = 0
    open_start: float | None = None
    open_zs: list[float] = []
    for t_s, position, _yaw in gt_poses:
        if _in_band(position.z, z_lo_m, z_hi_m):
            if open_start is not None:
                excursions.append(AltitudeExcursion(open_start, t_s, min(open_zs), max(open_zs)))
                open_start, open_zs = None, []
            continue
        n_out += 1
        if open_start is None:
            open_start = t_s
        open_zs.append(position.z)
    if open_start is not None:
        excursions.append(
            AltitudeExcursion(open_start, gt_poses[-1][0], min(open_zs), max(open_zs))
        )
    return AltitudeValidity(
        z_lo_m=z_lo_m,
        z_hi_m=z_hi_m,
        max_excursion_s=max_excursion_s,
        n_poses=len(gt_poses),
        n_poses_out_of_band=n_out,
        excursions=tuple(excursions),
    )


def in_band_poses(gt_poses: Sequence[GtPose], *, z_lo_m: float, z_hi_m: float) -> list[GtPose]:
    """The subset of ``gt_poses`` whose GT z lies inside ``[z_lo_m, z_hi_m]``."""
    return [p for p in gt_poses if _in_band(p[1].z, z_lo_m, z_hi_m)]

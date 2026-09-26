"""2D LiDAR scan -> point cloud back-projection (spec §18.2, §20).

Points are returned in the LiDAR's own sensor frame (``S_lidar``,
FLU-aligned per spec §19.1) — the caller composes with the sensor's
extrinsics then the vehicle pose to reach ``M``, exactly as with depth
(see :mod:`aeris.perception.depth.projection`).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from aeris.core.frames.vector import Vec3


@dataclass(frozen=True, slots=True)
class ScanGeometry:
    """A 2D LiDAR scan's angular/range geometry (gz ``LaserScan`` fields)."""

    angle_min_rad: float
    angle_step_rad: float
    range_min_m: float
    range_max_m: float


def scan_to_points(ranges: Sequence[float], geometry: ScanGeometry) -> list[Vec3]:
    """Back-project a flat 2D LiDAR range array into sensor-frame points.

    Ranges at or beyond ``range_max_m`` (gz reports ``range_max`` itself for
    a no-hit ray) or at/below ``range_min_m`` are dropped as invalid/no-return.
    """
    points: list[Vec3] = []
    for i, r in enumerate(ranges):
        if math.isnan(r) or math.isinf(r):
            continue
        if not (geometry.range_min_m < r < geometry.range_max_m):
            continue
        angle = geometry.angle_min_rad + i * geometry.angle_step_rad
        points.append(Vec3(r * math.cos(angle), r * math.sin(angle), 0.0))
    return points

"""Depth image -> point cloud back-projection (spec §18.2, §20; pinhole model).

Points are returned in the camera *optical* frame (REP-103: x right, y
down, z forward per spec §19.1's ``C_opt``) — the caller composes with
``T_B_C`` (extrinsics + the fixed optical<->body rotation, spec §19.3) to
reach the body frame, then with the vehicle's pose to reach ``M``.

Kept numpy-free: spec's dependency-minimal principle scopes ``numpy`` to
the ``learn`` extra, added in Phase 13 (see ``pyproject.toml``) — depth
buffers arrive from the Sensor Bridge as flat ``float32`` byte payloads and
are handled here via the stdlib ``array`` module.
"""

from __future__ import annotations

import math
from array import array
from collections.abc import Sequence
from dataclasses import dataclass

from aeris.core.frames.vector import Vec3


@dataclass(frozen=True, slots=True)
class CameraIntrinsics:
    """A pinhole camera's intrinsic parameters."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


def camera_intrinsics_from_k(width: int, height: int, k: Sequence[float]) -> CameraIntrinsics:
    """Build from a row-major 3x3 intrinsics matrix (gz ``CameraInfo.intrinsics``)."""
    if len(k) != 9:
        raise ValueError(f"expected a 3x3 (9-element) intrinsics matrix, got {len(k)} elements")
    return CameraIntrinsics(width=width, height=height, fx=k[0], fy=k[4], cx=k[2], cy=k[5])


def depth_to_points_camera(
    depth: array[float],
    intrinsics: CameraIntrinsics,
    *,
    min_depth_m: float = 0.05,
    max_depth_m: float = 100.0,
) -> list[Vec3]:
    """Back-project a flat, row-major depth buffer into camera-optical-frame points.

    ``depth[row * width + col]`` is the perpendicular (z-depth, not slant
    range) distance at that pixel, matching gz's ``depth_camera`` sensor
    convention. NaN/inf samples and samples outside
    ``[min_depth_m, max_depth_m]`` (no-return / clipped) are dropped.
    """
    width, height = intrinsics.width, intrinsics.height
    if len(depth) != width * height:
        raise ValueError(f"depth buffer has {len(depth)} samples, expected {width * height}")

    points: list[Vec3] = []
    for row in range(height):
        row_offset = row * width
        for col in range(width):
            z = depth[row_offset + col]
            if math.isnan(z) or math.isinf(z):
                continue
            if not (min_depth_m <= z <= max_depth_m):
                continue
            x = (col - intrinsics.cx) / intrinsics.fx * z
            y = (row - intrinsics.cy) / intrinsics.fy * z
            points.append(Vec3(x, y, z))
    return points

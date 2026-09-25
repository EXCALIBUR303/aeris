"""A minimal, dependency-free 3D vector.

``aeris.core`` stays free of numpy (spec §14.1: numpy lives in the
``learn`` extra, added in Phase 13) — pose/frame math at telemetry rates
(tens of Hz) has no need for it, and keeping ``aeris.core.frames``
dependency-free keeps it usable from any part of AERIS without dragging in
the learning stack.
"""

from __future__ import annotations

import math
from typing import NamedTuple


class Vec3(NamedTuple):
    """A 3D vector. Frame-agnostic — callers track which frame it's in."""

    x: float
    y: float
    z: float

    def __add__(self, other: Vec3) -> Vec3:  # type: ignore[override]
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vec3) -> Vec3:
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __neg__(self) -> Vec3:
        return Vec3(-self.x, -self.y, -self.z)

    def scale(self, s: float) -> Vec3:
        return Vec3(self.x * s, self.y * s, self.z * s)

    def dot(self, other: Vec3) -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: Vec3) -> Vec3:
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def norm(self) -> float:
        return math.sqrt(self.dot(self))

    def normalized(self) -> Vec3:
        n = self.norm()
        if n < 1e-12:
            raise ValueError("cannot normalize a near-zero-length vector")
        return self.scale(1.0 / n)


ZERO = Vec3(0.0, 0.0, 0.0)

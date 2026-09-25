"""A minimal, dependency-free unit quaternion for rotations.

Hamilton convention, ``(w, x, y, z)``, active rotation: ``q.rotate(v)``
rotates the vector ``v`` the same way ``q`` rotates a frame — i.e. if ``q``
represents the orientation of frame ``B`` in frame ``A`` (``q_A_B``), then
``q_A_B.rotate(v_B) == v_A``. This matches the convention used throughout
``aeris.core.frames`` and spec §19.2: "Rotations are stored as unit
quaternions ``(w, x, y, z)`` in code, normalized after every composition,
with a Hamilton product."
"""

from __future__ import annotations

import math
from typing import NamedTuple

from aeris.core.frames.vector import Vec3


class Quaternion(NamedTuple):
    w: float
    x: float
    y: float
    z: float

    def norm(self) -> float:
        return math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def normalized(self) -> Quaternion:
        n = self.norm()
        if n < 1e-12:
            raise ValueError("cannot normalize a near-zero-norm quaternion")
        return Quaternion(self.w / n, self.x / n, self.y / n, self.z / n)

    def conjugate(self) -> Quaternion:
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def __mul__(self, other: Quaternion) -> Quaternion:  # type: ignore[override]
        """Hamilton product ``self * other`` (composition: apply ``other`` then ``self``)."""
        w1, x1, y1, z1 = self
        w2, x2, y2, z2 = other
        return Quaternion(
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    def compose(self, other: Quaternion) -> Quaternion:
        """``q_A_C = q_A_B.compose(q_B_C)`` — spec §19.2's composition rule.

        Equivalent to the Hamilton product ``self * other``, renormalized
        (spec: "normalized after every composition") to guard against
        floating-point drift accumulating across many compositions.
        """
        return (self * other).normalized()

    def rotate(self, v: Vec3) -> Vec3:
        """Rotate vector ``v`` by this quaternion (active rotation)."""
        qv = Quaternion(0.0, v.x, v.y, v.z)
        result = self * qv * self.conjugate()
        return Vec3(result.x, result.y, result.z)

    def inverse(self) -> Quaternion:
        """For a unit quaternion, the inverse is the conjugate."""
        return self.conjugate()

    @staticmethod
    def identity() -> Quaternion:
        return Quaternion(1.0, 0.0, 0.0, 0.0)

    @staticmethod
    def from_axis_angle(axis: Vec3, angle_rad: float) -> Quaternion:
        a = axis.normalized()
        half = angle_rad / 2.0
        s = math.sin(half)
        return Quaternion(math.cos(half), a.x * s, a.y * s, a.z * s).normalized()

    @staticmethod
    def from_matrix_columns(col_x: Vec3, col_y: Vec3, col_z: Vec3) -> Quaternion:
        """Build a quaternion from a rotation matrix given as its three columns.

        ``col_x``/``col_y``/``col_z`` are where the source frame's unit
        basis vectors ``e_x``/``e_y``/``e_z`` land after rotation — i.e.
        the columns of the rotation matrix ``R`` such that ``R @ e_x ==
        col_x``. Uses the standard robust (branch-on-largest-diagonal-term)
        matrix-to-quaternion conversion, so it works for any proper
        rotation, not just well-conditioned ones near the identity.
        """
        m00, m10, m20 = col_x
        m01, m11, m21 = col_y
        m02, m12, m22 = col_z
        trace = m00 + m11 + m22

        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (m21 - m12) / s
            y = (m02 - m20) / s
            z = (m10 - m01) / s
        elif m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            w = (m21 - m12) / s
            x = 0.25 * s
            y = (m01 + m10) / s
            z = (m02 + m20) / s
        elif m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            w = (m02 - m20) / s
            x = (m01 + m10) / s
            y = 0.25 * s
            z = (m12 + m21) / s
        else:
            s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            w = (m10 - m01) / s
            x = (m02 + m20) / s
            y = (m12 + m21) / s
            z = 0.25 * s

        return Quaternion(w, x, y, z).normalized()

    @staticmethod
    def from_yaw(yaw_rad: float) -> Quaternion:
        """A pure yaw (rotation about +Z) quaternion — the common case for a
        planar heading with zero roll/pitch."""
        return Quaternion.from_axis_angle(Vec3(0.0, 0.0, 1.0), yaw_rad)

    def is_close(self, other: Quaternion, atol: float = 1e-9) -> bool:
        """True if ``self`` and ``other`` represent the same rotation.

        A unit quaternion and its negation represent the identical
        rotation (``q`` and ``-q`` are the same orientation) — compares
        both signs rather than raw component equality.
        """
        same = all(abs(a - b) <= atol for a, b in zip(self, other, strict=True))
        opposite = all(abs(a + b) <= atol for a, b in zip(self, other, strict=True))
        return same or opposite

"""A rigid-body transform (element of SE(3)): rotation + translation.

Spec §19.2 notation: ``T_A_B`` maps coordinates expressed in frame ``B``
into frame ``A``: ``p_A = T_A_B · p_B``. Composition: ``T_A_C = T_A_B · T_B_C``.
"""

from __future__ import annotations

from typing import NamedTuple

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import ZERO, Vec3


class Transform(NamedTuple):
    """``T_A_B``: the pose of frame ``B`` expressed in frame ``A``.

    ``rotation`` is ``q_A_B`` (rotates a vector from ``B``'s axes to
    ``A``'s), ``translation`` is the origin of ``B`` expressed in ``A``.
    """

    rotation: Quaternion
    translation: Vec3

    def apply(self, p_B: Vec3) -> Vec3:
        """Map a point ``p_B`` (expressed in frame B) into frame A."""
        return self.rotation.rotate(p_B) + self.translation

    def apply_vector(self, v_B: Vec3) -> Vec3:
        """Map a *free* vector (e.g. velocity) — rotation only, no translation."""
        return self.rotation.rotate(v_B)

    def compose(self, other: Transform) -> Transform:
        """``T_A_C = T_A_B.compose(T_B_C)`` (spec §19.2)."""
        # self = T_A_B, other = T_B_C
        new_rotation = self.rotation.compose(other.rotation)
        new_translation = self.translation + self.rotation.rotate(other.translation)
        return Transform(new_rotation, new_translation)

    def inverse(self) -> Transform:
        """``T_B_A`` from ``T_A_B`` (this transform)."""
        inv_rotation = self.rotation.inverse()
        inv_translation = -inv_rotation.rotate(self.translation)
        return Transform(inv_rotation, inv_translation)

    @staticmethod
    def identity() -> Transform:
        return Transform(Quaternion.identity(), ZERO)

    @staticmethod
    def from_translation(t: Vec3) -> Transform:
        return Transform(Quaternion.identity(), t)

    @staticmethod
    def from_rotation(q: Quaternion) -> Transform:
        return Transform(q, ZERO)

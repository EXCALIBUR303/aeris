"""aeris.core.frames — coordinate frame conventions and transforms (spec §19).

**Critical, per spec §19**: AERIS internals use ENU (world/odometry/map)
and FLU (body) exclusively (ADR-0008). NED/FRD conversion happens *only*
inside the PX4 vehicle adapter (:mod:`aeris.vehicle.px4_mavlink`) — an
import-linter contract enforces that nothing else imports the NED/FRD
conversion functions in :mod:`aeris.core.frames.conventions`.

May depend on ``aeris.core.units`` (import-linter contract; frames is in
the same layer as clock/config/logging).
"""

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import ZERO, Vec3

__all__ = ["ZERO", "Quaternion", "Transform", "Vec3"]

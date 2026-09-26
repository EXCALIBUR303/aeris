"""Collision / clearance metrics against ground-truth geometry (spec §41).

Needs a GT geometry source -- spec's own Phase 7 task list explicitly
allows "a ``WorldSpec`` stub or stock-world geometry until P9".
:class:`GroundTruthGeometry` represents exactly the world every AERIS
profile has actually used through Phase 7 (PX4's own ``default.sdf``):
a flat ground plane, no other obstacles. Collision/clearance against it
are honestly "never collides, unbounded lateral clearance" for *this*
world -- not a faked value -- and a real ``WorldSpec``-backed geometry
(Phase 9) plugs into the exact same interface.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

from aeris.core.frames.vector import Vec3


@dataclass(frozen=True, slots=True)
class SphereObstacle:
    center: Vec3
    radius_m: float


@dataclass(frozen=True, slots=True)
class GroundTruthGeometry:
    """A flat ground plane at ``ground_z`` plus zero or more spherical
    obstacles. Real ``WorldSpec``-generated geometry (Phase 9) is a strict
    superset of this shape."""

    ground_z: float = 0.0
    obstacles: tuple[SphereObstacle, ...] = field(default_factory=tuple)

    def clearance_at(self, position: Vec3) -> float:
        """Distance from ``position`` to the nearest occupied geometry."""
        clearance = position.z - self.ground_z
        for obstacle in self.obstacles:
            clearance = min(clearance, (position - obstacle.center).norm() - obstacle.radius_m)
        return clearance


def default_world_geometry() -> GroundTruthGeometry:
    """PX4's ``default.sdf`` -- the world every AERIS profile uses through
    Phase 7: a flat ground plane, no additional obstacles."""
    return GroundTruthGeometry(ground_z=0.0, obstacles=())


def minimum_clearance_m(positions: Sequence[Vec3], geometry: GroundTruthGeometry) -> float:
    """d_min = min_t d_GT(t) (spec §41)."""
    if not positions:
        raise ValueError("positions must be non-empty")
    return min(geometry.clearance_at(p) for p in positions)


def has_collision(
    positions: Sequence[Vec3], geometry: GroundTruthGeometry, vehicle_radius_m: float
) -> bool:
    """spec §41: 1[exists t: d_GT(t) < r_v]. (The contact-event half of the
    spec's fuller definition needs a real contact sensor -- Phase 8/9.)"""
    return minimum_clearance_m(positions, geometry) < vehicle_radius_m

"""Collision / clearance metrics against ground-truth geometry (spec §41).

Needs a GT geometry source -- spec's own Phase 7 task list explicitly
allows "a ``WorldSpec`` stub or stock-world geometry until P9".
:class:`GroundTruthGeometry` represents exactly the world every AERIS
profile has actually used through Phase 7 (PX4's own ``default.sdf``):
a flat ground plane, no other obstacles. Collision/clearance against it
are honestly "never collides, unbounded lateral clearance" for *this*
world -- not a faked value.

Phase 9 plugs real ``WorldSpec`` geometry into the exact same interface
via :func:`from_world_spec`: :class:`BoxObstacle`/:class:`CylinderObstacle`
compute an *analytic* clearance (not a voxel-quantized one) directly from
the same box/cylinder primitives ``aeris.simulation.worlds.sdf`` renders
into Gazebo and ``aeris.simulation.worlds.occupancy`` voxelizes -- the
same geometry, never authored twice, per ADR-007.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

from aeris.core.frames.vector import Vec3
from aeris.simulation.worlds.spec import WorldSpec


@dataclass(frozen=True, slots=True)
class SphereObstacle:
    center: Vec3
    radius_m: float

    def clearance_at(self, position: Vec3) -> float:
        return (position - self.center).norm() - self.radius_m


@dataclass(frozen=True, slots=True)
class BoxObstacle:
    """An axis-aligned box (Phase 9's rotated boxes are conservatively treated
    as their axis-aligned bounding box here -- exact oriented clearance
    isn't needed for this metric's purpose, a scalar minimum distance)."""

    center: Vec3
    half_extent: Vec3

    def clearance_at(self, position: Vec3) -> float:
        dx = abs(position.x - self.center.x) - self.half_extent.x
        dy = abs(position.y - self.center.y) - self.half_extent.y
        dz = abs(position.z - self.center.z) - self.half_extent.z
        # Outside on at least one axis: Euclidean distance to the nearest
        # face/edge/corner. Inside on all axes: negative (penetration) depth.
        outside = Vec3(max(dx, 0.0), max(dy, 0.0), max(dz, 0.0))
        if outside.x == 0.0 and outside.y == 0.0 and outside.z == 0.0:
            return max(dx, dy, dz)
        return outside.norm()


@dataclass(frozen=True, slots=True)
class CylinderObstacle:
    center: Vec3
    radius_m: float
    height_m: float

    def clearance_at(self, position: Vec3) -> float:
        radial = (
            (position.x - self.center.x) ** 2 + (position.y - self.center.y) ** 2
        ) ** 0.5 - self.radius_m
        axial = abs(position.z - self.center.z) - self.height_m / 2.0
        if radial <= 0.0 and axial <= 0.0:
            return float(max(radial, axial))
        return float((max(radial, 0.0) ** 2 + max(axial, 0.0) ** 2) ** 0.5)


Obstacle = SphereObstacle | BoxObstacle | CylinderObstacle


@dataclass(frozen=True, slots=True)
class GroundTruthGeometry:
    """A flat ground plane at ``ground_z`` plus zero or more obstacles."""

    ground_z: float = 0.0
    obstacles: tuple[Obstacle, ...] = field(default_factory=tuple)

    def clearance_at(self, position: Vec3) -> float:
        """Distance from ``position`` to the nearest occupied geometry."""
        clearance = position.z - self.ground_z
        for obstacle in self.obstacles:
            clearance = min(clearance, obstacle.clearance_at(position))
        return clearance


def default_world_geometry() -> GroundTruthGeometry:
    """PX4's ``default.sdf`` -- the world every AERIS profile uses through
    Phase 7: a flat ground plane, no additional obstacles."""
    return GroundTruthGeometry(ground_z=0.0, obstacles=())


def from_world_spec(spec: WorldSpec) -> GroundTruthGeometry:
    """Convert a :class:`WorldSpec` (Phase 9) into a :class:`GroundTruthGeometry`
    -- the real-geometry counterpart to :func:`default_world_geometry`."""
    boxes = tuple(
        BoxObstacle(center=Vec3(b.x, b.y, b.z), half_extent=b.half_extent) for b in spec.boxes
    )
    cylinders = tuple(
        CylinderObstacle(center=Vec3(c.x, c.y, c.z), radius_m=c.radius_m, height_m=c.height_m)
        for c in spec.cylinders
    )
    return GroundTruthGeometry(ground_z=spec.bounds.min_z, obstacles=boxes + cylinders)


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

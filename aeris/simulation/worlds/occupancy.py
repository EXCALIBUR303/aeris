"""Ground-truth occupancy voxelization + reachability (spec §9.3, §33's generators).

``voxelize()`` renders a :class:`WorldSpec` into a sparse-occupied voxel
grid — the evaluator-only ``GroundTruthRenderer`` half of ADR-007's "no
world geometry authored twice": the exact same box/cylinder primitives the
``SdfRenderer`` (sdf.py) turns into Gazebo models are what this module
tests points against.

Kept numpy-free (spec scopes ``numpy`` to the ``learn`` extra, Phase 13+):
worlds here are modest (tens of primitives, grids in the low hundreds of
thousands of cells), and occupied cells are stored sparsely as a
``frozenset`` of integer indices rather than a dense array, since most of
any world is free space.
"""

from __future__ import annotations

import functools
import math
from collections import deque
from dataclasses import dataclass

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import Vec3
from aeris.simulation.worlds.spec import Box, Cylinder, WorldSpec

Cell = tuple[int, int, int]

_NEIGHBOR_OFFSETS: tuple[Cell, ...] = (
    (1, 0, 0),
    (-1, 0, 0),
    (0, 1, 0),
    (0, -1, 0),
    (0, 0, 1),
    (0, 0, -1),
)


def _box_transform(box: Box) -> Transform:
    """``T_world_boxlocal`` -- box-local axes aligned with its own (roll, pitch, yaw)."""
    rotation = (
        Quaternion.from_axis_angle(Vec3(0.0, 0.0, 1.0), box.yaw_rad)
        .compose(Quaternion.from_axis_angle(Vec3(0.0, 1.0, 0.0), box.pitch_rad))
        .compose(Quaternion.from_axis_angle(Vec3(1.0, 0.0, 0.0), box.roll_rad))
    )
    return Transform(rotation, box.center)


@functools.lru_cache(maxsize=4096)
def _box_inverse(box: Box) -> Transform:
    """``T_boxlocal_world``, memoized per (immutable) box: the coverage
    raycaster (Phase 12) calls :func:`point_in_box` per ray sample per box,
    and rebuilding this from quaternions every time made scoring one
    episode take ~30 min (profiled in Phase 13). Same arithmetic, computed
    once -- results are bit-identical."""
    return _box_transform(box).inverse()


def _point_in_box_local(p_local: Vec3, half: Vec3) -> bool:
    return abs(p_local.x) <= half.x and abs(p_local.y) <= half.y and abs(p_local.z) <= half.z


def point_in_box(p: Vec3, box: Box) -> bool:
    """Whether ``p`` (world frame) falls inside ``box`` (which may be rotated).

    The box's inverse transform is memoized (:func:`_box_inverse`) --
    recomputing it per point made :func:`voxelize` over 100x slower than
    necessary (profiled during Phase 9 world generation), and the coverage
    raycaster hit the same cost again in Phase 13.
    """
    # Exact early-out: rotation preserves distance, so any point inside the
    # box is within its half-diagonal of the center. The relative margin
    # keeps a point the rotated test would accept (on a face, after float
    # rounding) from ever being rejected here.
    c, h = box.center, box.half_extent
    dx, dy, dz = p.x - c.x, p.y - c.y, p.z - c.z
    if dx * dx + dy * dy + dz * dz > (h.x * h.x + h.y * h.y + h.z * h.z) * (1.0 + 1e-9) + 1e-12:
        return False
    return _point_in_box_local(_box_inverse(box).apply(p), h)


def point_in_cylinder(p: Vec3, cyl: Cylinder) -> bool:
    dx, dy = p.x - cyl.x, p.y - cyl.y
    half_h = cyl.height_m / 2.0
    return (dx * dx + dy * dy) <= cyl.radius_m**2 and abs(p.z - cyl.z) <= half_h


def is_occupied(spec: WorldSpec, p: Vec3) -> bool:
    return any(point_in_box(p, b) for b in spec.boxes) or any(
        point_in_cylinder(p, c) for c in spec.cylinders
    )


def column_occupied_in_band(
    spec: WorldSpec, x: float, y: float, *, z_lo_m: float, z_hi_m: float, resolution_m: float
) -> bool:
    """Whether *any* point in the queried altitude band at ``(x, y)`` is
    occupied per the exact ``WorldSpec`` geometry -- sampled at
    ``resolution_m`` steps, matching how the agent's own band projection
    decides "any voxel in-band occupied" (spec §21.1), so both the
    evaluator's map-accuracy check (Phase 11) and its coverage raycaster
    (Phase 12) use the same effective vertical granularity as each other
    and as the agent's own map."""
    z = z_lo_m + resolution_m / 2.0
    while z <= z_hi_m:
        if is_occupied(spec, Vec3(x, y, z)):
            return True
        z += resolution_m
    return False


@dataclass(frozen=True, slots=True)
class OccupancyGrid:
    """A sparse voxel grid: only occupied cells are stored."""

    resolution_m: float
    origin: Vec3  # world position of cell (0, 0, 0)'s corner
    occupied: frozenset[Cell]

    def world_to_cell(self, p: Vec3) -> Cell:
        return (
            math.floor((p.x - self.origin.x) / self.resolution_m),
            math.floor((p.y - self.origin.y) / self.resolution_m),
            math.floor((p.z - self.origin.z) / self.resolution_m),
        )

    def cell_center(self, cell: Cell) -> Vec3:
        cx, cy, cz = cell
        r = self.resolution_m
        return Vec3(
            self.origin.x + (cx + 0.5) * r,
            self.origin.y + (cy + 0.5) * r,
            self.origin.z + (cz + 0.5) * r,
        )

    def is_cell_occupied(self, cell: Cell) -> bool:
        return cell in self.occupied

    def reachable_free_cells(self, start: Vec3, *, bounds_cells: tuple[Cell, Cell]) -> set[Cell]:
        """6-connected flood fill of free cells reachable from ``start``, clipped to ``bounds_cells``."""
        (min_cx, min_cy, min_cz), (max_cx, max_cy, max_cz) = bounds_cells
        start_cell = self.world_to_cell(start)
        if self.is_cell_occupied(start_cell):
            return set()

        visited: set[Cell] = {start_cell}
        queue: deque[Cell] = deque([start_cell])
        while queue:
            cx, cy, cz = queue.popleft()
            for dx, dy, dz in _NEIGHBOR_OFFSETS:
                nxt = (cx + dx, cy + dy, cz + dz)
                if nxt in visited:
                    continue
                if not (
                    min_cx <= nxt[0] <= max_cx
                    and min_cy <= nxt[1] <= max_cy
                    and min_cz <= nxt[2] <= max_cz
                ):
                    continue
                if self.is_cell_occupied(nxt):
                    continue
                visited.add(nxt)
                queue.append(nxt)
        return visited


def voxelize(spec: WorldSpec, *, resolution_m: float = 0.2) -> OccupancyGrid:
    """Render ``spec`` into an :class:`OccupancyGrid` at ``resolution_m``.

    Only cells whose *center* falls inside a primitive are marked occupied
    -- consistent with how :func:`is_occupied` is used for point checks
    elsewhere (sampling, reachability), so the two never disagree about
    the same point by construction.

    Precomputes each box's inverse transform once (not per voxel -- this
    was the dominant cost, profiled at >100x during Phase 9 world
    generation) and pre-filters by each box's bounding-sphere radius
    before the exact, rotation-aware check.
    """
    origin = Vec3(spec.bounds.min_x, spec.bounds.min_y, spec.bounds.min_z)
    n_x = math.ceil((spec.bounds.max_x - spec.bounds.min_x) / resolution_m)
    n_y = math.ceil((spec.bounds.max_y - spec.bounds.min_y) / resolution_m)
    n_z = math.ceil((spec.bounds.max_z - spec.bounds.min_z) / resolution_m)

    box_data = [
        (b.center, b.half_extent.norm(), _box_transform(b).inverse(), b.half_extent)
        for b in spec.boxes
    ]
    cyl_data = [(c.x, c.y, c.z, c.radius_m, c.height_m / 2.0) for c in spec.cylinders]

    occupied: set[Cell] = set()
    grid = OccupancyGrid(resolution_m=resolution_m, origin=origin, occupied=frozenset())
    for iz in range(n_z):
        for iy in range(n_y):
            for ix in range(n_x):
                center = grid.cell_center((ix, iy, iz))
                hit = False
                for box_center, bounding_radius, inv, half in box_data:
                    if (center - box_center).norm() > bounding_radius:
                        continue
                    if _point_in_box_local(inv.apply(center), half):
                        hit = True
                        break
                if not hit:
                    for cx, cy, cz, radius_m, half_h in cyl_data:
                        dx, dy = center.x - cx, center.y - cy
                        if dx * dx + dy * dy <= radius_m**2 and abs(center.z - cz) <= half_h:
                            hit = True
                            break
                if hit:
                    occupied.add((ix, iy, iz))
    return OccupancyGrid(resolution_m=resolution_m, origin=origin, occupied=frozenset(occupied))


def check_reachability(
    spec: WorldSpec, *, resolution_m: float = 0.5, min_free_fraction: float = 0.5
) -> bool:
    """Whether every spawn pose reaches at least ``min_free_fraction`` of the
    altitude band's total free volume via a 6-connected flood fill.

    A coarse resolution keeps this cheap; the intent is to catch a
    generator bug that accidentally seals a spawn point into a pocket, not
    to substitute for a real planner (Phase 12).
    """
    grid = voxelize(spec, resolution_m=resolution_m)
    origin = grid.origin
    n_x = math.ceil((spec.bounds.max_x - spec.bounds.min_x) / resolution_m)
    n_y = math.ceil((spec.bounds.max_y - spec.bounds.min_y) / resolution_m)
    alt_low, alt_high = spec.altitude_band_m
    min_cz = math.floor((alt_low - origin.z) / resolution_m)
    max_cz = math.ceil((alt_high - origin.z) / resolution_m)
    bounds_cells: tuple[Cell, Cell] = ((0, 0, min_cz), (n_x - 1, n_y - 1, max_cz))

    total_cells = (n_x) * (n_y) * (max_cz - min_cz + 1)
    occupied_in_band = sum(1 for (_, _, cz) in grid.occupied if min_cz <= cz <= max_cz)
    total_free = max(total_cells - occupied_in_band, 1)

    for spawn in spec.spawn_poses:
        reachable = grid.reachable_free_cells(spawn.position, bounds_cells=bounds_cells)
        if len(reachable) / total_free < min_free_fraction:
            return False
    return True

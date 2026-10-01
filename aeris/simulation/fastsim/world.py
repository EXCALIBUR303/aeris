"""FastSim worlds: ``WorldSpec`` rendered to a dense voxel occupancy volume
(spec §17.3), stacked into a :class:`WorldBank` so N parallel environments
can each sit in a different world inside one Numba-friendly array.

Voxelization uses the same rule as Phase 9's evaluator-side
:func:`aeris.simulation.worlds.occupancy.voxelize` -- a voxel is occupied
iff its *center* lies inside a primitive -- but is vectorized per
primitive over that primitive's own bounding box, instead of looping
every voxel in pure Python against every primitive (measured in Phase 12
at 14.8s for one 24x24x6m val world at 0.2m; this is milliseconds). An
equivalence test pins the two against each other.

The ground plane (``z = ground_z``) is analytic, not voxels: Gazebo
renders a ground plane the depth camera sees, so FastSim's raycaster must
see one too, at exact (not voxel-quantized) height.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from aeris.simulation.worlds.spec import WorldSpec

DEFAULT_RESOLUTION_M = 0.1


def _box_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`` -- the same composition as
    :func:`aeris.simulation.worlds.occupancy._box_transform`."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return np.asarray(rz @ ry @ rx)


@dataclass(frozen=True, slots=True)
class FastWorld:
    """One world's dense occupancy volume, in the world (Gazebo) frame."""

    name: str
    occ: np.ndarray  # uint8 [nx, ny, nz], 1 = occupied
    origin: np.ndarray  # float64 (3,), world position of voxel (0,0,0)'s min corner
    resolution_m: float
    ground_z: float
    spawn_world: np.ndarray  # float64 (3,)
    spawn_yaw_rad: float
    altitude_band_m: tuple[float, float]

    def world_to_index(self, p: np.ndarray) -> tuple[int, int, int]:
        idx = np.floor((p - self.origin) / self.resolution_m).astype(int)
        return int(idx[0]), int(idx[1]), int(idx[2])

    def is_occupied(self, p: np.ndarray) -> bool:
        if p[2] <= self.ground_z:
            return True
        i, j, k = self.world_to_index(p)
        nx, ny, nz = self.occ.shape
        if 0 <= i < nx and 0 <= j < ny and 0 <= k < nz:
            return bool(self.occ[i, j, k])
        return False


def build_world(spec: WorldSpec, *, resolution_m: float = DEFAULT_RESOLUTION_M) -> FastWorld:
    b = spec.bounds
    origin = np.array([b.min_x, b.min_y, b.min_z], dtype=np.float64)
    shape = (
        math.ceil((b.max_x - b.min_x) / resolution_m),
        math.ceil((b.max_y - b.min_y) / resolution_m),
        math.ceil((b.max_z - b.min_z) / resolution_m),
    )
    occ = np.zeros(shape, dtype=np.uint8)
    r = resolution_m

    def _index_range(lo: float, hi: float, o: float, n: int) -> tuple[int, int]:
        # Voxels whose *centers* could fall in [lo, hi].
        i0 = max(0, math.floor((lo - o) / r - 0.5))
        i1 = min(n - 1, math.ceil((hi - o) / r - 0.5))
        return i0, i1

    for box in spec.boxes:
        rot = _box_rotation(box.roll_rad, box.pitch_rad, box.yaw_rad)
        half = np.array([box.size_x, box.size_y, box.size_z]) / 2.0
        center = np.array([box.x, box.y, box.z])
        # World-frame AABB of the (possibly rotated) box.
        ext = np.abs(rot) @ half
        lo, hi = center - ext, center + ext
        (i0, i1), (j0, j1), (k0, k1) = (
            _index_range(lo[a], hi[a], origin[a], shape[a]) for a in range(3)
        )
        if i0 > i1 or j0 > j1 or k0 > k1:
            continue
        ii, jj, kk = np.meshgrid(
            np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), np.arange(k0, k1 + 1), indexing="ij"
        )
        centers = np.stack([ii, jj, kk], axis=-1) * r + origin + r / 2.0
        local = (centers - center) @ rot  # == (R^T @ (p - c)) row-wise
        inside = np.all(np.abs(local) <= half, axis=-1)
        occ[ii[inside], jj[inside], kk[inside]] = 1

    for cyl in spec.cylinders:
        lo = np.array([cyl.x - cyl.radius_m, cyl.y - cyl.radius_m, cyl.z - cyl.height_m / 2.0])
        hi = np.array([cyl.x + cyl.radius_m, cyl.y + cyl.radius_m, cyl.z + cyl.height_m / 2.0])
        (i0, i1), (j0, j1), (k0, k1) = (
            _index_range(lo[a], hi[a], origin[a], shape[a]) for a in range(3)
        )
        if i0 > i1 or j0 > j1 or k0 > k1:
            continue
        ii, jj, kk = np.meshgrid(
            np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), np.arange(k0, k1 + 1), indexing="ij"
        )
        centers = np.stack([ii, jj, kk], axis=-1) * r + origin + r / 2.0
        dx, dy = centers[..., 0] - cyl.x, centers[..., 1] - cyl.y
        inside = (dx * dx + dy * dy <= cyl.radius_m**2) & (
            np.abs(centers[..., 2] - cyl.z) <= cyl.height_m / 2.0
        )
        occ[ii[inside], jj[inside], kk[inside]] = 1

    spawn = spec.spawn_poses[0]
    return FastWorld(
        name=spec.name,
        occ=occ,
        origin=origin,
        resolution_m=resolution_m,
        ground_z=0.0,
        spawn_world=np.array([spawn.x, spawn.y, spawn.z], dtype=np.float64),
        spawn_yaw_rad=spawn.yaw_rad,
        altitude_band_m=spec.altitude_band_m,
    )


@dataclass(frozen=True, slots=True)
class WorldBank:
    """W worlds zero-padded into one ``[W, NX, NY, NZ]`` array, so a batched
    Numba kernel can index ``occ[world_idx[env], i, j, k]`` directly.
    Out-of-a-world's-own-dims voxels are free (padding is never occupied)."""

    worlds: tuple[FastWorld, ...]
    occ: np.ndarray  # uint8 [W, NX, NY, NZ]
    origins: np.ndarray  # float64 [W, 3]
    dims: np.ndarray  # int64 [W, 3]
    ground_z: np.ndarray  # float64 [W]
    resolution_m: float

    @staticmethod
    def from_worlds(worlds: Sequence[FastWorld]) -> WorldBank:
        if not worlds:
            raise ValueError("WorldBank needs at least one world")
        res = worlds[0].resolution_m
        if any(w.resolution_m != res for w in worlds):
            raise ValueError("every world in a bank must share one resolution")
        dims = np.array([w.occ.shape for w in worlds], dtype=np.int64)
        big = dims.max(axis=0)
        occ = np.zeros((len(worlds), *big), dtype=np.uint8)
        for n, w in enumerate(worlds):
            nx, ny, nz = w.occ.shape
            occ[n, :nx, :ny, :nz] = w.occ
        return WorldBank(
            worlds=tuple(worlds),
            occ=occ,
            origins=np.stack([w.origin for w in worlds]),
            dims=dims,
            ground_z=np.array([w.ground_z for w in worlds], dtype=np.float64),
            resolution_m=res,
        )

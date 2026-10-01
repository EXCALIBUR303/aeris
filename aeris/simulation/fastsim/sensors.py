"""FastSim sensors: Numba Amanatides--Woo voxel traversal against a
:class:`~aeris.simulation.fastsim.world.WorldBank` (spec §17.3), plus the
analytic ground plane.

Depth images are rendered as **z-depth** (distance along the optical
axis), not Euclidean range -- that's what a Gazebo depth camera publishes
and what :func:`aeris.perception.depth.projection.depth_to_points_camera`
back-projects from, so a FastSim depth image and a Tier H depth image are
the same quantity pixel for pixel. No-return pixels are ``inf``, matching
Gazebo's own no-return convention.

The camera ray set is built from the real depth camera's intrinsics
(``configs/vehicle/sensors.yaml``), strided the same way the Tier H
pipeline strides it, so "sensor raycast matching Tier H intrinsics" (spec
§51 Phase 13 task 3) holds by construction: pixel (u, v) in a FastSim
image is the same ray as pixel (u, v) in a Tier H image.
"""

from __future__ import annotations

import math

import numpy as np
from numba import njit, prange

from aeris.perception.depth.projection import CameraIntrinsics


@njit(cache=True, inline="always")
def _cast_one(
    occ: np.ndarray,
    w: int,
    origin: np.ndarray,
    dims: np.ndarray,
    res: float,
    ground_z: float,
    px: float,
    py: float,
    pz: float,
    dx: float,
    dy: float,
    dz: float,
    max_range: float,
) -> float:
    """Range (m) along the unit direction (dx, dy, dz) to the first
    occupied voxel or the ground plane, or ``inf`` if nothing is hit
    within ``max_range``."""
    t_hit = math.inf
    # Analytic ground plane.
    if dz < 0.0 and pz > ground_z:
        t_ground = (ground_z - pz) / dz
        if t_ground <= max_range:
            t_hit = t_ground

    ox, oy, oz = origin[w, 0], origin[w, 1], origin[w, 2]
    nx, ny, nz = dims[w, 0], dims[w, 1], dims[w, 2]
    gx0, gy0, gz0 = ox + nx * res, oy + ny * res, oz + nz * res

    # Clip the ray to the grid's AABB (slab method).
    t0, t1 = 0.0, min(max_range, t_hit)
    for a in range(3):
        if a == 0:
            p, d, lo, hi = px, dx, ox, gx0
        elif a == 1:
            p, d, lo, hi = py, dy, oy, gy0
        else:
            p, d, lo, hi = pz, dz, oz, gz0
        if abs(d) < 1e-12:
            if p < lo or p >= hi:
                return t_hit
        else:
            ta, tb = (lo - p) / d, (hi - p) / d
            if ta > tb:
                ta, tb = tb, ta
            if ta > t0:
                t0 = ta
            if tb < t1:
                t1 = tb
            if t0 > t1:
                return t_hit

    # Entry point, nudged inside.
    t = t0
    ex, ey, ez = px + dx * (t + 1e-9), py + dy * (t + 1e-9), pz + dz * (t + 1e-9)
    ix = min(max(int(math.floor((ex - ox) / res)), 0), nx - 1)  # noqa: RUF046 -- numba math.floor returns float
    iy = min(max(int(math.floor((ey - oy) / res)), 0), ny - 1)  # noqa: RUF046 -- numba math.floor returns float
    iz = min(max(int(math.floor((ez - oz) / res)), 0), nz - 1)  # noqa: RUF046 -- numba math.floor returns float

    step_x = 1 if dx > 0 else -1
    step_y = 1 if dy > 0 else -1
    step_z = 1 if dz > 0 else -1
    inf = math.inf
    tdx = res / abs(dx) if dx != 0.0 else inf
    tdy = res / abs(dy) if dy != 0.0 else inf
    tdz = res / abs(dz) if dz != 0.0 else inf
    bx = ox + (ix + (1 if dx > 0 else 0)) * res
    by = oy + (iy + (1 if dy > 0 else 0)) * res
    bz = oz + (iz + (1 if dz > 0 else 0)) * res
    tmx = (bx - px) / dx if dx != 0.0 else inf
    tmy = (by - py) / dy if dy != 0.0 else inf
    tmz = (bz - pz) / dz if dz != 0.0 else inf

    t_cur = t0
    while t_cur <= t1:
        if occ[w, ix, iy, iz] != 0:
            return t_cur
        if tmx < tmy:
            if tmx < tmz:
                t_cur = tmx
                ix += step_x
                tmx += tdx
            else:
                t_cur = tmz
                iz += step_z
                tmz += tdz
        else:
            if tmy < tmz:
                t_cur = tmy
                iy += step_y
                tmy += tdy
            else:
                t_cur = tmz
                iz += step_z
                tmz += tdz
        if ix < 0 or iy < 0 or iz < 0 or ix >= nx or iy >= ny or iz >= nz:
            break
    return t_hit


@njit(cache=True, parallel=True)
def cast_rays_batch(
    occ: np.ndarray,
    origins: np.ndarray,
    dims: np.ndarray,
    ground_z: np.ndarray,
    res: float,
    world_idx: np.ndarray,  # int64 [N]
    sensor_pos: np.ndarray,  # float64 [N, 3], world frame
    sensor_rot: np.ndarray,  # float64 [N, 3, 3], R_world_sensor
    dirs_sensor: np.ndarray,  # float64 [K, 3], unit vectors in the sensor frame
    max_range: float,
) -> np.ndarray:
    """Ranges ``[N, K]`` (Euclidean, m; ``inf`` = no return)."""
    n = sensor_pos.shape[0]
    k = dirs_sensor.shape[0]
    out = np.empty((n, k), dtype=np.float64)
    for flat in prange(n * k):  # type: ignore[no-untyped-call,attr-defined]
        e = flat // k
        j = flat - e * k
        r = sensor_rot[e]
        ds = dirs_sensor[j]
        dx = r[0, 0] * ds[0] + r[0, 1] * ds[1] + r[0, 2] * ds[2]
        dy = r[1, 0] * ds[0] + r[1, 1] * ds[1] + r[1, 2] * ds[2]
        dz = r[2, 0] * ds[0] + r[2, 1] * ds[1] + r[2, 2] * ds[2]
        w = world_idx[e]
        out[e, j] = _cast_one(
            occ,
            w,
            origins,
            dims,
            res,
            ground_z[w],
            sensor_pos[e, 0],
            sensor_pos[e, 1],
            sensor_pos[e, 2],
            dx,
            dy,
            dz,
            max_range,
        )
    return out


def camera_ray_dirs(intrinsics: CameraIntrinsics, *, stride: int) -> tuple[np.ndarray, np.ndarray]:
    """Unit ray directions in the **optical** frame (x right, y down, z
    forward) for every ``stride``-th pixel -- the same (u, v) sampling as
    :func:`~aeris.perception.depth.projection.depth_to_points_camera`'s own
    ``stride`` -- plus each ray's z-component (range -> z-depth factor).

    Returns ``(dirs [H'*W', 3], z_factor [H'*W'])`` in row-major (v, u) order.
    """
    us = np.arange(0, intrinsics.width, stride, dtype=np.float64)
    vs = np.arange(0, intrinsics.height, stride, dtype=np.float64)
    vv, uu = np.meshgrid(vs, us, indexing="ij")
    x = (uu - intrinsics.cx) / intrinsics.fx
    y = (vv - intrinsics.cy) / intrinsics.fy
    d = np.stack([x, y, np.ones_like(x)], axis=-1).reshape(-1, 3)
    norm = np.linalg.norm(d, axis=1, keepdims=True)
    d = d / norm
    return d, d[:, 2].copy()


def lidar_ray_dirs(n_rays: int) -> np.ndarray:
    """A horizontal 360-degree fan in the body (FLU) frame, ray 0 = +x (forward),
    counter-clockwise."""
    a = np.arange(n_rays, dtype=np.float64) * (2.0 * math.pi / n_rays)
    return np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], axis=-1)


def ranges_to_zdepth(ranges: np.ndarray, z_factor: np.ndarray) -> np.ndarray:
    """Euclidean range -> z-depth (``inf`` stays ``inf``)."""
    return np.asarray(ranges * z_factor)

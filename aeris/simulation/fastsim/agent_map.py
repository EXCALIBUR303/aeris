"""FastSim's agent-side 2D map: depth rays -> per-column log-odds, the fast
stand-in for Tier H's voxel map + ``project_band(free_rule="any_observed")``
(Phase 11/12). Rays are cast from the vehicle's *estimated* pose (the map
is agent data, built from what the agent believes its pose is -- the
same registration error Tier H's EKF-posed map has).

Per ray, samples every half fine-cell along the ray; a sample inside the
altitude band marks its column observed-free (``l_free``), and the hit
point, if inside the band, marks its column observed-occupied (``l_occ``).
Hits below the band (the ground) mark nothing -- exactly as a ground
return can't make a 2D column occupied in Tier H's band projection.

Approximation (documented, not hidden): Tier H keeps log-odds per *voxel*
and calls a column occupied if *any* in-band voxel is occupied; this keeps
one log-odds per *column*. A column seen both free (at one height) and
occupied (at another) can therefore net out differently. Gate item 6
(scripted frontier coverage vs Tier H) measures whether that matters for
the end-to-end behavior.
"""

from __future__ import annotations

import math

import numpy as np
from numba import njit

UNKNOWN, FREE, OCCUPIED = 0, 1, 2


@njit(cache=True)
def integrate_rays_2d(
    logodds: np.ndarray,  # [NX, NY] float64, in/out
    touched: np.ndarray,  # [NX, NY] bool, in/out
    min_ix: int,
    min_iy: int,
    res: float,
    origin: np.ndarray,  # (3,) estimated camera position (world)
    dirs: np.ndarray,  # [K, 3] unit ray directions (world, estimated attitude)
    ranges: np.ndarray,  # [K] true ranges (inf = no return)
    max_range: float,
    z_lo: float,
    z_hi: float,
    l_occ: float,
    l_free: float,
    l_min: float,
    l_max: float,
) -> int:
    """Returns the number of columns that went from untouched to touched."""
    nx, ny = logodds.shape
    newly = 0
    step = res * 0.5
    for k in range(dirs.shape[0]):
        r = ranges[k]
        hit = math.isfinite(r) and r <= max_range
        length = r if hit else max_range
        dx, dy, dz = dirs[k, 0], dirs[k, 1], dirs[k, 2]
        n = int(length / step)
        last_i, last_j = -1, -1
        for s in range(n):
            t = s * step
            z = origin[2] + dz * t
            if z < z_lo or z > z_hi:
                continue
            i = int(math.floor((origin[0] + dx * t) / res)) - min_ix  # noqa: RUF046
            j = int(math.floor((origin[1] + dy * t) / res)) - min_iy  # noqa: RUF046
            if i == last_i and j == last_j:
                continue
            last_i, last_j = i, j
            if i < 0 or j < 0 or i >= nx or j >= ny:
                continue
            if not touched[i, j]:
                touched[i, j] = True
                newly += 1
            logodds[i, j] = max(l_min, logodds[i, j] + l_free)
        if hit:
            z = origin[2] + dz * r
            if z_lo <= z <= z_hi:
                i = int(math.floor((origin[0] + dx * r) / res)) - min_ix  # noqa: RUF046
                j = int(math.floor((origin[1] + dy * r) / res)) - min_iy  # noqa: RUF046
                if 0 <= i < nx and 0 <= j < ny:
                    if not touched[i, j]:
                        touched[i, j] = True
                        newly += 1
                    # Undo this ray's own free update on its hit column, then mark occupied.
                    if i == last_i and j == last_j:
                        logodds[i, j] -= l_free
                    logodds[i, j] = min(l_max, logodds[i, j] + l_occ)
    return newly


def map_state(logodds: np.ndarray, touched: np.ndarray) -> np.ndarray:
    """int8 UNKNOWN/FREE/OCCUPIED per column."""
    st = np.zeros(logodds.shape, dtype=np.int8)
    st[touched & (logodds <= 0.0)] = FREE
    st[touched & (logodds > 0.0)] = OCCUPIED
    return st


def dilate_occupied(state: np.ndarray, radius_cells: int) -> np.ndarray:
    """``project_band``'s inflation, densely: occupied grows by a Chebyshev
    ball of ``radius_cells`` (its iterated 8-neighbour dilation); free cells
    swallowed by it become occupied, unknown cells stay unknown unless reached."""
    if radius_cells <= 0:
        return state.copy()
    occ = state == OCCUPIED
    grown = occ.copy()
    nx, ny = occ.shape
    for di in range(-radius_cells, radius_cells + 1):
        for dj in range(-radius_cells, radius_cells + 1):
            si = slice(max(0, di), nx + min(0, di))
            ti = slice(max(0, -di), nx + min(0, -di))
            sj = slice(max(0, dj), ny + min(0, dj))
            tj = slice(max(0, -dj), ny + min(0, -dj))
            grown[si, sj] |= occ[ti, tj]
    out = state.copy()
    out[grown] = OCCUPIED
    return out

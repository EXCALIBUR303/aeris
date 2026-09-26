"""Numba-accelerated per-frame ray integration (spec §21.1, ADR-0009: "Numba
ray integration").

One call integrates an *entire depth/LiDAR frame* (hundreds to thousands of
rays sharing one sensor origin) and returns, per voxel touched by any ray
this frame, the *net* log-odds delta to apply -- "free along the ray,
occupied at the hit" (or free all the way to ``max_range_m`` for a
no-return ray, spec's "max-range free-space policy": never treated as
occupied at max).

Why a frame-level accumulator, not a per-ray one: profiling during Phase
11 development showed the geometry (tracing each ray) is essentially free
in Numba (~0.6 microseconds/ray, warm) but applying updates to the
block-hashed Python-side map one *raw* ray-voxel touch at a time (~144,000
touches for a 4,800-point frame, most of them overlapping) cost ~120ms --
comfortably blowing the spec's 50ms p95 budget. Accumulating all of a
frame's touches into a Numba ``typed.Dict`` first and returning only the
~15,000 *unique* voxels actually touched cut the Python-side apply step by
~10x (measured), bringing a full frame under 25ms with margin. Voxel
coordinates are encoded into a single ``int64`` key (see :func:`encode_key`)
so Numba's dict can use a primitive key type instead of a tuple.
"""

from __future__ import annotations

import math

import numpy as np
from numba import njit
from numba.core import types
from numba.typed import Dict

# Keeps per-axis voxel indices in [-(OFFSET), OFFSET-1] representable
# without overflowing the encoded int64 key (max key ~= SPAN**3 ~= 2**57,
# safely under int64's 2**63-1) -- at any resolution >= 0.05m this covers
# +/-13,000m per axis, far beyond any AERIS world (spec worlds are tens of
# meters).
OFFSET = 1 << 18
SPAN = 1 << 19


def encode_key(ix: int, iy: int, iz: int) -> int:
    """Pack a signed 3D voxel index into one non-negative ``int64`` key."""
    return ((ix + OFFSET) * SPAN + (iy + OFFSET)) * SPAN + (iz + OFFSET)


def decode_key(key: int) -> tuple[int, int, int]:
    """Inverse of :func:`encode_key`."""
    iz = key % SPAN - OFFSET
    key //= SPAN
    iy = key % SPAN - OFFSET
    key //= SPAN
    ix = key - OFFSET
    return ix, iy, iz


@njit(cache=True)
def _integrate_frame_jit(
    ox: float,
    oy: float,
    oz: float,
    ex: np.ndarray,
    ey: np.ndarray,
    ez: np.ndarray,
    is_hit: np.ndarray,
    resolution_m: float,
    max_range_m: float,
    l_occ: float,
    l_free: float,
    offset: int,
    span: int,
) -> tuple[np.ndarray, np.ndarray]:
    acc = Dict.empty(key_type=types.int64, value_type=types.float64)
    n_points = ex.shape[0]
    for p in range(n_points):
        dx = ex[p] - ox
        dy = ey[p] - oy
        dz = ez[p] - oz
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length < 1e-9:
            continue
        ux, uy, uz = (
            dx / length,
            dy / length,
            dz / length,
        )  # unit direction from the ORIGINAL length
        hit = is_hit[p]
        if length > max_range_m:
            length = max_range_m
            hit = False  # clipped to max range -> never a real hit (spec: "not occupied at max")
        # int(...) casts below are load-bearing inside this @njit function:
        # Numba's math.floor/math.ceil on a float64 return float64 (unlike
        # plain Python, where they already return int), so ruff's
        # "already an integer" reading doesn't hold in this context.
        n_steps = max(1, int(math.ceil(length / (resolution_m * 0.5))))  # noqa: RUF046
        step = length / n_steps
        prev_ix, prev_iy, prev_iz = 1 << 30, 1 << 30, 1 << 30
        # i in [0, n_steps]: every sample but the last is an interpolated
        # point along the ray (only needs to be dense enough to not skip a
        # voxel, not exact); the last is forced to the ray's true endpoint
        # (`length` * unit direction) rather than `n_steps * step`, since
        # floating-point step accumulation can otherwise land one voxel
        # short of the actual endpoint right at a voxel boundary -- caught
        # live by a hand-computed "fan of rays against a flat wall" test
        # whose rightmost ray's hit voxel was silently never marked
        # occupied at all.
        for i in range(n_steps + 1):
            if i == n_steps:
                px, py, pz = ox + ux * length, oy + uy * length, oz + uz * length
            else:
                t = i * step
                px, py, pz = ox + ux * t, oy + uy * t, oz + uz * t
            ix = int(math.floor(px / resolution_m))  # noqa: RUF046
            iy = int(math.floor(py / resolution_m))  # noqa: RUF046
            iz = int(math.floor(pz / resolution_m))  # noqa: RUF046
            if ix == prev_ix and iy == prev_iy and iz == prev_iz:
                continue
            prev_ix, prev_iy, prev_iz = ix, iy, iz
            is_last = i == n_steps
            delta = (l_occ if hit else l_free) if is_last else l_free
            key = ((ix + offset) * span + (iy + offset)) * span + (iz + offset)
            if key in acc:
                acc[key] += delta
            else:
                acc[key] = delta

    n = len(acc)
    keys_out = np.empty(n, dtype=np.int64)
    deltas_out = np.empty(n, dtype=np.float64)
    i = 0
    for k, v in acc.items():  # enumerate() over a Numba typed.Dict isn't supported in nopython mode
        keys_out[i] = k
        deltas_out[i] = v
        i += 1  # noqa: SIM113
    return keys_out, deltas_out


def integrate_frame(
    origin_m: tuple[float, float, float],
    endpoints_m: np.ndarray,
    is_hit: np.ndarray,
    *,
    resolution_m: float,
    max_range_m: float,
    l_occ: float,
    l_free: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Integrate one frame's rays (one common origin, many endpoints).

    ``endpoints_m`` is ``(N, 3)`` float64, ``is_hit`` is ``(N,)`` bool --
    ``True`` for a real sensor return (mark free along the ray, occupied at
    the endpoint), ``False`` for a no-return ray (mark free all the way to
    ``max_range_m``, per spec's max-range free-space policy; the endpoint
    itself is then expected to already be the max-range point along that
    ray's direction).

    Returns ``(keys, deltas)``: every voxel touched by *any* ray this
    frame, encoded via :func:`encode_key`, with the *net* (summed) log-odds
    delta from this frame's rays -- callers apply ``prior + delta`` then
    clamp, matching :class:`aeris.mapping.voxel.VoxelMap`'s own semantics.
    """
    ox, oy, oz = origin_m
    ex = np.ascontiguousarray(endpoints_m[:, 0], dtype=np.float64)
    ey = np.ascontiguousarray(endpoints_m[:, 1], dtype=np.float64)
    ez = np.ascontiguousarray(endpoints_m[:, 2], dtype=np.float64)
    hit = np.ascontiguousarray(is_hit, dtype=np.bool_)
    return _integrate_frame_jit(
        ox, oy, oz, ex, ey, ez, hit, resolution_m, max_range_m, l_occ, l_free, OFFSET, SPAN
    )


def warm_up() -> None:
    """Forces Numba's JIT compilation now (~0.5s, one-time) rather than on
    the first real frame -- call once at startup, outside any timed loop."""
    integrate_frame(
        (0.0, 0.0, 0.0),
        np.array([[1.0, 0.0, 0.0]]),
        np.array([True]),
        resolution_m=0.2,
        max_range_m=15.0,
        l_occ=0.85,
        l_free=-0.4,
    )

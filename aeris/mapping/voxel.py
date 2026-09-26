"""Sparse, block-hashed 3D voxel log-odds occupancy map (spec §21.1, ADR-0009).

Storage: a ``dict`` keyed by block coordinate (``ix // 8, iy // 8, iz //
8``), each value an ``8x8x8`` ``float32`` NumPy array of log-odds, lazily
allocated on first touch. A voxel that has never been touched is ``nan``
(distinct from a touched-but-net-zero voxel, which is a real ``0.0`` --
using ``0.0`` as the "never touched" sentinel would make the two
indistinguishable) -- :meth:`VoxelMap.is_occupied` returns ``None`` exactly
for ``nan`` voxels.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from aeris.core.frames.vector import Vec3
from aeris.mapping.raycast import decode_key, integrate_frame

BLOCK_SIZE = 8
Block = tuple[int, int, int]


@dataclass(frozen=True, slots=True)
class MappingConfig:
    """spec §21.1's configured parameters (defaults per spec's own text)."""

    resolution_m: float = 0.2  # spec: "0.15-0.25 m (configured; default 0.2 m)"
    l_occ: float = 0.85
    l_free: float = -0.4
    l_min: float = -4.0
    l_max: float = 4.0
    max_range_m: float = 15.0
    occupied_threshold: float = 0.0  # log-odds > this => occupied


@dataclass(slots=True)
class VoxelMap:
    """A live-buildable, sparse voxel log-odds map in frame ``M`` (spec §21.1)."""

    config: MappingConfig = field(default_factory=MappingConfig)
    _blocks: dict[Block, np.ndarray] = field(default_factory=dict)

    def _voxel_index(self, p: Vec3) -> tuple[int, int, int]:
        r = self.config.resolution_m
        return (math.floor(p.x / r), math.floor(p.y / r), math.floor(p.z / r))

    def voxel_center(self, index: tuple[int, int, int]) -> Vec3:
        r = self.config.resolution_m
        ix, iy, iz = index
        return Vec3((ix + 0.5) * r, (iy + 0.5) * r, (iz + 0.5) * r)

    def log_odds_at_index(self, index: tuple[int, int, int]) -> float:
        """``0.0`` (the neutral prior) if unblocked/untouched -- distinct
        from :meth:`is_occupied_at_index`, which reports unknown as
        ``None`` rather than folding it into a numeric 0.0."""
        ix, iy, iz = index
        block = self._blocks.get((ix // BLOCK_SIZE, iy // BLOCK_SIZE, iz // BLOCK_SIZE))
        if block is None:
            return 0.0
        v = float(block[ix % BLOCK_SIZE, iy % BLOCK_SIZE, iz % BLOCK_SIZE])
        return 0.0 if math.isnan(v) else v

    def is_occupied_at_index(self, index: tuple[int, int, int]) -> bool | None:
        """``None`` = never touched (unknown); otherwise per ``occupied_threshold``."""
        ix, iy, iz = index
        block = self._blocks.get((ix // BLOCK_SIZE, iy // BLOCK_SIZE, iz // BLOCK_SIZE))
        if block is None:
            return None
        v = float(block[ix % BLOCK_SIZE, iy % BLOCK_SIZE, iz % BLOCK_SIZE])
        if math.isnan(v):
            return None
        return v > self.config.occupied_threshold

    def log_odds_at(self, p: Vec3) -> float:
        return self.log_odds_at_index(self._voxel_index(p))

    def is_occupied(self, p: Vec3) -> bool | None:
        return self.is_occupied_at_index(self._voxel_index(p))

    def touched_block_count(self) -> int:
        return len(self._blocks)

    def _apply_index_delta(self, ix: int, iy: int, iz: int, delta: float) -> None:
        bkey = (ix // BLOCK_SIZE, iy // BLOCK_SIZE, iz // BLOCK_SIZE)
        block = self._blocks.get(bkey)
        if block is None:
            block = np.full((BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE), np.nan, dtype=np.float32)
            self._blocks[bkey] = block
        lx, ly, lz = ix % BLOCK_SIZE, iy % BLOCK_SIZE, iz % BLOCK_SIZE
        cur = block[lx, ly, lz]
        prior = 0.0 if math.isnan(cur) else float(cur)
        block[lx, ly, lz] = min(max(prior + delta, self.config.l_min), self.config.l_max)

    def integrate_ray(self, origin_m: Vec3, endpoint_m: Vec3, *, is_hit: bool) -> None:
        """Integrate a single ray -- free along it, occupied at the
        endpoint if ``is_hit`` (else free all the way to the endpoint,
        which the caller is responsible for placing at ``max_range_m``
        along the ray's direction for a genuine no-return case)."""
        endpoints = np.array([[endpoint_m.x, endpoint_m.y, endpoint_m.z]], dtype=np.float64)
        hits = np.array([is_hit], dtype=np.bool_)
        keys, deltas = integrate_frame(
            (origin_m.x, origin_m.y, origin_m.z),
            endpoints,
            hits,
            resolution_m=self.config.resolution_m,
            max_range_m=self.config.max_range_m,
            l_occ=self.config.l_occ,
            l_free=self.config.l_free,
        )
        for key, delta in zip(keys.tolist(), deltas.tolist(), strict=True):
            ix, iy, iz = decode_key(key)
            self._apply_index_delta(ix, iy, iz, delta)

    def integrate_points(
        self, origin_m: Vec3, points_m: list[Vec3], *, is_hit: list[bool] | None = None
    ) -> None:
        """Integrate one frame's worth of rays sharing ``origin_m`` (spec
        §21.2's pipeline: "ray integration"). ``is_hit`` defaults to all
        ``True`` (every point a real return) -- the common case for a
        depth camera, whose invalid/no-return pixels are already dropped
        upstream by :func:`aeris.perception.depth.projection.depth_to_points_camera`
        (a documented Phase 11 scope limitation: the max-range free-space
        policy for no-return *camera* pixels is implemented and unit
        tested, but not exercised end-to-end by the live depth pipeline
        this phase, which never sees a no-return pixel in the first
        place)."""
        if not points_m:
            return
        endpoints = np.array([[p.x, p.y, p.z] for p in points_m], dtype=np.float64)
        hits = np.array(is_hit if is_hit is not None else [True] * len(points_m), dtype=np.bool_)
        keys, deltas = integrate_frame(
            (origin_m.x, origin_m.y, origin_m.z),
            endpoints,
            hits,
            resolution_m=self.config.resolution_m,
            max_range_m=self.config.max_range_m,
            l_occ=self.config.l_occ,
            l_free=self.config.l_free,
        )
        for key, delta in zip(keys.tolist(), deltas.tolist(), strict=True):
            ix, iy, iz = decode_key(key)
            self._apply_index_delta(ix, iy, iz, delta)

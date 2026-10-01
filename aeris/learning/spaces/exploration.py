"""Learned-exploration observation/action spaces (spec §26.2, §30, ADR-0015),
shared verbatim by FastSim training and Tier H deployment.

**Map interchange.** Tier H's agent map is a :class:`~aeris.mapping.projection.BandGrid`
(frozensets of cells); FastSim keeps a dense array for speed. Both reduce to
:class:`DenseAgentMap` before anything here touches them, and the action
mask/decoder go back through ``BandGrid`` so they reuse *the same*
:mod:`aeris.autonomy.exploration.base` geometry and
:func:`aeris.autonomy.planning.astar.astar` reachability check the classical
baselines use -- the learned policy's action set is exactly as constrained,
and as safe, as theirs (ADR-0015).

**Observation (spec §30).** Two heading-aligned egocentric crops of the
agent's own map (64x64 at 0.2 m, 64x64 at 0.8 m), channels
free/occupied/unknown/visited-trail per scale, plus low-dim time remaining,
speed and fraction explored. The coarse scale is max-pooled from the fine
map in absolute-index-aligned 4x4 blocks (occupied if any cell occupied,
else free if any free, else unknown) rather than point-sampled, so a
single occupied fine cell is never dropped by the coarse view. The
frontier channel is an ablation factor (spec §42 A3), not in v1.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from aeris.autonomy.exploration.base import (
    AgentPose,
    egocentric_candidates,
    snap_to_nearest_free_cell,
    world_to_cell,
)
from aeris.autonomy.planning.astar import astar
from aeris.core.types import Provenance
from aeris.learning.spaces.spec import FieldSpec, ObservationSpec
from aeris.mapping.projection import BandGrid, Cell2D

SPEC_VERSION = "exploration/v1"
N_ACTIONS = 25
ROTATE_ACTION = 24
UNKNOWN, FREE, OCCUPIED = 0, 1, 2


@dataclass(frozen=True, slots=True)
class ExplorationSpaceConfig:
    crop_size: int = 64
    fine_res_m: float = 0.2
    coarse_res_m: float = 0.8
    v_max_mps: float = 2.0
    episode_time_s: float = 180.0
    explored_area_norm_m2: float = 400.0


def exploration_spec(cfg: ExplorationSpaceConfig) -> ObservationSpec:
    return ObservationSpec(
        name="exploration",
        version=SPEC_VERSION,
        fields=(
            FieldSpec(
                "map",
                (8, cfg.crop_size, cfg.crop_size),
                0.0,
                1.0,
                Provenance.ESTIMATE,
                "agent's own map (sensor data registered with the estimated pose): "
                "[free, occupied, unknown, visited] x [fine, coarse], heading-aligned",
            ),
            FieldSpec(
                "state",
                (3,),
                0.0,
                1.0,
                Provenance.ESTIMATE,
                "time remaining (MISSION_INPUT budget), speed / v_max, explored area / norm",
            ),
        ),
    )


@dataclass(slots=True)
class DenseAgentMap:
    """``state[i, j]`` is the cell ``(min_cell[0] + i, min_cell[1] + j)``."""

    state: np.ndarray  # int8 [NX, NY]
    visited: np.ndarray  # bool [NX, NY]
    min_cell: tuple[int, int]
    resolution_m: float

    @staticmethod
    def from_bandgrid(
        grid: BandGrid, visited_cells: frozenset[Cell2D] = frozenset()
    ) -> DenseAgentMap:
        (x0, y0), (x1, y1) = grid.min_cell, grid.max_cell
        st = np.zeros((x1 - x0 + 1, y1 - y0 + 1), dtype=np.int8)
        vis = np.zeros(st.shape, dtype=np.bool_)
        for cx, cy in grid.free:
            if grid.in_bounds((cx, cy)):
                st[cx - x0, cy - y0] = FREE
        for cx, cy in grid.occupied:
            if grid.in_bounds((cx, cy)):
                st[cx - x0, cy - y0] = OCCUPIED
        for cx, cy in visited_cells:
            if grid.in_bounds((cx, cy)):
                vis[cx - x0, cy - y0] = True
        return DenseAgentMap(st, vis, (x0, y0), grid.resolution_m)

    def to_bandgrid(self) -> BandGrid:
        x0, y0 = self.min_cell
        occ = np.argwhere(self.state == OCCUPIED)
        free = np.argwhere(self.state == FREE)
        return BandGrid(
            resolution_m=self.resolution_m,
            min_cell=(x0, y0),
            max_cell=(x0 + self.state.shape[0] - 1, y0 + self.state.shape[1] - 1),
            occupied=frozenset((int(i) + x0, int(j) + y0) for i, j in occ),
            free=frozenset((int(i) + x0, int(j) + y0) for i, j in free),
        )

    def known_area_m2(self) -> float:
        return float(np.count_nonzero(self.state != UNKNOWN)) * self.resolution_m**2


def _pooled(m: DenseAgentMap, k: int) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    """Absolute-index-aligned k x k max-pool: returns (state, visited, coarse min_cell)."""
    x0, y0 = m.min_cell
    cx0, cy0 = math.floor(x0 / k), math.floor(y0 / k)
    px, py = x0 - cx0 * k, y0 - cy0 * k
    nx = math.ceil((px + m.state.shape[0]) / k)
    ny = math.ceil((py + m.state.shape[1]) / k)
    st = np.zeros((nx * k, ny * k), dtype=np.int8)
    vis = np.zeros(st.shape, dtype=np.bool_)
    st[px : px + m.state.shape[0], py : py + m.state.shape[1]] = m.state
    vis[px : px + m.state.shape[0], py : py + m.state.shape[1]] = m.visited
    blocks = st.reshape(nx, k, ny, k)
    any_occ = (blocks == OCCUPIED).any(axis=(1, 3))
    any_free = (blocks == FREE).any(axis=(1, 3))
    pooled = np.where(any_occ, OCCUPIED, np.where(any_free, FREE, UNKNOWN)).astype(np.int8)
    pvis = vis.reshape(nx, k, ny, k).any(axis=(1, 3))
    return pooled, pvis, (cx0, cy0)


def _crop(
    st: np.ndarray,
    vis: np.ndarray,
    min_cell: tuple[int, int],
    res: float,
    pose_xy: tuple[float, float],
    yaw: float,
    size: int,
) -> np.ndarray:
    """Heading-aligned crop: row 0 is farthest ahead, column 0 farthest left."""
    r = np.arange(size)
    fwd = (size / 2.0 - r - 0.5) * res
    left = (size / 2.0 - r - 0.5) * res
    f, lft = np.meshgrid(fwd, left, indexing="ij")
    c, s = math.cos(yaw), math.sin(yaw)
    wx = pose_xy[0] + c * f - s * lft
    wy = pose_xy[1] + s * f + c * lft
    ix = np.floor(wx / res).astype(np.int64) - min_cell[0]
    iy = np.floor(wy / res).astype(np.int64) - min_cell[1]
    inside = (ix >= 0) & (iy >= 0) & (ix < st.shape[0]) & (iy < st.shape[1])
    ixc, iyc = np.clip(ix, 0, st.shape[0] - 1), np.clip(iy, 0, st.shape[1] - 1)
    cell = np.where(inside, st[ixc, iyc], UNKNOWN)
    v = np.where(inside, vis[ixc, iyc], False)
    return np.stack([cell == FREE, cell == OCCUPIED, cell == UNKNOWN, v]).astype(np.float32)


def build_observation(
    cfg: ExplorationSpaceConfig,
    *,
    agent_map: DenseAgentMap,
    pose_xy: tuple[float, float],
    yaw: float,
    speed_mps: float,
    time_remaining_s: float,
) -> dict[str, np.ndarray]:
    if abs(agent_map.resolution_m - cfg.fine_res_m) > 1e-9:
        raise ValueError("agent map resolution must equal the spec's fine_res_m")
    k = round(cfg.coarse_res_m / cfg.fine_res_m)
    fine = _crop(
        agent_map.state,
        agent_map.visited,
        agent_map.min_cell,
        cfg.fine_res_m,
        pose_xy,
        yaw,
        cfg.crop_size,
    )
    pst, pvis, pmin = _pooled(agent_map, k)
    coarse = _crop(pst, pvis, pmin, cfg.coarse_res_m, pose_xy, yaw, cfg.crop_size)
    state = np.array(
        [
            np.clip(time_remaining_s / cfg.episode_time_s, 0.0, 1.0),
            np.clip(speed_mps / cfg.v_max_mps, 0.0, 1.0),
            np.clip(agent_map.known_area_m2() / cfg.explored_area_norm_m2, 0.0, 1.0),
        ],
        dtype=np.float32,
    )
    return {"map": np.concatenate([fine, coarse]), "state": state}


def candidate_targets(
    grid: BandGrid, pose: AgentPose, *, unknown_cost_multiplier: float = 1.0
) -> list[Cell2D | None]:
    """For each of the 24 egocentric actions (bearing-major, ADR-0015's
    ordering from :func:`egocentric_candidates`), the reachable free cell it
    snaps to, or ``None`` if it's masked. The rotate action (index 24) is
    never masked and has no target cell.

    A* starts from the raw cell under the pose, not a snapped one -- the
    same start :meth:`ExplorationEnv.execute_subgoal` and Tier H's loop
    plan from -- so an unmasked action is exactly one execution can plan.
    (When that cell is inflated-occupied every move is masked and only the
    rotate scan remains, which is Tier H's own fallback there.)"""
    start = world_to_cell(grid, pose.position_m)
    out: list[Cell2D | None] = []
    for cand in egocentric_candidates(pose):
        cell = snap_to_nearest_free_cell(grid, cand)
        if cell is None or cell == start:
            out.append(None)
            continue
        ok = (
            astar(grid, start, cell, unknown_cost_multiplier=unknown_cost_multiplier).reason == "ok"
        )
        out.append(cell if ok else None)
    return out


def action_mask(targets: list[Cell2D | None]) -> np.ndarray:
    mask = np.zeros(N_ACTIONS, dtype=np.bool_)
    mask[:24] = [t is not None for t in targets]
    mask[ROTATE_ACTION] = True
    return mask

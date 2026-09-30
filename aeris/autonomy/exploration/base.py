"""Shared exploration types (spec §51 Phase 12 task 2): the ``Subgoal``
value type and the ``ExplorationStrategy`` interface every classical
baseline implements -- and that Phase 19's learned exploration policy
will implement too (spec's own line: "the subgoal interface shared with
the future learned policy"), so a later phase can swap in a learned
strategy without touching the episode runner or the other baselines.

Also holds the egocentric candidate-subgoal geometry from ADR-0015 (12
bearings x 2 ranges + an implicit "rotate in place" null option = 25
actions) -- ADR-0015 itself is scoped to Phase 19's *learned, masked*
version of this action space, but spec's own "Random" baseline
definition (§25) requires sampling from "the same egocentric action
space as the learned policy" now, so the geometry is built once here and
reused by :mod:`random`, not duplicated ahead of Phase 19.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

from aeris.core.frames.vector import Vec3
from aeris.mapping.projection import BandGrid, Cell2D, CellState

N_BEARINGS = 12
RANGE_OPTIONS_M: tuple[float, float] = (2.0, 5.0)


@dataclass(frozen=True, slots=True)
class AgentPose:
    """The agent's own (possibly EKF-estimated) pose, in the map's own frame."""

    position_m: Vec3
    yaw_rad: float


@dataclass(frozen=True, slots=True)
class Subgoal:
    cell: Cell2D
    position_m: Vec3


class ExplorationStrategy(Protocol):
    def select_subgoal(self, grid: BandGrid, pose: AgentPose, t_s: float) -> Subgoal | None:
        """Choose the next subgoal, or ``None`` when nothing reachable
        remains to explore (an in-place scan is the caller's fallback)."""
        ...


def world_to_cell(grid: BandGrid, position_m: Vec3) -> Cell2D:
    res = grid.resolution_m
    return (math.floor(position_m.x / res), math.floor(position_m.y / res))


def egocentric_candidates(pose: AgentPose) -> list[Vec3]:
    """The 12-bearing x 2-range egocentric candidate world positions
    (ADR-0015's action geometry), at the agent's own altitude."""
    candidates: list[Vec3] = []
    for i in range(N_BEARINGS):
        bearing = pose.yaw_rad + 2.0 * math.pi * i / N_BEARINGS
        for r in RANGE_OPTIONS_M:
            dx, dy = r * math.cos(bearing), r * math.sin(bearing)
            candidates.append(
                Vec3(pose.position_m.x + dx, pose.position_m.y + dy, pose.position_m.z)
            )
    return candidates


def nearest_cell_to_point(
    cells: frozenset[Cell2D], point_xy: tuple[float, float], grid: BandGrid
) -> Cell2D:
    """The cell in ``cells`` (assumed non-empty) whose center is nearest
    ``point_xy`` -- used to turn a :class:`~aeris.mapping.frontier.FrontierCluster`'s
    ``centroid_xy`` (which may not itself be a cell in the cluster) into a
    concrete, drivable target cell."""
    px, py = point_xy
    return min(
        cells,
        key=lambda c: (grid.cell_center_xy(c)[0] - px) ** 2 + (grid.cell_center_xy(c)[1] - py) ** 2,
    )


def snap_to_nearest_free_cell(
    grid: BandGrid, position_m: Vec3, *, max_search_radius_cells: int = 5
) -> Cell2D | None:
    """The true nearest (by cell-center Euclidean distance) ``FREE`` cell
    to ``position_m``, among cells within a ``max_search_radius_cells``
    square window -- ``None`` if none is found that close."""
    origin = world_to_cell(grid, position_m)
    best: Cell2D | None = None
    best_dist_sq = math.inf
    for dx in range(-max_search_radius_cells, max_search_radius_cells + 1):
        for dy in range(-max_search_radius_cells, max_search_radius_cells + 1):
            cell = (origin[0] + dx, origin[1] + dy)
            if not grid.in_bounds(cell) or grid.state_at(cell) != CellState.FREE:
                continue
            cx, cy = grid.cell_center_xy(cell)
            dist_sq = (cx - position_m.x) ** 2 + (cy - position_m.y) ** 2
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best = cell
    return best

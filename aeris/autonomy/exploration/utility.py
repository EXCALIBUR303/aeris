"""Utility-frontier exploration (spec §25): the strong classical baseline
for RQ1. Score = expected information gain / (path length + lambda *
turn cost); greedy, replanning is the episode runner's job (on arrival or
a significant map change), not this strategy's.

Information gain is estimated from the agent's own map only (no
privileged ground truth): a full-circle raycast from each frontier
candidate over the agent's own :class:`~aeris.mapping.projection.BandGrid`,
counting distinct ``UNKNOWN`` cells reached before a ray is blocked by an
``OCCUPIED`` one. A full circle (not the sensor's actual FOV, and not
committing to any particular arrival heading) is a deliberate
simplification -- the candidate's eventual facing direction on arrival
isn't decided by this strategy, so estimating gain independent of heading
avoids silently favoring one arbitrary assumed heading over another.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from aeris.autonomy.exploration.base import (
    AgentPose,
    Subgoal,
    nearest_cell_to_point,
    snap_to_nearest_free_cell,
    world_to_cell,
)
from aeris.autonomy.planning.astar import astar
from aeris.mapping.frontier import detect_frontiers
from aeris.mapping.projection import BandGrid, Cell2D, CellState, cell_center_world


def _wrap_to_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _estimate_unknown_gain(
    grid: BandGrid, xy: tuple[float, float], *, max_range_m: float, n_rays: int
) -> int:
    x0, y0 = xy
    res = grid.resolution_m
    n_steps = max(1, int(max_range_m / res))
    seen_unknown: set[Cell2D] = set()
    for i in range(n_rays):
        angle = 2.0 * math.pi * i / n_rays
        dx, dy = math.cos(angle), math.sin(angle)
        for step in range(1, n_steps + 1):
            r = step * res
            cell = (math.floor((x0 + dx * r) / res), math.floor((y0 + dy * r) / res))
            if not grid.in_bounds(cell):
                break
            state = grid.state_at(cell)
            if state == CellState.OCCUPIED:
                break
            if state == CellState.UNKNOWN:
                seen_unknown.add(cell)
    return len(seen_unknown)


@dataclass(frozen=True, slots=True)
class UtilityFrontierExploration:
    sensor_max_range_m: float = 15.0
    lambda_turn: float = 1.0
    # See NearestFrontierExploration's identical field for why 1, not a
    # noise-filtering 2+: live-tested at 2 and found it starved early
    # exploration entirely when only single-cell frontiers existed yet.
    min_cluster_size: int = 1
    unknown_cost_multiplier: float = 1.0
    n_gain_rays: int = 24

    def select_subgoal(self, grid: BandGrid, pose: AgentPose, t_s: float) -> Subgoal | None:
        clusters = detect_frontiers(grid, min_cluster_size=self.min_cluster_size)
        if not clusters:
            return None

        start_cell = world_to_cell(grid, pose.position_m)
        if grid.state_at(start_cell) != CellState.FREE:
            snapped = snap_to_nearest_free_cell(grid, pose.position_m)
            if snapped is None:
                return None
            start_cell = snapped

        best_cell: Cell2D | None = None
        best_score = float("-inf")
        for cluster in clusters:
            # See NearestFrontierExploration's identical exclusion: the
            # agent's own current cell can itself be a frontier (even a
            # whole cluster's centroid can coincide with it, for a
            # symmetric shape) -- excluded from the candidate pool before
            # picking the nearest-to-centroid cell, not after, so the
            # cluster's other real cells are never thrown away with it.
            candidate_cells = cluster.cells - {start_cell}
            if not candidate_cells:
                continue
            target_cell = nearest_cell_to_point(candidate_cells, cluster.centroid_xy, grid)
            result = astar(
                grid, start_cell, target_cell, unknown_cost_multiplier=self.unknown_cost_multiplier
            )
            if result.reason != "ok":
                continue

            candidate_xy = grid.cell_center_xy(target_cell)
            gain = _estimate_unknown_gain(
                grid, candidate_xy, max_range_m=self.sensor_max_range_m, n_rays=self.n_gain_rays
            )
            bearing = math.atan2(
                candidate_xy[1] - pose.position_m.y, candidate_xy[0] - pose.position_m.x
            )
            turn_cost = abs(_wrap_to_pi(bearing - pose.yaw_rad))
            denom = result.path_length_m + self.lambda_turn * turn_cost
            score = gain / denom if denom > 0 else float(gain)
            if score > best_score:
                best_score = score
                best_cell = target_cell

        if best_cell is None:
            return None
        return Subgoal(
            cell=best_cell, position_m=cell_center_world(grid, best_cell, z_m=pose.position_m.z)
        )

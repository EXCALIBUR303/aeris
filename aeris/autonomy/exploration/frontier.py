"""Nearest-frontier exploration (spec §25, Yamauchi 1997): the standard
classical baseline. Goes to the reachable frontier cluster with the
shortest A* path, not the geometrically nearest centroid -- spec's own
wording ("nearest reachable cluster centroid (A* path length)")."""

from __future__ import annotations

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
from aeris.mapping.projection import BandGrid, CellState, cell_center_world


@dataclass(frozen=True, slots=True)
class NearestFrontierExploration:
    # A lone free cell bordering unknown space is a genuine frontier by
    # Yamauchi's own definition, not noise -- live-tested at 2 (this
    # class's earlier default) and found it starved early exploration
    # entirely: right after takeoff, with only a small observed area, the
    # only frontiers that exist yet are frequently single cells at the
    # observed area's own ragged edge, and filtering all of them out left
    # the strategy with nothing to select for the whole episode.
    min_cluster_size: int = 1
    unknown_cost_multiplier: float = 1.0

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

        best_cell = None
        best_length_m = float("inf")
        for cluster in clusters:
            # The agent's own current cell can itself be a frontier (it
            # borders unknown space -- the literal definition of standing
            # at one), and for a cluster shaped symmetrically around the
            # agent its centroid can even coincide with the agent's own
            # cell -- excluded from the candidate pool *before* picking
            # the nearest-to-centroid cell, so a cluster that merely
            # touches the agent's position doesn't lose its other,
            # perfectly real cells too.
            candidate_cells = cluster.cells - {start_cell}
            if not candidate_cells:
                continue
            target_cell = nearest_cell_to_point(candidate_cells, cluster.centroid_xy, grid)
            result = astar(
                grid, start_cell, target_cell, unknown_cost_multiplier=self.unknown_cost_multiplier
            )
            if result.reason == "ok" and result.path_length_m < best_length_m:
                best_length_m = result.path_length_m
                best_cell = target_cell

        if best_cell is None:
            return None
        return Subgoal(
            cell=best_cell, position_m=cell_center_world(grid, best_cell, z_m=pose.position_m.z)
        )

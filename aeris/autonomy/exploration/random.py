"""Random exploration (spec §25): the lower-bound / H0.1 sanity baseline.

Samples uniformly from the currently-*reachable* subset of the same
egocentric candidate geometry the future learned policy uses (ADR-0015,
:mod:`base`'s ``egocentric_candidates``), plus the same "rotate in place"
null option counted as one more equally-likely slot -- a candidate is
reachable if it snaps to a known ``FREE`` cell that A* can actually path
to from the agent's current cell. The rotate option is always in the
sampling pool (not just a last-resort fallback when nothing else is
reachable): a "random" baseline that only ever rotates when truly stuck
would be an artificially easy target for the frontier baselines to beat,
understating H0.1's actual effect size.
"""

from __future__ import annotations

import random as _random
from dataclasses import dataclass, field

from aeris.autonomy.exploration.base import (
    AgentPose,
    Subgoal,
    egocentric_candidates,
    snap_to_nearest_free_cell,
    world_to_cell,
)
from aeris.autonomy.planning.astar import astar
from aeris.mapping.projection import BandGrid, Cell2D, CellState, cell_center_world


@dataclass(slots=True)
class RandomExploration:
    rng: _random.Random = field(default_factory=_random.Random)
    unknown_cost_multiplier: float = 1.0

    def select_subgoal(self, grid: BandGrid, pose: AgentPose, t_s: float) -> Subgoal | None:
        start_cell = world_to_cell(grid, pose.position_m)
        if grid.state_at(start_cell) != CellState.FREE:
            snapped = snap_to_nearest_free_cell(grid, pose.position_m)
            if snapped is None:
                return None
            start_cell = snapped

        reachable: list[Cell2D] = []
        for candidate_xy in egocentric_candidates(pose):
            cell = snap_to_nearest_free_cell(grid, candidate_xy)
            if cell is None or cell == start_cell:
                # A candidate that snaps back to the agent's own current
                # cell (plausible near the map's edge, where most of the
                # egocentric ring has nothing else nearby to snap to) is
                # never a meaningful "go somewhere" choice -- excluded
                # rather than padding the pool with a no-op duplicate.
                continue
            result = astar(
                grid, start_cell, cell, unknown_cost_multiplier=self.unknown_cost_multiplier
            )
            if result.reason == "ok":
                reachable.append(cell)

        pool: list[Cell2D | None] = [*reachable, None]  # the rotate-in-place slot
        choice = self.rng.choice(pool)
        if choice is None:
            return None
        return Subgoal(
            cell=choice, position_m=cell_center_world(grid, choice, z_m=pose.position_m.z)
        )

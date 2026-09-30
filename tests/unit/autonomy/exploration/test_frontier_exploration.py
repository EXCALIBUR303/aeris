"""Unit tests for :mod:`aeris.autonomy.exploration.frontier`."""

from __future__ import annotations

from aeris.autonomy.exploration.base import AgentPose
from aeris.autonomy.exploration.frontier import NearestFrontierExploration
from aeris.core.frames.vector import Vec3
from aeris.mapping.projection import BandGrid

_RES = 1.0
_MIN_CELL = (-10, -10)
_MAX_CELL = (10, 10)


def _grid(*, occupied: set = frozenset(), free: set = frozenset()) -> BandGrid:
    return BandGrid(
        resolution_m=_RES,
        min_cell=_MIN_CELL,
        max_cell=_MAX_CELL,
        occupied=frozenset(occupied),
        free=frozenset(free),
    )


def test_selects_a_reachable_frontier() -> None:
    free = {(x, 0) for x in range(0, 6)}  # (5, 0) borders unknown -> a frontier
    grid = _grid(free=free)
    strategy = NearestFrontierExploration(min_cluster_size=1)
    pose = AgentPose(position_m=Vec3(0.5, 0.5, 1.0), yaw_rad=0.0)
    subgoal = strategy.select_subgoal(grid, pose, 0.0)
    assert subgoal is not None
    assert subgoal.cell != (0, 0)  # never "go to where you already are"


def test_returns_none_when_there_are_no_frontiers() -> None:
    # A free region exactly matching the grid's own window: every boundary
    # cell is excluded from frontier status by its own "touches the
    # window's edge" rule, and every interior cell's neighbors are all
    # FREE -- so no cell anywhere qualifies.
    free = {
        (x, y)
        for x in range(_MIN_CELL[0], _MAX_CELL[0] + 1)
        for y in range(_MIN_CELL[1], _MAX_CELL[1] + 1)
    }
    grid = _grid(free=free)
    strategy = NearestFrontierExploration(min_cluster_size=1)
    pose = AgentPose(position_m=Vec3(0.5, 0.5, 1.0), yaw_rad=0.0)
    assert strategy.select_subgoal(grid, pose, 0.0) is None


def test_returns_none_when_the_only_frontier_is_truly_sealed_off() -> None:
    # A wall spanning the grid's *entire* window height, so there is no
    # detour around it (even through UNKNOWN space) within the finite
    # queried window -- the only real test of "unreachable," since a
    # finite-height wall alone doesn't block a detour around its ends.
    wall = {(1, y) for y in range(_MIN_CELL[1], _MAX_CELL[1] + 1)}
    agent_room = {(0, 0)}
    sealed_frontier = {(2, 0)}
    grid = _grid(free=agent_room | sealed_frontier, occupied=wall)
    strategy = NearestFrontierExploration(min_cluster_size=1)
    pose = AgentPose(position_m=Vec3(0.5, 0.5, 1.0), yaw_rad=0.0)
    assert strategy.select_subgoal(grid, pose, 0.0) is None


def test_never_selects_the_agents_own_cell_even_when_it_is_the_only_frontier() -> None:
    # The agent's own current cell borders unknown space (it's standing
    # right at the frontier) and no other frontier exists anywhere --
    # this must return None, not the degenerate zero-length "subgoal" of
    # the agent's own position.
    grid = _grid(free={(0, 0)})
    strategy = NearestFrontierExploration(min_cluster_size=1)
    pose = AgentPose(position_m=Vec3(0.5, 0.5, 1.0), yaw_rad=0.0)
    assert strategy.select_subgoal(grid, pose, 0.0) is None

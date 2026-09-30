"""Unit tests for :mod:`aeris.autonomy.exploration.utility`."""

from __future__ import annotations

from aeris.autonomy.exploration.base import AgentPose
from aeris.autonomy.exploration.utility import UtilityFrontierExploration, _estimate_unknown_gain
from aeris.core.frames.vector import Vec3
from aeris.mapping.projection import BandGrid

_RES = 1.0
_MIN_CELL = (-15, -15)
_MAX_CELL = (15, 15)


def _grid(*, occupied: set = frozenset(), free: set = frozenset()) -> BandGrid:
    return BandGrid(
        resolution_m=_RES,
        min_cell=_MIN_CELL,
        max_cell=_MAX_CELL,
        occupied=frozenset(occupied),
        free=frozenset(free),
    )


def test_estimate_unknown_gain_counts_zero_in_a_fully_free_region() -> None:
    free = {(x, y) for x in range(-5, 6) for y in range(-5, 6)}
    grid = _grid(free=free)
    gain = _estimate_unknown_gain(grid, (0.5, 0.5), max_range_m=5.0, n_rays=16)
    assert gain == 0


def test_estimate_unknown_gain_is_positive_next_to_unknown_space() -> None:
    grid = _grid(free={(0, 0)})
    gain = _estimate_unknown_gain(grid, (0.5, 0.5), max_range_m=5.0, n_rays=16)
    assert gain > 0


def test_estimate_unknown_gain_stops_at_an_occupied_cell() -> None:
    """A wall one cell away in every direction should make the gain
    exactly 0 -- no ray can reach past it to any unknown cell beyond."""
    ring = {(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)}
    grid = _grid(free={(0, 0)}, occupied=ring)
    gain = _estimate_unknown_gain(grid, (0.5, 0.5), max_range_m=5.0, n_rays=16)
    assert gain == 0


def test_selects_a_reachable_frontier_and_never_its_own_cell() -> None:
    # (0,0) is itself a frontier (isolated, bordering unknown) but must be
    # excluded; (5,0) is a separate isolated frontier cell, reachable via
    # A*'s default willingness to traverse unknown space.
    grid = _grid(free={(0, 0), (5, 0)})
    strategy = UtilityFrontierExploration(min_cluster_size=1, sensor_max_range_m=5.0)
    pose = AgentPose(position_m=Vec3(0.5, 0.5, 1.0), yaw_rad=0.0)
    subgoal = strategy.select_subgoal(grid, pose, 0.0)
    assert subgoal is not None
    assert subgoal.cell == (5, 0)


def test_returns_none_when_there_are_no_frontiers() -> None:
    free = {
        (x, y)
        for x in range(_MIN_CELL[0], _MAX_CELL[0] + 1)
        for y in range(_MIN_CELL[1], _MAX_CELL[1] + 1)
    }
    grid = _grid(free=free)
    strategy = UtilityFrontierExploration(min_cluster_size=1)
    pose = AgentPose(position_m=Vec3(0.5, 0.5, 1.0), yaw_rad=0.0)
    assert strategy.select_subgoal(grid, pose, 0.0) is None


def test_prefers_the_higher_information_gain_candidate_over_a_boxed_in_one() -> None:
    """Two isolated candidate cells -- one surrounded by occupied cells on
    7 of its 8 neighbors (a single narrow gap keeps it A*-reachable, but
    sharply limits the unknown space actually visible from it), the other
    fully open. With turn cost disabled (lambda_turn=0), utility scoring
    must prefer the open one on gain (and, if anything, its path is also
    the shorter one, reinforcing the same preference)."""
    boxed_ring = {
        (-1, 2),
        (0, 2),
        (1, 2),
        (-1, 3),
        (1, 3),
        (-1, 4),
        (0, 4),
    }  # 7 of 8 neighbors of (0, 3)
    free = {(0, 0), (0, 3), (0, -3)}
    grid = _grid(free=free, occupied=boxed_ring)
    strategy = UtilityFrontierExploration(
        min_cluster_size=1, sensor_max_range_m=8.0, n_gain_rays=32, lambda_turn=0.0
    )
    pose = AgentPose(position_m=Vec3(0.5, 0.5, 1.0), yaw_rad=0.0)
    subgoal = strategy.select_subgoal(grid, pose, 0.0)
    assert subgoal is not None
    assert subgoal.cell == (0, -3)  # the open one, not the boxed-in (0, 3)


def test_turn_cost_penalizes_a_candidate_behind_the_agent() -> None:
    """Two isolated, equally-open (equal-gain) candidate cells at equal
    path length -- one straight ahead of the agent's current yaw, one
    directly behind it. With a large lambda_turn, the ahead candidate
    must win despite identical gain and path cost."""
    ahead = (3, 0)  # due +x -- yaw=0 points straight at it, turn cost 0
    behind = (-3, 0)  # due -x -- a full about-face, turn cost = pi
    grid = _grid(free={(0, 0), ahead, behind})
    strategy = UtilityFrontierExploration(
        min_cluster_size=1, sensor_max_range_m=8.0, n_gain_rays=32, lambda_turn=10.0
    )
    pose = AgentPose(position_m=Vec3(0.5, 0.5, 1.0), yaw_rad=0.0)
    subgoal = strategy.select_subgoal(grid, pose, 0.0)
    assert subgoal is not None
    assert subgoal.cell == ahead

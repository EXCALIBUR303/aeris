"""Unit tests for :mod:`aeris.autonomy.exploration.random`."""

from __future__ import annotations

import random as _random

from aeris.autonomy.exploration.base import AgentPose
from aeris.autonomy.exploration.random import RandomExploration
from aeris.core.frames.vector import Vec3
from aeris.mapping.projection import BandGrid

_RES = 1.0


def _open_grid(*, half_size: int = 15) -> BandGrid:
    free = {
        (x, y) for x in range(-half_size, half_size + 1) for y in range(-half_size, half_size + 1)
    }
    return BandGrid(
        resolution_m=_RES,
        min_cell=(-half_size, -half_size),
        max_cell=(half_size, half_size),
        occupied=frozenset(),
        free=frozenset(free),
    )


def test_selects_a_reachable_egocentric_candidate_in_open_space() -> None:
    grid = _open_grid()
    strategy = RandomExploration(rng=_random.Random(0))
    pose = AgentPose(position_m=Vec3(0.5, 0.5, 1.0), yaw_rad=0.0)
    subgoal = strategy.select_subgoal(grid, pose, 0.0)
    assert subgoal is not None
    assert grid.state_at(subgoal.cell).value == "free"


def test_can_choose_rotate_in_place_even_when_options_are_reachable() -> None:
    """Over many draws with a fully-open map, some fraction of choices
    must land on the rotate-in-place null option (spec: it's one of the
    25 equally-likely slots, not just a last-resort fallback)."""
    grid = _open_grid()
    strategy = RandomExploration(rng=_random.Random(1))
    pose = AgentPose(position_m=Vec3(0.5, 0.5, 1.0), yaw_rad=0.0)
    results = [strategy.select_subgoal(grid, pose, float(i)) for i in range(200)]
    assert any(r is None for r in results)
    assert any(r is not None for r in results)


def test_returns_none_when_nothing_reachable_is_close_by() -> None:
    # Agent's only free cell is fully isolated -- no candidate can snap
    # to a FREE cell within the default search radius, and there's no
    # other cell to path to either.
    grid = BandGrid(
        resolution_m=_RES,
        min_cell=(-20, -20),
        max_cell=(20, 20),
        occupied=frozenset(),
        free=frozenset({(0, 0)}),
    )
    strategy = RandomExploration(rng=_random.Random(2))
    pose = AgentPose(position_m=Vec3(0.5, 0.5, 1.0), yaw_rad=0.0)
    # With nothing else FREE anywhere, every egocentric candidate fails to
    # snap -- the only valid pool entry is the rotate-in-place option.
    subgoal = strategy.select_subgoal(grid, pose, 0.0)
    assert subgoal is None

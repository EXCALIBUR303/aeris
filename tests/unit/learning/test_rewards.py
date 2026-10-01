"""Reward functions v1 (spec §27.5) against hand-computed transitions."""

from __future__ import annotations

import numpy as np
import pytest

from aeris.learning.envs.rewards import (
    ExplorationRewardWeights,
    LocalNavRewardWeights,
    exploration_reward,
    local_nav_reward,
)

_W = LocalNavRewardWeights()  # progress 1.0, time 0.01, collision 10, shield 0.05, goal 10


def _ln(prev: float, now: float, *, col: bool = False, sh: bool = False, ok: bool = False) -> float:
    return float(
        local_nav_reward(
            _W,
            prev_geodesic_m=np.array([prev]),
            geodesic_m=np.array([now]),
            collided=np.array([col]),
            shield_intervened=np.array([sh]),
            success=np.array([ok]),
        )[0]
    )


def test_local_nav_progress_minus_time_cost() -> None:
    assert _ln(5.0, 4.8) == pytest.approx(0.2 - 0.01)


def test_local_nav_moving_away_is_penalized() -> None:
    assert _ln(4.8, 5.0) == pytest.approx(-0.2 - 0.01)


def test_local_nav_collision_and_shield_penalties_stack() -> None:
    assert _ln(5.0, 5.0, col=True, sh=True) == pytest.approx(-0.01 - 10.0 - 0.05)


def test_local_nav_success_bonus() -> None:
    assert _ln(0.6, 0.45, ok=True) == pytest.approx(0.15 - 0.01 + 10.0)


@pytest.mark.parametrize(("prev", "now"), [(np.inf, 3.0), (3.0, np.inf), (np.inf, np.inf)])
def test_local_nav_non_finite_geodesic_gives_zero_progress_not_inf(prev: float, now: float) -> None:
    assert _ln(prev, now) == pytest.approx(-0.01)


def test_local_nav_is_elementwise_over_a_batch() -> None:
    r = local_nav_reward(
        _W,
        prev_geodesic_m=np.array([5.0, 5.0, np.inf]),
        geodesic_m=np.array([4.0, 6.0, 1.0]),
        collided=np.array([False, True, False]),
        shield_intervened=np.array([True, False, False]),
        success=np.array([False, False, False]),
    )
    np.testing.assert_allclose(r, [1.0 - 0.01 - 0.05, -1.0 - 0.01 - 10.0, -0.01])


def test_exploration_reward_hand_transition() -> None:
    w = (
        ExplorationRewardWeights()
    )  # explore 0.1/m^2, time 0.01/s, collision 10, shield 0.01, failed 0.5
    r = exploration_reward(
        w,
        new_explored_m2=12.0,
        decision_duration_s=6.5,
        collided=False,
        shield_interventions=3,
        subgoal_failed=True,
    )
    assert r == pytest.approx(1.2 - 0.065 - 0.03 - 0.5)


def test_exploration_collision_dominates() -> None:
    w = ExplorationRewardWeights()
    r = exploration_reward(
        w,
        new_explored_m2=40.0,
        decision_duration_s=2.0,
        collided=True,
        shield_interventions=0,
        subgoal_failed=False,
    )
    assert r == pytest.approx(4.0 - 0.02 - 10.0)
    assert r < 0

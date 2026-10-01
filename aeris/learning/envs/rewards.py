"""Reward functions v1 (spec §27.5): pure functions over one transition's
already-extracted quantities, versioned (the version string is written into
every run manifest), unit-tested against hand-built transitions.

**Privileged-reward notes (spec §27.5, recorded, not hidden).**

- Local navigation's progress term uses the *geodesic* distance to the goal,
  computed by Dijkstra on the FastSim ground-truth map
  (:func:`aeris.simulation.fastsim.batch.geodesic_field`). This is
  legitimate -- reward is a training signal, not an observation, and the
  agent never observes the geodesic field -- but it *is* privileged
  information, and is declared as such here and in the env's
  ``privileged_reward_notes``.
- Collision is judged from ground-truth geometry (the evaluator's own
  definition, spec §41) during training in FastSim. Also reward-only.
- Exploration's area term uses the agent's *own* map (legitimate, not
  privileged).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

LOCAL_NAV_REWARD_VERSION = "local_nav_reward/v1"
EXPLORATION_REWARD_VERSION = "exploration_reward/v1"


@dataclass(frozen=True, slots=True)
class LocalNavRewardWeights:
    w_progress: float = 1.0  # per metre of geodesic distance gained
    w_time: float = 0.01  # per control step
    w_collision: float = 10.0
    w_shield: float = 0.05  # per step the shield had to intervene
    r_goal: float = 10.0


def local_nav_reward(
    w: LocalNavRewardWeights,
    *,
    prev_geodesic_m: np.ndarray,
    geodesic_m: np.ndarray,
    collided: np.ndarray,
    shield_intervened: np.ndarray,
    success: np.ndarray,
) -> np.ndarray:
    """r = w_p*(d_prev - d) - w_t - w_c*1[collision] - w_s*1[shield] + R_goal*1[success].

    A non-finite geodesic on either side (the vehicle sits in a cell the
    inflated true map calls blocked -- possible transiently at its edge)
    contributes zero progress, never +/-inf."""
    both = np.isfinite(prev_geodesic_m) & np.isfinite(geodesic_m)
    safe_prev = np.where(both, prev_geodesic_m, 0.0)
    safe_now = np.where(both, geodesic_m, 0.0)
    progress = safe_prev - safe_now
    return (
        w.w_progress * progress
        - w.w_time
        - w.w_collision * collided.astype(np.float64)
        - w.w_shield * shield_intervened.astype(np.float64)
        + w.r_goal * success.astype(np.float64)
    )


@dataclass(frozen=True, slots=True)
class ExplorationRewardWeights:
    w_explore: float = 0.1  # per m^2 newly observed in the agent's own map
    w_time: float = 0.01  # per second of decision duration
    w_collision: float = 10.0
    w_shield: float = 0.01  # per shield-intervening tick during the subgoal
    w_failed: float = 0.5  # subgoal abandoned without arrival


def exploration_reward(
    w: ExplorationRewardWeights,
    *,
    new_explored_m2: float,
    decision_duration_s: float,
    collided: bool,
    shield_interventions: int,
    subgoal_failed: bool,
) -> float:
    return (
        w.w_explore * new_explored_m2
        - w.w_time * decision_duration_s
        - w.w_collision * float(collided)
        - w.w_shield * shield_interventions
        - w.w_failed * float(subgoal_failed)
    )

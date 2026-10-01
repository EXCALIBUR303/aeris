"""Known-answer tasks for the PPO test suite (spec §27.4 #7) and the SB3
reference comparison. Each has a known optimal behavior, so "did PPO learn"
has a crisp answer.

All are native batched vector envs with the contract the trainer expects
from :class:`~aeris.learning.envs.local_nav.LocalNavVectorEnv`: dict
observations ``[N, ...]``, same-step autoreset with the pre-reset
observation in ``info["final_obs"]``, ``info["success"]`` on finished
episodes, and ``info["action_mask"]`` for the masked task.

- :class:`ContextualBandit`: one-step episodes; the context (one-hot) picks
  which arm pays 1.
- :class:`MaskedBandit`: one-step; a random subset of arms is unavailable
  each episode (the mask is also observed); arms have fixed values, and the
  best *available* arm is optimal. Exercises masked categorical end to end.
- :class:`PointGoal1D`: continuous 1-D point-goal; success = within 0.05 of
  the goal before the 50-step time limit (a truncation).
- :class:`MemoryCue`: a cue shown only at t=0; at t=``delay`` (flagged) the
  agent must name it. Without memory the best possible success rate is 50%.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from gymnasium import spaces
from gymnasium.vector import AutoresetMode

Obs = dict[str, np.ndarray]


class _ToyVectorEnv:
    metadata = {"autoreset_mode": AutoresetMode.SAME_STEP}  # noqa: RUF012
    single_observation_space: spaces.Dict
    single_action_space: spaces.Space[Any]

    def __init__(self, num_envs: int, seed: int = 0) -> None:
        self.num_envs = num_envs
        self.rng = np.random.default_rng(seed)
        self.t = np.zeros(num_envs, dtype=np.int64)

    # subclasses: _reset_envs(idx), _obs(), _step(actions) -> (reward, term, trunc, success)
    def _reset_envs(self, idx: np.ndarray) -> None:
        raise NotImplementedError

    def _obs(self) -> Obs:
        raise NotImplementedError

    def _step(self, actions: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        raise NotImplementedError

    def _info(self) -> dict[str, Any]:
        return {}

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[Obs, dict[str, Any]]:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.t[:] = 0
        self._reset_envs(np.arange(self.num_envs))
        return self._obs(), self._info()

    def step(
        self, actions: np.ndarray
    ) -> tuple[Obs, np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
        reward, term, trunc, success = self._step(np.asarray(actions))
        self.t += 1
        done = term | trunc
        info: dict[str, Any] = {"success": success & done}
        if done.any():
            info["final_obs"] = {k: v.copy() for k, v in self._obs().items()}
            idx = np.nonzero(done)[0]
            self.t[idx] = 0
            self._reset_envs(idx)
        info.update(self._info())
        return self._obs(), reward.astype(np.float32), term, trunc, info


class ContextualBandit(_ToyVectorEnv):
    def __init__(
        self, num_envs: int, seed: int = 0, *, n_contexts: int = 4, n_arms: int = 4
    ) -> None:
        super().__init__(num_envs, seed)
        self.n_ctx, self.n_arms = n_contexts, n_arms
        self.optimal = np.random.default_rng(1234).permutation(n_arms)[
            np.arange(n_contexts) % n_arms
        ]
        self.ctx = np.zeros(num_envs, dtype=np.int64)
        self.single_observation_space = spaces.Dict(
            {"state": spaces.Box(0.0, 1.0, (n_contexts,), np.float32)}
        )
        self.single_action_space = spaces.Discrete(n_arms)

    def _reset_envs(self, idx: np.ndarray) -> None:
        self.ctx[idx] = self.rng.integers(self.n_ctx, size=len(idx))

    def _obs(self) -> Obs:
        return {"state": np.eye(self.n_ctx, dtype=np.float32)[self.ctx]}

    def _step(self, actions: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        ok = actions.astype(np.int64) == self.optimal[self.ctx]
        ones = np.ones(self.num_envs, dtype=np.bool_)
        return ok.astype(np.float32), ones, ~ones, ok


class MaskedBandit(_ToyVectorEnv):
    VALUES = np.array([0.2, 0.4, 0.6, 0.8, 1.0, 0.1], dtype=np.float32)

    def __init__(self, num_envs: int, seed: int = 0, *, p_available: float = 0.5) -> None:
        super().__init__(num_envs, seed)
        self.n_arms = len(self.VALUES)
        self.p = p_available
        self.mask = np.ones((num_envs, self.n_arms), dtype=np.bool_)
        self.single_observation_space = spaces.Dict(
            {"state": spaces.Box(0.0, 1.0, (self.n_arms,), np.float32)}
        )
        self.single_action_space = spaces.Discrete(self.n_arms)

    def _reset_envs(self, idx: np.ndarray) -> None:
        m = self.rng.random((len(idx), self.n_arms)) < self.p
        none = ~m.any(1)
        m[none, self.rng.integers(self.n_arms, size=int(none.sum()))] = True
        self.mask[idx] = m

    def _obs(self) -> Obs:
        return {"state": self.mask.astype(np.float32)}

    def _info(self) -> dict[str, Any]:
        return {"action_mask": self.mask.copy()}

    def _step(self, actions: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        a = actions.astype(np.int64)
        rows = np.arange(self.num_envs)
        if not self.mask[rows, a].all():
            raise ValueError("MaskedBandit: a masked action was taken")
        best = np.where(self.mask, self.VALUES, -np.inf).argmax(1)
        ones = np.ones(self.num_envs, dtype=np.bool_)
        return self.VALUES[a], ones, ~ones, a == best


class PointGoal1D(_ToyVectorEnv):
    STEP, TOL, LIMIT = 0.1, 0.05, 50

    def __init__(self, num_envs: int, seed: int = 0) -> None:
        super().__init__(num_envs, seed)
        self.x = np.zeros(num_envs)
        self.g = np.zeros(num_envs)
        self.single_observation_space = spaces.Dict(
            {"state": spaces.Box(-2.0, 2.0, (3,), np.float32)}
        )
        self.single_action_space = spaces.Box(-1.0, 1.0, (1,), np.float32)

    def _reset_envs(self, idx: np.ndarray) -> None:
        self.x[idx] = self.rng.uniform(-1.0, 1.0, len(idx))
        self.g[idx] = self.rng.uniform(-0.8, 0.8, len(idx))

    def _obs(self) -> Obs:
        return {"state": np.stack([self.x, self.g, self.g - self.x], 1).astype(np.float32)}

    def _step(self, actions: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        a = np.clip(actions.reshape(self.num_envs), -1.0, 1.0)
        d0 = np.abs(self.g - self.x)
        self.x = np.clip(self.x + self.STEP * a, -1.0, 1.0)
        d1 = np.abs(self.g - self.x)
        success = d1 < self.TOL
        reward = 10.0 * (d0 - d1) - 0.01 + 1.0 * success
        trunc = ~success & (self.t + 1 >= self.LIMIT)
        return reward, success, trunc, success


class MemoryCue(_ToyVectorEnv):
    """obs ``[cue_0, cue_1, query]``; the cue is visible only at t=0, the
    query flag only at t=``delay``, where the answer is scored and the
    episode terminates."""

    def __init__(self, num_envs: int, seed: int = 0, *, delay: int = 5) -> None:
        super().__init__(num_envs, seed)
        self.delay = delay
        self.cue = np.zeros(num_envs, dtype=np.int64)
        self.single_observation_space = spaces.Dict(
            {"state": spaces.Box(0.0, 1.0, (3,), np.float32)}
        )
        self.single_action_space = spaces.Discrete(2)

    def _reset_envs(self, idx: np.ndarray) -> None:
        self.cue[idx] = self.rng.integers(2, size=len(idx))

    def _obs(self) -> Obs:
        o = np.zeros((self.num_envs, 3), dtype=np.float32)
        first = self.t == 0
        o[first, self.cue[first]] = 1.0
        o[self.t == self.delay, 2] = 1.0
        return {"state": o}

    def _step(self, actions: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        query = self.t == self.delay
        correct = query & (actions.astype(np.int64) == self.cue)
        reward = np.where(query, np.where(correct, 1.0, -1.0), 0.0)
        return reward, query, np.zeros(self.num_envs, dtype=np.bool_), correct

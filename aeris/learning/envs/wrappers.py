"""Training-side env wrappers.

:class:`ObservationContract` / :class:`VectorObservationContract` enforce the
observation spec at *runtime*, every step (spec §51 Phase 13 task 6,
"provenance enforcement"; §27.6). The static half of that enforcement --
no ORACLE field in a shipped spec unless ``oracle_baseline``, and
``aeris.learning.spaces`` never importing a simulator -- is
:meth:`ObservationSpec.check_provenance` plus the import-linter contracts.
This is the dynamic half: an env whose builder silently drifts from its
declared spec (an extra key, a reshaped field, an out-of-bounds or NaN
value) fails loudly at the first offending step instead of training a
policy on inputs the deployment builder will never produce.

Both wrappers expose ``spec_hash`` -- the value a trainer stamps into every
checkpoint and :func:`~aeris.learning.spaces.spec.verify_checkpoint_spec`
checks at deployment.
"""

from __future__ import annotations

from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium.vector import VectorEnv, VectorWrapper

from aeris.learning.spaces.spec import ObservationSpec


class ObservationContractError(ValueError):
    """An env produced an observation its declared spec does not allow."""


def check_observation(
    spec: ObservationSpec, obs: dict[str, np.ndarray], *, batch: int | None = None
) -> None:
    """Raise :class:`ObservationContractError` unless ``obs`` has exactly the
    spec's keys, each float32 with the declared shape (with a leading
    ``batch`` axis when given), finite, and inside ``[low, high]``."""
    expected = {f.name for f in spec.fields}
    if set(obs) != expected:
        raise ObservationContractError(
            f"{spec.name}: observation keys {sorted(obs)} != spec fields {sorted(expected)}"
        )
    for f in spec.fields:
        v = obs[f.name]
        shape = f.shape if batch is None else (batch, *f.shape)
        if v.shape != shape or v.dtype != np.float32:
            raise ObservationContractError(
                f"{spec.name}.{f.name}: got {v.dtype}{list(v.shape)}, spec says float32{list(shape)}"
            )
        if not np.all(np.isfinite(v)):
            raise ObservationContractError(f"{spec.name}.{f.name}: non-finite value")
        lo, hi = float(v.min()), float(v.max())
        if lo < f.low or hi > f.high:
            raise ObservationContractError(
                f"{spec.name}.{f.name}: range [{lo:.4g}, {hi:.4g}] outside "
                f"spec bounds [{f.low}, {f.high}]"
            )


class ObservationContract(gym.Wrapper[dict[str, np.ndarray], Any, dict[str, np.ndarray], Any]):
    def __init__(
        self,
        env: gym.Env[dict[str, np.ndarray], Any],
        spec: ObservationSpec,
        *,
        oracle_baseline: bool = False,
    ) -> None:
        super().__init__(env)
        spec.check_provenance(oracle_baseline=oracle_baseline)
        self.obs_contract = spec
        self.spec_hash = spec.hash()

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        obs, info = self.env.reset(seed=seed, options=options)
        check_observation(self.obs_contract, obs)
        return obs, info

    def step(self, action: Any) -> tuple[dict[str, np.ndarray], Any, bool, bool, dict[str, Any]]:
        obs, r, term, trunc, info = self.env.step(action)
        check_observation(self.obs_contract, obs)
        return obs, r, term, trunc, info


class VectorObservationContract(VectorWrapper):
    def __init__(
        self,
        env: VectorEnv[Any, Any, Any],
        spec: ObservationSpec,
        *,
        oracle_baseline: bool = False,
    ) -> None:
        super().__init__(env)
        spec.check_provenance(oracle_baseline=oracle_baseline)
        self.obs_contract = spec
        self.spec_hash = spec.hash()

    def reset(
        self, *, seed: int | list[int] | None = None, options: dict[str, Any] | None = None
    ) -> tuple[Any, dict[str, Any]]:
        # gymnasium's VectorWrapper.reset accepts a per-env seed list but
        # VectorEnv.reset is typed int-only; pass through what we were given.
        obs, info = self.env.reset(seed=seed, options=options)  # type: ignore[arg-type]
        check_observation(self.obs_contract, obs, batch=self.num_envs)
        return obs, info

    def step(self, actions: Any) -> tuple[Any, Any, Any, Any, dict[str, Any]]:
        obs, r, term, trunc, info = self.env.step(actions)
        check_observation(self.obs_contract, obs, batch=self.num_envs)
        if "final_obs" in info:
            done = np.asarray(term) | np.asarray(trunc)
            final = {k: v[done] for k, v in info["final_obs"].items()}
            check_observation(self.obs_contract, final, batch=int(done.sum()))
        return obs, r, term, trunc, info

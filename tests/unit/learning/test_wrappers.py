"""Runtime observation-contract wrappers (spec §27.6 provenance enforcement)."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

import gymnasium as gym
import numpy as np
import pytest

from aeris.core.errors import OracleObservationError
from aeris.core.types import Provenance
from aeris.learning.envs.local_nav import LocalNavEnv, LocalNavEnvConfig, LocalNavVectorEnv
from aeris.learning.envs.wrappers import (
    ObservationContract,
    ObservationContractError,
    VectorObservationContract,
    check_observation,
)
from aeris.learning.spaces.local_nav import LocalNavSpaceConfig, local_nav_spec
from aeris.simulation.worlds.spec import Bounds, Box, SpawnPose, WorldFamily, WorldSpec, WorldSplit

_SPEC = local_nav_spec(LocalNavSpaceConfig())
_CFG = LocalNavEnvConfig(episode_time_s=1.0)


def _world() -> WorldSpec:
    return WorldSpec(
        name="wrap_test",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TRAIN,
        seed=0,
        bounds=Bounds(min_x=-6.0, min_y=-6.0, max_x=6.0, max_y=6.0, max_z=3.0),
        altitude_band_m=(0.3, 2.5),
        boxes=(Box(x=1.0, y=1.0, z=1.5, size_x=1.0, size_y=1.0, size_z=3.0),),
        spawn_poses=(SpawnPose(x=0.0, y=0.0, z=0.1),),
    )


def _valid() -> dict[str, np.ndarray]:
    return {f.name: np.zeros(f.shape, np.float32) for f in _SPEC.fields}


def test_valid_observation_passes() -> None:
    check_observation(_SPEC, _valid())


@pytest.mark.parametrize(
    ("mutate", "match"),
    [
        (lambda o: o.update(extra=np.zeros(1, np.float32)), "keys"),
        (lambda o: o.pop("state"), "keys"),
        (lambda o: o.update(state=np.zeros(11, np.float32)), "float32"),
        (lambda o: o.update(state=np.zeros(10, np.float64)), "float32"),
        (lambda o: o["state"].__setitem__(0, np.nan), "non-finite"),
        (lambda o: o["depth"].__setitem__((0, 0, 0), 5.0), "outside"),
    ],
)
def test_contract_violations_raise(mutate: Any, match: str) -> None:
    obs = _valid()
    mutate(obs)
    with pytest.raises(ObservationContractError, match=match):
        check_observation(_SPEC, obs)


def test_wrapped_single_env_runs_episodes_and_exposes_spec_hash() -> None:
    env = ObservationContract(LocalNavEnv([_world()], config=_CFG), _SPEC)
    assert env.spec_hash == _SPEC.hash()
    env.reset(seed=0)
    rng = np.random.default_rng(0)
    for _ in range(25):  # 1 s episodes at 10 Hz: crosses terminal steps
        _, _, term, trunc, _ = env.step(rng.uniform(-1, 1, 3).astype(np.float32))
        if term or trunc:
            env.reset()


def test_wrapped_vector_env_checks_batched_and_final_observations() -> None:
    env = VectorObservationContract(
        LocalNavVectorEnv([_world()], num_envs=4, config=_CFG, seed=0), _SPEC
    )
    env.reset(seed=0)
    rng = np.random.default_rng(1)
    for _ in range(25):  # autoreset path exercises the final_obs check
        env.step(rng.uniform(-1, 1, (4, 3)))


def test_wrapper_refuses_an_oracle_spec_unless_declared_baseline() -> None:
    oracle = replace(
        _SPEC,
        fields=(*_SPEC.fields[:-1], replace(_SPEC.fields[-1], provenance=Provenance.ORACLE)),
    )
    base = LocalNavEnv([_world()], config=_CFG)
    with pytest.raises(OracleObservationError):
        ObservationContract(base, oracle)
    ObservationContract(base, oracle, oracle_baseline=True)


def test_wrapper_catches_a_drifting_builder() -> None:
    class Drift(gym.ObservationWrapper[dict[str, np.ndarray], Any, dict[str, np.ndarray]]):
        def observation(self, observation: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
            return {**observation, "state": observation["state"] * 1e3}

    env = ObservationContract(Drift(LocalNavEnv([_world()], config=_CFG)), _SPEC)
    with pytest.raises(ObservationContractError, match="outside"):
        env.reset(seed=0)

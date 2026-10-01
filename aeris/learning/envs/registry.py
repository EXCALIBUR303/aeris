"""Build the vector env a training config names, plus the provenance the
trainer records with it (observation-spec hash, reward version, privileged
-reward notes).

FastSim tasks draw worlds from the *train* split for training and the *val*
split for evaluation (spec §27.2 "Evaluation environments are separate from
training environments and use validation-split worlds"); the envs' own
split guards enforce it a second time.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any

from gymnasium import spaces

from aeris.learning.envs.exploration import ExplorationEnvConfig, ExplorationVectorEnv
from aeris.learning.envs.local_nav import LocalNavEnvConfig, LocalNavVectorEnv, SplitMode
from aeris.learning.envs.toy import ContextualBandit, MaskedBandit, MemoryCue, PointGoal1D
from aeris.learning.networks.heads import ActionSpec
from aeris.learning.ppo.config import EnvSettings
from aeris.simulation.worlds.batch import generate_world
from aeris.simulation.worlds.spec import WorldFamily, WorldSpec, WorldSplit
from aeris.simulation.worlds.splits import SEED_RANGES

_TOYS = {
    "toy_bandit": ContextualBandit,
    "toy_masked_bandit": MaskedBandit,
    "toy_point_goal_1d": PointGoal1D,
    "toy_memory": MemoryCue,
}


@dataclass
class EnvHandle:
    env: Any
    obs_shapes: dict[str, tuple[int, ...]]
    action: ActionSpec
    action_low: Any = None
    action_high: Any = None
    obs_spec_hash: str | None = None
    reward_version: str | None = None
    privileged_reward_notes: tuple[str, ...] = field(default_factory=tuple)


def _worlds(families: tuple[str, ...], n: int, split: WorldSplit) -> list[WorldSpec]:
    start = SEED_RANGES[split].start
    return [generate_world(WorldFamily(f), split, start + i) for f in families for i in range(n)]


def make_env(
    cfg: EnvSettings, *, seed: int, evaluation: bool = False, num_envs: int | None = None
) -> EnvHandle:
    n = num_envs or cfg.num_envs
    env: Any
    if cfg.id in _TOYS:
        env = _TOYS[cfg.id](n, seed, **cfg.params)
    else:
        split = WorldSplit.VAL if evaluation else WorldSplit.TRAIN
        mode: SplitMode = "val" if evaluation else "train"
        worlds = _worlds(
            cfg.families, cfg.eval_worlds_per_family if evaluation else cfg.worlds_per_family, split
        )
        if cfg.id == "local_nav":
            ln_cfg = replace(LocalNavEnvConfig.identified(), **cfg.params)
            env = LocalNavVectorEnv(worlds, num_envs=n, config=ln_cfg, split_mode=mode, seed=seed)
        else:
            ex_cfg = replace(ExplorationEnvConfig.identified(), **cfg.params)
            env = ExplorationVectorEnv(
                worlds, num_envs=n, config=ex_cfg, split_mode=mode, seed=seed
            )

    obs_space: spaces.Dict = env.single_observation_space
    act_space = env.single_action_space
    handle = EnvHandle(
        env=env,
        obs_shapes={k: tuple(int(d) for d in (s.shape or ())) for k, s in obs_space.spaces.items()},
        action=(
            ActionSpec("discrete", int(act_space.n))
            if isinstance(act_space, spaces.Discrete)
            else ActionSpec("continuous", int(act_space.shape[0]))
        ),
    )
    if isinstance(act_space, spaces.Box):
        handle.action_low, handle.action_high = act_space.low, act_space.high
    spec = getattr(env, "obs_spec", None)  # toy envs have no deployment spec
    if spec is not None:
        handle.obs_spec_hash = spec.hash()
    handle.reward_version = getattr(env, "reward_version", None)
    handle.privileged_reward_notes = tuple(getattr(env, "privileged_reward_notes", ()))
    return handle

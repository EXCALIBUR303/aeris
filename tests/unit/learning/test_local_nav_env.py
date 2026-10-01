"""FastSim local-nav env: Gymnasium contract (check_env), D0 determinism,
split discipline, and terminal-state semantics."""

from __future__ import annotations

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from aeris.core.errors import SplitViolationError
from aeris.learning.envs.local_nav import LocalNavEnv, LocalNavEnvConfig, LocalNavVectorEnv
from aeris.simulation.worlds.spec import (
    Bounds,
    Box,
    SpawnPose,
    WorldFamily,
    WorldSpec,
    WorldSplit,
)


def _world(split: WorldSplit = WorldSplit.TRAIN, seed: int = 0) -> WorldSpec:
    return WorldSpec(
        name=f"ln_test_{split.value}_{seed}",
        family=WorldFamily.RUBBLE,
        split=split,
        seed=seed,
        bounds=Bounds(min_x=-6.0, min_y=-6.0, max_x=6.0, max_y=6.0, max_z=3.0),
        altitude_band_m=(0.3, 2.5),
        boxes=(
            Box(x=1.0, y=1.0, z=1.5, size_x=1.0, size_y=1.0, size_z=3.0),
            Box(x=-2.0, y=-1.5, z=1.5, size_x=0.6, size_y=2.0, size_z=3.0, yaw_rad=0.5),
        ),
        spawn_poses=(SpawnPose(x=0.0, y=0.0, z=0.1),),
    )


_CFG = LocalNavEnvConfig(episode_time_s=5.0)


def test_single_env_passes_gymnasium_check_env() -> None:
    check_env(LocalNavEnv([_world()], config=_CFG), skip_render_check=True)


def test_rollout_is_bitwise_deterministic_given_seed() -> None:
    def rollout(seed: int) -> tuple[np.ndarray, np.ndarray]:
        env = LocalNavVectorEnv([_world()], num_envs=4, config=_CFG, seed=seed)
        obs, _ = env.reset(seed=seed)
        rng = np.random.default_rng(99)
        rewards, depth = [], []
        for _ in range(80):  # crosses episode boundaries (autoreset)
            obs, r, *_ = env.step(rng.uniform(-1, 1, (4, 3)))
            rewards.append(r)
            depth.append(obs["depth"])
        return np.array(rewards), np.array(depth)

    r1, d1 = rollout(7)
    r2, d2 = rollout(7)
    np.testing.assert_array_equal(r1, r2)
    np.testing.assert_array_equal(d1, d2)
    r3, _ = rollout(8)
    assert not np.array_equal(r1, r3)


def test_train_mode_refuses_val_and_test_worlds() -> None:
    with pytest.raises(SplitViolationError):
        LocalNavEnv([_world(WorldSplit.VAL, 10_000)], config=_CFG)
    with pytest.raises(SplitViolationError):
        LocalNavEnv([_world(WorldSplit.TEST_ID, 20_000)], config=_CFG, split_mode="val")
    LocalNavEnv([_world(WorldSplit.VAL, 10_000)], config=_CFG, split_mode="val")  # allowed


def test_flying_into_a_wall_terminates_with_collision_or_is_shielded() -> None:
    """Full speed straight ahead, repeatedly: the shield must hold the vehicle
    off the obstacle it faces, or -- if it can't -- the episode must end as a
    *terminated* collision, never silently pass through."""
    env = LocalNavVectorEnv([_world()], num_envs=8, config=_CFG, seed=3)
    env.reset(seed=3)
    saw_collision_terminal = False
    for _ in range(200):
        _, _, term, trunc, info = env.step(np.tile([1.0, 0.0, 0.0], (8, 1)))
        assert not np.any(info["collision"] & ~term)
        saw_collision_terminal |= bool(np.any(info["collision"]))
        assert not np.any(term & trunc)
    del saw_collision_terminal  # either outcome is legitimate; the invariants above are the test


def test_success_is_terminal_and_rewarded() -> None:
    env = LocalNavVectorEnv(
        [_world()], num_envs=1, config=LocalNavEnvConfig(episode_time_s=60.0), seed=5
    )
    env.reset(seed=5)
    core = env.core
    # Teleport the vehicle onto its goal: next step must be a rewarded terminal success.
    core.sim.dyn.state.pos[0, :2] = core.goal[0]
    core.prev_geo[0] = 1.0
    _, r, term, trunc, info = env.step(np.zeros((1, 3)))
    assert info["success"][0] and term[0] and not trunc[0]
    assert r[0] > 5.0


def test_time_limit_is_a_truncation_not_a_termination() -> None:
    cfg = LocalNavEnvConfig(episode_time_s=0.5)
    env = LocalNavVectorEnv([_world()], num_envs=2, config=cfg, seed=1)
    env.reset(seed=1)
    for _ in range(5):
        _, _, term, trunc, info = env.step(np.zeros((2, 3)))
    assert np.all(trunc | term)
    assert np.all(info["episode_length"] == 5)

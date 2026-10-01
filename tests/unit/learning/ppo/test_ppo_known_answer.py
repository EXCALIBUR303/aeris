"""Spec §27.4 #7: known-answer learning tests (SMOKE, each < 2 min CPU).

(a) contextual bandit -> optimal arm; (a') masked bandit -> best *available*
arm, never a masked one (the env raises if it is); (b) 1-D point goal ->
>= 95% success (deterministic policy); (c) memory cue -> solved by the GRU
and *not* by the MLP, which proves the memory path works."""

from __future__ import annotations

import time
from typing import Any

from aeris.learning.ppo.config import TrainConfig, load_train_config
from aeris.learning.ppo.trainer import PPOTrainer

BUDGET_S = 120.0


def _train(cfg: TrainConfig) -> tuple[dict[str, float], float]:
    t0 = time.perf_counter()
    tr = PPOTrainer(cfg, log=None)
    tr.train()
    result = tr.evaluate(deterministic=True)
    return result, time.perf_counter() - t0


def _bandit(env_id: str) -> TrainConfig:
    raw: dict[str, Any] = {
        "name": env_id,
        "total_steps": 20_000,
        "env": {"id": env_id, "num_envs": 16},
        "network": {"trunk_sizes": [64]},
        "ppo": {"num_steps": 32, "ent_coef": 0.01},
        "eval": {"episodes": 400, "num_envs": 16},
    }
    return TrainConfig.model_validate(raw)


def test_contextual_bandit_converges_to_optimal_arm() -> None:
    r, dt = _train(_bandit("toy_bandit"))
    assert r["success_rate"] >= 0.99 and dt < BUDGET_S


def test_masked_bandit_picks_best_available_arm_and_never_a_masked_one() -> None:
    r, dt = _train(_bandit("toy_masked_bandit"))  # MaskedBandit raises on a masked action
    assert r["success_rate"] >= 0.99 and dt < BUDGET_S


def test_point_goal_1d_reaches_95_percent_success() -> None:
    cfg, _, _ = load_train_config("ppo_toy_point_goal")
    r, dt = _train(cfg)
    assert r["success_rate"] >= 0.95, r
    assert dt < BUDGET_S


def test_memory_task_is_solved_by_gru_and_not_by_mlp() -> None:
    gru_cfg, _, _ = load_train_config("ppo_toy_memory_gru")
    mlp_cfg, _, _ = load_train_config("ppo_toy_memory_gru", ("network.recurrent=false",))
    gru, dt_gru = _train(gru_cfg)
    mlp, dt_mlp = _train(mlp_cfg)
    assert gru["success_rate"] >= 0.95, gru
    assert mlp["success_rate"] <= 0.65, mlp  # chance is 0.5 without memory
    assert dt_gru < BUDGET_S and dt_mlp < BUDGET_S

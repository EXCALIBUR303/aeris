#!/usr/bin/env python3
"""Reference sanity check (spec §27.1, §51 Phase 14): AERIS PPO vs
Stable-Baselines3 PPO on the same toy task (1-D point goal), matched
hyperparameters, several seeds, learning curves on a common evaluator.

The purpose is to catch gross bugs, not to match exactly: the two differ in
network layout (AERIS: one shared 64-unit trunk after a 64-unit obs MLP;
SB3: separate [64, 64] actor and critic MLPs) and in observation
normalization details.

SB3 is a *dev-only* dependency and is deliberately not in pyproject.toml;
run in an ephemeral environment::

    uv run --with "stable-baselines3>=2.3" python scripts/learning/sb3_reference.py

Writes ``results/learning/sb3_reference.json``.
"""

from __future__ import annotations

import json
import sys
import time
from collections.abc import Callable
from importlib.metadata import version
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from aeris.learning.envs.toy import PointGoal1D
from aeris.learning.ppo.config import load_train_config
from aeris.learning.ppo.trainer import PPOTrainer

_OUT = Path(__file__).resolve().parents[2] / "results" / "learning" / "sb3_reference.json"
SEEDS = (0, 1, 2)
TOTAL, EVERY, N_ENVS, N_STEPS = 200_000, 25_000, 16, 32


class PointGoalGym(gym.Env[np.ndarray, np.ndarray]):
    """Single-env gymnasium view of the batched :class:`PointGoal1D`."""

    def __init__(self, seed: int) -> None:
        self.inner = PointGoal1D(1, seed)
        self.observation_space = gym.spaces.Box(-2.0, 2.0, (3,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        obs, _ = self.inner.reset(seed=seed)
        return obs["state"][0], {}

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        obs, r, te, tr, info = self.inner.step(np.asarray(action).reshape(1, 1))
        final = info["final_obs"]["state"][0] if (te[0] or tr[0]) else obs["state"][0]
        return final, float(r[0]), bool(te[0]), bool(tr[0]), {}


def success_rate(policy: Callable[[np.ndarray], np.ndarray], episodes: int = 400) -> float:
    env = PointGoal1D(16, seed=999)
    obs, _ = env.reset(seed=999)
    done_eps: list[bool] = []
    while len(done_eps) < episodes:
        obs, _, te, tr, info = env.step(policy(obs["state"]))
        done_eps += list(info["success"][te | tr])
    return float(np.mean(done_eps[:episodes]))


def _near_goal_states(n: int = 400) -> np.ndarray:
    """States 0.05-0.1 from the goal: an ideal controller needs |a| < 1 here
    (one 0.1-step at full action overshoots the 0.05 window about half the
    time), so a mean action saturated at the bound is the deterministic
    failure mode."""
    rng = np.random.default_rng(7)
    x = rng.uniform(-0.7, 0.7, n)
    g = x + rng.choice([-1.0, 1.0], n) * rng.uniform(0.05, 0.1, n)
    return np.stack([x, g, g - x], 1).astype(np.float32)


_NEAR = _near_goal_states()


def saturation(mean: np.ndarray) -> float:
    return float(np.mean(np.abs(np.asarray(mean).ravel()) >= 1.0))


def run_sb3(seed: int) -> list[tuple[int, float, float, float]]:
    venv = VecNormalize(
        DummyVecEnv([lambda i=i: PointGoalGym(seed * 100 + i) for i in range(N_ENVS)]),
        norm_obs=True,
        norm_reward=False,
    )
    model = PPO(
        "MlpPolicy",
        venv,
        n_steps=N_STEPS,
        batch_size=N_ENVS * N_STEPS // 4,
        n_epochs=4,
        learning_rate=lambda f: 3e-4 * f,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        vf_coef=0.5,
        max_grad_norm=0.5,
        policy_kwargs={"log_std_init": -0.5},
        seed=seed,
        device="cpu",
        verbose=0,
    )
    curve = []
    for step in range(EVERY, TOTAL + 1, EVERY):
        model.learn(EVERY, reset_num_timesteps=False)
        venv.training = False
        with torch.no_grad():
            dist = model.policy.get_distribution(torch.as_tensor(venv.normalize_obs(_NEAR)))
            sat = saturation(dist.distribution.mean.numpy())
        curve.append(
            (
                step,
                success_rate(lambda o: model.predict(venv.normalize_obs(o), deterministic=True)[0]),
                success_rate(
                    lambda o: model.predict(venv.normalize_obs(o), deterministic=False)[0]
                ),
                sat,
            )
        )
        venv.training = True
    return curve


def run_aeris(seed: int) -> list[tuple[int, float, float, float]]:
    cfg, _, _ = load_train_config("ppo_toy_point_goal", (f"seed={seed}", f"total_steps={TOTAL}"))
    tr = PPOTrainer(cfg, log=None)
    per = EVERY // cfg.batch_size
    curve = []

    def policy(o: np.ndarray, deterministic: bool = True) -> np.ndarray:
        obs = tr._prep_obs({"state": o}, update_stats=False)
        with torch.no_grad():
            d, _, _ = tr.model.step(obs, tr.model.initial_state(len(o)), torch.ones(len(o)))
            a = d.mode() if deterministic else d.sample()
        return np.asarray(np.clip(a.numpy(), -1.0, 1.0), dtype=np.float32)

    for i in range(TOTAL // EVERY):
        tr.train(max_updates=per)
        grid = tr._prep_obs({"state": _NEAR}, update_stats=False)
        with torch.no_grad():
            d, _, _ = tr.model.step(
                grid, tr.model.initial_state(len(_NEAR)), torch.ones(len(_NEAR))
            )
        curve.append(
            (
                (i + 1) * per * cfg.batch_size,
                success_rate(policy),
                success_rate(lambda o: policy(o, deterministic=False)),
                saturation(d.mode().numpy()),
            )
        )
    return curve


def main() -> int:
    torch.set_num_threads(4)
    out: dict[str, Any] = {
        "task": "toy_point_goal_1d (success rate over 400 episodes, deterministic and stochastic)",
        "stable_baselines3": version("stable-baselines3"),
        "torch": torch.__version__,
        "aeris": {},
        "sb3": {},
    }
    for seed in SEEDS:
        for name, fn in (("aeris", run_aeris), ("sb3", run_sb3)):
            t0 = time.perf_counter()
            curve = fn(seed)
            out[name][str(seed)] = {"curve": curve, "wall_s": round(time.perf_counter() - t0, 1)}
            print(name, seed, curve, f"{time.perf_counter() - t0:.0f}s", flush=True)
    out["curve_columns"] = [
        "env_steps",
        "det_success",
        "stochastic_success",
        "near_goal_saturation",
    ]
    for name in ("aeris", "sb3"):
        last = [out[name][str(s)]["curve"][-1] for s in SEEDS]
        out[name]["final_det_success_mean"] = float(np.mean([r[1] for r in last]))
        out[name]["final_stochastic_success_mean"] = float(np.mean([r[2] for r in last]))
        out[name]["final_near_goal_saturation_mean"] = float(np.mean([r[3] for r in last]))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    _OUT.write_text(json.dumps(out, indent=2))
    print("wrote", _OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())

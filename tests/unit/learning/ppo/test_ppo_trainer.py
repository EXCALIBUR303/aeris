"""Trainer-level checks: the NaN guard (spec §27.4 #10), the masked
categorical path on the real FastSim exploration env, frozen evaluation
normalization, and the ``aeris train`` CLI end to end."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import torch

from aeris.cli import main as cli_main
from aeris.core.errors import NonFiniteTrainingError
from aeris.learning.envs.registry import make_env
from aeris.learning.envs.toy import PointGoal1D
from aeris.learning.ppo import run as run_module
from aeris.learning.ppo.config import TrainConfig
from aeris.learning.ppo.trainer import PPOTrainer


def _cfg(env_id: str = "toy_point_goal_1d", **env: Any) -> TrainConfig:
    raw: dict[str, Any] = {
        "name": "t",
        "total_steps": 512,
        "env": {"id": env_id, "num_envs": 4, **env},
        "network": {"trunk_sizes": [32], "image_dim": 32, "vector_dim": 16},
        "ppo": {"num_steps": 16, "num_minibatches": 2, "update_epochs": 2},
        "eval": {"episodes": 4, "num_envs": 2},
    }
    return TrainConfig.model_validate(raw)


class _NaNReward(PointGoal1D):
    def __init__(self, num_envs: int, seed: int = 0, at_step: int = 5) -> None:
        super().__init__(num_envs, seed)
        self.calls, self.at = 0, at_step

    def step(self, actions: np.ndarray) -> Any:
        out = list(super().step(actions))
        self.calls += 1
        if self.calls == self.at:
            out[1] = out[1].copy()
            out[1][0] = np.nan
        return tuple(out)


def _with_env(cfg: TrainConfig, env: Any) -> PPOTrainer:
    handle = make_env(cfg.env, seed=0)
    handle.env = env
    return PPOTrainer(cfg, env=handle, log=None)


def test_nan_guard_trips_on_an_injected_nan_reward() -> None:
    tr = _with_env(_cfg(), _NaNReward(4))
    with pytest.raises(NonFiniteTrainingError) as exc:
        tr.train(max_updates=1)
    d = exc.value.diagnostics
    assert "loss" in d["non_finite"] and d["update"] == 0 and d["epoch"] == 0


def test_nan_guard_trips_on_a_corrupted_parameter() -> None:
    tr = _with_env(_cfg(), PointGoal1D(4))
    tr.train(max_updates=1)
    with torch.no_grad():
        tr.model.actor.weight[0, 0] = float("nan")
    with pytest.raises(NonFiniteTrainingError):
        tr.train(max_updates=1)


def test_run_writes_nan_diagnostics_and_fails_the_manifest(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    real = run_module.PPOTrainer

    def patched(cfg: TrainConfig, **kw: Any) -> PPOTrainer:
        handle = make_env(cfg.env, seed=cfg.seed)
        handle.env = _NaNReward(cfg.env.num_envs)
        return real(cfg, env=handle, **kw)

    monkeypatch.setattr(run_module, "PPOTrainer", patched)
    cfg_path = tmp_path / "c.yaml"
    cfg_path.write_text(json.dumps(_cfg().model_dump(mode="json")))
    with pytest.raises(NonFiniteTrainingError):
        run_module.run_training(str(cfg_path), run_dir=tmp_path / "run", log=None)
    diag = json.loads((tmp_path / "run" / "nan_diagnostics.json").read_text())
    assert "loss" in diag["non_finite"]
    manifest = json.loads((tmp_path / "run" / "manifest.json").read_text())
    assert manifest["status"] == "failed" and "NaN guard" in manifest["status_reason"]


def test_exploration_env_masked_actions_are_never_sampled() -> None:
    cfg = _cfg("exploration", families=("f2_office",), worlds_per_family=1)
    cfg = cfg.model_copy(
        update={
            "ppo": cfg.ppo.model_copy(update={"num_steps": 4}),
            "network": cfg.network.model_copy(update={"recurrent": True, "hidden_size": 16}),
        }
    )
    tr = PPOTrainer(cfg, log=None)
    tr.collect()
    b = tr.buffer
    assert b.masks is not None
    t_idx, n_idx = torch.meshgrid(torch.arange(4), torch.arange(4), indexing="ij")
    assert bool(b.masks[t_idx, n_idx, b.actions].all())
    assert bool((~b.masks).any())  # the mask actually masked something
    tr.learn()  # and the update runs on the real env's data


def test_evaluation_freezes_observation_normalization() -> None:
    tr = PPOTrainer(_cfg(), log=None)
    tr.train(max_updates=1)
    before = {k: r.state_dict() for k, r in tr.obs_rms.items()}
    tr.evaluate(deterministic=True)
    tr.evaluate(deterministic=False)
    for k, r in tr.obs_rms.items():
        for f, v in r.state_dict().items():
            assert torch.equal(v, before[k][f])


def test_cli_train_end_to_end(tmp_path: Path) -> None:
    run_dir = tmp_path / "run"
    rc = cli_main(
        [
            "train",
            "--config",
            "ppo_toy_point_goal",
            "--set",
            "total_steps=1024",
            "--set",
            "eval.episodes=8",
            "--run-dir",
            str(run_dir),
        ]
    )
    assert rc == 0
    manifest = json.loads((run_dir / "manifest.json").read_text())
    assert manifest["status"] == "completed" and manifest["method"] == "ppo_mlp"
    assert manifest["checkpoint_hash"] and (run_dir / "checkpoints" / "final.pt").is_file()
    rows = [json.loads(x) for x in (run_dir / "metrics.jsonl").read_text().splitlines()]
    assert len(rows) == 2
    for key in (
        "approx_kl",
        "clip_fraction",
        "entropy",
        "value_loss",
        "explained_variance",
        "grad_norm",
        "sps",
        "learning_rate",
    ):
        assert key in rows[0]
    summary = json.loads((run_dir / "summary.json").read_text())
    assert set(summary["final_eval"]) == {"deterministic", "stochastic"}

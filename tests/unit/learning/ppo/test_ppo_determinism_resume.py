"""Spec §27.4 #8 (same seed -> identical first-N-update losses on CPU) and
#9 (checkpoint -> resume -> identical subsequent losses), including on the
FastSim local-nav env; plus checkpoint compatibility refusals."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
import torch

from aeris.core.errors import CheckpointMismatchError
from aeris.learning.ppo.checkpoint import load_checkpoint, restore_trainer, save_checkpoint
from aeris.learning.ppo.config import TrainConfig
from aeris.learning.ppo.trainer import PPOTrainer


def _cfg(env_id: str, *, recurrent: bool = False, seed: int = 0, **env: Any) -> TrainConfig:
    raw: dict[str, Any] = {
        "name": f"det_{env_id}",
        "seed": seed,
        "total_steps": 10_000_000,  # never reached; tests drive max_updates
        "env": {"id": env_id, "num_envs": 4, **env},
        "network": {
            "trunk_sizes": [32],
            "image_dim": 32,
            "vector_dim": 16,
            "recurrent": recurrent,
            "hidden_size": 16,
        },
        "ppo": {"num_steps": 16, "num_minibatches": 2, "update_epochs": 2, "scale_rewards": True},
        "eval": {"episodes": 2, "num_envs": 2},
    }
    return TrainConfig.model_validate(raw)


CASES = [
    pytest.param(_cfg("toy_point_goal_1d"), id="mlp-gaussian"),
    pytest.param(_cfg("toy_memory", recurrent=True), id="gru-categorical"),
    pytest.param(_cfg("toy_masked_bandit"), id="masked-categorical"),
]


def _losses(tr: PPOTrainer, n: int) -> list[float]:
    return [x for s in tr.train(max_updates=n) for x in s.loss_trace]


@pytest.mark.parametrize("cfg", CASES)
def test_same_seed_gives_identical_losses(cfg: TrainConfig) -> None:
    a = _losses(PPOTrainer(cfg, log=None), 3)
    b = _losses(PPOTrainer(cfg, log=None), 3)
    assert a == b  # bitwise, not approx
    c = _losses(PPOTrainer(cfg.model_copy(update={"seed": 1}), log=None), 3)
    assert a != c


@pytest.mark.parametrize(
    "cfg",
    [
        *CASES,
        pytest.param(
            _cfg("local_nav", families=("f1_rubble",), worlds_per_family=1), id="fastsim-local-nav"
        ),
    ],
)
def test_checkpoint_resume_is_bitwise_identical(cfg: TrainConfig, tmp_path: Path) -> None:
    straight = PPOTrainer(cfg, config_hash="h", log=None)
    _losses(straight, 2)
    ckpt = tmp_path / "mid.pt"
    save_checkpoint(straight, ckpt)
    tail_straight = _losses(straight, 2)

    resumed = PPOTrainer(cfg, config_hash="h", log=None)
    assert restore_trainer(resumed, ckpt) is True
    assert resumed.update == 2 and resumed.global_step == straight.global_step - 2 * 16 * 4
    tail_resumed = _losses(resumed, 2)
    assert tail_resumed == tail_straight
    for p1, p2 in zip(straight.model.parameters(), resumed.model.parameters(), strict=True):
        assert torch.equal(p1, p2)


def test_checkpoint_loads_with_weights_only_and_records_provenance(tmp_path: Path) -> None:
    cfg = _cfg("local_nav", families=("f1_rubble",), worlds_per_family=1)
    tr = PPOTrainer(cfg, config_hash="abc", log=None)
    tr.train(max_updates=1)
    save_checkpoint(tr, tmp_path / "c.pt")
    state = load_checkpoint(tmp_path / "c.pt")  # torch.load(weights_only=True) inside
    assert state["config_hash"] == "abc"
    assert state["obs_spec_hash"] == tr.handle.obs_spec_hash is not None
    assert state["reward_version"] == "local_nav_reward/v1"
    assert state["update"] == 1 and "git_sha" in state


def test_resume_refuses_mismatched_checkpoints(tmp_path: Path) -> None:
    cfg = _cfg("toy_point_goal_1d")
    tr = PPOTrainer(cfg, config_hash="h1", log=None)
    save_checkpoint(tr, tmp_path / "c.pt")
    with pytest.raises(CheckpointMismatchError, match="config hash"):
        restore_trainer(PPOTrainer(cfg, config_hash="h2", log=None), tmp_path / "c.pt")
    restore_trainer(
        PPOTrainer(cfg, config_hash="h2", log=None), tmp_path / "c.pt", allow_config_change=True
    )
    wider = cfg.model_copy(
        update={"network": cfg.network.model_copy(update={"trunk_sizes": (64,)})}
    )
    with pytest.raises(CheckpointMismatchError, match="network"):
        restore_trainer(PPOTrainer(wider, config_hash="h1", log=None), tmp_path / "c.pt")
    with pytest.raises(CheckpointMismatchError, match=r"action space|observation"):
        restore_trainer(
            PPOTrainer(_cfg("toy_memory"), config_hash="h1", log=None), tmp_path / "c.pt"
        )


def test_resume_without_env_snapshot_is_valid_but_not_bitwise(tmp_path: Path) -> None:
    cfg = _cfg("toy_point_goal_1d")
    tr = PPOTrainer(cfg, config_hash="h", log=None)
    tr.train(max_updates=1)
    save_checkpoint(tr, tmp_path / "c.pt", env_snapshot=False)
    resumed = PPOTrainer(cfg, config_hash="h", log=None)
    assert restore_trainer(resumed, tmp_path / "c.pt") is False
    assert resumed.update == 1
    resumed.train(max_updates=1)  # keeps training from restored weights

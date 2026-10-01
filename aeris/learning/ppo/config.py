"""PPO training config (spec §27.2 defaults, §38.1 composition).

Loaded with :func:`aeris.core.config.load_resolved_config` (YAML +
``extends:`` + CLI ``key=value`` overrides), validated here, and hashed; the
hash goes into the run id, the manifest and every checkpoint. ``extra =
"forbid"`` everywhere, so a misspelled key fails loudly instead of silently
training with a default.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

from aeris.core.config import load_resolved_config

_STRICT = ConfigDict(extra="forbid", frozen=True)
CONFIG_DIR = Path(__file__).resolve().parents[3] / "configs" / "learning"

EnvId = Literal[
    "local_nav",
    "exploration",
    "toy_bandit",
    "toy_masked_bandit",
    "toy_point_goal_1d",
    "toy_memory",
]


class EnvSettings(BaseModel):
    model_config = _STRICT
    id: EnvId
    num_envs: int = Field(16, ge=1)
    # FastSim tasks: procedural world families and how many seeds of each,
    # taken from the front of the train split (training) / val split (eval).
    families: tuple[str, ...] = ("f1_rubble", "f2_office", "f3_warehouse")
    worlds_per_family: int = Field(2, ge=1)
    eval_worlds_per_family: int = Field(1, ge=1)
    # Overrides of the env's own config dataclass / constructor keywords.
    params: dict[str, Any] = Field(default_factory=dict)


class NetworkSettings(BaseModel):
    model_config = _STRICT
    image_dim: int = 256
    vector_dim: int = 64
    trunk_sizes: tuple[int, ...] = (256,)
    recurrent: bool = False
    hidden_size: int = 256
    log_std_init: float = -0.5


class PPOSettings(BaseModel):
    model_config = _STRICT
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_eps: float = 0.2
    num_steps: int = Field(128, ge=1)
    num_minibatches: int = Field(4, ge=1)
    update_epochs: int = Field(4, ge=1)
    learning_rate: float = 3e-4
    adam_eps: float = 1e-5
    anneal_lr: bool = True
    max_grad_norm: float = 0.5
    vf_coef: float = 0.5
    ent_coef: float = 0.0
    clip_value_loss: bool = False
    normalize_advantages: bool = True
    target_kl: float = 0.02
    kl_early_stop: bool = False
    normalize_obs: bool = True
    scale_rewards: bool = False


class EvalSettings(BaseModel):
    model_config = _STRICT
    episodes: int = Field(32, ge=1)
    every_updates: int = Field(0, ge=0)  # 0 = only at the end
    num_envs: int = Field(8, ge=1)
    seed_offset: int = 10_000


class TrainConfig(BaseModel):
    model_config = _STRICT
    name: str
    budget: Literal["smoke", "dev", "full"] = "smoke"
    seed: int = 0
    device: Literal["cpu", "mps"] = "cpu"
    torch_threads: int | None = None
    total_steps: int = Field(..., ge=1)
    checkpoint_every_updates: int = Field(0, ge=0)  # 0 = only the final checkpoint
    env: EnvSettings
    network: NetworkSettings = NetworkSettings()
    ppo: PPOSettings = PPOSettings()
    eval: EvalSettings = EvalSettings()

    @property
    def batch_size(self) -> int:
        return self.ppo.num_steps * self.env.num_envs

    @property
    def num_updates(self) -> int:
        return max(1, self.total_steps // self.batch_size)


def resolve_config_path(name_or_path: str) -> Path:
    """``ppo_localnav_smoke`` -> ``configs/learning/ppo_localnav_smoke.yaml``;
    an existing path is used as given."""
    p = Path(name_or_path)
    if p.suffix in (".yaml", ".yml") and p.is_file():
        return p
    return CONFIG_DIR / f"{name_or_path}.yaml"


def load_train_config(
    name_or_path: str, overrides: tuple[str, ...] = ()
) -> tuple[TrainConfig, dict[str, Any], str]:
    """Returns ``(validated config, resolved dict, config hash)``."""
    raw, digest = load_resolved_config(resolve_config_path(name_or_path), overrides=overrides)
    return TrainConfig.model_validate(raw), raw, digest

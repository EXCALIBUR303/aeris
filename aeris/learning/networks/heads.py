"""Actor-critic network (spec §27.2): encoder -> [GRU] -> actor + critic heads.

- Actor, continuous: linear mean + a **state-independent** log-std
  parameter (init -0.5), diagonal Gaussian.
- Actor, discrete: linear logits over ``n`` actions, masked categorical.
- Critic: linear -> scalar V(s), sharing the trunk (spec default).
- Output-layer gains: 0.01 for the policy, 1.0 for the value (Huang et al.
  2022, detail #2): a near-uniform initial policy, an unbiased initial value
  scale.

Two entry points, so rollout and training go through the *same* code:
:meth:`ActorCritic.step` (one time step, used while collecting) and
:meth:`ActorCritic.evaluate` (a ``[T, B]`` block, used in the update; for
the GRU variant it replays from the stored initial hidden state with the
stored episode-start flags).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Literal

import torch
from torch import nn

from aeris.learning.networks.distributions import DiagGaussian, MaskedCategorical
from aeris.learning.networks.encoders import MultiModalEncoder, orthogonal_
from aeris.learning.networks.recurrent import GRUCore

Distribution = DiagGaussian | MaskedCategorical


@dataclass(frozen=True, slots=True)
class ActionSpec:
    kind: Literal["continuous", "discrete"]
    n: int  # action dims (continuous) or number of actions (discrete)


@dataclass(frozen=True, slots=True)
class NetworkConfig:
    image_dim: int = 256
    vector_dim: int = 64
    trunk_sizes: tuple[int, ...] = (256,)
    recurrent: bool = False
    hidden_size: int = 256
    log_std_init: float = -0.5


class ActorCritic(nn.Module):
    def __init__(
        self,
        obs_shapes: Mapping[str, Sequence[int]],
        action: ActionSpec,
        config: NetworkConfig | None = None,
    ) -> None:
        super().__init__()
        cfg = config or NetworkConfig()
        self.config, self.action = cfg, action
        self.encoder = MultiModalEncoder(
            {k: tuple(v) for k, v in obs_shapes.items()},
            image_dim=cfg.image_dim,
            vector_dim=cfg.vector_dim,
            trunk_sizes=cfg.trunk_sizes,
        )
        feat = self.encoder.out_dim
        self.core: GRUCore | None = None
        if cfg.recurrent:
            self.core = GRUCore(feat, cfg.hidden_size)
            feat = cfg.hidden_size
        self.actor = orthogonal_(nn.Linear(feat, action.n), gain=0.01)
        self.critic = orthogonal_(nn.Linear(feat, 1), gain=1.0)
        self.log_std: nn.Parameter | None = None
        if action.kind == "continuous":
            self.log_std = nn.Parameter(torch.full((action.n,), cfg.log_std_init))

    # -- recurrent state -------------------------------------------------------------
    @property
    def hidden_size(self) -> int:
        return self.core.hidden_size if self.core is not None else 0

    def initial_state(self, n: int, device: torch.device | str = "cpu") -> torch.Tensor:
        """Zeros ``[n, H]`` (``H = 0`` for the feed-forward variant)."""
        return torch.zeros(n, self.hidden_size, device=device)

    # -- heads ----------------------------------------------------------------------------
    def _dist(self, feat: torch.Tensor, mask: torch.Tensor | None) -> Distribution:
        out = self.actor(feat)
        if self.action.kind == "continuous":
            assert self.log_std is not None
            return DiagGaussian(out, self.log_std)
        return MaskedCategorical(out, mask)

    def step(
        self,
        obs: Mapping[str, torch.Tensor],
        h: torch.Tensor,
        starts: torch.Tensor,
        mask: torch.Tensor | None = None,
    ) -> tuple[Distribution, torch.Tensor, torch.Tensor]:
        """One step for ``B`` envs -> ``(dist, value [B], h_next [B, H])``."""
        feat = self.encoder(obs)
        if self.core is not None:
            h = self.core.step(feat, h, starts)
            feat = h
        return self._dist(feat, mask), self.critic(feat).squeeze(-1), h

    def evaluate(
        self,
        obs: Mapping[str, torch.Tensor],
        h0: torch.Tensor,
        starts: torch.Tensor,
        actions: torch.Tensor,
        masks: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Re-evaluate a ``[T, B]`` block -> ``(log_prob, entropy, value)``,
        each ``[T, B]``. Feed-forward ignores ``h0``/``starts``."""
        t, b = starts.shape
        flat = {k: v.reshape(t * b, *v.shape[2:]) for k, v in obs.items()}
        feat = self.encoder(flat)
        if self.core is not None:
            seq, _ = self.core.sequence(feat.reshape(t, b, -1), h0, starts)
            feat = seq.reshape(t * b, -1)
        flat_mask = masks.reshape(t * b, -1) if masks is not None else None
        dist = self._dist(feat, flat_mask)
        flat_actions = actions.reshape(t * b, *actions.shape[2:])
        return (
            dist.log_prob(flat_actions).reshape(t, b),
            dist.entropy().reshape(t, b),
            self.critic(feat).squeeze(-1).reshape(t, b),
        )

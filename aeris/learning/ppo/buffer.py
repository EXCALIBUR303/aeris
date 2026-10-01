"""Rollout storage (spec §27.2): ``[T, N, ...]`` tensors for observations,
actions, log-probs, rewards, terminated, truncated, values, the
final-observation values used for truncation bootstrapping, action masks,
episode-start flags, and the GRU state at the start of the chunk.

Minibatching (spec §27.2 "Batching"):

- feed-forward: shuffle all ``T*N`` samples into ``M`` minibatches;
- recurrent: split the ``N`` envs into ``M`` groups and keep each env's
  whole ``T``-step sequence together, so BPTT replays from that env's stored
  ``h0`` with its own episode-start flags and never mixes envs (§27.4 #6).

Both yield :class:`Minibatch` blocks shaped ``[T', B, ...]`` (``T' = 1`` for
feed-forward), so the update loop has one code path.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass

import torch

from aeris.learning.networks.heads import ActionSpec


@dataclass(frozen=True, slots=True)
class Minibatch:
    obs: dict[str, torch.Tensor]
    actions: torch.Tensor
    old_log_prob: torch.Tensor
    old_values: torch.Tensor
    advantages: torch.Tensor
    returns: torch.Tensor
    starts: torch.Tensor
    masks: torch.Tensor | None
    h0: torch.Tensor
    env_index: torch.Tensor  # which envs (recurrent) / flat sample ids (feed-forward)


class RolloutBuffer:
    def __init__(
        self,
        *,
        num_steps: int,
        num_envs: int,
        obs_shapes: Mapping[str, Sequence[int]],
        action: ActionSpec,
        hidden_size: int = 0,
        device: torch.device | str = "cpu",
    ) -> None:
        t, n = num_steps, num_envs
        self.num_steps, self.num_envs, self.action = t, n, action
        self.obs = {k: torch.zeros(t, n, *s, device=device) for k, s in obs_shapes.items()}
        if action.kind == "continuous":
            self.actions = torch.zeros(t, n, action.n, device=device)
            self.masks: torch.Tensor | None = None
        else:
            self.actions = torch.zeros(t, n, dtype=torch.long, device=device)
            self.masks = torch.ones(t, n, action.n, dtype=torch.bool, device=device)
        self.log_probs = torch.zeros(t, n, device=device)
        self.rewards = torch.zeros(t, n, device=device)
        self.values = torch.zeros(t, n, device=device)
        self.final_values = torch.zeros(t, n, device=device)
        self.terminated = torch.zeros(t, n, dtype=torch.bool, device=device)
        self.truncated = torch.zeros(t, n, dtype=torch.bool, device=device)
        self.starts = torch.zeros(t, n, device=device)
        self.h0 = torch.zeros(n, hidden_size, device=device)

    def add(
        self,
        t: int,
        *,
        obs: Mapping[str, torch.Tensor],
        action: torch.Tensor,
        log_prob: torch.Tensor,
        value: torch.Tensor,
        reward: torch.Tensor,
        terminated: torch.Tensor,
        truncated: torch.Tensor,
        final_value: torch.Tensor,
        start: torch.Tensor,
        mask: torch.Tensor | None = None,
    ) -> None:
        for k, v in obs.items():
            self.obs[k][t] = v
        self.actions[t] = action
        self.log_probs[t] = log_prob
        self.values[t] = value
        self.rewards[t] = reward
        self.terminated[t] = terminated
        self.truncated[t] = truncated
        self.final_values[t] = final_value
        self.starts[t] = start
        if self.masks is not None and mask is not None:
            self.masks[t] = mask

    def minibatches(
        self,
        advantages: torch.Tensor,
        returns: torch.Tensor,
        *,
        num_minibatches: int,
        recurrent: bool,
        generator: torch.Generator,
    ) -> Iterator[Minibatch]:
        if recurrent:
            yield from self._recurrent(advantages, returns, num_minibatches, generator)
        else:
            yield from self._feed_forward(advantages, returns, num_minibatches, generator)

    def _feed_forward(
        self, adv: torch.Tensor, ret: torch.Tensor, m: int, g: torch.Generator
    ) -> Iterator[Minibatch]:
        total = self.num_steps * self.num_envs

        def flat(x: torch.Tensor) -> torch.Tensor:
            return x.reshape(total, *x.shape[2:])

        obs = {k: flat(v) for k, v in self.obs.items()}
        masks = flat(self.masks) if self.masks is not None else None
        perm = torch.randperm(total, generator=g).to(self.rewards.device)
        for idx in perm.chunk(m):

            def pick(x: torch.Tensor, idx: torch.Tensor = idx) -> torch.Tensor:
                return flat(x)[idx].unsqueeze(0)

            yield Minibatch(
                obs={k: v[idx].unsqueeze(0) for k, v in obs.items()},
                actions=pick(self.actions),
                old_log_prob=pick(self.log_probs),
                old_values=pick(self.values),
                advantages=pick(adv),
                returns=pick(ret),
                starts=pick(self.starts),
                masks=masks[idx].unsqueeze(0) if masks is not None else None,
                h0=self.h0.new_zeros(len(idx), self.h0.shape[1]),
                env_index=idx,
            )

    def _recurrent(
        self, adv: torch.Tensor, ret: torch.Tensor, m: int, g: torch.Generator
    ) -> Iterator[Minibatch]:
        if self.num_envs < m:
            raise ValueError(f"recurrent minibatching needs num_envs >= {m} minibatches")
        perm = torch.randperm(self.num_envs, generator=g).to(self.rewards.device)
        for envs in perm.chunk(m):
            yield Minibatch(
                obs={k: v[:, envs] for k, v in self.obs.items()},
                actions=self.actions[:, envs],
                old_log_prob=self.log_probs[:, envs],
                old_values=self.values[:, envs],
                advantages=adv[:, envs],
                returns=ret[:, envs],
                starts=self.starts[:, envs],
                masks=self.masks[:, envs] if self.masks is not None else None,
                h0=self.h0[envs],
                env_index=envs,
            )

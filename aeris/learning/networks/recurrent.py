"""GRU memory core (spec §28).

The hidden state is reset at the *input* of every step:
``h_t <- h_t * (1 - start_t)``, where ``start_t`` = "step t is the first of
an episode" (= ``done_{t-1}``). The same masking is applied when a stored
rollout chunk is replayed from its stored initial state during training, so
a sequence never carries memory across an episode boundary.

``GRUCell`` stepped in a Python loop is deliberate: masking between steps
is then explicit and identical in rollout and replay, at a cost that is
negligible for the T <= 128 rollouts this project uses.
"""

from __future__ import annotations

import torch
from torch import nn


class GRUCore(nn.Module):
    def __init__(self, in_dim: int, hidden_size: int = 256) -> None:
        super().__init__()
        self.cell = nn.GRUCell(in_dim, hidden_size)
        for name, p in self.cell.named_parameters():
            if "weight" in name:
                nn.init.orthogonal_(p, 1.0)
            else:
                nn.init.zeros_(p)
        self.hidden_size = hidden_size

    def step(self, x: torch.Tensor, h: torch.Tensor, starts: torch.Tensor) -> torch.Tensor:
        """One step: ``x [B, in]``, ``h [B, H]``, ``starts [B]`` -> ``h' [B, H]``."""
        h = h * (1.0 - starts).unsqueeze(-1)
        out: torch.Tensor = self.cell(x, h)
        return out

    def sequence(
        self, x: torch.Tensor, h0: torch.Tensor, starts: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Replay ``x [T, B, in]`` from ``h0 [B, H]`` with ``starts [T, B]``.
        Returns ``(outputs [T, B, H], h_T [B, H])``."""
        h = h0
        outs = []
        for t in range(x.shape[0]):
            h = self.step(x[t], h, starts[t])
            outs.append(h)
        return torch.stack(outs), h

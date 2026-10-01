"""Action distributions (spec §27.2), written as explicit closed forms so the
math is visible and unit-tested against ``torch.distributions`` (§27.4 #5).

- :class:`DiagGaussian`: mean from the actor head, **state-independent**
  log-std parameter. Log-prob is computed on the *unclipped* sample; the
  environment's ``ActionDecoder`` clips to bounds (CleanRL convention,
  Huang et al. 2022, "The 37 Implementation Details of PPO", detail #4 of
  the continuous-action section).
- :class:`MaskedCategorical`: logits over discrete actions with invalid
  actions set to ``MASKED_LOGIT`` (-1e9), so they get exactly zero
  probability after softmax in float32, and contribute nothing to entropy
  (Huang & Ontañón 2022, "A Closer Look at Invalid Action Masking in Policy
  Gradient Algorithms").
"""

from __future__ import annotations

import math

import torch

MASKED_LOGIT = -1e9
_LOG_SQRT_2PI = 0.5 * math.log(2.0 * math.pi)


class DiagGaussian:
    """Diagonal Gaussian over ``A`` action dims; ``log_prob``/``entropy`` sum
    over the action dimension and return shape ``[B]``."""

    def __init__(self, mean: torch.Tensor, log_std: torch.Tensor) -> None:
        self.mean = mean
        self.log_std = log_std.expand_as(mean)
        self.std = self.log_std.exp()

    def sample(self, generator: torch.Generator | None = None) -> torch.Tensor:
        noise = torch.randn(
            self.mean.shape, generator=generator, device=self.mean.device, dtype=self.mean.dtype
        )
        return self.mean + self.std * noise

    def mode(self) -> torch.Tensor:
        return self.mean

    def log_prob(self, action: torch.Tensor) -> torch.Tensor:
        z = (action - self.mean) / self.std
        return (-0.5 * z * z - self.log_std - _LOG_SQRT_2PI).sum(-1)

    def entropy(self) -> torch.Tensor:
        return (0.5 + _LOG_SQRT_2PI + self.log_std).sum(-1)


class MaskedCategorical:
    """Categorical over ``K`` actions with an optional boolean ``mask``
    (``True`` = allowed). Every row must allow at least one action."""

    def __init__(self, logits: torch.Tensor, mask: torch.Tensor | None = None) -> None:
        if mask is not None:
            if not bool(mask.any(-1).all()):
                raise ValueError("MaskedCategorical: a row has every action masked")
            logits = torch.where(mask, logits, torch.full_like(logits, MASKED_LOGIT))
        self.mask = mask
        self.log_probs = logits - logits.logsumexp(-1, keepdim=True)
        self.probs = self.log_probs.exp()

    def sample(self, generator: torch.Generator | None = None) -> torch.Tensor:
        return torch.multinomial(self.probs, 1, generator=generator).squeeze(-1)

    def mode(self) -> torch.Tensor:
        return self.probs.argmax(-1)

    def log_prob(self, action: torch.Tensor) -> torch.Tensor:
        return self.log_probs.gather(-1, action.long().unsqueeze(-1)).squeeze(-1)

    def entropy(self) -> torch.Tensor:
        plogp = self.probs * self.log_probs
        if self.mask is not None:
            plogp = torch.where(self.mask, plogp, torch.zeros_like(plogp))
        return -plogp.sum(-1)

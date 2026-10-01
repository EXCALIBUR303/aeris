"""Training diagnostics and running statistics (spec §27.2-27.3).

- :func:`explained_variance`: ``1 - Var[R - V] / Var[R]``.
- :class:`RunningMeanStd`: parallel-Welford running mean/variance (Chan et
  al. 1979), used for low-dimensional observation normalization (frozen at
  evaluation) and for return-based reward scaling. Kept in float64 and
  saved in checkpoints.
- :class:`ReturnScaler`: divides rewards by the running std of the
  discounted return (Engstrom et al. 2020, "Implementation Matters in Deep
  Policy Gradients"; the ``NormalizeReward`` convention).
- :func:`assert_finite`: the NaN guard (spec §27.4 #10). It raises
  :class:`~aeris.core.errors.NonFiniteTrainingError` naming every non-finite
  tensor, so the trainer halts with diagnostics instead of stepping
  corrupted weights.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import torch

from aeris.core.errors import NonFiniteTrainingError


def explained_variance(values: torch.Tensor, returns: torch.Tensor) -> float:
    var_r = returns.var()
    if not torch.isfinite(var_r) or float(var_r) == 0.0:
        return float("nan")
    return float(1.0 - (returns - values).var() / var_r)


class RunningMeanStd:
    def __init__(self, shape: tuple[int, ...], epsilon: float = 1e-4) -> None:
        self.mean = torch.zeros(shape, dtype=torch.float64)
        self.var = torch.ones(shape, dtype=torch.float64)
        self.count = torch.tensor(epsilon, dtype=torch.float64)

    def update(self, batch: torch.Tensor) -> None:
        b = batch.detach().to("cpu", torch.float64).reshape(-1, *self.mean.shape)
        b_mean, b_var, b_count = b.mean(0), b.var(0, unbiased=False), b.shape[0]
        delta = b_mean - self.mean
        total = self.count + b_count
        self.mean = self.mean + delta * b_count / total
        m2 = self.var * self.count + b_var * b_count + delta**2 * self.count * b_count / total
        self.var = m2 / total
        self.count = total

    def normalize(self, x: torch.Tensor, clip: float = 10.0) -> torch.Tensor:
        mean = self.mean.to(x.device, x.dtype)
        std = torch.sqrt(self.var + 1e-8).to(x.device, x.dtype)
        return torch.clamp((x - mean) / std, -clip, clip)

    def state_dict(self) -> dict[str, torch.Tensor]:
        return {"mean": self.mean.clone(), "var": self.var.clone(), "count": self.count.clone()}

    def load_state_dict(self, state: Mapping[str, torch.Tensor]) -> None:
        self.mean, self.var, self.count = (
            state["mean"].clone(),
            state["var"].clone(),
            state["count"].clone(),
        )


class ReturnScaler:
    """Scale rewards by the running std of each env's discounted return."""

    def __init__(self, num_envs: int, gamma: float) -> None:
        self.gamma = gamma
        self.returns = torch.zeros(num_envs, dtype=torch.float64)
        self.rms = RunningMeanStd(())

    def __call__(self, reward: torch.Tensor, done: torch.Tensor) -> torch.Tensor:
        r = reward.detach().to("cpu", torch.float64)
        self.returns = self.returns * self.gamma + r
        self.rms.update(self.returns)
        self.returns = torch.where(done.cpu().bool(), torch.zeros_like(self.returns), self.returns)
        scaled = r / torch.sqrt(self.rms.var + 1e-8)
        return scaled.to(reward.device, reward.dtype)

    def state_dict(self) -> dict[str, Any]:
        return {"returns": self.returns.clone(), "rms": self.rms.state_dict()}

    def load_state_dict(self, state: Mapping[str, Any]) -> None:
        self.returns = state["returns"].clone()
        self.rms.load_state_dict(state["rms"])


def assert_finite(tensors: Mapping[str, torch.Tensor], context: Mapping[str, Any]) -> None:
    """Raise :class:`NonFiniteTrainingError` if any tensor has a NaN/inf."""
    bad = {
        name: int((~torch.isfinite(t)).sum())
        for name, t in tensors.items()
        if not bool(torch.isfinite(t).all())
    }
    if bad:
        diagnostics = {"non_finite": bad, **dict(context)}
        raise NonFiniteTrainingError(f"non-finite values in {sorted(bad)}", diagnostics)

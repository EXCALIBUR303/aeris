"""Generalized Advantage Estimation with Gymnasium termination/truncation
semantics (spec §27.3; Schulman et al. 2016, "High-Dimensional Continuous
Control Using Generalized Advantage Estimation").

    delta_t = r_t + gamma * (1 - term_t) * Vhat_{t+1} - V(s_t)
    Vhat_{t+1} = V(s_final^(t))   if trunc_t   (value of the true final obs,
                                               before auto-reset)
               = V(s_{t+1})       otherwise    (next stored value, or the
                                               bootstrap value after step T-1)
    A_t = delta_t + gamma * lambda * (1 - done_t) * A_{t+1},   done = term | trunc
    R_t = A_t + V(s_t)

Termination cuts the bootstrap (a true terminal has no future); truncation
keeps it but takes it from the *final* observation, because after an
auto-reset ``s_{t+1}`` is the next episode's first observation (Pardo et al.
2018, "Time Limits in Reinforcement Learning"). Either one stops the
recursion so advantages never cross an episode boundary.
"""

from __future__ import annotations

import torch


def compute_gae(
    *,
    rewards: torch.Tensor,
    values: torch.Tensor,
    next_value: torch.Tensor,
    terminated: torch.Tensor,
    truncated: torch.Tensor,
    final_values: torch.Tensor,
    gamma: float,
    lam: float,
) -> tuple[torch.Tensor, torch.Tensor]:
    """All inputs ``[T, N]`` except ``next_value [N]`` (V of the observation
    after step ``T-1``). ``final_values[t]`` is read only where
    ``truncated[t]``. Returns ``(advantages, returns)``, each ``[T, N]``."""
    t_len = rewards.shape[0]
    term = terminated.to(rewards.dtype)
    trunc = truncated.to(rewards.dtype)
    done = torch.clamp(term + trunc, max=1.0)
    adv = torch.zeros_like(rewards)
    last = torch.zeros_like(next_value)
    for t in reversed(range(t_len)):
        v_next = next_value if t == t_len - 1 else values[t + 1]
        v_next = torch.where(truncated[t].bool(), final_values[t], v_next)
        delta = rewards[t] + gamma * (1.0 - term[t]) * v_next - values[t]
        last = delta + gamma * lam * (1.0 - done[t]) * last
        adv[t] = last
    return adv, adv + values

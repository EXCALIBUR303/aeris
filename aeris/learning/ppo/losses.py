"""PPO losses (spec §27.2-27.3; Schulman et al. 2017, "Proximal Policy
Optimization Algorithms").

    rho_t  = exp(log pi_theta(a_t|s_t) - log pi_old(a_t|s_t))
    L_clip = -E[min(rho_t A_t, clip(rho_t, 1-eps, 1+eps) A_t)]
    L_V    = 0.5 E[(V(s_t) - R_t)^2]   (optionally clipped around V_old)
    L      = L_clip + c_v L_V - c_e H[pi]

Advantages are normalized per minibatch (Huang et al. 2022, detail #7).
Diagnostics: approx-KL = E[(rho - 1) - log rho] (Schulman's unbiased,
always-non-negative k3 estimator), clip fraction = E[|rho - 1| > eps].
"""

from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass(frozen=True, slots=True)
class LossTerms:
    loss: torch.Tensor
    policy_loss: torch.Tensor
    value_loss: torch.Tensor
    entropy: torch.Tensor
    approx_kl: torch.Tensor
    clip_fraction: torch.Tensor


def normalize_advantages(adv: torch.Tensor, eps: float = 1e-8) -> torch.Tensor:
    if adv.numel() < 2:
        return adv - adv.mean()
    return (adv - adv.mean()) / (adv.std() + eps)


def ppo_loss(
    *,
    new_log_prob: torch.Tensor,
    old_log_prob: torch.Tensor,
    advantages: torch.Tensor,
    new_values: torch.Tensor,
    old_values: torch.Tensor,
    returns: torch.Tensor,
    entropy: torch.Tensor,
    clip_eps: float,
    vf_coef: float,
    ent_coef: float,
    clip_value_loss: bool = False,
) -> LossTerms:
    """All tensors share one shape (flattened minibatch). ``advantages`` are
    used as given -- normalize first with :func:`normalize_advantages`."""
    log_ratio = new_log_prob - old_log_prob
    ratio = log_ratio.exp()
    surr1 = ratio * advantages
    surr2 = torch.clamp(ratio, 1.0 - clip_eps, 1.0 + clip_eps) * advantages
    policy_loss = -torch.min(surr1, surr2).mean()

    if clip_value_loss:
        v_clipped = old_values + torch.clamp(new_values - old_values, -clip_eps, clip_eps)
        v_loss = 0.5 * torch.max((new_values - returns) ** 2, (v_clipped - returns) ** 2).mean()
    else:
        v_loss = 0.5 * ((new_values - returns) ** 2).mean()

    ent = entropy.mean()
    with torch.no_grad():
        approx_kl = ((ratio - 1.0) - log_ratio).mean()
        clip_fraction = ((ratio - 1.0).abs() > clip_eps).float().mean()
    return LossTerms(
        loss=policy_loss + vf_coef * v_loss - ent_coef * ent,
        policy_loss=policy_loss,
        value_loss=v_loss,
        entropy=ent,
        approx_kl=approx_kl,
        clip_fraction=clip_fraction,
    )

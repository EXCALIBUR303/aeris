"""Spec §27.4 #3-#4: PPO clipped-surrogate identities and clip gradients."""

from __future__ import annotations

import pytest
import torch

from aeris.learning.ppo.losses import normalize_advantages, ppo_loss


def _loss(new_lp: torch.Tensor, old_lp: torch.Tensor, adv: torch.Tensor, eps: float = 0.2):  # type: ignore[no-untyped-def]
    z = torch.zeros_like(adv)
    return ppo_loss(
        new_log_prob=new_lp,
        old_log_prob=old_lp,
        advantages=adv,
        new_values=z,
        old_values=z,
        returns=z,
        entropy=z,
        clip_eps=eps,
        vf_coef=0.5,
        ent_coef=0.0,
    )


def test_identity_at_theta_equals_theta_old() -> None:
    torch.manual_seed(0)
    lp, adv = torch.randn(64), torch.randn(64)
    t = _loss(lp, lp.clone(), adv)
    assert float(t.clip_fraction) == 0.0
    assert float(t.approx_kl) == 0.0
    assert float(t.policy_loss) == pytest.approx(-float(adv.mean()))


@pytest.mark.parametrize(
    ("ratio", "adv", "grad_is_zero"),
    [
        (1.5, +1.0, True),  # A > 0, rho > 1+eps: clipped, no incentive to push further
        (0.5, -1.0, True),  # A < 0, rho < 1-eps: mirror case
        (1.1, +1.0, False),  # inside the trust region: gradient flows
        (0.5, +1.0, False),  # A > 0 but rho < 1-eps: the min() keeps the unclipped term
        (1.5, -1.0, False),  # A < 0 but rho > 1+eps: likewise
    ],
)
def test_clip_gradient_wrt_log_prob(ratio: float, adv: float, grad_is_zero: bool) -> None:
    new_lp = torch.tensor([float(torch.log(torch.tensor(ratio)))], requires_grad=True)
    t = _loss(new_lp, torch.zeros(1), torch.tensor([adv]))
    t.policy_loss.backward()  # type: ignore[no-untyped-call]
    assert new_lp.grad is not None
    assert (float(new_lp.grad) == 0.0) is grad_is_zero


def test_clip_fraction_and_kl_are_correct() -> None:
    ratios = torch.tensor([0.7, 0.9, 1.0, 1.1, 1.3])
    t = _loss(ratios.log(), torch.zeros(5), torch.ones(5))
    assert float(t.clip_fraction) == pytest.approx(2 / 5)
    expected_kl = ((ratios - 1) - ratios.log()).mean()
    assert float(t.approx_kl) == pytest.approx(float(expected_kl))
    assert float(t.approx_kl) > 0


def test_value_loss_and_clipped_value_loss() -> None:
    new_v, old_v, ret = torch.tensor([2.0]), torch.tensor([1.0]), torch.tensor([3.0])
    z = torch.zeros(1)
    plain = ppo_loss(
        new_log_prob=z,
        old_log_prob=z,
        advantages=z,
        new_values=new_v,
        old_values=old_v,
        returns=ret,
        entropy=z,
        clip_eps=0.2,
        vf_coef=0.5,
        ent_coef=0.0,
    )
    assert float(plain.value_loss) == pytest.approx(0.5 * 1.0)
    clipped = ppo_loss(
        new_log_prob=z,
        old_log_prob=z,
        advantages=z,
        new_values=new_v,
        old_values=old_v,
        returns=ret,
        entropy=z,
        clip_eps=0.2,
        vf_coef=0.5,
        ent_coef=0.0,
        clip_value_loss=True,
    )
    # v_clipped = 1.2 -> (1.2 - 3)^2 = 3.24 > (2 - 3)^2 = 1 -> max picks 3.24
    assert float(clipped.value_loss) == pytest.approx(0.5 * 3.24)


def test_total_loss_composition() -> None:
    lp, adv = torch.zeros(4), torch.tensor([1.0, -1.0, 2.0, 0.0])
    nv, ret, ent = torch.tensor([1.0] * 4), torch.zeros(4), torch.full((4,), 0.5)
    t = ppo_loss(
        new_log_prob=lp,
        old_log_prob=lp,
        advantages=adv,
        new_values=nv,
        old_values=nv,
        returns=ret,
        entropy=ent,
        clip_eps=0.2,
        vf_coef=0.5,
        ent_coef=0.01,
    )
    assert float(t.loss) == pytest.approx(-0.5 + 0.5 * 0.5 - 0.01 * 0.5)


def test_advantage_normalization() -> None:
    a = normalize_advantages(torch.tensor([1.0, 2.0, 3.0, 10.0]))
    assert float(a.mean()) == pytest.approx(0.0, abs=1e-6)
    assert float(a.std()) == pytest.approx(1.0, abs=1e-5)

"""Spec §27.4 #5: Gaussian log-prob/entropy against torch.distributions;
masked categorical gives masked actions zero probability and its entropy
ignores them."""

from __future__ import annotations

import pytest
import torch
from torch.distributions import Categorical, Normal

from aeris.learning.networks.distributions import DiagGaussian, MaskedCategorical


def test_diag_gaussian_matches_torch_closed_forms() -> None:
    torch.manual_seed(0)
    mean, log_std = torch.randn(32, 3), torch.randn(3) * 0.5
    a = torch.randn(32, 3) * 2
    d = DiagGaussian(mean, log_std)
    ref = Normal(mean, log_std.exp().expand_as(mean))
    torch.testing.assert_close(d.log_prob(a), ref.log_prob(a).sum(-1))
    torch.testing.assert_close(d.entropy(), ref.entropy().sum(-1))


def test_diag_gaussian_log_prob_is_on_the_unclipped_sample() -> None:
    d = DiagGaussian(torch.zeros(1, 1), torch.zeros(1))
    a = torch.tensor([[3.0]])  # outside [-1, 1]: the decoder clips, the log-prob must not
    assert float(d.log_prob(a)) == pytest.approx(
        float(Normal(0.0, 1.0).log_prob(torch.tensor(3.0)))
    )


def test_masked_categorical_zero_probability_and_never_sampled() -> None:
    torch.manual_seed(0)
    logits = torch.randn(4, 6) * 3
    mask = torch.tensor(
        [[1, 0, 1, 0, 1, 0], [0, 0, 0, 0, 0, 1], [1] * 6, [0, 1, 1, 0, 0, 0]], dtype=torch.bool
    )
    d = MaskedCategorical(logits, mask)
    assert bool((d.probs[~mask] == 0).all())
    torch.testing.assert_close(d.probs.sum(-1), torch.ones(4))
    g = torch.Generator().manual_seed(1)
    for _ in range(500):
        a = d.sample(g)
        assert bool(mask[torch.arange(4), a].all())
    assert bool(mask[torch.arange(4), d.mode()].all())


def test_masked_categorical_entropy_and_log_prob_ignore_masked_actions() -> None:
    torch.manual_seed(1)
    logits = torch.randn(5, 7)
    mask = torch.rand(5, 7) > 0.4
    mask[:, 0] = True
    d = MaskedCategorical(logits, mask)
    for i in range(5):
        valid = mask[i].nonzero().squeeze(-1)
        ref = Categorical(logits=logits[i, valid])
        assert float(d.entropy()[i]) == pytest.approx(float(ref.entropy()), abs=1e-6)
        for j, a in enumerate(valid):
            lp = d.log_prob(torch.full((5,), int(a)))[i]
            assert float(lp) == pytest.approx(float(ref.log_prob(torch.tensor(j))), abs=1e-6)


def test_unmasked_categorical_matches_torch() -> None:
    logits = torch.randn(8, 5)
    d, ref = MaskedCategorical(logits), Categorical(logits=logits)
    a = torch.randint(0, 5, (8,))
    torch.testing.assert_close(d.log_prob(a), ref.log_prob(a))
    torch.testing.assert_close(d.entropy(), ref.entropy())


def test_masked_categorical_gradients_are_finite_and_masked_logits_get_none() -> None:
    logits = torch.randn(3, 4, requires_grad=True)
    mask = torch.tensor([[1, 1, 0, 0], [0, 1, 0, 1], [1, 0, 0, 0]], dtype=torch.bool)
    d = MaskedCategorical(logits, mask)
    (d.entropy().sum() + d.log_prob(torch.tensor([0, 3, 0])).sum()).backward()  # type: ignore[no-untyped-call]
    assert logits.grad is not None and bool(torch.isfinite(logits.grad).all())
    assert bool((logits.grad[~mask] == 0).all())


def test_fully_masked_row_is_rejected() -> None:
    with pytest.raises(ValueError, match="every action masked"):
        MaskedCategorical(torch.zeros(2, 3), torch.tensor([[1, 0, 0], [0, 0, 0]], dtype=torch.bool))

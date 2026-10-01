"""Spec §27.4 #1-#2: GAE against hand-computed values, and truncation
bootstrapping from the final-observation value."""

from __future__ import annotations

import pytest
import torch

from aeris.learning.ppo.gae import compute_gae

# One env, 5 steps: terminated at t=1, truncated at t=3 (true final-obs value
# 0.9), still running after t=4 (bootstrap V = 0.7).
R = [1.0, 2.0, 3.0, 4.0, 5.0]
V = [0.5, 0.4, 0.3, 0.2, 0.1]
NEXT_V, FINAL_V3 = 0.7, 0.9


def _hand(g: float, lam: float) -> list[float]:
    """Written out from spec §27.3 term by term."""
    d0 = R[0] + g * V[1] - V[0]
    d1 = R[1] - V[1]  # terminated: no bootstrap
    d2 = R[2] + g * V[3] - V[2]
    d3 = R[3] + g * FINAL_V3 - V[3]  # truncated: bootstrap from the final obs, not V[4]
    d4 = R[4] + g * NEXT_V - V[4]
    a4 = d4
    a3 = d3  # done at t=3: recursion stops
    a2 = d2 + g * lam * a3
    a1 = d1  # done at t=1
    a0 = d0 + g * lam * a1
    return [a0, a1, a2, a3, a4]


def _run(g: float, lam: float, *, v4: float = V[4], final3: float = FINAL_V3) -> torch.Tensor:
    t = lambda x: torch.tensor(x, dtype=torch.float64).unsqueeze(1)  # noqa: E731
    adv, ret = compute_gae(
        rewards=t(R),
        values=t([*V[:4], v4]),
        next_value=torch.tensor([NEXT_V], dtype=torch.float64),
        terminated=t([0, 1, 0, 0, 0]).bool(),
        truncated=t([0, 0, 0, 1, 0]).bool(),
        final_values=t([0.0, 0.0, 0.0, final3, 0.0]),
        gamma=g,
        lam=lam,
    )
    torch.testing.assert_close(ret, adv + t([*V[:4], v4]))
    return adv.squeeze(1)


@pytest.mark.parametrize(("g", "lam"), [(0.99, 0.95), (1.0, 1.0), (0.9, 0.0)])
def test_gae_matches_hand_computation(g: float, lam: float) -> None:
    torch.testing.assert_close(_run(g, lam), torch.tensor(_hand(g, lam), dtype=torch.float64))


def test_gae_lambda_one_gamma_one_is_return_to_go_minus_value() -> None:
    # Sanity on the (1, 1) case: A_t = sum of rewards to the episode cut + bootstrap - V_t.
    adv = _run(1.0, 1.0)
    assert float(adv[0]) == pytest.approx(R[0] + R[1] - V[0])
    assert float(adv[2]) == pytest.approx(R[2] + R[3] + FINAL_V3 - V[2])


def test_truncation_bootstraps_from_final_obs_value_not_reset_obs() -> None:
    """V[4] here is the value of the *reset* observation after the truncation
    at t=3. Changing it must not change A_3; changing the final-obs value must."""
    base = _run(0.99, 0.95)
    other_reset = _run(0.99, 0.95, v4=123.0)
    assert float(other_reset[3]) == pytest.approx(float(base[3]))
    other_final = _run(0.99, 0.95, final3=-5.0)
    assert float(other_final[3]) == pytest.approx(float(base[3]) + 0.99 * (-5.0 - FINAL_V3))


def test_gae_is_independent_per_env() -> None:
    torch.manual_seed(0)
    r, v = torch.randn(6, 3), torch.randn(6, 3)
    term = torch.zeros(6, 3, dtype=torch.bool)
    term[2, 1] = True
    kw = {
        "next_value": torch.randn(3),
        "truncated": torch.zeros(6, 3, dtype=torch.bool),
        "final_values": torch.zeros(6, 3),
        "gamma": 0.99,
        "lam": 0.95,
    }
    adv, _ = compute_gae(rewards=r, values=v, terminated=term, **kw)
    for e in range(3):
        a_e, _ = compute_gae(
            rewards=r[:, e : e + 1],
            values=v[:, e : e + 1],
            terminated=term[:, e : e + 1],
            next_value=kw["next_value"][e : e + 1],
            truncated=kw["truncated"][:, e : e + 1],
            final_values=kw["final_values"][:, e : e + 1],
            gamma=0.99,
            lam=0.95,
        )
        torch.testing.assert_close(adv[:, e : e + 1], a_e)

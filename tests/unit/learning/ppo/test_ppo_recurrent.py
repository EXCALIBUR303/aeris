"""Spec §28 tests: (a) no memory leaks across a done, (c) hidden-state shape
/device; plus rollout/replay equivalence -- the stepwise rollout path and the
sequence replay used in training must produce identical outputs."""

from __future__ import annotations

import torch

from aeris.learning.networks.heads import ActionSpec, ActorCritic, NetworkConfig

SHAPES = {"state": (4,), "img": (1, 8, 8)}


def _obs(t: int, b: int, seed: int = 0) -> dict[str, torch.Tensor]:
    g = torch.Generator().manual_seed(seed)
    return {
        "state": torch.randn(t, b, 4, generator=g),
        "img": torch.rand(t, b, 1, 8, 8, generator=g),
    }


def _model(recurrent: bool, kind: str = "discrete") -> ActorCritic:
    torch.manual_seed(0)
    return ActorCritic(
        SHAPES,
        ActionSpec(kind, 3),  # type: ignore[arg-type]
        NetworkConfig(
            image_dim=16, vector_dim=8, trunk_sizes=(16,), recurrent=recurrent, hidden_size=12
        ),
    )


def test_hidden_state_shape_and_device() -> None:
    m = _model(True)
    h = m.initial_state(5)
    assert h.shape == (5, 12) and h.device.type == "cpu" and bool((h == 0).all())
    assert _model(False).initial_state(5).shape == (5, 0)
    o = {k: v[0] for k, v in _obs(1, 5).items()}
    _, v, h2 = m.step(o, h, torch.ones(5))
    assert v.shape == (5,) and h2.shape == (5, 12)


def test_leak_output_after_done_equals_fresh_agent() -> None:
    m = _model(True)
    o = {k: v[0] for k, v in _obs(1, 4, seed=3).items()}
    stale = torch.randn(4, 12) * 5
    d1, v1, h1 = m.step(o, stale, torch.ones(4))  # episode start: stale memory must be wiped
    d2, v2, h2 = m.step(o, m.initial_state(4), torch.zeros(4))
    assert isinstance(d1.probs, torch.Tensor)  # categorical
    torch.testing.assert_close(d1.probs, d2.probs)  # type: ignore[union-attr]
    torch.testing.assert_close(v1, v2)
    torch.testing.assert_close(h1, h2)


def test_sequence_replay_matches_stepwise_rollout_including_resets() -> None:
    for recurrent in (True, False):
        m = _model(recurrent)
        t_len, b = 7, 3
        obs = _obs(t_len, b, seed=5)
        starts = torch.zeros(t_len, b)
        starts[0] = 1.0
        starts[3, 1] = 1.0  # env 1 begins a new episode at t=3
        h0 = torch.randn(b, m.hidden_size)
        actions = torch.randint(0, 3, (t_len, b))
        h, lps, vals = h0, [], []
        for t in range(t_len):
            d, v, h = m.step({k: x[t] for k, x in obs.items()}, h, starts[t])
            lps.append(d.log_prob(actions[t]))
            vals.append(v)
        lp_r, _, v_r = m.evaluate(obs, h0, starts, actions)
        torch.testing.assert_close(lp_r, torch.stack(lps))
        torch.testing.assert_close(v_r, torch.stack(vals))


def test_reset_mid_sequence_matches_a_fresh_replay_of_the_tail() -> None:
    m = _model(True)
    obs = _obs(6, 1, seed=7)
    starts = torch.zeros(6, 1)
    starts[2, 0] = 1.0
    actions = torch.zeros(6, 1, dtype=torch.long)
    lp_full, _, v_full = m.evaluate(obs, torch.randn(1, 12), starts, actions)
    tail = {k: v[2:] for k, v in obs.items()}
    lp_tail, _, v_tail = m.evaluate(tail, m.initial_state(1), torch.zeros(4, 1), actions[2:])
    torch.testing.assert_close(lp_full[2:], lp_tail)
    torch.testing.assert_close(v_full[2:], v_tail)


def test_initialization_gains() -> None:
    m = _model(False, "continuous")
    assert m.log_std is not None and bool((m.log_std == -0.5).all())
    # policy output gain 0.01 -> tiny weights; value gain 1.0 -> much larger.
    with torch.no_grad():
        assert float(m.actor.weight.abs().max()) < 0.05
        assert float(m.critic.weight.norm()) > 10 * float(m.actor.weight.norm())

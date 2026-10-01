"""Spec §27.4 #6: rollout-buffer shapes, and recurrent minibatches that keep
whole env sequences together with no cross-env mixing."""

from __future__ import annotations

import torch

from aeris.learning.networks.heads import ActionSpec
from aeris.learning.ppo.buffer import RolloutBuffer

T, N, H = 8, 6, 5


def _filled(action: ActionSpec) -> RolloutBuffer:
    b = RolloutBuffer(
        num_steps=T,
        num_envs=N,
        obs_shapes={"state": (2,), "img": (1, 3, 4)},
        action=action,
        hidden_size=H,
    )
    for t in range(T):
        tag = torch.arange(N, dtype=torch.float32) * 1000 + t  # encodes (env, t)
        b.add(
            t,
            obs={
                "state": tag.unsqueeze(1).repeat(1, 2),
                "img": tag.view(N, 1, 1, 1).repeat(1, 1, 3, 4),
            },
            action=tag.unsqueeze(1).repeat(1, action.n)
            if action.kind == "continuous"
            else torch.arange(N) % action.n,
            log_prob=tag,
            value=tag,
            reward=tag,
            terminated=torch.zeros(N, dtype=torch.bool),
            truncated=torch.zeros(N, dtype=torch.bool),
            final_value=tag,
            start=(tag % 3 == 0).float(),
            mask=torch.ones(N, action.n, dtype=torch.bool) if action.kind == "discrete" else None,
        )
    b.h0 = torch.arange(N, dtype=torch.float32).unsqueeze(1).repeat(1, H)
    return b


def test_shapes() -> None:
    b = _filled(ActionSpec("continuous", 3))
    assert b.obs["state"].shape == (T, N, 2) and b.obs["img"].shape == (T, N, 1, 3, 4)
    assert b.actions.shape == (T, N, 3) and b.masks is None
    for x in (
        b.log_probs,
        b.rewards,
        b.values,
        b.final_values,
        b.terminated,
        b.truncated,
        b.starts,
    ):
        assert x.shape == (T, N)
    d = _filled(ActionSpec("discrete", 4))
    assert d.actions.shape == (T, N) and d.actions.dtype == torch.long
    assert d.masks is not None and d.masks.shape == (T, N, 4)


def test_feed_forward_minibatches_cover_every_sample_once() -> None:
    b = _filled(ActionSpec("continuous", 1))
    adv = b.values.clone()
    seen = []
    for mb in b.minibatches(
        adv, adv, num_minibatches=4, recurrent=False, generator=torch.Generator().manual_seed(0)
    ):
        assert mb.obs["state"].shape[0] == 1
        seen += mb.old_values.reshape(-1).tolist()
        torch.testing.assert_close(mb.advantages, mb.old_values)  # rows stay aligned
    assert sorted(seen) == sorted(b.values.reshape(-1).tolist())


def test_recurrent_minibatches_keep_whole_sequences_without_mixing_envs() -> None:
    b = _filled(ActionSpec("discrete", 4))
    adv = b.values.clone()
    envs_seen: list[int] = []
    for mb in b.minibatches(
        adv, adv, num_minibatches=3, recurrent=True, generator=torch.Generator().manual_seed(0)
    ):
        assert mb.obs["state"].shape[:2] == (T, len(mb.env_index))
        for j, e in enumerate(mb.env_index.tolist()):
            envs_seen.append(e)
            expected = (
                torch.arange(T, dtype=torch.float32) + 1000 * e
            )  # env e's own t = 0..T-1, in order
            torch.testing.assert_close(mb.obs["state"][:, j, 0], expected)
            torch.testing.assert_close(mb.obs["img"][:, j, 0, 0, 0], expected)
            torch.testing.assert_close(mb.old_log_prob[:, j], expected)
            torch.testing.assert_close(mb.starts[:, j], (expected % 3 == 0).float())
            torch.testing.assert_close(mb.h0[j], torch.full((H,), float(e)))
    assert sorted(envs_seen) == list(range(N))

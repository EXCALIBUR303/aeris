"""PPO trainer (spec §27.2): collect a ``[T, N]`` rollout, compute GAE with
truncation bootstrapping, run ``K`` epochs of minibatch updates, log the
§27.3 diagnostics, evaluate on separate (validation-split) envs, checkpoint.

Everything that evolves during training lives on the trainer -- current
observation, GRU state, episode-start flags, normalizers, reward scaler,
the two torch generators (action sampling, minibatch permutation), the
update counter -- so :mod:`aeris.learning.ppo.checkpoint` can capture it
completely and resume bit-for-bit on CPU (§27.4 #9).
"""

from __future__ import annotations

import json
import math
import random
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import torch

from aeris.learning.envs.registry import EnvHandle, make_env
from aeris.learning.networks.heads import ActorCritic, NetworkConfig
from aeris.learning.ppo.buffer import RolloutBuffer
from aeris.learning.ppo.config import TrainConfig
from aeris.learning.ppo.diagnostics import (
    ReturnScaler,
    RunningMeanStd,
    assert_finite,
    explained_variance,
)
from aeris.learning.ppo.gae import compute_gae
from aeris.learning.ppo.losses import normalize_advantages, ppo_loss


@dataclass
class UpdateStats:
    update: int
    global_step: int
    learning_rate: float
    policy_loss: float
    value_loss: float
    entropy: float
    approx_kl: float
    clip_fraction: float
    explained_variance: float
    grad_norm: float
    sps: float
    epochs_run: int
    kl_early_stopped: bool
    episodes: int
    episode_return: float
    episode_length: float
    success_rate: float
    loss_trace: list[float] = field(default_factory=list)  # every minibatch's total loss


def resolve_device(requested: str) -> torch.device:
    """``mps`` falls back to ``cpu`` when unavailable (spec §7: CPU fallback
    is mandatory and automatic on this macOS)."""
    if requested == "mps" and not torch.backends.mps.is_available():
        return torch.device("cpu")
    return torch.device(requested)


def seed_everything(seed: int) -> None:
    random.seed(seed)
    torch.manual_seed(seed)


class PPOTrainer:
    def __init__(
        self,
        cfg: TrainConfig,
        *,
        config_hash: str = "",
        run_dir: Path | None = None,
        env: EnvHandle | None = None,
        log: Callable[[str], None] | None = print,
    ) -> None:
        self.cfg, self.config_hash, self.run_dir, self._log = cfg, config_hash, run_dir, log
        if cfg.torch_threads:
            torch.set_num_threads(cfg.torch_threads)
        seed_everything(cfg.seed)
        self.device = resolve_device(cfg.device)
        self.handle = env or make_env(cfg.env, seed=cfg.seed)
        n, p = self.handle.env.num_envs, cfg.ppo
        net = cfg.network
        self.model = ActorCritic(
            self.handle.obs_shapes,
            self.handle.action,
            NetworkConfig(
                image_dim=net.image_dim,
                vector_dim=net.vector_dim,
                trunk_sizes=tuple(net.trunk_sizes),
                recurrent=net.recurrent,
                hidden_size=net.hidden_size,
                log_std_init=net.log_std_init,
            ),
        ).to(self.device)
        self.optimizer = torch.optim.Adam(
            self.model.parameters(), lr=p.learning_rate, eps=p.adam_eps
        )
        self.obs_rms = (
            {k: RunningMeanStd(s) for k, s in self.handle.obs_shapes.items() if len(s) == 1}
            if p.normalize_obs
            else {}
        )
        self.reward_scaler = ReturnScaler(n, p.gamma) if p.scale_rewards else None
        self.sample_gen = torch.Generator(device=self.device).manual_seed(cfg.seed)
        self.perm_gen = torch.Generator().manual_seed(cfg.seed + 1)
        self.buffer = RolloutBuffer(
            num_steps=p.num_steps,
            num_envs=n,
            obs_shapes=self.handle.obs_shapes,
            action=self.handle.action,
            hidden_size=self.model.hidden_size,
            device=self.device,
        )
        self.update = 0
        self.global_step = 0
        self.best_eval: tuple[float, float] | None = None
        self._eval_handle: EnvHandle | None = None
        obs_np, info = self.handle.env.reset(seed=cfg.seed)
        self.obs = self._prep_obs(obs_np, update_stats=True)
        self.mask = self._mask(info)
        self.h = self.model.initial_state(n, self.device)
        self.starts = torch.ones(n, device=self.device)
        self.ep_return = np.zeros(n)
        self.ep_length = np.zeros(n, dtype=np.int64)

    # -- observation plumbing ---------------------------------------------------------
    def _prep_obs(
        self, obs: dict[str, np.ndarray], *, update_stats: bool
    ) -> dict[str, torch.Tensor]:
        out = {}
        for k, v in obs.items():
            t = torch.as_tensor(np.asarray(v, dtype=np.float32))
            if k in self.obs_rms:
                if update_stats:
                    self.obs_rms[k].update(t)
                t = self.obs_rms[k].normalize(t)
            out[k] = t.to(self.device)
        return out

    def _mask(self, info: dict[str, Any]) -> torch.Tensor | None:
        m = info.get("action_mask")
        return (
            None
            if m is None
            else torch.as_tensor(np.asarray(m, dtype=np.bool_), device=self.device)
        )

    def _env_action(self, action: torch.Tensor) -> np.ndarray:
        a = action.cpu().numpy()
        if self.handle.action.kind == "continuous":
            a = np.clip(
                a, self.handle.action_low, self.handle.action_high
            )  # log-prob stays unclipped
        return a

    # -- rollout --------------------------------------------------------------------------
    def collect(self) -> list[tuple[float, int, bool]]:
        env, buf, n = self.handle.env, self.buffer, self.handle.env.num_envs
        buf.h0.copy_(self.h)
        episodes: list[tuple[float, int, bool]] = []
        for t in range(self.cfg.ppo.num_steps):
            with torch.no_grad():
                dist, value, h_next = self.model.step(self.obs, self.h, self.starts, self.mask)
                action = dist.sample(self.sample_gen)
                log_prob = dist.log_prob(action)
            obs_np, rew, term, trunc, info = env.step(self._env_action(action))
            term_t = torch.as_tensor(np.asarray(term, dtype=np.bool_), device=self.device)
            trunc_t = torch.as_tensor(np.asarray(trunc, dtype=np.bool_), device=self.device)
            done = np.asarray(term, dtype=np.bool_) | np.asarray(trunc, dtype=np.bool_)
            final_value = torch.zeros(n, device=self.device)
            if bool(trunc_t.any()):
                final_obs = self._prep_obs(info["final_obs"], update_stats=False)
                with torch.no_grad():  # V of the true final obs, with the pre-reset memory
                    _, v_final, _ = self.model.step(
                        final_obs, h_next, torch.zeros(n, device=self.device)
                    )
                final_value = torch.where(trunc_t, v_final, final_value)
            reward = torch.as_tensor(np.asarray(rew, dtype=np.float32), device=self.device)
            if self.reward_scaler is not None:
                reward = self.reward_scaler(reward, torch.as_tensor(done))
            buf.add(
                t,
                obs=self.obs,
                action=action,
                log_prob=log_prob,
                value=value,
                reward=reward,
                terminated=term_t,
                truncated=trunc_t,
                final_value=final_value,
                start=self.starts,
                mask=self.mask,
            )
            self.ep_return += np.asarray(rew, dtype=np.float64)
            self.ep_length += 1
            success = np.asarray(info.get("success", np.zeros(n, dtype=np.bool_)), dtype=np.bool_)
            for i in np.nonzero(done)[0]:
                episodes.append(
                    (float(self.ep_return[i]), int(self.ep_length[i]), bool(success[i]))
                )
                self.ep_return[i], self.ep_length[i] = 0.0, 0
            self.obs = self._prep_obs(obs_np, update_stats=True)
            self.mask = self._mask(info)
            self.h = h_next
            self.starts = torch.as_tensor(done, dtype=torch.float32, device=self.device)
            self.global_step += n
        return episodes

    # -- update ---------------------------------------------------------------------------
    def learn(self) -> UpdateStats:
        cfg, p, buf = self.cfg, self.cfg.ppo, self.buffer
        t0 = time.perf_counter()
        episodes = self.collect()
        with torch.no_grad():
            _, next_value, _ = self.model.step(self.obs, self.h, self.starts, self.mask)
        adv, ret = compute_gae(
            rewards=buf.rewards,
            values=buf.values,
            next_value=next_value,
            terminated=buf.terminated,
            truncated=buf.truncated,
            final_values=buf.final_values,
            gamma=p.gamma,
            lam=p.gae_lambda,
        )
        lr = p.learning_rate
        if p.anneal_lr:
            lr *= 1.0 - self.update / cfg.num_updates
        for g in self.optimizer.param_groups:
            g["lr"] = lr

        acc: dict[str, list[float]] = {
            k: [] for k in ("pg", "v", "ent", "kl", "clip", "gn", "loss")
        }
        epochs_run, early = 0, False
        for epoch in range(p.update_epochs):
            epoch_kl = []
            for mb in buf.minibatches(
                adv,
                ret,
                num_minibatches=p.num_minibatches,
                recurrent=self.model.core is not None,
                generator=self.perm_gen,
            ):
                log_prob, entropy, values = self.model.evaluate(
                    mb.obs, mb.h0, mb.starts, mb.actions, mb.masks
                )
                a = mb.advantages.reshape(-1)
                if p.normalize_advantages:
                    a = normalize_advantages(a)
                terms = ppo_loss(
                    new_log_prob=log_prob.reshape(-1),
                    old_log_prob=mb.old_log_prob.reshape(-1),
                    advantages=a,
                    new_values=values.reshape(-1),
                    old_values=mb.old_values.reshape(-1),
                    returns=mb.returns.reshape(-1),
                    entropy=entropy.reshape(-1),
                    clip_eps=p.clip_eps,
                    vf_coef=p.vf_coef,
                    ent_coef=p.ent_coef,
                    clip_value_loss=p.clip_value_loss,
                )
                ctx = {"update": self.update, "epoch": epoch, "global_step": self.global_step}
                assert_finite(
                    {
                        "loss": terms.loss,
                        "policy_loss": terms.policy_loss,
                        "value_loss": terms.value_loss,
                    },
                    ctx,
                )
                self.optimizer.zero_grad(set_to_none=True)
                terms.loss.backward()  # type: ignore[no-untyped-call]
                grad_norm = torch.nn.utils.clip_grad_norm_(self.model.parameters(), p.max_grad_norm)
                assert_finite({"grad_norm": grad_norm}, ctx)
                self.optimizer.step()
                assert_finite(dict(self.model.named_parameters()), ctx)
                acc["pg"].append(float(terms.policy_loss.detach()))
                acc["v"].append(float(terms.value_loss.detach()))
                acc["ent"].append(float(terms.entropy.detach()))
                acc["kl"].append(float(terms.approx_kl))
                acc["clip"].append(float(terms.clip_fraction))
                acc["gn"].append(float(grad_norm))
                acc["loss"].append(float(terms.loss.detach()))
                epoch_kl.append(float(terms.approx_kl))
            epochs_run = epoch + 1
            if p.kl_early_stop and float(np.mean(epoch_kl)) > 1.5 * p.target_kl:
                early = True
                break
        self.update += 1
        elapsed = time.perf_counter() - t0
        rets = [e[0] for e in episodes]
        return UpdateStats(
            update=self.update,
            global_step=self.global_step,
            learning_rate=lr,
            policy_loss=float(np.mean(acc["pg"])),
            value_loss=float(np.mean(acc["v"])),
            entropy=float(np.mean(acc["ent"])),
            approx_kl=float(np.mean(acc["kl"])),
            clip_fraction=float(np.mean(acc["clip"])),
            explained_variance=explained_variance(buf.values.flatten(), ret.flatten()),
            grad_norm=float(np.mean(acc["gn"])),
            sps=buf.num_steps * buf.num_envs / elapsed,
            epochs_run=epochs_run,
            kl_early_stopped=early,
            episodes=len(episodes),
            episode_return=float(np.mean(rets)) if rets else float("nan"),
            episode_length=float(np.mean([e[1] for e in episodes])) if episodes else float("nan"),
            success_rate=float(np.mean([e[2] for e in episodes])) if episodes else float("nan"),
            loss_trace=acc["loss"],
        )

    # -- evaluation -------------------------------------------------------------------------
    def evaluate(self, *, deterministic: bool, episodes: int | None = None) -> dict[str, float]:
        """Run the current policy on the *evaluation* envs (validation-split
        worlds for FastSim tasks), observation normalization frozen."""
        ev = self.cfg.eval
        if self._eval_handle is None:
            self._eval_handle = make_env(
                self.cfg.env,
                seed=self.cfg.seed + ev.seed_offset,
                evaluation=True,
                num_envs=ev.num_envs,
            )
        return run_policy(
            self.model,
            self._eval_handle,
            episodes=episodes or ev.episodes,
            deterministic=deterministic,
            seed=self.cfg.seed + ev.seed_offset,
            prep_obs=lambda o: self._prep_obs(o, update_stats=False),
            device=self.device,
        )

    # -- main loop ----------------------------------------------------------------------------
    def train(self, *, max_updates: int | None = None) -> list[UpdateStats]:
        from aeris.learning.ppo.checkpoint import save_checkpoint

        history = []
        stop = (
            self.cfg.num_updates
            if max_updates is None
            else min(self.cfg.num_updates, self.update + max_updates)
        )
        while self.update < stop:
            stats = self.learn()
            history.append(stats)
            self._record(stats)
            every = self.cfg.eval.every_updates
            if every and self.update % every == 0 and self.update < self.cfg.num_updates:
                self._evaluate_and_maybe_keep_best()
            ck = self.cfg.checkpoint_every_updates
            if self.run_dir is not None and ck and self.update % ck == 0:
                save_checkpoint(self, self.run_dir / "checkpoints" / f"update_{self.update:06d}.pt")
        return history

    def _evaluate_and_maybe_keep_best(self) -> dict[str, dict[str, float]]:
        from aeris.learning.ppo.checkpoint import save_checkpoint

        det = self.evaluate(deterministic=True)
        sto = self.evaluate(deterministic=False)
        result = {"deterministic": det, "stochastic": sto}
        self._write_jsonl(
            "eval.jsonl", {"update": self.update, "global_step": self.global_step, **result}
        )
        key = (
            det["success_rate"] if not math.isnan(det["success_rate"]) else 0.0,
            det["mean_return"],
        )
        if self.best_eval is None or key > self.best_eval:
            self.best_eval = key
            if self.run_dir is not None:
                save_checkpoint(self, self.run_dir / "checkpoints" / "best.pt")
        if self._log:
            self._log(
                f"[eval @ {self.update}] det return {det['mean_return']:.3f} "
                f"success {det['success_rate']:.2f} | sto return {sto['mean_return']:.3f} "
                f"success {sto['success_rate']:.2f}"
            )
        return result

    def _write_jsonl(self, name: str, row: dict[str, Any]) -> None:
        if self.run_dir is None:
            return
        self.run_dir.mkdir(parents=True, exist_ok=True)
        with (self.run_dir / name).open("a") as f:
            f.write(json.dumps(row) + "\n")

    def _record(self, s: UpdateStats) -> None:
        row = asdict(s)
        row.pop("loss_trace")
        self._write_jsonl("metrics.jsonl", row)
        if self._log:
            self._log(
                f"[{s.update}/{self.cfg.num_updates}] step {s.global_step} "
                f"ret {s.episode_return:.3f} succ {s.success_rate:.2f} "
                f"pg {s.policy_loss:.4f} v {s.value_loss:.4f} ent {s.entropy:.3f} "
                f"kl {s.approx_kl:.4f} clip {s.clip_fraction:.3f} ev {s.explained_variance:.2f} "
                f"sps {s.sps:.0f}"
            )


def run_policy(
    model: ActorCritic,
    handle: EnvHandle,
    *,
    episodes: int,
    deterministic: bool,
    seed: int,
    prep_obs: Callable[[dict[str, np.ndarray]], dict[str, torch.Tensor]],
    device: torch.device,
    max_steps: int = 1_000_000,
) -> dict[str, float]:
    """Roll ``model`` until ``episodes`` episodes finish (first-finished
    order); report mean return/length and success rate."""
    env, n = handle.env, handle.env.num_envs
    gen = torch.Generator(device=device).manual_seed(seed)
    obs_np, info = env.reset(seed=seed)
    obs = prep_obs(obs_np)
    mask = info.get("action_mask")
    h = model.initial_state(n, device)
    starts = torch.ones(n, device=device)
    ret, length = np.zeros(n), np.zeros(n, dtype=np.int64)
    done_eps: list[tuple[float, int, bool]] = []
    for _ in range(max_steps):
        m = None if mask is None else torch.as_tensor(np.asarray(mask), device=device)
        with torch.no_grad():
            dist, _, h = model.step(obs, h, starts, m)
            a = dist.mode() if deterministic else dist.sample(gen)
        a_np = a.cpu().numpy()
        if handle.action.kind == "continuous":
            a_np = np.clip(a_np, handle.action_low, handle.action_high)
        obs_np, rew, term, trunc, info = env.step(a_np)
        done = np.asarray(term, dtype=np.bool_) | np.asarray(trunc, dtype=np.bool_)
        ret += np.asarray(rew, dtype=np.float64)
        length += 1
        succ = np.asarray(info.get("success", np.zeros(n, dtype=np.bool_)), dtype=np.bool_)
        for i in np.nonzero(done)[0]:
            done_eps.append((float(ret[i]), int(length[i]), bool(succ[i])))
            ret[i], length[i] = 0.0, 0
        if len(done_eps) >= episodes:
            break
        obs, mask = prep_obs(obs_np), info.get("action_mask")
        starts = torch.as_tensor(done, dtype=torch.float32, device=device)
    eps = done_eps[:episodes]
    return {
        "episodes": float(len(eps)),
        "mean_return": float(np.mean([e[0] for e in eps])) if eps else float("nan"),
        "mean_length": float(np.mean([e[1] for e in eps])) if eps else float("nan"),
        "success_rate": float(np.mean([e[2] for e in eps])) if eps else float("nan"),
    }

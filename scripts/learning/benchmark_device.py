#!/usr/bin/env python3
"""CPU vs MPS benchmark for PPO training (spec §51 Phase 14 validation gate:
"SPS benchmark recorded (CPU vs MPS) with the device decision"; spec §7's
ASSUMPTION that small RL nets run faster on CPU, to be tested here).

Three measurements per device:

1. ``train_sps``: end-to-end PPO training throughput (rollout + GAE + 4
   epochs of updates) on the real configs, local-nav (depth CNN + state
   MLP) and the GRU memory task, after one warm-up update.
2. ``update_ms``: learner-only time for one minibatch forward+backward+Adam
   step on the exploration encoder (8x64x64 map crops + state, GRU), the
   heaviest network Phase 19 will train.
3. ``act_ms``: one batched policy forward (action selection), N=16.

Writes ``results/learning/device_benchmark.json``.

Usage::

    uv run python scripts/learning/benchmark_device.py
"""

from __future__ import annotations

import json
import platform
import statistics
import sys
import time
from pathlib import Path

import torch

from aeris.learning.networks.heads import ActionSpec, ActorCritic, NetworkConfig
from aeris.learning.ppo.config import load_train_config
from aeris.learning.ppo.trainer import PPOTrainer

_OUT = Path(__file__).resolve().parents[2] / "results" / "learning" / "device_benchmark.json"


def _sync(device: torch.device) -> None:
    if device.type == "mps":
        torch.mps.synchronize()


def train_sps(config: str, device: str, overrides: tuple[str, ...], updates: int = 3) -> float:
    cfg, _, _ = load_train_config(config, (*overrides, f"device={device}", "total_steps=100000000"))
    tr = PPOTrainer(cfg, log=None)
    tr.learn()  # warm-up (allocations, kernel compilation)
    _sync(tr.device)
    t0 = time.perf_counter()
    for _ in range(updates):
        tr.learn()
    _sync(tr.device)
    return updates * cfg.batch_size / (time.perf_counter() - t0)


def exploration_update_ms(
    device: torch.device, batch_envs: int = 8, t_len: int = 16, reps: int = 10
) -> float:
    torch.manual_seed(0)
    m = ActorCritic(
        {"map": (8, 64, 64), "state": (3,)},
        ActionSpec("discrete", 25),
        NetworkConfig(recurrent=True),
    ).to(device)
    opt = torch.optim.Adam(m.parameters(), lr=3e-4, eps=1e-5)
    obs = {
        "map": torch.rand(t_len, batch_envs, 8, 64, 64, device=device),
        "state": torch.rand(t_len, batch_envs, 3, device=device),
    }
    starts = torch.zeros(t_len, batch_envs, device=device)
    actions = torch.randint(0, 25, (t_len, batch_envs), device=device)
    h0 = m.initial_state(batch_envs, device)
    times = []
    for i in range(reps + 2):
        _sync(device)
        t0 = time.perf_counter()
        lp, ent, v = m.evaluate(obs, h0, starts, actions)
        loss = -(lp.mean() + 0.01 * ent.mean()) + 0.5 * (v**2).mean()
        opt.zero_grad()
        loss.backward()  # type: ignore[no-untyped-call]
        opt.step()
        _sync(device)
        if i >= 2:
            times.append((time.perf_counter() - t0) * 1e3)
    return statistics.median(times)


def act_ms(device: torch.device, reps: int = 50) -> float:
    torch.manual_seed(0)
    m = ActorCritic(
        {"depth": (1, 30, 40), "state": (10,)}, ActionSpec("continuous", 3), NetworkConfig()
    ).to(device)
    obs = {
        "depth": torch.rand(16, 1, 30, 40, device=device),
        "state": torch.rand(16, 10, device=device),
    }
    h, s = m.initial_state(16, device), torch.zeros(16, device=device)
    times = []
    with torch.no_grad():
        for i in range(reps + 5):
            _sync(device)
            t0 = time.perf_counter()
            d, _, _ = m.step(obs, h, s)
            d.sample().cpu()  # include the device->host copy the env step needs
            if i >= 5:
                times.append((time.perf_counter() - t0) * 1e3)
    return statistics.median(times)


def mps_parity() -> dict[str, float]:
    """Same weights, same batch on CPU and MPS: max |difference| of the
    log-probs, values and gradients (spec §51 Phase 14 known risk: "MPS
    numerical differences")."""
    torch.manual_seed(0)
    cpu = ActorCritic(
        {"map": (8, 64, 64), "state": (3,)},
        ActionSpec("discrete", 25),
        NetworkConfig(recurrent=True),
    )
    mps = ActorCritic(
        {"map": (8, 64, 64), "state": (3,)},
        ActionSpec("discrete", 25),
        NetworkConfig(recurrent=True),
    ).to("mps")
    mps.load_state_dict(cpu.state_dict())
    g = torch.Generator().manual_seed(1)
    obs = {
        "map": torch.rand(16, 4, 8, 64, 64, generator=g),
        "state": torch.rand(16, 4, 3, generator=g),
    }
    starts = torch.zeros(16, 4)
    starts[0] = 1.0
    actions = torch.randint(0, 25, (16, 4), generator=g)
    out = {}
    lps, vals, grads = [], [], []
    for m, dev in ((cpu, "cpu"), (mps, "mps")):
        lp, ent, v = m.evaluate(
            {k: x.to(dev) for k, x in obs.items()},
            m.initial_state(4, dev),
            starts.to(dev),
            actions.to(dev),
        )
        (lp.mean() + v.mean() + 0.01 * ent.mean()).backward()  # type: ignore[no-untyped-call]
        lps.append(lp.detach().cpu())
        vals.append(v.detach().cpu())
        grads.append(
            torch.cat(
                [p.grad.detach().cpu().flatten() for p in m.parameters() if p.grad is not None]
            )
        )
    out["max_abs_log_prob_diff"] = float((lps[0] - lps[1]).abs().max())
    out["max_abs_value_diff"] = float((vals[0] - vals[1]).abs().max())
    out["max_abs_grad_diff"] = float((grads[0] - grads[1]).abs().max())
    out["max_abs_grad"] = float(grads[0].abs().max())
    return out


def main() -> int:
    devices = ["cpu"] + (["mps"] if torch.backends.mps.is_available() else [])
    results: dict[str, dict[str, float]] = {}
    for dev in devices:
        d = torch.device(dev)
        r = {
            "train_sps_local_nav_mlp": train_sps(
                "ppo_localnav_smoke", dev, ("checkpoint_every_updates=0",)
            ),
            "train_sps_memory_gru": train_sps("ppo_toy_memory_gru", dev, ()),
            "update_ms_exploration_gru_8x16": exploration_update_ms(d),
            "act_ms_local_nav_n16": act_ms(d),
        }
        results[dev] = {k: round(v, 2) for k, v in r.items()}
        print(dev, results[dev], flush=True)
    parity = mps_parity() if "mps" in results else {}
    print("parity", parity, flush=True)

    def pick(key: str) -> str:
        if "mps" not in results:
            return "cpu"
        return "mps" if results["mps"][key] > 1.1 * results["cpu"][key] else "cpu"

    decision = {
        "image_observation_tasks (local_nav, exploration)": pick("train_sps_local_nav_mlp"),
        "low_dimensional_tasks (toy known-answer tasks)": pick("train_sps_memory_gru"),
    }
    doc = {
        "torch": torch.__version__,
        "macos": platform.mac_ver()[0],
        "machine": platform.machine(),
        "torch_threads": torch.get_num_threads(),
        "mps_available": torch.backends.mps.is_available(),
        "results": results,
        "mps_parity": parity,
        "decision_rule": "per task class: mps only if its end-to-end train SPS is > 1.1x CPU's",
        "decision": decision,
    }
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    _OUT.write_text(json.dumps(doc, indent=2))
    print("decision:", decision, "->", _OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())

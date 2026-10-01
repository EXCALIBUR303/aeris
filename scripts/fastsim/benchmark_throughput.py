#!/usr/bin/env python3
"""FastSim local-nav throughput benchmark (spec §51 Phase 13 validation gate
item 4: >= 2000 env-steps/s aggregate on this Mac).

Runs the native batched ``LocalNavVectorEnv`` on real *train*-split
procedural worlds (one per family: f1_rubble/f2_office/f3_warehouse),
with the identified dynamics/pose-noise config and random actions, and
reports aggregate env-steps/s (after a warm-up that absorbs Numba JIT
compilation). Every step includes: action decoding, the S3 shield, 5
dynamics substeps, collision checking, reward, a full depth render
(30x40 rays), and observation building.

Usage::

    uv run python scripts/fastsim/benchmark_throughput.py [--num-envs 16 32 64]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

from aeris.learning.envs.local_nav import LocalNavEnvConfig, LocalNavVectorEnv
from aeris.simulation.worlds.batch import generate_batch, load_world_spec
from aeris.simulation.worlds.spec import WorldFamily, WorldSpec, WorldSplit

_REPO_ROOT = Path(__file__).resolve().parents[2]
_WORLDS = _REPO_ROOT / "results" / "worlds"
_OUT = _REPO_ROOT / "results" / "fastsim" / "throughput.json"


def train_worlds() -> list[WorldSpec]:
    specs = []
    for fam in (WorldFamily.RUBBLE, WorldFamily.OFFICE, WorldFamily.WAREHOUSE):
        path = _WORLDS / fam.value / "train" / f"{fam.value}_0.json"
        if not path.exists():
            generate_batch(
                family=fam, split=WorldSplit.TRAIN, n=1, out_dir=_WORLDS, render_sdf=False
            )
        specs.append(load_world_spec(path))
    return specs


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--num-envs", type=int, nargs="+", default=[16, 32, 64])
    parser.add_argument("--steps", type=int, default=300)
    args = parser.parse_args()
    worlds = train_worlds()
    results = []
    for n in args.num_envs:
        env = LocalNavVectorEnv(worlds, num_envs=n, config=LocalNavEnvConfig.identified(), seed=0)
        env.reset(seed=0)
        rng = np.random.default_rng(0)
        for _ in range(20):  # warm-up: JIT + caches
            env.step(rng.uniform(-1, 1, (n, 3)))
        t0 = time.perf_counter()
        for _ in range(args.steps):
            env.step(rng.uniform(-1, 1, (n, 3)))
        dt = time.perf_counter() - t0
        sps = n * args.steps / dt
        results.append(
            {
                "num_envs": n,
                "steps": args.steps,
                "wall_s": round(dt, 3),
                "env_steps_per_s": round(sps, 1),
            }
        )
        print(results[-1], flush=True)
    best = max(r["env_steps_per_s"] for r in results)
    summary = {
        "gate": ">= 2000 env-steps/s aggregate (local nav)",
        "best_env_steps_per_s": best,
        "passed": best >= 2000,
        "runs": results,
    }
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    _OUT.write_text(json.dumps(summary, indent=2))
    print(f"best {best:.0f} env-steps/s -> {'PASS' if summary['passed'] else 'FAIL'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

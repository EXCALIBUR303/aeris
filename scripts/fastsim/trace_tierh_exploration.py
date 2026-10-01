#!/usr/bin/env python3
"""Instrumented re-run of one Phase 12 Tier H exploration episode, for the
Phase 13 gate-6 diagnosis (FastSim's mirror of the loop stalled where Tier H
didn't).

Runs Phase 12's own episode code unmodified (``_attempt_episode`` from
``scripts/run_p12_exploration_experiment.py``); the only addition is a
pass-through strategy wrapper that records every ``select_subgoal`` call
(sim time, EKF pose, yaw, chosen cell) -- i.e. every replan Tier H made --
plus the episode's GT trajectory, which Phase 12 did not persist.

Usage::

    uv run python scripts/fastsim/trace_tierh_exploration.py \
        --world f1_rubble --method nearest_frontier
"""

from __future__ import annotations

import argparse
import asyncio
import itertools
import json
import math
import sys
from pathlib import Path
from typing import Any

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO_ROOT / "scripts"))

import run_p12_exploration_experiment as p12  # noqa: E402
from aeris.autonomy.exploration.base import AgentPose, ExplorationStrategy, Subgoal  # noqa: E402
from aeris.evaluation.metrics.coverage import (  # noqa: E402
    reachable_free_cells_gt,
    running_coverage_trace,
)
from aeris.mapping.projection import BandGrid  # noqa: E402
from aeris.simulation.worlds.spec import WorldFamily, WorldSplit  # noqa: E402

_OUT_DIR = _REPO_ROOT / "results" / "fastsim" / "tierh_trace"


class _Recording:
    def __init__(self, inner: ExplorationStrategy) -> None:
        self.inner = inner
        self.calls: list[dict[str, Any]] = []

    def select_subgoal(self, grid: BandGrid, pose: AgentPose, t_s: float) -> Subgoal | None:
        sub = self.inner.select_subgoal(grid, pose, t_s)
        self.calls.append(
            {
                "t_s": round(t_s, 3),
                "x": round(pose.position_m.x, 3),
                "y": round(pose.position_m.y, 3),
                "yaw": round(pose.yaw_rad, 4),
                "cell": list(sub.cell) if sub is not None else None,
            }
        )
        return sub


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="f1_rubble")
    ap.add_argument("--method", default="nearest_frontier")
    args = ap.parse_args()

    config = yaml.safe_load(
        (_REPO_ROOT / "configs" / "experiments" / "p12_exploration.yaml").read_text()
    )
    spec = p12._ensure_world(WorldFamily(args.world), WorldSplit.VAL, config["seed_per_family"])
    recorder: list[_Recording] = []
    make = p12._make_strategy

    def wrapped(method: str, *, lambda_turn: float) -> ExplorationStrategy:
        r = _Recording(make(method, lambda_turn=lambda_turn))
        recorder.append(r)
        return r

    p12._make_strategy = wrapped
    run_dir = _OUT_DIR / f"{spec.name}_{args.method}"
    run_dir.mkdir(parents=True, exist_ok=True)
    episode = await p12._attempt_episode(
        world_spec=spec,
        method=args.method,
        lambda_turn=config["utility_lambda_turn"],
        config=config,
        run_dir=run_dir,
    )
    res = config["coverage_resolution_m"]
    lo, hi = spec.altitude_band_m
    trace = running_coverage_trace(
        spec,
        episode.gt_trajectory,
        reachable_free_cells=reachable_free_cells_gt(spec, resolution_m=res),
        z_lo_m=lo,
        z_hi_m=hi,
        resolution_m=res,
    )
    out = {
        "world": spec.name,
        "method": args.method,
        "reason": episode.reason,
        "coverage_final": trace[-1][1] if trace else 0.0,
        "n_subgoals_chosen": episode.n_subgoals_chosen,
        "n_rotate_scans": episode.n_rotate_scans,
        "shield_intervention_rate": episode.shield_intervention_rate,
        "replans": recorder[-1].calls if recorder else [],
        "gt_trajectory": [
            [round(t, 3), round(p.x, 3), round(p.y, 3), round(p.z, 3), round(yaw, 4)]
            for t, p, yaw in episode.gt_trajectory
        ],
    }
    (run_dir / "trace.json").write_text(json.dumps(out))
    gt = out["gt_trajectory"]
    path_len = sum(math.dist(a[1:3], b[1:3]) for a, b in itertools.pairwise(gt))
    print(
        f"C={out['coverage_final']:.3f} subgoals={episode.n_subgoals_chosen} "
        f"rotations={episode.n_rotate_scans} shield={episode.shield_intervention_rate:.2f} "
        f"replans={len(out['replans'])} gt_poses={len(gt)} path_len={path_len:.1f} m",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))

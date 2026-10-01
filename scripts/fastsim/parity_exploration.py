#!/usr/bin/env python3
"""Scripted-frontier coverage parity, FastSim vs Tier H (spec §51 Phase 13
validation gate item 6: "a scripted frontier policy running inside the
FastSim exploration env reproduces Tier H frontier coverage within a
reported gap").

Runs Phase 12's own classical strategies (nearest_frontier -- the gate --
and random, for context) through :meth:`ExplorationEnv.execute_subgoal` on
the same 3 val-split worlds Phase 12 used, from the same spawn poses, 3
seeds each, and scores C(180 s) with the same evaluator
(``aeris.evaluation.metrics.coverage`` at Phase 12's 0.5 m coverage
resolution). Three FastSim conditions:

- ``nominal``: 10 Hz control, fixed heading (Phase 12's loop logic).
- ``tierh_rate``: control period matched to Tier H's *measured* ~0.5 s
  loop period (its synchronous GT poll), fixed heading.
- ``face_travel``: 10 Hz, vehicle yaws toward its direction of travel --
  tests whether Phase 12's heavy shield throttling (and random beating
  frontier) comes from flying legs outside the camera's FOV.

Tier H numbers are read from Phase 12's own ``runs.jsonl``, not retyped.

**Startup scan.** Every Tier H episode's first ``select_subgoal`` runs
before any depth frame is integrated, returns ``None``, and so starts with a
2 s rotate scan (Phase 12's logs: >= 1 scan in 8/9 frontier episodes; the
Phase 13 instrumented trace, ``trace_tierh_exploration.py``, shows it at
t=0). FastSim integrates a frame at reset, so the scripted runner issues
that scan explicitly -- without it the vehicle never turns and, in the
shield-deadlocked worlds, sees only its spawn heading.

Usage::

    uv run python scripts/fastsim/parity_exploration.py
"""

from __future__ import annotations

import itertools
import json
import math
import random
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

from aeris.autonomy.exploration.base import ExplorationStrategy
from aeris.autonomy.exploration.frontier import NearestFrontierExploration
from aeris.autonomy.exploration.random import RandomExploration
from aeris.evaluation.metrics.coverage import reachable_free_cells_gt, running_coverage_trace
from aeris.evaluation.statistics import paired_bootstrap_ci
from aeris.learning.envs.exploration import ExplorationEnv, ExplorationEnvConfig
from aeris.simulation.worlds.batch import load_world_spec

_REPO_ROOT = Path(__file__).resolve().parents[2]
_P12 = _REPO_ROOT / "results" / "experiments" / "p12_exploration" / "runs.jsonl"
_OUT = _REPO_ROOT / "results" / "fastsim" / "parity_exploration.json"
_ROWS = _OUT.with_suffix(".rows.jsonl")  # per-episode checkpoint: a killed run resumes
_WORLDS = [
    _REPO_ROOT / "results" / "worlds" / f / "val" / f"{f}_10000.json"
    for f in ("f1_rubble", "f2_office", "f3_warehouse")
]
_CONDITIONS = {
    "nominal": {"control_dt_s": 0.1, "face_travel_direction": False},
    "tierh_rate": {"control_dt_s": 0.5, "face_travel_direction": False},
    "face_travel": {"control_dt_s": 0.1, "face_travel_direction": True},
}
_SEEDS = (0, 1, 2)
_COVERAGE_RES_M = 0.5  # Phase 12's coverage_resolution_m


def _strategy(method: str, seed: int) -> ExplorationStrategy:
    if method == "nearest_frontier":
        return NearestFrontierExploration()
    return RandomExploration(rng=random.Random(20261001 + seed))


def run_episode(spec: Any, method: str, cond: dict[str, Any], seed: int) -> dict[str, float]:
    env = ExplorationEnv([spec], config=ExplorationEnvConfig.identified(**cond), split_mode="val")
    env.reset(seed=seed, options={"world_index": 0})
    strat = _strategy(method, seed)
    collided = False
    rotations = int(env.execute_subgoal(None).rotated)  # Tier H's startup scan (see docstring)
    while not env.time_up:
        sub = strat.select_subgoal(env.planning_grid(), env.agent_pose(), env.t_s)
        res = env.execute_subgoal(sub.cell if sub is not None else None)
        rotations += int(res.rotated)
        if res.collided:
            collided = True
            break
    gt = env.gt_trajectory
    path_len = sum(math.hypot(b[1].x - a[1].x, b[1].y - a[1].y) for a, b in itertools.pairwise(gt))
    lo, hi = spec.altitude_band_m
    f = reachable_free_cells_gt(spec, resolution_m=_COVERAGE_RES_M)
    trace = running_coverage_trace(
        spec,
        env.gt_trajectory,
        reachable_free_cells=f,
        z_lo_m=lo,
        z_hi_m=hi,
        resolution_m=_COVERAGE_RES_M,
    )
    return {
        "coverage": trace[-1][1] if trace else 0.0,
        "shield_rate": env.shield_interventions / max(env.shield_ticks, 1),
        "decisions": env.decisions,
        "rotate_scans": rotations,
        "path_length_m": round(path_len, 2),
        "collided": collided,
    }


def main() -> int:
    tier_h: dict[tuple[str, str], list[float]] = defaultdict(list)
    tier_h_shield: dict[tuple[str, str], list[float]] = defaultdict(list)
    for line in _P12.read_text().splitlines():
        r = json.loads(line)
        if r["status"] == "completed" and r["method"] in ("nearest_frontier", "random"):
            tier_h[(r["world"], r["method"])].append(r["coverage_final"])
            tier_h_shield[(r["world"], r["method"])].append(r["shield_intervention_rate"])

    specs = [load_world_spec(p) for p in _WORLDS]
    _ROWS.parent.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = (
        [json.loads(ln) for ln in _ROWS.read_text().splitlines()] if _ROWS.exists() else []
    )
    done = {(r["condition"], r["world"], r["method"], r["seed"]) for r in rows}
    t0 = time.perf_counter()
    for cname, cond in _CONDITIONS.items():
        for spec in specs:
            for method in ("nearest_frontier", "random"):
                for seed in _SEEDS:
                    if (cname, spec.name, method, seed) in done:
                        continue
                    r = run_episode(spec, method, cond, seed)
                    row = {
                        "condition": cname,
                        "world": spec.name,
                        "method": method,
                        "seed": seed,
                        **r,
                    }
                    rows.append(row)
                    with _ROWS.open("a") as f:
                        f.write(json.dumps(row) + "\n")
                    print(row, f"[{time.perf_counter() - t0:.0f}s]", flush=True)

    summary: dict[str, Any] = {"conditions": {}}
    for cname in _CONDITIONS:
        per_world: dict[str, dict[str, dict[str, Any]]] = {}
        for spec in specs:
            entry: dict[str, dict[str, Any]] = {}
            for method in ("nearest_frontier", "random"):
                fs = [
                    float(r["coverage"])
                    for r in rows
                    if r["condition"] == cname and r["world"] == spec.name and r["method"] == method
                ]
                sh = [
                    float(r["shield_rate"])
                    for r in rows
                    if r["condition"] == cname and r["world"] == spec.name and r["method"] == method
                ]
                th = tier_h[(spec.name, method)]
                entry[method] = {
                    "fastsim_C180_mean": round(statistics.mean(fs), 4),
                    "tierh_C180_mean": round(statistics.mean(th), 4),
                    "fastsim_C180_range": [round(min(fs), 4), round(max(fs), 4)],
                    "tierh_C180_range": [round(min(th), 4), round(max(th), 4)],
                    "gap_fastsim_minus_tierh": round(statistics.mean(fs) - statistics.mean(th), 4),
                    "fastsim_shield_rate": round(statistics.mean(sh), 3),
                    "tierh_shield_rate": round(
                        statistics.mean(tier_h_shield[(spec.name, method)]), 3
                    ),
                }
            per_world[spec.name] = entry
        nf = [
            float(r["coverage"])
            for r in rows
            if r["condition"] == cname and r["method"] == "nearest_frontier"
        ]
        rd = [
            float(r["coverage"])
            for r in rows
            if r["condition"] == cname and r["method"] == "random"
        ]
        lo, hi = paired_bootstrap_ci(nf, rd, seed=12345)
        summary["conditions"][cname] = {
            "per_world": per_world,
            "frontier_minus_random": {
                "point": round(statistics.mean(nf) - statistics.mean(rd), 4),
                "ci95": [round(lo, 4), round(hi, 4)],
            },
            "frontier_abs_gap_vs_tierh_mean": round(
                statistics.mean(
                    abs(per_world[s.name]["nearest_frontier"]["gap_fastsim_minus_tierh"])
                    for s in specs
                ),
                4,
            ),
        }
    summary["rows"] = rows
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    _OUT.write_text(json.dumps(summary, indent=2))
    for cname, c in summary["conditions"].items():
        print(
            cname,
            "frontier-random",
            c["frontier_minus_random"],
            "| mean |gap| vs Tier H",
            c["frontier_abs_gap_vs_tierh_mean"],
        )
        for w, e in c["per_world"].items():
            print("   ", w, e)
    return 0


if __name__ == "__main__":
    sys.exit(main())

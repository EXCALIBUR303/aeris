#!/usr/bin/env python3
"""Analyze ``results/experiments/p12_exploration/runs.jsonl`` (spec §51
Phase 12): per-world/method coverage means, plus the H0.1 decision --
a paired bootstrap CI on ``coverage_final`` (frontier - random), pooled
across all (world, repeat) pairs, per the pre-registered rule in
``configs/experiments/preregistration/p12_h0_1.md``.
"""

from __future__ import annotations

import json
import statistics
import sys
from pathlib import Path
from typing import Any

from aeris.evaluation.statistics import paired_bootstrap_ci

_REPO_ROOT = Path(__file__).resolve().parents[1]
_RUNS_PATH = _REPO_ROOT / "results" / "experiments" / "p12_exploration" / "runs.jsonl"
_OUT_PATH = _REPO_ROOT / "results" / "experiments" / "p12_exploration" / "summary.md"

_METRICS = [
    ("coverage_final", "C(180s) [H0.1's primary metric]"),
    ("n_subgoals_chosen", "Subgoals chosen"),
    ("n_rotate_scans", "Rotate-in-place scans"),
    ("shield_intervention_rate", "Shield intervention rate"),
]

_MIN_MEANINGFUL_EFFECT_PP = 0.10  # spec §6: H0.1's minimum meaningful effect, +10 percentage points


def load_records(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def main() -> int:
    records = load_records(_RUNS_PATH)
    worlds = sorted({r["world"] for r in records})
    methods = sorted({r["method"] for r in records})

    lines = ["# AERIS Phase 12 exploration experiment: results summary\n"]
    lines.append(f"Total records: {len(records)}\n")

    for world in worlds:
        lines.append(f"\n## {world}\n")
        for method in methods:
            group = [r for r in records if r["world"] == world and r["method"] == method]
            completed = [r for r in group if r["status"] == "completed"]
            failed = [r for r in group if r["status"] != "completed"]
            lines.append(
                f"\n### {method} ({len(completed)}/{len(group)} completed, {len(failed)} failed)\n"
            )
            if not completed:
                continue
            lines.append("| Metric | Mean |")
            lines.append("|---|---|")
            for key, label in _METRICS:
                values = [float(r[key]) for r in completed]
                lines.append(f"| {label} | {statistics.mean(values):.4f} |")

    # H0.1: paired bootstrap CI on coverage_final, frontier - random,
    # pooled across every (world, repeat) pair (spec §6: "every method
    # runs on the same (world seed, start pose) pairs").
    lines.append("\n## H0.1 decision: nearest_frontier - random, coverage_final\n")
    frontier_by_key = {
        (r["world"], r["repeat"]): r["coverage_final"]
        for r in records
        if r["method"] == "nearest_frontier" and r["status"] == "completed"
    }
    random_by_key = {
        (r["world"], r["repeat"]): r["coverage_final"]
        for r in records
        if r["method"] == "random" and r["status"] == "completed"
    }
    paired_keys = sorted(set(frontier_by_key) & set(random_by_key))
    lines.append(f"Paired episodes: {len(paired_keys)}\n")

    if len(paired_keys) >= 2:
        a = [frontier_by_key[k] for k in paired_keys]
        b = [random_by_key[k] for k in paired_keys]
        point_estimate = statistics.mean(a) - statistics.mean(b)
        lo, hi = paired_bootstrap_ci(a, b, seed=12345)
        excludes_zero = lo > 0.0 or hi < 0.0
        meaningful = point_estimate > _MIN_MEANINGFUL_EFFECT_PP
        rejected = excludes_zero and meaningful and point_estimate > 0
        lines.append(f"Point estimate (frontier - random): {point_estimate:+.4f}\n")
        lines.append(f"95% paired bootstrap CI: [{lo:+.4f}, {hi:+.4f}]\n")
        lines.append(f"CI excludes 0: {excludes_zero}\n")
        lines.append(
            f"Point estimate exceeds +{_MIN_MEANINGFUL_EFFECT_PP:.2f} (10pp) minimum meaningful effect: {meaningful}\n"
        )
        lines.append(
            f"**H0.1 {'REJECTED in favor of H1 (frontier > random)' if rejected else 'NOT rejected'}**\n"
        )
    else:
        lines.append("Fewer than 2 paired episodes -- cannot compute a bootstrap CI.\n")

    summary_text = "\n".join(lines) + "\n"
    _OUT_PATH.write_text(summary_text)
    print(summary_text)
    print(f"Wrote {_OUT_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

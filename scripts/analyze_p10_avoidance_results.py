#!/usr/bin/env python3
"""Analyze ``results/experiments/p10_avoidance/runs.jsonl`` (spec §51 Phase 10):
per-scenario/method summary stats plus a paired bootstrap CI on
reactive-vs-straight_line, for each metric spec's own "Research
considerations" line names (collision rate, d_min, path efficiency eta,
smoothness, completion).
"""

from __future__ import annotations

import json
import statistics
import sys
from pathlib import Path
from typing import Any

from aeris.evaluation.statistics import paired_bootstrap_ci

_REPO_ROOT = Path(__file__).resolve().parents[1]
_RUNS_PATH = _REPO_ROOT / "results" / "experiments" / "p10_avoidance" / "runs.jsonl"
_OUT_PATH = _REPO_ROOT / "results" / "experiments" / "p10_avoidance" / "summary.md"

_METRICS = [
    ("collided", "Collision rate", False),  # False = lower is better
    ("d_min_m", "Mean min clearance d_min (m)", True),
    ("path_efficiency", "Mean path efficiency eta", True),
    ("control_smoothness", "Mean control smoothness (lower = smoother)", False),
    ("arrived", "Completion rate", True),
]


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
    scenarios = sorted({r["scenario"] for r in records})
    methods = sorted({r["method"] for r in records})

    lines = ["# AERIS Phase 10 avoidance experiment: results summary\n"]
    lines.append(f"Total records: {len(records)}\n")

    for scenario in scenarios:
        lines.append(f"\n## {scenario}\n")
        by_method = {}
        for method in methods:
            group = [r for r in records if r["scenario"] == scenario and r["method"] == method]
            completed = [r for r in group if r["status"] == "completed"]
            failed = [r for r in group if r["status"] != "completed"]
            by_method[method] = completed
            lines.append(
                f"\n### {method} ({len(completed)}/{len(group)} completed, {len(failed)} failed)\n"
            )
            if not completed:
                continue
            lines.append("| Metric | Mean |")
            lines.append("|---|---|")
            for key, label, _higher_better in _METRICS:
                values = [float(r[key]) for r in completed]
                lines.append(f"| {label} | {statistics.mean(values):.3f} |")

        if len(methods) == 2 and all(
            len(by_method[m]) == len(by_method[methods[0]]) for m in methods
        ):
            a_name, b_name = methods
            a, b = by_method[a_name], by_method[b_name]
            if a and b and len(a) == len(b):
                lines.append(f"\n### Paired bootstrap CI: {a_name} - {b_name} (95%, n={len(a)})\n")
                lines.append("| Metric | Diff CI |")
                lines.append("|---|---|")
                for key, label, _higher_better in _METRICS:
                    va = [float(r[key]) for r in a]
                    vb = [float(r[key]) for r in b]
                    try:
                        lo, hi = paired_bootstrap_ci(va, vb, seed=12345)
                        lines.append(f"| {label} | [{lo:.3f}, {hi:.3f}] |")
                    except ValueError as exc:
                        lines.append(f"| {label} | (n/a: {exc}) |")

    summary_text = "\n".join(lines) + "\n"
    _OUT_PATH.write_text(summary_text)
    print(summary_text)
    print(f"Wrote {_OUT_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

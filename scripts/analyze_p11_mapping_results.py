#!/usr/bin/env python3
"""Analyze ``results/experiments/p11_mapping/runs.jsonl`` (spec §51 Phase 11):
per-world, per-pose-source-condition map-accuracy metrics (precision,
recall, free-space false-occupied rate, coverage, ATE, RPE), plus the
drift contribution (GT-pose accuracy minus EKF/noisy-pose accuracy) spec's
own research consideration asks to quantify.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

_REPO_ROOT = Path(__file__).resolve().parents[1]
_RUNS_PATH = _REPO_ROOT / "results" / "experiments" / "p11_mapping" / "runs.jsonl"
_OUT_PATH = _REPO_ROOT / "results" / "experiments" / "p11_mapping" / "summary.md"

_CONDITIONS = ["gt", "ekf", "noisy"]
_METRICS = [
    ("occupied_precision", "Occupied precision"),
    ("occupied_recall", "Occupied recall"),
    ("free_false_occupied_rate", "Free-space false-occupied rate"),
    ("map_coverage", "Map coverage"),
    ("ate_m", "ATE (m)"),
    ("rpe_m", "RPE (m, 10m segments)"),
]


def load_records(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def _fmt(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.3f}"


def main() -> int:
    records = load_records(_RUNS_PATH)
    completed = [r for r in records if r["status"] == "completed"]
    failed = [r for r in records if r["status"] != "completed"]

    lines = ["# AERIS Phase 11 mapping experiment: results summary\n"]
    lines.append(
        f"Total records: {len(records)} ({len(completed)} completed, {len(failed)} failed)\n"
    )

    ekf_precisions: list[float] = []
    ekf_recalls: list[float] = []
    ekf_p95s: list[float] = []

    for record in completed:
        scenario = record["scenario"]
        lines.append(f"\n## {scenario}\n")
        lines.append(
            f"n_frames={record['n_frames']}, latency_p95_ms={_fmt(record['latency_p95_ms'])}\n"
        )
        lines.append("| Condition | " + " | ".join(label for _key, label in _METRICS) + " |")
        lines.append("|---|" + "---|" * len(_METRICS))
        for condition in _CONDITIONS:
            row = record["conditions"][condition]
            values = " | ".join(_fmt(row.get(key)) for key, _label in _METRICS)
            lines.append(f"| {condition} | {values} |")

        ekf = record["conditions"]["ekf"]
        ekf_precisions.append(ekf["occupied_precision"])
        ekf_recalls.append(ekf["occupied_recall"])
        if record["latency_p95_ms"] is not None:
            ekf_p95s.append(record["latency_p95_ms"])

        gt_precision = record["conditions"]["gt"]["occupied_precision"]
        gt_recall = record["conditions"]["gt"]["occupied_recall"]
        noisy_precision = record["conditions"]["noisy"]["occupied_precision"]
        noisy_recall = record["conditions"]["noisy"]["occupied_recall"]
        lines.append(
            f"\nDrift contribution (GT - EKF): precision {gt_precision - ekf['occupied_precision']:+.3f}, "
            f"recall {gt_recall - ekf['occupied_recall']:+.3f}\n"
        )
        lines.append(
            f"Drift contribution (GT - noisy): precision {gt_precision - noisy_precision:+.3f}, "
            f"recall {gt_recall - noisy_recall:+.3f}\n"
        )

    if failed:
        lines.append("\n## Failed\n")
        for record in failed:
            lines.append(f"- {record['scenario']}: {record.get('reason', '')}\n")

    if ekf_precisions:
        lines.append("\n## Validation gate (spec §51 Phase 11, EKF-pose condition)\n")
        lines.append(
            f"min occupied_precision across worlds: {min(ekf_precisions):.3f} (gate: >= 0.9)\n"
        )
        lines.append(f"min occupied_recall across worlds: {min(ekf_recalls):.3f} (gate: >= 0.8)\n")
        if ekf_p95s:
            lines.append(f"max latency_p95_ms across worlds: {max(ekf_p95s):.2f} (gate: < 50)\n")

    summary_text = "\n".join(lines) + "\n"
    _OUT_PATH.write_text(summary_text)
    print(summary_text)
    print(f"Wrote {_OUT_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

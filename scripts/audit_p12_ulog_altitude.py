#!/usr/bin/env python3
"""Retroactive altitude audit of Phase 12's exploration episodes from PX4's
own ULogs (Phase 13 follow-up; see docs/exploration.md).

Phase 12's runs.jsonl saved no GT trajectories, so its coverage numbers
cannot be re-checked directly for the grounded-vehicle failure modes
Phase 13 found. Each run's PX4 ULog still records PX4's internal
ground truth (``vehicle_local_position_groundtruth``, world-frame NED,
taken from Gazebo inside PX4 -- independent of AERIS's own GT poll) and
its EKF estimate, which is enough for an altitude-only verdict: this
applies :func:`aeris.evaluation.metrics.altitude.check_altitude_band`
to PX4's GT altitude over each episode's autonomy window.

The window runs from the takeoff handover (EKF altitude first reaching
``hover_altitude_m - 0.5`` while in OFFBOARD, the same criterion
``run_exploration_episode`` uses) to PX4 entering AUTO_LAND. Each run's
ULog is found from the ``logger: Opened full log file`` line of its own
``run_logs/<label>/logs/px4.log``; px4.log is rewritten per launch, so
this is the attempt whose result runs.jsonl recorded. It also reports
IMU delivery loss (``sensor_combined`` integration time missing vs.
elapsed), the trigger found for the f1_rubble mid-episode drop.

Needs ``uvx`` (pyulog is run through it, not added as a dependency).

Usage::

    uv run python scripts/audit_p12_ulog_altitude.py
"""

from __future__ import annotations

import csv
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml

from aeris.core.frames.vector import Vec3
from aeris.evaluation.metrics.altitude import check_altitude_band
from aeris.simulation.px4_paths import resolve_px4_layout
from aeris.simulation.worlds.batch import load_world_spec

_REPO_ROOT = Path(__file__).resolve().parents[1]
_P12_DIR = _REPO_ROOT / "results" / "experiments" / "p12_exploration"
_WORLDS_DIR = _REPO_ROOT / "results" / "worlds"
_NAV_STATE_OFFBOARD = 14
_NAV_STATE_AUTO_LAND = 18
_TOPICS = (
    "vehicle_local_position",
    "vehicle_local_position_groundtruth",
    "vehicle_status",
    "sensor_combined",
)
_ULOG_LINE = re.compile(r"Opened full log file: \./(log/\S+\.ulg)")

Table = dict[str, np.ndarray]


def _read_csv(path: Path) -> Table:
    with path.open() as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [[float(x) if x not in ("", "nan") else np.nan for x in row] for row in reader]
    data = np.array(rows)
    return {key: data[:, i] for i, key in enumerate(header)}


def _extract(ulg: Path) -> dict[str, Table]:
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(
            ["uvx", "--from", "pyulog", "ulog2csv", "-m", ",".join(_TOPICS), "-o", tmp, str(ulg)],
            check=True,
            capture_output=True,
        )
        return {t: _read_csv(Path(tmp) / f"{ulg.stem}_{t}_0.csv") for t in _TOPICS}


def _autonomy_window(
    lp: Table, status: Table, *, handover_z_m: float
) -> tuple[float, float] | None:
    t_status = status["timestamp"] / 1e6
    offboard = status["nav_state"] == _NAV_STATE_OFFBOARD
    land = np.where(status["nav_state"] == _NAV_STATE_AUTO_LAND)[0]
    if not offboard.any():
        return None
    t_lp = lp["timestamp"] / 1e6
    in_offboard = np.interp(t_lp, t_status, offboard.astype(float), left=0.0) > 0.5
    reached = np.where(in_offboard & (-lp["z"] >= handover_z_m))[0]
    if len(reached) == 0:
        return None
    t0 = float(t_lp[reached[0]])
    land_after = [float(t_status[i]) for i in land if t_status[i] > t0]
    t1 = land_after[0] if land_after else float(t_lp[-1])
    return t0, t1


def _imu_loss(sc: Table, t0: float, t1: float) -> tuple[float, float]:
    """(fraction of IMU integration time missing over the window, worst 1-s window)."""
    t = sc["timestamp"] / 1e6
    idt = sc["accelerometer_integral_dt"] / 1e6
    m = (t >= t0) & (t < t1)
    total = 1.0 - idt[m][1:].sum() / (t[m][-1] - t[m][0])
    worst = 0.0
    for w in np.arange(t0, t1 - 1.0, 1.0):
        mw = (t >= w) & (t < w + 1.0)
        if mw.sum() > 2:
            worst = max(worst, 1.0 - idt[mw][1:].sum() / (t[mw][-1] - t[mw][0]))
    # Clamped at 0: timestamp rounding can make the integral sum exceed
    # elapsed time by a few microseconds.
    return max(0.0, float(total)), max(0.0, float(worst))


def main() -> int:
    config = yaml.safe_load(
        (_REPO_ROOT / "configs" / "experiments" / "p12_exploration.yaml").read_text()
    )
    handover_z_m = config["hover_altitude_m"] - 0.5
    max_excursion_s = config["max_altitude_excursion_s"]
    rootfs = resolve_px4_layout().rootfs_dir

    records = [json.loads(line) for line in (_P12_DIR / "runs.jsonl").open()]
    out: list[dict[str, object]] = []
    for rec in records:
        label = rec["label"]
        px4_log = _P12_DIR / "run_logs" / label / "logs" / "px4.log"
        matches = (
            _ULOG_LINE.findall(px4_log.read_text(errors="replace")) if px4_log.exists() else []
        )
        ulg = rootfs / matches[-1] if matches else None
        row: dict[str, object] = {
            "label": label,
            "coverage_final_p12": round(rec["coverage_final"], 3),
        }
        if ulg is None or not ulg.exists():
            row["audit"] = "no ULog"
            out.append(row)
            print(f"{label:42s} no ULog", flush=True)
            continue
        family = rec["world"].rsplit("_", 1)[0]
        spec = load_world_spec(_WORLDS_DIR / family / "val" / f"{rec['world']}.json")
        z_lo, z_hi = spec.altitude_band_m
        topics = _extract(ulg)
        window = _autonomy_window(
            topics["vehicle_local_position"], topics["vehicle_status"], handover_z_m=handover_z_m
        )
        if window is None:
            row["audit"] = "no autonomy window (never handed over)"
            out.append(row)
            print(f"{label:42s} never handed over", flush=True)
            continue
        t0, t1 = window
        gt = topics["vehicle_local_position_groundtruth"]
        t_gt = gt["timestamp"] / 1e6
        idx = np.where((t_gt >= t0) & (t_gt <= t1))[0][::5]  # ~50 Hz -> ~10 Hz
        poses = [(float(t_gt[i]), Vec3(0.0, 0.0, float(-gt["z"][i])), 0.0) for i in idx]
        validity = check_altitude_band(
            poses, z_lo_m=z_lo, z_hi_m=z_hi, max_excursion_s=max_excursion_s
        )
        loss_total, loss_worst = _imu_loss(topics["sensor_combined"], t0, t1)
        row.update(
            {
                "ulog": str(ulg.relative_to(rootfs)),
                "window_s": [round(t0, 1), round(t1, 1)],
                "audit": "valid" if validity.valid else "invalid",
                "detail": validity.describe(),
                "time_out_of_band_s": round(sum(e.duration_s for e in validity.excursions), 1),
                "min_gt_z_m": round(min(p[1].z for p in poses), 3),
                "imu_loss_total": round(loss_total, 4),
                "imu_loss_worst_1s": round(loss_worst, 3),
            }
        )
        out.append(row)
        print(
            f"{label:42s} C={rec['coverage_final']:.3f} {row['audit']:8s} "
            f"out={row['time_out_of_band_s']:6.1f}s min_z={row['min_gt_z_m']:6.2f} "
            f"imu_loss={loss_total * 100:4.1f}% (worst 1s {loss_worst * 100:4.1f}%)",
            flush=True,
        )

    out_path = _P12_DIR / "ulog_altitude_audit.json"
    out_path.write_text(json.dumps(out, indent=1))
    print(f"Wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

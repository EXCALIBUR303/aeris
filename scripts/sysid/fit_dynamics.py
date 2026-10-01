#!/usr/bin/env python3
"""Fit FastSim dynamics to the Tier H step-response session recorded by
``record_tier_h.py --session steps`` (spec §51 Phase 13 task 1, validation
gate item 1).

Repetition 0 of every step is the training set, repetition 1 the held-out
set. Per-axis fits (vx, vy, yaw) are reported with bootstrap CIs; FastSim
uses one horizontal model (``tau_xy``/``delay_xy``/``accel_max_xy``), so
vx and vy are also fit *pooled* -- that pooled fit is what's written to
``configs/fastsim/dynamics.yaml`` and scored on held-out data, with the
per-axis fits kept alongside so any real vx/vy asymmetry is visible, not
averaged away silently.

Usage::

    uv run python scripts/sysid/fit_dynamics.py
"""

from __future__ import annotations

import datetime as _dt
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

from aeris.simulation.fastsim.sysid import (
    StepWindow,
    bootstrap_ci,
    fit_axis,
    held_out_rmse,
    resample_window,
)

_REPO_ROOT = Path(__file__).resolve().parents[2]
_DATA = _REPO_ROOT / "results" / "sysid" / "steps"
_OUT = _REPO_ROOT / "configs" / "fastsim" / "dynamics.yaml"

_COL = {"t": 0, "vx": 4, "vy": 5, "yaw": 7}
_CMD_COL = {"vx": 8, "vy": 9, "yaw": 10}
_PRE_S = 0.5
_POST_S = 2.5 + 1.5  # the step hold, then most of the following rest (captures ramp-down too)
_N_BOOT = 200


def load_windows(rows: np.ndarray, tags: list[str]) -> list[StepWindow]:
    t = rows[:, _COL["t"]]
    keep = np.concatenate([[True], np.diff(t) > 1e-6])  # telemetry can repeat between polls
    rows, t = rows[keep], t[keep]
    tags = [tg for tg, k in zip(tags, keep, strict=True) if k]

    windows = []
    seen: set[str] = set()
    for i, tag in enumerate(tags):
        if tag in seen or tag.endswith(":rest") or not tag.startswith("rep"):
            continue
        seen.add(tag)
        rep_s, axis, amp_s = tag.split(":")
        t_on = t[i]
        t0, t1 = t_on - _PRE_S, t_on + _POST_S
        cmd, meas = resample_window(t, rows[:, _CMD_COL[axis]], rows[:, _COL[axis]], t0, t1)
        i0 = int(np.searchsorted(t, t0))
        windows.append(
            StepWindow(
                axis=axis,
                amplitude=float(amp_s),
                rep=int(rep_s[3:]),
                cmd=cmd,
                measured=meas,
                v0=float(rows[i0, _COL[axis]]),
                cmd_before=float(rows[i0, _CMD_COL[axis]]),
            )
        )
    return windows


def main() -> int:
    rows = np.load(_DATA / "rows.npy")
    tags = json.loads((_DATA / "tags.json").read_text())
    windows = load_windows(rows, tags)

    def sel(axes: tuple[str, ...], rep: int) -> list[StepWindow]:
        return [w for w in windows if w.axis in axes and w.rep == rep]

    report: dict[str, dict[str, object]] = {}
    for name, axes in (
        ("vx", ("vx",)),
        ("vy", ("vy",)),
        ("xy_pooled", ("vx", "vy")),
        ("yaw", ("yaw",)),
    ):
        train, held = sel(axes, 0), sel(axes, 1)
        fit = fit_axis(train)
        ci = bootstrap_ci(train, n_boot=_N_BOOT, seed=13)
        report[name] = {
            "n_train_steps": len(train),
            "n_heldout_steps": len(held),
            "tau_s": round(fit.tau_s, 4),
            "delay_s": round(fit.delay_s, 4),
            "accel_max": round(fit.accel_max, 4),
            "train_rmse": round(fit.train_rmse, 4),
            "heldout_rmse": round(held_out_rmse(held, fit), 4),
            "ci95": {k: [round(v[0], 4), round(v[1], 4)] for k, v in ci.items()},
        }
        print(name, report[name], flush=True)

    xy, yw = report["xy_pooled"], report["yaw"]
    sha = subprocess.run(
        ["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, cwd=_REPO_ROOT
    ).stdout.strip()
    doc = {
        "provenance": {
            "dataset": "results/sysid/steps (scripts/sysid/record_tier_h.py --session steps)",
            "fitted_utc": _dt.datetime.now(_dt.UTC).isoformat(timespec="seconds"),
            "git_sha": sha,
            "model": "first-order lag + integer delay + accel clip, explicit Euler at dt_sim=0.02s",
            "note": "rep0 = training, rep1 = held-out; heldout_rmse is gate item 1 (<= 0.15 m/s)",
        },
        "identified": {
            "tau_xy_s": xy["tau_s"],
            "delay_xy_s": xy["delay_s"],
            "accel_max_xy_mps2": xy["accel_max"],
            "tau_yaw_s": yw["tau_s"],
            "delay_yaw_s": yw["delay_s"],
            "yaw_accel_max_radps2": yw["accel_max"],
        },
        "fits": report,
    }
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    _OUT.write_text(yaml.safe_dump(doc, sort_keys=False))
    print(f"wrote {_OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

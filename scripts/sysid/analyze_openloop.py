#!/usr/bin/env python3
"""Analyze the ``record_tier_h.py --session openloop`` dataset (spec §51
Phase 13 tasks 4 and 9; validation gate item 2).

1. **Pose-error model** -> ``configs/fastsim/pose_noise.yaml``. Residual
   e(t) = EKF position - (GT position - constant T_W_O offset). Under the
   FastSim model (random-walk drift + white noise), the variogram is
   gamma(L) = 2*sigma^2 + q*L, with L in control steps (0.1 s): the
   intercept gives the white-noise sigma, the slope the per-step drift
   variance q = walk_step^2. Only pairs where the vehicle was nearly
   stationary (EKF speed < 0.3 m/s *and* |yaw rate| < 0.1 rad/s) are
   used: each GT poll spans a
   ~0.24 s window of unknown exact sample time, and at speed that timing
   uncertainty alone would masquerade as pose error.
2. **Open-loop trajectory parity** -> ``results/sysid/parity_trajectory.json``.
   Each sequence's logged command stream is replayed through FastSim's
   identified dynamics (:class:`BatchDynamics`, the same integrator the
   envs use) from the measured initial velocity; horizontal position RMSE
   vs the Tier H EKF trajectory over the sequence's 20 s is gate item 2
   (<= 0.5 m, every one of the 10 sequences).

Usage::

    uv run python scripts/sysid/analyze_openloop.py
"""

from __future__ import annotations

import datetime as _dt
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from aeris.simulation.fastsim.dynamics import DT_SIM_S, BatchDynamics, DynamicsParams

_REPO_ROOT = Path(__file__).resolve().parents[2]
_DATA = _REPO_ROOT / "results" / "sysid" / "openloop"
_DYN = _REPO_ROOT / "configs" / "fastsim" / "dynamics.yaml"
_POSE_OUT = _REPO_ROOT / "configs" / "fastsim" / "pose_noise.yaml"
_PARITY_OUT = _REPO_ROOT / "results" / "sysid" / "parity_trajectory.json"
_CONTROL_DT_S = 0.1
_STATIONARY_MPS = 0.3
_STATIONARY_RADPS = 0.1


def _wrap(a: np.ndarray) -> np.ndarray:
    return (a + math.pi) % (2 * math.pi) - math.pi


def fit_pose_noise(rows: np.ndarray, pairs: list[list[float]]) -> dict[str, object]:
    t = rows[:, 0]
    speed = np.hypot(rows[:, 4], rows[:, 5])
    samples = []
    for t0, t1, gx, gy, _gz, gyaw in pairs:
        if not (math.isfinite(t0) and math.isfinite(t1)):
            continue
        tm = 0.5 * (t0 + t1)
        if np.interp(tm, t, speed) > _STATIONARY_MPS:
            continue
        if abs(np.interp(tm, t, rows[:, 7])) > _STATIONARY_RADPS:
            continue  # rotating: the poll-window timing uncertainty would alias into yaw error
        ex, ey = np.interp(tm, t, rows[:, 1]), np.interp(tm, t, rows[:, 2])
        eyaw = np.interp(tm, t, np.unwrap(rows[:, 6]))
        samples.append((tm, ex - gx, ey - gy, _wrap(np.array(eyaw - gyaw)).item()))
    s = np.array(samples)
    if len(s) < 20:
        raise RuntimeError(f"only {len(s)} stationary pose pairs -- not enough to fit")
    offset = s[:, 1:3].mean(axis=0)  # constant T_W_O translation (spec §19.3 alignment)
    e = s[:, 1:3] - offset

    lags, gammas = [], []
    for i in range(len(s)):
        for j in range(i + 1, len(s)):
            lag = (s[j, 0] - s[i, 0]) / _CONTROL_DT_S
            if lag > 300:  # <= 30 s
                break
            lags.append(lag)
            gammas.append(float(np.mean((e[j] - e[i]) ** 2)))  # per-axis
    lags_a, g_a = np.array(lags), np.array(gammas)
    # Bin by lag, fit gamma = 2 sigma^2 + q L by least squares on bin means.
    bins = np.linspace(0, 300, 16)
    idx = np.digitize(lags_a, bins)
    bl, bg = [], []
    for b in range(1, len(bins)):
        m = idx == b
        if m.sum() >= 5:
            bl.append(lags_a[m].mean())
            bg.append(g_a[m].mean())
    slope, intercept = np.polyfit(bl, bg, 1)
    sigma = math.sqrt(max(intercept, 0.0) / 2.0)
    walk = math.sqrt(max(slope, 0.0))
    yaw_resid = s[:, 3] - s[:, 3].mean()
    return {
        "n_pairs_used": len(s),
        "t_w_o_offset_xy_m": [round(float(v), 4) for v in offset],
        "residual_std_xy_m": [round(float(v), 4) for v in e.std(axis=0)],
        "variogram_intercept_m2": round(float(intercept), 6),
        "variogram_slope_m2_per_step": round(float(slope), 8),
        "identified": {
            "noise_std_m": round(sigma, 4),
            "walk_step_m": round(walk, 5),
            "yaw_noise_std_rad": round(float(yaw_resid.std()), 4),
        },
    }


def trajectory_parity(
    rows: np.ndarray, tags: list[str], params: DynamicsParams
) -> list[dict[str, Any]]:
    t = rows[:, 0]
    out = []
    seqs = sorted(
        {tg.split(":")[0] for tg in tags if tg.startswith("seq")}, key=lambda x: int(x[3:])
    )
    for name in seqs:
        idx = [i for i, tg in enumerate(tags) if tg.startswith(name + ":seg")]
        i0, i1 = idx[0], idx[-1]
        t0, t1 = t[i0], t[i1]
        grid = np.arange(t0, t1, DT_SIM_S)
        k = np.clip(np.searchsorted(t, grid, side="right") - 1, 0, len(t) - 1)
        cmd = rows[k][:, 8:11]
        dyn = BatchDynamics([params])
        dyn.reset(
            np.array([0]),
            pos=np.array([[rows[i0, 1], rows[i0, 2], rows[i0, 3]]]),
            yaw=np.array([rows[i0, 6]]),
        )
        dyn.state.vel[0] = rows[i0, 4:6]
        dyn.state.yaw_rate[0] = rows[i0, 7]
        pred = np.empty((len(grid), 2))
        for n in range(len(grid)):
            dyn.step(cmd[n : n + 1])
            pred[n] = dyn.state.pos[0, :2]
        meas = np.column_stack(
            [np.interp(grid + DT_SIM_S, t, rows[:, 1]), np.interp(grid + DT_SIM_S, t, rows[:, 2])]
        )
        err = np.hypot(*(pred - meas).T)
        out.append(
            {
                "sequence": name,
                "duration_s": round(float(t1 - t0), 2),
                "path_length_m": round(float(np.hypot(*np.diff(meas, axis=0).T).sum()), 2),
                "position_rmse_m": round(float(np.sqrt(np.mean(err**2))), 4),
                "final_error_m": round(float(err[-1]), 4),
                "max_error_m": round(float(err.max()), 4),
            }
        )
    return out


def main() -> int:
    rows = np.load(_DATA / "rows.npy")
    tags = json.loads((_DATA / "tags.json").read_text())
    keep = np.concatenate([[True], np.diff(rows[:, 0]) > 1e-6])
    rows = rows[keep]
    tags = [tg for tg, k in zip(tags, keep, strict=True) if k]
    pairs = json.loads((_DATA / "pose_pairs.json").read_text())

    pose = fit_pose_noise(rows, pairs)
    print("pose noise:", pose)
    _POSE_OUT.write_text(
        yaml.safe_dump(
            {
                "provenance": {
                    "dataset": "results/sysid/openloop (record_tier_h.py --session openloop)",
                    "fitted_utc": _dt.datetime.now(_dt.UTC).isoformat(timespec="seconds"),
                    "model": "estimate = truth + random-walk drift (per 0.1 s control step) + white noise",
                },
                **pose,
            },
            sort_keys=False,
        )
    )

    params = DynamicsParams.from_yaml(_DYN)
    parity = trajectory_parity(rows, tags, params)
    for p in parity:
        print(p)
    worst = max(p["position_rmse_m"] for p in parity)
    summary = {
        "gate": "position RMSE <= 0.5 m over 20 s, each of 10 sequences",
        "n_sequences": len(parity),
        "worst_rmse_m": worst,
        "mean_rmse_m": round(float(np.mean([p["position_rmse_m"] for p in parity])), 4),
        "passed": bool(len(parity) >= 10 and worst <= 0.5),
        "sequences": parity,
    }
    _PARITY_OUT.write_text(json.dumps(summary, indent=2))
    print(f"worst RMSE {worst:.3f} m -> {'PASS' if summary['passed'] else 'FAIL'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""System identification for FastSim's dynamics (spec §51 Phase 13 task 1):
fit ``(tau, delay, accel_max)`` per axis to Tier H step responses, with
bootstrap confidence intervals, and score held-out prediction RMSE.

The fitter's inner simulator (:func:`simulate_axis`) is a Numba scalar
copy of :class:`~aeris.simulation.fastsim.dynamics.BatchDynamics`'s own
per-axis update (same explicit Euler, same delay ring semantics, same
accel clip) -- a test pins the two to identical outputs, so the parameters
fitted here are parameters *for the integrator the envs actually run*,
not for a look-alike.

Data preparation: telemetry is logged at ~50 Hz but not on a uniform
grid, so each step window is resampled onto the ``dt_sim`` grid -- the
command by zero-order hold (it's piecewise constant, set at submission
time), the measured velocity by linear interpolation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numba import njit

from aeris.simulation.fastsim.dynamics import DT_SIM_S, MAX_DELAY_STEPS


@njit(cache=True)
def simulate_axis(
    cmd: np.ndarray,
    v0: float,
    cmd_before: float,
    tau: float,
    delay_steps: int,
    a_max: float,
    dt: float,
) -> np.ndarray:
    """One axis of :class:`BatchDynamics`: ``cmd[t]`` is the command
    submitted at step ``t``; the effective command is ``delay_steps`` old
    (``cmd_before`` fills the history before the window starts)."""
    n = cmd.shape[0]
    out = np.empty(n)
    v = v0
    lim = a_max * dt
    k = dt / tau
    for t in range(n):
        j = t - delay_steps
        c = cmd[j] if j >= 0 else cmd_before
        dv = (c - v) * k
        if dv > lim:
            dv = lim
        elif dv < -lim:
            dv = -lim
        v += dv
        out[t] = v
    return out


@dataclass(frozen=True, slots=True)
class StepWindow:
    """One step response, resampled onto the ``dt_sim`` grid."""

    axis: str
    amplitude: float
    rep: int
    cmd: np.ndarray  # [T]
    measured: np.ndarray  # [T]
    v0: float
    cmd_before: float


def resample_window(
    t: np.ndarray,
    cmd: np.ndarray,
    measured: np.ndarray,
    t_start: float,
    t_end: float,
    dt: float = DT_SIM_S,
) -> tuple[np.ndarray, np.ndarray]:
    grid = np.arange(t_start + dt, t_end, dt)
    idx = np.searchsorted(t, grid, side="right") - 1
    idx = np.clip(idx, 0, len(t) - 1)
    return cmd[idx], np.interp(grid, t, measured)


@njit(cache=True)
def _weighted_rmse(
    cmd: np.ndarray,
    meas: np.ndarray,
    starts: np.ndarray,
    lens: np.ndarray,
    v0s: np.ndarray,
    befores: np.ndarray,
    weights: np.ndarray,
    tau: float,
    d: int,
    a: float,
    dt: float,
) -> float:
    se = 0.0
    n = 0.0
    for w in range(starts.shape[0]):
        if weights[w] == 0.0:
            continue
        s0, ln = starts[w], lens[w]
        pred = simulate_axis(cmd[s0 : s0 + ln], v0s[w], befores[w], tau, d, a, dt)
        acc = 0.0
        for i in range(ln):  # explicit loop: numba's np.dot needs scipy's BLAS
            e = pred[i] - meas[s0 + i]
            acc += e * e
        se += weights[w] * acc
        n += weights[w] * ln
    return float(np.sqrt(se / max(n, 1.0)))


@dataclass(frozen=True, slots=True)
class _Packed:
    cmd: np.ndarray
    meas: np.ndarray
    starts: np.ndarray
    lens: np.ndarray
    v0s: np.ndarray
    befores: np.ndarray

    @staticmethod
    def of(windows: list[StepWindow]) -> _Packed:
        lens = np.array([len(w.cmd) for w in windows], dtype=np.int64)
        starts = np.concatenate([[0], np.cumsum(lens)[:-1]]).astype(np.int64)
        return _Packed(
            cmd=np.concatenate([w.cmd for w in windows]).astype(np.float64),
            meas=np.concatenate([w.measured for w in windows]).astype(np.float64),
            starts=starts,
            lens=lens,
            v0s=np.array([w.v0 for w in windows], dtype=np.float64),
            befores=np.array([w.cmd_before for w in windows], dtype=np.float64),
        )

    def rmse(self, weights: np.ndarray, tau: float, d: int, a: float) -> float:
        return float(
            _weighted_rmse(
                self.cmd,
                self.meas,
                self.starts,
                self.lens,
                self.v0s,
                self.befores,
                weights,
                tau,
                d,
                a,
                DT_SIM_S,
            )
        )


@dataclass(frozen=True, slots=True)
class AxisFit:
    tau_s: float
    delay_s: float
    accel_max: float
    train_rmse: float


_TAU_GRID = np.round(np.arange(0.05, 1.501, 0.05), 4)
_ACC_GRID = np.round(np.arange(0.5, 12.01, 0.5), 4)
_DELAY_GRID = np.arange(
    0, MAX_DELAY_STEPS + 1
)  # 0 .. 0.50 s at dt_sim = 0.02 (the full ring buffer)


def _fit_packed(p: _Packed, weights: np.ndarray) -> AxisFit:
    best: tuple[float, float, int, float] = (float("inf"), 0.0, 0, 0.0)
    for d in _DELAY_GRID:
        if d > MAX_DELAY_STEPS:
            continue
        for tau in _TAU_GRID:
            for a in _ACC_GRID:
                r = p.rmse(weights, float(tau), int(d), float(a))
                if r < best[0]:
                    best = (r, float(tau), int(d), float(a))
    _, tau0, d0, a0 = best
    for tau in np.linspace(max(0.03, tau0 - 0.05), tau0 + 0.05, 11):
        for a in np.linspace(max(0.3, a0 - 0.5), a0 + 0.5, 11):
            r = p.rmse(weights, float(tau), d0, float(a))
            if r < best[0]:
                best = (r, float(tau), d0, float(a))
    r_best, tau_best, d_best, a_best = best
    return AxisFit(tau_s=tau_best, delay_s=d_best * DT_SIM_S, accel_max=a_best, train_rmse=r_best)


def fit_axis(windows: list[StepWindow]) -> AxisFit:
    """Grid search (delay x tau x accel_max), then one local refinement
    pass around the best cell. Exhaustive rather than gradient-based: the
    delay is integer-valued and the accel clip makes the loss non-smooth."""
    if not windows:
        raise ValueError("no step windows to fit")
    return _fit_packed(_Packed.of(windows), np.ones(len(windows)))


def held_out_rmse(windows: list[StepWindow], fit: AxisFit) -> float:
    p = _Packed.of(windows)
    return p.rmse(np.ones(len(windows)), fit.tau_s, round(fit.delay_s / DT_SIM_S), fit.accel_max)


def bootstrap_ci(
    windows: list[StepWindow], *, n_boot: int, seed: int, alpha: float = 0.05
) -> dict[str, tuple[float, float]]:
    """Percentile CIs from refitting on step windows resampled with
    replacement (as per-window multiplicity weights -- equivalent to
    refitting on the resampled list, without repacking every time)."""
    p = _Packed.of(windows)
    rng = np.random.default_rng(seed)
    taus, delays, accs = [], [], []
    for _ in range(n_boot):
        w = np.bincount(rng.integers(0, len(windows), size=len(windows)), minlength=len(windows))
        f = _fit_packed(p, w.astype(np.float64))
        taus.append(f.tau_s)
        delays.append(f.delay_s)
        accs.append(f.accel_max)
    q = (100 * alpha / 2, 100 * (1 - alpha / 2))

    def ci(v: list[float]) -> tuple[float, float]:
        lo, hi = np.percentile(v, q)
        return float(lo), float(hi)

    return {"tau_s": ci(taus), "delay_s": ci(delays), "accel_max": ci(accs)}

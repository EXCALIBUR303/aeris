"""System-ID fitter: its inner simulator matches BatchDynamics exactly, and
it recovers known parameters from synthetic step responses."""

from __future__ import annotations

import numpy as np
import pytest

from aeris.simulation.fastsim.dynamics import DT_SIM_S, DynamicsParams, simulate_single
from aeris.simulation.fastsim.sysid import (
    StepWindow,
    bootstrap_ci,
    fit_axis,
    held_out_rmse,
    resample_window,
    simulate_axis,
)


def _windows(tau: float, delay: float, acc: float, *, noise: float, seed: int) -> list[StepWindow]:
    rng = np.random.default_rng(seed)
    p = DynamicsParams(tau_xy_s=tau, delay_xy_s=delay, accel_max_xy_mps2=acc)
    out = []
    for amp in (0.5, 1.0, 1.5, 2.0, -0.5, -1.0, -1.5, -2.0):
        cmd = np.zeros((225, 3))
        cmd[25:150, 0] = amp  # 0.5s pre-roll at rest, 2.5s step, 1.5s rest
        v = simulate_single(p, cmd)[:, 0] + rng.normal(0.0, noise, 225)
        out.append(StepWindow("vx", amp, 0, cmd[:, 0].copy(), v, 0.0, 0.0))
    return out


def test_numba_axis_simulator_equals_batch_dynamics_exactly() -> None:
    p = DynamicsParams(tau_xy_s=0.37, delay_xy_s=0.12, accel_max_xy_mps2=2.3)
    cmd = np.zeros((300, 3))
    cmd[20:120, 0] = 1.7
    cmd[120:220, 0] = -0.8
    ref = simulate_single(p, cmd)[:, 0]
    got = simulate_axis(cmd[:, 0], 0.0, 0.0, 0.37, round(0.12 / DT_SIM_S), 2.3, DT_SIM_S)
    np.testing.assert_allclose(got, ref, rtol=0, atol=1e-12)


def test_fit_recovers_known_parameters_from_noisy_synthetic_steps() -> None:
    fit = fit_axis(_windows(0.40, 0.10, 2.5, noise=0.02, seed=1))
    assert fit.tau_s == pytest.approx(0.40, abs=0.04)
    assert fit.delay_s == pytest.approx(0.10, abs=0.021)
    assert fit.accel_max == pytest.approx(2.5, abs=0.3)
    assert fit.train_rmse < 0.03


def test_held_out_rmse_is_near_noise_floor_for_the_true_model() -> None:
    train = _windows(0.30, 0.06, 3.0, noise=0.02, seed=2)
    held = _windows(0.30, 0.06, 3.0, noise=0.02, seed=3)
    assert held_out_rmse(held, fit_axis(train)) < 0.03


def test_bootstrap_ci_brackets_the_truth() -> None:
    ci = bootstrap_ci(_windows(0.40, 0.10, 2.5, noise=0.02, seed=4), n_boot=20, seed=0)
    lo, hi = ci["tau_s"]
    assert lo - 0.05 <= 0.40 <= hi + 0.05


def test_resample_uses_zero_order_hold_for_command_and_interpolates_measurement() -> None:
    t = np.array([0.0, 0.03, 0.05, 0.11])
    cmd = np.array([0.0, 1.0, 1.0, 2.0])
    meas = np.array([0.0, 0.3, 0.5, 1.1])
    c, m = resample_window(t, cmd, meas, 0.0, 0.1, dt=0.02)
    np.testing.assert_allclose(c, [0.0, 1.0, 1.0, 1.0])  # grid 0.02, 0.04, 0.06, 0.08
    np.testing.assert_allclose(m, [0.2, 0.4, 0.6, 0.8])

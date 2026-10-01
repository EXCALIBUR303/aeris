"""FastSim dynamics against analytic step responses (spec's own testing line)."""

from __future__ import annotations

import numpy as np
import pytest

from aeris.simulation.fastsim.dynamics import (
    DT_SIM_S,
    BatchDynamics,
    DynamicsParams,
    simulate_single,
)

_NO_LIMIT = DynamicsParams(
    tau_xy_s=0.4,
    delay_xy_s=0.0,
    accel_max_xy_mps2=1e9,
    tau_yaw_s=0.3,
    delay_yaw_s=0.0,
    yaw_accel_max_radps2=1e9,
)


def _step(t: int, vx: float = 0.0, vy: float = 0.0, r: float = 0.0) -> np.ndarray:
    c = np.zeros((t, 3))
    c[:, 0], c[:, 1], c[:, 2] = vx, vy, r
    return c


def test_first_order_step_matches_the_discrete_closed_form_exactly() -> None:
    out = simulate_single(_NO_LIMIT, _step(100, vx=1.0))
    n = np.arange(1, 101)
    expected = 1.0 - (1.0 - DT_SIM_S / 0.4) ** n
    np.testing.assert_allclose(out[:, 0], expected, rtol=1e-12)
    np.testing.assert_allclose(out[:, 1], 0.0)


def test_first_order_step_approximates_the_continuous_response() -> None:
    out = simulate_single(_NO_LIMIT, _step(200, vy=2.0))
    t = np.arange(1, 201) * DT_SIM_S
    np.testing.assert_allclose(out[:, 1], 2.0 * (1.0 - np.exp(-t / 0.4)), atol=0.05)


def test_delay_holds_the_response_at_zero_for_exactly_d_steps() -> None:
    p = DynamicsParams(tau_xy_s=0.4, delay_xy_s=0.1, accel_max_xy_mps2=1e9)
    out = simulate_single(p, _step(20, vx=1.0))
    d = round(0.1 / DT_SIM_S)
    np.testing.assert_allclose(out[:d, 0], 0.0)
    assert out[d, 0] > 0.0


def test_acceleration_limit_bounds_the_ramp_slope() -> None:
    p = DynamicsParams(tau_xy_s=0.1, delay_xy_s=0.0, accel_max_xy_mps2=1.0)
    out = simulate_single(p, _step(50, vx=2.0))
    slopes = np.diff(np.concatenate([[0.0], out[:, 0]])) / DT_SIM_S
    assert slopes.max() <= 1.0 + 1e-9
    # Saturated ramp: 1 m/s^2 -> 0.02 m/s per step for the first steps.
    np.testing.assert_allclose(out[:10, 0], np.arange(1, 11) * 0.02, rtol=1e-9)


def test_accel_limit_applies_to_the_horizontal_vector_not_per_axis() -> None:
    p = DynamicsParams(tau_xy_s=0.02, delay_xy_s=0.0, accel_max_xy_mps2=1.0)
    out = simulate_single(p, _step(1, vx=3.0, vy=4.0))
    assert np.hypot(out[0, 0], out[0, 1]) == pytest.approx(1.0 * DT_SIM_S)
    assert out[0, 1] / out[0, 0] == pytest.approx(4.0 / 3.0)


def test_yaw_rate_tracks_its_own_first_order_lag() -> None:
    out = simulate_single(_NO_LIMIT, _step(50, r=0.5))
    n = np.arange(1, 51)
    np.testing.assert_allclose(out[:, 2], 0.5 * (1.0 - (1.0 - DT_SIM_S / 0.3) ** n), rtol=1e-12)


def test_position_and_yaw_integrate_velocity() -> None:
    dyn = BatchDynamics([_NO_LIMIT])
    dyn.reset(np.array([0]), pos=np.array([[1.0, 2.0, 1.5]]), yaw=np.array([0.0]))
    for _ in range(500):
        dyn.step(np.array([[1.0, 0.0, 0.2]]))
    # Settled: ~1 m/s for most of 10s -> x advanced ~9.6m; z untouched.
    assert dyn.state.pos[0, 0] == pytest.approx(1.0 + 10.0 - 0.4, abs=0.05)
    assert dyn.state.pos[0, 2] == 1.5
    assert -np.pi <= dyn.state.yaw[0] < np.pi


def test_per_env_parameters_are_independent() -> None:
    fast = DynamicsParams(tau_xy_s=0.1, delay_xy_s=0.0, accel_max_xy_mps2=1e9)
    slow = DynamicsParams(tau_xy_s=1.0, delay_xy_s=0.0, accel_max_xy_mps2=1e9)
    dyn = BatchDynamics([fast, slow])
    for _ in range(10):
        dyn.step(np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
    assert dyn.state.vel[0, 0] > dyn.state.vel[1, 0] + 0.3


def test_reset_clears_state_and_the_delay_buffer() -> None:
    p = DynamicsParams(tau_xy_s=0.2, delay_xy_s=0.2, accel_max_xy_mps2=1e9)
    dyn = BatchDynamics([p])
    for _ in range(20):
        dyn.step(np.array([[2.0, 0.0, 0.0]]))
    dyn.reset(np.array([0]), pos=np.zeros((1, 3)), yaw=np.zeros(1))
    dyn.step(np.array([[0.0, 0.0, 0.0]]))
    assert dyn.state.vel[0, 0] == 0.0  # no stale 2 m/s command leaking out of the buffer


def test_tau_below_dt_is_rejected() -> None:
    with pytest.raises(ValueError):
        BatchDynamics([DynamicsParams(tau_xy_s=0.001)])

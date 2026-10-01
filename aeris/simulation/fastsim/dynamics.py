"""FastSim vehicle dynamics (spec §17.3): a point mass whose velocity tracks
the (delayed, acceleration-limited) commanded velocity through a
first-order lag,

    v_dot = clip((v_sp(t - d) - v) / tau_v,  |.| <= a_max)

with the same structure for yaw rate. Altitude is held (spec §26.2: "Altitude
is held by the altitude-hold setpoint") -- z is constant, and only the
horizontal and yaw axes are modeled. Every parameter is **identified from
Tier H step responses** (``scripts/sysid``, spec §51 Phase 13 task 1), not
assumed; :class:`DynamicsParams` defaults are only placeholders until
``configs/fastsim/dynamics.yaml`` is loaded.

Integration is explicit Euler at ``dt_sim``, vectorized over N envs, each
with its own parameters (so domain randomization is per-env, not per-batch).
The delay is a per-env integer number of ``dt_sim`` steps, realized with a
command ring buffer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

DT_SIM_S = 0.02  # 50 Hz -- the Tier H telemetry rate step responses are logged at
MAX_DELAY_STEPS = 25  # 0.5 s ring buffer


@dataclass(frozen=True, slots=True)
class DynamicsParams:
    tau_xy_s: float = 0.35
    delay_xy_s: float = 0.10
    accel_max_xy_mps2: float = 3.0
    tau_yaw_s: float = 0.25
    delay_yaw_s: float = 0.10
    yaw_accel_max_radps2: float = 4.0

    @staticmethod
    def from_yaml(path: Path) -> DynamicsParams:
        doc = yaml.safe_load(path.read_text())
        fields = {f: float(doc["identified"][f]) for f in DynamicsParams.__dataclass_fields__}
        return DynamicsParams(**fields)


@dataclass(slots=True)
class BatchState:
    pos: np.ndarray  # [N, 3] world frame
    vel: np.ndarray  # [N, 2] horizontal world-frame velocity
    yaw: np.ndarray  # [N]
    yaw_rate: np.ndarray  # [N]


class BatchDynamics:
    """N independent vehicles, each with its own :class:`DynamicsParams`."""

    def __init__(self, params: list[DynamicsParams], *, dt_sim_s: float = DT_SIM_S) -> None:
        self.n = len(params)
        self.dt = dt_sim_s
        self.tau_xy = np.array([p.tau_xy_s for p in params])
        self.tau_yaw = np.array([p.tau_yaw_s for p in params])
        self.a_max = np.array([p.accel_max_xy_mps2 for p in params])
        self.yaw_a_max = np.array([p.yaw_accel_max_radps2 for p in params])
        self.d_xy = np.array([self._delay_steps(p.delay_xy_s) for p in params], dtype=np.int64)
        self.d_yaw = np.array([self._delay_steps(p.delay_yaw_s) for p in params], dtype=np.int64)
        if np.any(self.tau_xy < dt_sim_s) or np.any(self.tau_yaw < dt_sim_s):
            raise ValueError("tau must be >= dt_sim for explicit-Euler stability")
        self._buf = np.zeros((self.n, MAX_DELAY_STEPS + 1, 3))  # (vx, vy, yaw_rate) commands
        self._head = 0
        self._rows = np.arange(self.n)
        self.state = BatchState(
            pos=np.zeros((self.n, 3)),
            vel=np.zeros((self.n, 2)),
            yaw=np.zeros(self.n),
            yaw_rate=np.zeros(self.n),
        )

    def _delay_steps(self, delay_s: float) -> int:
        d = round(delay_s / self.dt)
        if not 0 <= d <= MAX_DELAY_STEPS:
            raise ValueError(f"delay {delay_s}s outside [0, {MAX_DELAY_STEPS * self.dt}]s")
        return d

    def set_params(self, idx: int, p: DynamicsParams) -> None:
        """Swap one env's dynamics parameters (per-episode domain randomization)."""
        if p.tau_xy_s < self.dt or p.tau_yaw_s < self.dt:
            raise ValueError("tau must be >= dt_sim for explicit-Euler stability")
        self.tau_xy[idx], self.tau_yaw[idx] = p.tau_xy_s, p.tau_yaw_s
        self.a_max[idx], self.yaw_a_max[idx] = p.accel_max_xy_mps2, p.yaw_accel_max_radps2
        self.d_xy[idx] = self._delay_steps(p.delay_xy_s)
        self.d_yaw[idx] = self._delay_steps(p.delay_yaw_s)

    def reset(self, idx: np.ndarray, *, pos: np.ndarray, yaw: np.ndarray) -> None:
        """Reset envs ``idx`` to rest at ``pos`` [len(idx), 3] facing ``yaw``."""
        self.state.pos[idx] = pos
        self.state.vel[idx] = 0.0
        self.state.yaw[idx] = yaw
        self.state.yaw_rate[idx] = 0.0
        self._buf[idx] = 0.0

    def step(self, cmd: np.ndarray) -> None:
        """Advance one ``dt_sim`` with commands ``cmd`` [N, 3] = world-frame
        (vx, vy) + yaw rate, held constant over the step."""
        size = MAX_DELAY_STEPS + 1
        self._head = (self._head + 1) % size
        self._buf[:, self._head] = cmd
        c_xy = self._buf[self._rows, (self._head - self.d_xy) % size, :2]
        c_yaw = self._buf[self._rows, (self._head - self.d_yaw) % size, 2]

        s = self.state
        dv = (c_xy - s.vel) * (self.dt / self.tau_xy)[:, None]
        dv_norm = np.linalg.norm(dv, axis=1)
        lim = self.a_max * self.dt
        scale = np.where(dv_norm > lim, lim / np.maximum(dv_norm, 1e-12), 1.0)
        s.vel += dv * scale[:, None]

        dr = (c_yaw - s.yaw_rate) * (self.dt / self.tau_yaw)
        lim_r = self.yaw_a_max * self.dt
        s.yaw_rate += np.clip(dr, -lim_r, lim_r)

        s.pos[:, :2] += s.vel * self.dt
        s.yaw = (s.yaw + s.yaw_rate * self.dt + math.pi) % (2.0 * math.pi) - math.pi


def simulate_single(
    params: DynamicsParams,
    cmd: np.ndarray,
    *,
    dt_sim_s: float = DT_SIM_S,
    v0: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> np.ndarray:
    """Replay a command sequence ``cmd`` [T, 3] (world vx, vy, yaw rate, one
    row per ``dt_sim``) through one vehicle from rest (or ``v0``); returns
    the achieved ``[T, 3]`` (vx, vy, yaw rate) *after* each step. Used by
    system identification and the parity experiments -- the exact same
    integrator the training envs use, not a separate reimplementation."""
    dyn = BatchDynamics([params], dt_sim_s=dt_sim_s)
    dyn.state.vel[0] = v0[:2]
    dyn.state.yaw_rate[0] = v0[2]
    dyn._buf[0, :] = v0
    out = np.empty_like(cmd, dtype=np.float64)
    for t in range(cmd.shape[0]):
        dyn.step(cmd[t : t + 1])
        out[t, :2] = dyn.state.vel[0]
        out[t, 2] = dyn.state.yaw_rate[0]
    return out

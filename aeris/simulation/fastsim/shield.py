"""Batched Numba port of the S3 collision shield (:mod:`aeris.safety.shield`)
for FastSim's training envs -- the reference implementation works on
Python lists of ``Vec3`` and runs at a few hundred ticks/s, far below the
>=2000 env-steps/s this phase's throughput gate needs.

Semantics are copied, not reinterpreted: same 72 sectors, same
"in-FOV-and-empty = clear at max range" default, same last-seen memory
for out-of-FOV sectors (starting fully blocked, 0.0 m), same "full FOV +
3 sectors nearest the heading" relevance set, same uniform-scale formula
and the same 1e-6 intervention epsilon. A randomized test drives both
implementations with identical inputs, tick after tick, and requires
identical outputs -- shielded RL in FastSim is only fair to Tier H if the
shield is literally the same function.

FastSim vehicles are level (yaw only), so the odom<->body rotation is a
planar rotation by yaw; the reference applies a full quaternion, which is
the same thing for a level vehicle.
"""

from __future__ import annotations

import math

import numpy as np
from numba import njit

from aeris.safety.shield import N_SECTORS, SECTOR_WIDTH_RAD, ShieldConfig, fov_sector_indices


def fov_mask(config: ShieldConfig) -> np.ndarray:
    m = np.zeros(N_SECTORS, dtype=np.bool_)
    for i in fov_sector_indices(config.fov_half_angle_rad):
        m[i] = True
    return m


@njit(cache=True, inline="always")
def _sector_index(x: float, y: float) -> int:
    two_pi = 2.0 * math.pi
    angle = math.atan2(y, x) % two_pi
    return int(angle // SECTOR_WIDTH_RAD) % N_SECTORS


@njit(cache=True)
def shield_batch(
    v_cmd_odom: np.ndarray,  # [N, 2] horizontal odom-frame command
    yaw: np.ndarray,  # [N]
    points_body: np.ndarray,  # [N, K, 3]; rows with non-finite x are skipped
    last_seen: np.ndarray,  # [N, 72] in/out: out-of-FOV memory
    fov: np.ndarray,  # [72] bool
    max_range: float,
    a_brake: float,
    d_safe: float,
    tau: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Returns ``(v_shielded_odom [N, 2], intervened [N])`` and updates
    ``last_seen`` in place, exactly as one ``CollisionShield.project`` call
    per env would."""
    n = v_cmd_odom.shape[0]
    out = np.empty((n, 2))
    intervened = np.zeros(n, dtype=np.bool_)
    obs = np.empty(N_SECTORS)
    for e in range(n):
        # 1. Sector clearances observed this tick (in-FOV only).
        for i in range(N_SECTORS):
            obs[i] = max_range if fov[i] else -1.0
        for k in range(points_body.shape[1]):
            px, py = points_body[e, k, 0], points_body[e, k, 1]
            if not (math.isfinite(px) and math.isfinite(py)):
                continue
            r = math.hypot(px, py)
            if r <= 0.0 or r > max_range:
                continue
            idx = _sector_index(px, py)
            if not fov[idx]:
                continue
            if r < obs[idx]:
                obs[idx] = r
        for i in range(N_SECTORS):
            if obs[i] >= 0.0:
                last_seen[e, i] = obs[i]

        # 2. Command into the body frame.
        c, s = math.cos(yaw[e]), math.sin(yaw[e])
        vx_o, vy_o = v_cmd_odom[e, 0], v_cmd_odom[e, 1]
        bx = c * vx_o + s * vy_o
        by = -s * vx_o + c * vy_o

        # 3. Uniform scale over the relevant sectors.
        scale = 1.0
        moving = not (bx == 0.0 and by == 0.0)
        dom = _sector_index(bx, by) if moving else -10
        for i in range(N_SECTORS):
            relevant = fov[i]
            if moving and not relevant:
                d = (i - dom) % N_SECTORS
                relevant = d == 0 or d == 1 or d == N_SECTORS - 1
            if not relevant:
                continue
            ang = (i + 0.5) * SECTOR_WIDTH_RAD
            dot = bx * math.cos(ang) + by * math.sin(ang)
            if dot <= 0.0:
                continue
            v_i_max = max(
                0.0, math.sqrt(2.0 * a_brake * max(0.0, last_seen[e, i] - d_safe)) - a_brake * tau
            )
            cand = v_i_max / dot
            if cand < scale:
                scale = cand
        if scale < 0.0:
            scale = 0.0
        sbx, sby = bx * scale, by * scale
        ox = c * sbx - s * sby
        oy = s * sbx + c * sby
        out[e, 0] = ox
        out[e, 1] = oy
        intervened[e] = math.hypot(ox - vx_o, oy - vy_o) > 1e-6
    return out, intervened

"""The batched Numba shield must be the same function as the reference
:class:`aeris.safety.shield.CollisionShield` -- identical outputs, tick
after tick, including the stateful out-of-FOV memory."""

from __future__ import annotations

import math

import numpy as np

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import Vec3
from aeris.safety.shield import N_SECTORS, CollisionShield, ShieldConfig
from aeris.simulation.fastsim.shield import fov_mask, shield_batch


def test_numba_shield_matches_reference_over_random_multi_tick_sequences() -> None:
    cfg = ShieldConfig()
    rng = np.random.default_rng(7)
    n_env, n_ticks, k = 6, 40, 30
    refs = [CollisionShield(config=cfg) for _ in range(n_env)]
    last_seen = np.zeros((n_env, N_SECTORS))
    fov = fov_mask(cfg)
    for _ in range(n_ticks):
        yaw = rng.uniform(-math.pi, math.pi, n_env)
        v = rng.uniform(-2.0, 2.0, (n_env, 2))
        v[0] = 0.0  # the zero-command branch
        pts = np.stack(
            [
                rng.uniform(-6, 6, (n_env, k)),
                rng.uniform(-6, 6, (n_env, k)),
                rng.uniform(-1, 1, (n_env, k)),
            ],
            axis=-1,
        )
        pts[:, :3, 0] = np.inf  # no-return rows must be skipped
        got, got_int = shield_batch(
            v, yaw, pts, last_seen, fov, cfg.max_range_m, cfg.a_brake_mps2, cfg.d_safe_m, cfg.tau_s
        )
        for e in range(n_env):
            points = [Vec3(*map(float, p)) for p in pts[e] if math.isfinite(p[0])]
            exp, exp_int = refs[e].project(
                Vec3(float(v[e, 0]), float(v[e, 1]), 0.0),
                orientation_odom_body=Quaternion.from_yaw(float(yaw[e])),
                points_body=points,
            )
            assert math.isclose(got[e, 0], exp.x, abs_tol=1e-9)
            assert math.isclose(got[e, 1], exp.y, abs_tol=1e-9)
            assert bool(got_int[e]) == exp_int
            np.testing.assert_allclose(last_seen[e], refs[e].sector_clearances_m(), atol=1e-12)


def test_lateral_command_is_blocked_until_that_sector_has_been_seen() -> None:
    cfg = ShieldConfig()
    last_seen = np.zeros((1, N_SECTORS))
    out, intervened = shield_batch(
        np.array([[0.0, 1.0]]),  # pure +y (left) in odom; yaw 0 -> outside the forward FOV
        np.zeros(1),
        np.full((1, 1, 3), np.inf),
        last_seen,
        fov_mask(cfg),
        cfg.max_range_m,
        cfg.a_brake_mps2,
        cfg.d_safe_m,
        cfg.tau_s,
    )
    assert np.allclose(out, 0.0)
    assert intervened[0]

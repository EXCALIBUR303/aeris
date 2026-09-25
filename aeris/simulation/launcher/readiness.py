"""Readiness detection: is the Gazebo world up? Is PX4 up and heartbeating?

Spec §17.2: the launcher "waits for readiness (gz topics up, PX4 heartbeat,
EKF converged)" before considering a profile started.

The Gazebo check mirrors PX4's *own* readiness check exactly (transcribed
from ``ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim``'s ``check_scene_info``
function, read directly from the pinned checkout in Phase 3) rather than
inventing a different one — PX4 itself waits on
``gz service -i --service "/world/<world>/scene/info"`` containing
"Service providers", so AERIS does too.
"""

from __future__ import annotations

import subprocess
import time
from collections.abc import Mapping

from pymavlink import mavutil

from aeris.core.clock import Clock, WallClock
from aeris.core.errors import SimulationLaunchError
from aeris.core.logging import get_logger

_logger = get_logger(component="simulation.readiness")

_POLL_INTERVAL_S = 1.0


def wait_for_gz_world_ready(
    *,
    world: str,
    env: Mapping[str, str],
    timeout_s: float,
    clock: Clock | None = None,
) -> None:
    """Block until Gazebo reports the named world's scene-info service is up.

    Raises :class:`SimulationLaunchError` on timeout, with the last
    ``gz service -i`` output attached for diagnosis.
    """
    clock = clock or WallClock()
    deadline = clock.now() + timeout_s
    service = f"/world/{world}/scene/info"
    last_output = ""

    while clock.now() < deadline:
        try:
            result = subprocess.run(
                ["gz", "service", "-i", "--service", service],
                env=dict(env),
                capture_output=True,
                text=True,
                timeout=min(5.0, timeout_s),
            )
            last_output = (result.stdout or "") + (result.stderr or "")
        except subprocess.TimeoutExpired:
            # `gz service -i` on a service with no provider yet doesn't
            # return promptly — it blocks until its own internal timeout.
            # That's "not ready yet," not an error: keep polling.
            last_output = "(gz service -i timed out)"
        if "Service providers" in last_output:
            _logger.info("gz_world.ready", world=world)
            return
        _logger.debug("gz_world.waiting", world=world)
        time.sleep(_POLL_INTERVAL_S)

    raise SimulationLaunchError(
        f"Gazebo world {world!r} did not become ready within {timeout_s}s. "
        f"Last `gz service -i` output:\n{last_output}"
    )


def wait_for_px4_heartbeat(
    *,
    port: int,
    timeout_s: float,
) -> mavutil.mavlink_connection:
    """Block until a MAVLink heartbeat arrives from PX4 on ``port``.

    Returns the open :class:`~pymavlink.mavutil.mavlink_connection` on
    success — callers that also want EKF status (:func:`wait_for_ekf_ok`)
    should reuse this connection rather than opening a second one. Raises
    :class:`SimulationLaunchError` on timeout.
    """
    conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}")
    msg = conn.wait_heartbeat(timeout=timeout_s)
    if msg is None:
        conn.close()
        raise SimulationLaunchError(
            f"no MAVLink heartbeat received on port {port} within {timeout_s}s"
        )
    _logger.info("px4.heartbeat", port=port, system=conn.target_system)
    return conn


def wait_for_ekf_ok(
    *,
    conn: mavutil.mavlink_connection,
    timeout_s: float,
    min_pos_horiz_accuracy_m: float | None = None,
) -> None:
    """Block until PX4's ``ESTIMATOR_STATUS`` reports a healthy EKF.

    "Healthy" here means a message was received (EKF2 is running and
    reporting) and, if ``min_pos_horiz_accuracy_m`` is given, its
    ``pos_horiz_accuracy`` is at or below that bound. This is deliberately
    looser than spec §7.3's eventual full convergence gate — Phase 3 only
    needs "the estimator is alive," not "converged to spec"; later phases
    (5+) that need a converged position estimate before arming should
    layer their own stricter wait on top.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = max(0.1, deadline - time.monotonic())
        msg = conn.recv_match(type="ESTIMATOR_STATUS", blocking=True, timeout=min(1.0, remaining))
        if msg is not None:
            if (
                min_pos_horiz_accuracy_m is not None
                and msg.pos_horiz_accuracy > min_pos_horiz_accuracy_m
            ):
                continue
            _logger.info("px4.ekf_ok", flags=msg.flags, pos_horiz_accuracy=msg.pos_horiz_accuracy)
            return
    raise SimulationLaunchError(f"no healthy ESTIMATOR_STATUS received within {timeout_s}s")

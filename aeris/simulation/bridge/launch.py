"""Launching the Sensor Bridge process (spec §18.4).

The bridge (:mod:`aeris.simulation.bridge._gz_process`) is the only AERIS
code that imports ``gz.*`` — it needs Homebrew's ``gz-transport13``/
``gz-msgs10`` Python bindings on its import path plus their native
libraries resolvable, neither of which the rest of AERIS should carry.
This module builds that one process's environment and launches it via the
same tracked-PID :class:`ManagedProcess` every other AERIS-owned process
uses (spec §51: "Tracked-PID shutdown only") — it is not a special case.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from aeris.simulation.launcher.process import ManagedProcess

# Phase 8 finding (docs/sensors.md): Homebrew's gz-transport13/gz-msgs10
# ship precompiled Python bindings ABI-compatible with this project's own
# uv-managed CPython 3.12 — no separate Homebrew-Python venv is needed,
# only these two paths plus the `protobuf` package in this venv.
_HOMEBREW_LIB_DIR = "/opt/homebrew/lib"
_HOMEBREW_SITE_PACKAGES = "/opt/homebrew/lib/python3.12/site-packages"

_BRIDGE_PORT_BASE = 19000  # AERIS-owned convention, unrelated to PX4's own MAVLink ports.


def bridge_zmq_endpoint(instance: int = 0) -> str:
    """The ZeroMQ PUB endpoint the bridge binds and :class:`GzBridgeClient` connects to."""
    return f"tcp://127.0.0.1:{_BRIDGE_PORT_BASE + instance}"


def rgb_topic_for_model(*, world: str, model_instance: str) -> str:
    """The gz topic name for an ``x500_depth`` instance's RGB (``IMX214``) camera.

    Namespaced per model instance (verified live, Phase 8), unlike the
    depth/camera_info topics, which the sensor's own SDF forces to the
    fixed global names ``/depth_camera``/``/camera_info`` via an explicit
    ``<topic>`` override.
    """
    return f"/world/{world}/model/{model_instance}/link/camera_link/sensor/IMX214/image"


def bridge_env(
    *,
    homebrew_lib_dir: str = _HOMEBREW_LIB_DIR,
    homebrew_site_packages: str = _HOMEBREW_SITE_PACKAGES,
) -> dict[str, str]:
    env = dict(os.environ)
    # Every other AERIS-launched process sets this (see launcher.py's
    # `_gz_env`/`_start_px4`) -- without it, `gz.transport13.Node` can't
    # find the running instance's transport partition at all (confirmed
    # live, Phase 8: `gz topic -l` itself returns nothing without it either).
    env["GZ_IP"] = "127.0.0.1"
    existing_pythonpath = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = (
        f"{homebrew_site_packages}{os.pathsep}{existing_pythonpath}"
        if existing_pythonpath
        else homebrew_site_packages
    )
    existing_dyld = env.get("DYLD_LIBRARY_PATH", "")
    env["DYLD_LIBRARY_PATH"] = (
        f"{homebrew_lib_dir}{os.pathsep}{existing_dyld}" if existing_dyld else homebrew_lib_dir
    )
    return env


def start_bridge(
    *,
    world: str = "default",
    model_instance: str = "x500_depth_0",
    instance: int = 0,
    duration_s: float | None = None,
    run_dir: Path | None = None,
) -> tuple[ManagedProcess, str]:
    """Launch the Sensor Bridge for one running vehicle instance.

    Returns ``(process, zmq_endpoint)`` — callers connect a
    :class:`~aeris.simulation.bridge.client.GzBridgeClient` to the endpoint
    once the process is up (there is no readiness handshake yet; the
    caller is expected to retry a first ``recv()`` briefly, matching how
    ZeroMQ PUB/SUB itself tolerates a late-connecting subscriber).
    """
    endpoint = bridge_zmq_endpoint(instance)
    argv = [
        sys.executable,
        "-m",
        "aeris.simulation.bridge._gz_process",
        "--zmq-endpoint",
        endpoint,
        "--rgb-topic",
        rgb_topic_for_model(world=world, model_instance=model_instance),
    ]
    if duration_s is not None:
        argv += ["--duration-s", str(duration_s)]

    log_path = None if run_dir is None else run_dir / "logs" / "sensor-bridge.log"
    process = ManagedProcess(name="sensor-bridge", argv=argv, env=bridge_env(), log_path=log_path)
    process.start()
    return process, endpoint

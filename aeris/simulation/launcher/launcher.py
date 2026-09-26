"""The PX4 SITL + Gazebo simulation launcher (spec §17.2, §51 Phase 3).

:class:`SimulationLauncher` owns the two OS processes a Tier-H run needs —
the Gazebo server and PX4 SITL — end to end: build their environment,
start them in the right order, block until each is actually ready, and
tear both down cleanly by tracked PID (spec: "Tracked-PID shutdown only").

Architecture (spec §17.2, mirrors PX4's own
``ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim``, read directly from the
pinned checkout in Phase 3):

1. AERIS launches ``gz sim -s <world.sdf>`` itself (``PX4_GZ_STANDALONE=1``
   tells PX4 *not* to launch its own copy) — AERIS therefore owns the
   Gazebo server's process lifecycle, not PX4.
2. AERIS waits for the world to report ready, using the exact same
   ``gz service -i --service "/world/<world>/scene/info"`` check PX4
   itself uses.
3. AERIS launches the built ``px4`` binary. PX4 handles spawning its own
   vehicle model into the already-running world and starting its internal
   ``gz_bridge`` — that part is PX4's job, not reimplemented here.
4. AERIS waits for a MAVLink heartbeat, then (optionally) a healthy
   ``ESTIMATOR_STATUS``.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from pathlib import Path

from pymavlink import mavutil

from aeris.core.clock import Clock, WallClock
from aeris.core.errors import SimulationLaunchError
from aeris.core.logging import get_logger
from aeris.simulation.launcher.params import apply_params
from aeris.simulation.launcher.ports import InstancePorts, instance_ports
from aeris.simulation.launcher.process import ManagedProcess
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.launcher.readiness import (
    wait_for_ekf_ok,
    wait_for_gz_world_ready,
    wait_for_px4_heartbeat,
)
from aeris.simulation.px4_paths import Px4Layout

_logger = get_logger(component="simulation.launcher")


@dataclass(frozen=True, slots=True)
class LaunchResult:
    """What a successful :meth:`SimulationLauncher.start` produced."""

    profile: SimulationProfile
    ports: InstancePorts
    gz_pid: int | None
    px4_pid: int
    world_ready_s: float
    px4_ready_s: float


class SimulationLauncher:
    """Starts, monitors, and tears down one PX4 SITL + Gazebo instance.

    One launcher manages exactly one vehicle instance at a time — spawn a
    second :class:`SimulationLauncher` (with a different ``instance``
    number in its profile) for multi-vehicle work, which is Linux-only
    per spec §9.1/§12 and out of scope until Phase 33 anyway.
    """

    def __init__(
        self,
        layout: Px4Layout,
        *,
        run_dir: Path | None = None,
        clock: Clock | None = None,
    ) -> None:
        self._layout = layout
        self._run_dir = run_dir
        self._clock = clock or WallClock()

        self._gz_process: ManagedProcess | None = None
        self._gz_gui_process: ManagedProcess | None = None
        self._px4_process: ManagedProcess | None = None
        self._conn: mavutil.mavlink_connection | None = None
        self._profile: SimulationProfile | None = None

    # --- public API --------------------------------------------------------

    @property
    def mavlink_connection(self) -> mavutil.mavlink_connection | None:
        """The open MAVLink connection to the offboard link, once started."""
        return self._conn

    def is_running(self) -> bool:
        gz_ok = self._gz_process is None or self._gz_process.is_running()
        px4_ok = self._px4_process is not None and self._px4_process.is_running()
        return px4_ok and gz_ok

    def start(
        self,
        profile: SimulationProfile,
        *,
        params: dict[str, float] | None = None,
    ) -> LaunchResult:
        """Launch Gazebo (if standalone) then PX4, and block until both are ready.

        Raises :class:`SimulationLaunchError` (via the readiness helpers)
        if either stage doesn't come up in time; whatever was already
        started is left running for post-mortem inspection — call
        :meth:`stop` explicitly rather than relying on this to clean up
        after a failed start, since the failure itself is diagnostic
        signal a caller may want to log before tearing down.
        """
        if self._px4_process is not None:
            raise RuntimeError("SimulationLauncher.start() called while already started")

        self._layout.ensure_built()
        self._profile = profile
        ports = instance_ports(profile.instance)
        t_start = self._clock.now()

        gz_pid: int | None = None
        if profile.standalone:
            gz_pid = self._start_gazebo(profile)
            wait_for_gz_world_ready(
                world=profile.world,
                env=self._gz_env(profile),
                timeout_s=profile.world_ready_timeout_s,
                clock=self._clock,
            )
        world_ready_s = self._clock.now() - t_start

        t_px4 = self._clock.now()
        px4_pid = self._start_px4(profile, ports)
        conn = wait_for_px4_heartbeat(
            port=ports.offboard_remote, timeout_s=profile.heartbeat_timeout_s
        )
        self._conn = conn
        wait_for_ekf_ok(conn=conn, timeout_s=profile.heartbeat_timeout_s)
        px4_ready_s = self._clock.now() - t_px4

        if params:
            apply_params(conn, params)

        _logger.info(
            "launcher.ready",
            profile=profile.name,
            world_ready_s=round(world_ready_s, 2),
            px4_ready_s=round(px4_ready_s, 2),
        )
        return LaunchResult(
            profile=profile,
            ports=ports,
            gz_pid=gz_pid,
            px4_pid=px4_pid,
            world_ready_s=world_ready_s,
            px4_ready_s=px4_ready_s,
        )

    def stop(self) -> None:
        """Tear down PX4 then Gazebo, by tracked PID only. Idempotent."""
        if self._conn is not None:
            self._conn.close()
            self._conn = None
        if self._px4_process is not None:
            self._px4_process.stop()
            self._px4_process = None
        if self._gz_gui_process is not None:
            self._gz_gui_process.stop()
            self._gz_gui_process = None
        if self._gz_process is not None:
            self._gz_process.stop()
            self._gz_process = None
        self._profile = None

    def reset(self, params: dict[str, float] | None = None) -> tuple[LaunchResult, float]:
        """Full stop + restart of the last-started profile.

        Spec §17.2: "full PX4 + gz restart between evaluation episodes is
        the correct default, because it guarantees a clean EKF and
        failsafe state." Returns ``(result, wall_time_s)`` so callers can
        record the cost (spec §51 Phase 3: "measure its cost").
        """
        if self._profile is None:
            raise RuntimeError("reset() called before a successful start()")
        profile = self._profile
        t0 = self._clock.now()
        self.stop()
        result = self.start(profile, params=params)
        return result, self._clock.now() - t0

    # --- internals -----------------------------------------------------------

    def _gz_env(self, profile: SimulationProfile) -> dict[str, str]:
        env = dict(os.environ)
        env["GZ_IP"] = "127.0.0.1"
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
            [str(self._layout.models_dir), str(self._layout.worlds_dir)]
        )
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(self._layout.gz_plugins_dir)
        env["GZ_SIM_SERVER_CONFIG_PATH"] = str(self._layout.gz_server_config)
        return env

    def _start_gazebo(self, profile: SimulationProfile) -> int:
        world_sdf = profile.world_sdf_path or self._layout.world_sdf(profile.world)
        if not world_sdf.is_file():
            raise SimulationLaunchError(f"world file not found: {world_sdf}")

        argv = ["gz", "sim", f"--verbose={profile.gz_verbose}", "-r", "-s", str(world_sdf)]
        if profile.render_engine:
            argv += ["--render-engine", profile.render_engine]
        # When not headless, the GUI is a *second* `gz sim -g` process
        # (same split PX4's own non-standalone path uses) — started below,
        # after the server exists.

        proc = ManagedProcess(
            name="gz-sim",
            argv=argv,
            env=self._gz_env(profile),
            log_path=self._log_path("gz-sim.log"),
        )
        proc.start()
        self._gz_process = proc

        if not profile.headless:
            gui = ManagedProcess(
                name="gz-sim-gui",
                argv=["gz", "sim", "-g"],
                env=self._gz_env(profile),
                log_path=self._log_path("gz-sim-gui.log"),
            )
            gui.start()
            # The GUI is cosmetic (spec: "GUI failure alone does not fail
            # the gate if headless works") — track it so stop() can tear it
            # down too, but don't block readiness on it.
            self._gz_gui_process = gui

        assert proc.pid is not None
        return proc.pid

    def _start_px4(self, profile: SimulationProfile, ports: InstancePorts) -> int:
        env = dict(os.environ)
        env["GZ_IP"] = "127.0.0.1"
        env["PX4_SIM_MODEL"] = profile.model
        env["PX4_GZ_WORLD"] = profile.world
        env["PX4_GZ_STANDALONE"] = "1"
        env["PX4_GZ_MODELS"] = str(self._layout.models_dir)
        env["PX4_GZ_WORLDS"] = str(self._layout.worlds_dir)
        if profile.spawn_pose is not None:
            env["PX4_GZ_MODEL_POSE"] = ",".join(str(v) for v in profile.spawn_pose)
        if profile.speed_factor != 1.0:
            env["PX4_SIM_SPEED_FACTOR"] = str(profile.speed_factor)
        if profile.render_engine:
            env["PX4_GZ_SIM_RENDER_ENGINE"] = profile.render_engine

        argv = [str(self._layout.binary_path)]
        if profile.instance != 0:
            argv += ["-i", str(profile.instance)]

        proc = ManagedProcess(
            name="px4",
            argv=argv,
            env=env,
            log_path=self._log_path("px4.log"),
        )
        proc.start()
        self._px4_process = proc
        assert proc.pid is not None
        return proc.pid

    def _log_path(self, filename: str) -> Path | None:
        if self._run_dir is None:
            return None
        return self._run_dir / "logs" / filename


def wait_ready_or_raise(launcher: SimulationLauncher, timeout_s: float) -> None:
    """Small helper for callers that already called :meth:`start` and just
    want to assert readiness held (e.g. after a manual sleep in a test).
    Not used by :meth:`SimulationLauncher.start` itself, which does its own
    precise waiting — this is for defensive re-checks only.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if launcher.is_running():
            return
        time.sleep(0.2)
    raise SimulationLaunchError("simulation processes are not both running")

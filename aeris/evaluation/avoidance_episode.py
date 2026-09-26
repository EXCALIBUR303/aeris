"""Wires :mod:`aeris.autonomy.navigation`'s follower/reactive policies and
:mod:`aeris.safety.shield` into one live control loop and runs it to a goal
(spec §51 Phase 10: "the first formal experiment"). Lives beside
:mod:`aeris.evaluation.episode` (Phase 7's identical "one live episode, one
result" pattern) rather than in ``aeris.autonomy`` itself, since this
module owns simulation/IPC concerns (the Sensor Bridge connection, the
control-loop timing) that ``aeris.autonomy.navigation``'s own modules
deliberately don't -- they're pure goal-seeking/avoidance math.

The GT collision check (:func:`aeris.evaluation.metrics.collision.has_collision`)
is computed *after* the flight, from the recorded trajectory, never fed to
the agent mid-flight -- spec §17.4's provenance rule: the agent only ever
sees the depth points this loop already collects for the shield/reactive
planner, never simulator ground truth.
"""

from __future__ import annotations

import asyncio
import time
from array import array
from dataclasses import dataclass, field
from typing import Literal

from aeris.autonomy.navigation.follower import PathFollower
from aeris.autonomy.navigation.reactive import ReactiveAvoidance
from aeris.core.clock import Clock
from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import Vec3
from aeris.perception.depth.projection import CameraIntrinsics, depth_to_points_camera
from aeris.safety.shield import (
    CollisionShield,
    ShieldConfig,
    compute_sector_clearances,
    fov_sector_indices,
)
from aeris.safety.supervisor import SafetySupervisor
from aeris.simulation.bridge.client import GzBridgeClient
from aeris.vehicle.interface import VehicleEndpoint, VelocitySetpoint

Method = Literal["reactive", "straight_line"]

_TICK_PERIOD_S = 0.1
_ARRIVAL_RADIUS_M = 0.4


@dataclass(frozen=True, slots=True)
class AvoidanceEpisodeResult:
    method: Method
    positions: tuple[Vec3, ...]
    velocities: tuple[Vec3, ...]
    arrived: bool
    wall_time_s: float
    shield_intervention_rate: float
    shield_tick_count: int
    reason: str = ""


@dataclass(slots=True)
class _DepthSource:
    """Polls the Sensor Bridge for the latest depth frame and back-projects
    it into body-frame points, ready for the shield/reactive planner."""

    endpoint: str
    intrinsics: CameraIntrinsics
    t_body_optical: Transform
    fov_sectors: frozenset[int] = field(default_factory=lambda: fov_sector_indices(0.637))
    # Back-projecting all 307,200 pixels of a 640x480 depth image in pure
    # Python took ~150ms per call, measured directly (Phase 10) -- more
    # than the entire intended 100ms control-tick period, which visibly
    # destabilized the vertical velocity control into a growing
    # oscillation (the loop was always reacting to a stale, seconds-old
    # position). Stride 8 (~4800 points, ~3ms) is far more than enough to
    # populate 72 angular sectors with a robust per-sector minimum.
    depth_stride: int = 8
    _client: GzBridgeClient | None = None

    def __enter__(self) -> _DepthSource:
        self._client = GzBridgeClient(self.endpoint, ["depth"], conflate=True, rcv_timeout_ms=50)
        return self

    def __exit__(self, *exc: object) -> None:
        if self._client is not None:
            self._client.close()

    def latest_points_body(self) -> list[Vec3]:
        assert self._client is not None
        msg = self._client.recv()
        if msg is None:
            return []
        payload = msg["payload"]
        buf: array[float] = array("f")
        buf.frombytes(payload["data"])
        if len(buf) != payload["width"] * payload["height"]:
            return []
        points_optical = depth_to_points_camera(buf, self.intrinsics, stride=self.depth_stride)
        return [self.t_body_optical.apply(p) for p in points_optical]


def _sector_clearances_for_reactive(
    points_body: list[Vec3], *, fov_sectors: frozenset[int], max_range_m: float
) -> list[float]:
    observed = compute_sector_clearances(
        points_body, max_range_m=max_range_m, fov_sectors=fov_sectors
    )
    return [d if d is not None else 0.0 for d in observed]


async def run_avoidance_episode(
    *,
    supervisor: SafetySupervisor,
    endpoint: VehicleEndpoint,
    bridge_endpoint: str,
    camera_intrinsics: CameraIntrinsics,
    t_body_optical: Transform,
    goal_odom: Vec3,
    method: Method,
    takeoff_altitude_m: float = 1.5,
    cruise_speed_mps: float = 1.0,
    timeout_s: float = 60.0,
    shield_config: ShieldConfig | None = None,
    clock: Clock,
) -> AvoidanceEpisodeResult:
    shield = CollisionShield(config=shield_config or ShieldConfig())
    follower = PathFollower(cruise_speed_mps=cruise_speed_mps)
    reactive = ReactiveAvoidance(cruise_speed_mps=cruise_speed_mps)

    positions: list[Vec3] = []
    velocities: list[Vec3] = []
    arrived = False
    reason = ""
    t_start = time.monotonic()

    with _DepthSource(bridge_endpoint, camera_intrinsics, t_body_optical) as depth_source:
        try:
            await supervisor.connect(endpoint)
            if not await supervisor.preflight_check():
                return AvoidanceEpisodeResult(
                    method,
                    (),
                    (),
                    False,
                    time.monotonic() - t_start,
                    0.0,
                    0,
                    "preflight_check failed",
                )

            arm_result = await supervisor.arm_and_takeoff(takeoff_altitude_m)
            if not arm_result.ok:
                return AvoidanceEpisodeResult(
                    method,
                    (),
                    (),
                    False,
                    time.monotonic() - t_start,
                    0.0,
                    0,
                    "arm_and_takeoff rejected",
                )

            deadline = clock.now() + 30.0
            reached_altitude = False
            while clock.now() < deadline:
                state = await supervisor.get_vehicle_state()
                if state.pose_odom.z >= takeoff_altitude_m - 0.5:
                    reached_altitude = True
                    break
                await asyncio.sleep(0.5)
            if not reached_altitude:
                reason = "never reached takeoff altitude"

            await supervisor.confirm_airborne()
            await supervisor.start_autonomy()

            deadline = clock.now() + timeout_s
            while clock.now() < deadline:
                state = await supervisor.get_vehicle_state()
                positions.append(state.pose_odom)

                points_body = depth_source.latest_points_body()

                if method == "reactive":
                    clearances = _sector_clearances_for_reactive(
                        points_body,
                        fov_sectors=depth_source.fov_sectors,
                        max_range_m=shield.config.max_range_m,
                    )
                    v_desired = reactive.compute_velocity(state.pose_odom, goal_odom, clearances)
                else:
                    v_desired = follower.compute_velocity(state.pose_odom, goal_odom)

                v_shielded, _intervened = shield.project(
                    v_desired, orientation_odom_body=state.orientation_odom, points_body=points_body
                )
                velocities.append(v_shielded)

                sp = VelocitySetpoint(velocity=v_shielded, frame="odom")
                await supervisor.submit_setpoint(sp)
                supervisor.autonomy_heartbeat()
                await supervisor.tick()

                if (state.pose_odom - goal_odom).norm() <= _ARRIVAL_RADIUS_M:
                    arrived = True
                    break

                await asyncio.sleep(_TICK_PERIOD_S)
            else:
                if not reason:
                    reason = "timeout"

            await supervisor.stop_autonomy()
            await supervisor.land()
            deadline = clock.now() + 30.0
            while clock.now() < deadline:
                state = await supervisor.get_vehicle_state()
                if state.landed_state.value == "on_ground":
                    break
                await asyncio.sleep(0.5)
            supervisor.confirm_landed()
            await supervisor.disarm()
        finally:
            await supervisor.disconnect()

    return AvoidanceEpisodeResult(
        method=method,
        positions=tuple(positions),
        velocities=tuple(velocities),
        arrived=arrived,
        wall_time_s=time.monotonic() - t_start,
        shield_intervention_rate=shield.intervention_rate,
        shield_tick_count=shield.tick_count,
        reason=reason,
    )

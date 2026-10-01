"""Runs one live exploration episode (spec §51 Phase 12's "exploration
episodes" sim-test line): wires an :class:`~aeris.autonomy.exploration.base.ExplorationStrategy`
into a live control loop that builds its own map, re-plans a subgoal
periodically, and drives there via A* + :class:`~aeris.autonomy.navigation.follower.PathFollower`
under the collision shield -- the same "shared planner, follower, shield"
spec §25's fairness rules require of every classical baseline (and,
later, the learned exploration policy).

Lives beside :mod:`aeris.evaluation.avoidance_episode`/`mapping_episode`
for the same reason both of those do: this module owns simulation/IPC
concerns (Sensor Bridge, GT polling, control-loop timing) that
``aeris.mapping``/``aeris.autonomy`` deliberately don't.

The agent's own map is built from its **EKF pose only** -- exactly what a
real deployment would see (spec §17.4's provenance rule). GT pose is
polled purely so the evaluator can compute coverage/localization metrics
*after* the flight from the recorded trajectory (:mod:`aeris.evaluation.metrics.coverage`);
it is never fed into the agent's own map, exploration decisions, or A*
planning.

Every command carries an explicit altitude hold (:class:`~aeris.autonomy.navigation.altitude.AltitudeHold`,
on the EKF altitude), and the result carries a GT-altitude validity
verdict (:func:`~aeris.evaluation.metrics.altitude.check_altitude_band`)
so a vehicle that ends up on the floor is flagged rather than scored --
both added after Phase 13's instrumented re-runs found grounded episodes
in Phase 12's harness (``docs/exploration.md``).
"""

from __future__ import annotations

import asyncio
import gc
import math
import time
from array import array
from dataclasses import dataclass, field

import numpy as np

from aeris.autonomy.exploration.base import AgentPose, ExplorationStrategy
from aeris.autonomy.navigation.altitude import AltitudeHold
from aeris.autonomy.navigation.follower import PathFollower
from aeris.autonomy.planning.astar import astar
from aeris.core.clock import Clock
from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import Vec3
from aeris.evaluation.metrics.altitude import (
    DEFAULT_MAX_EXCURSION_S,
    AltitudeValidity,
    check_altitude_band,
)
from aeris.localization.px4_ekf import Px4EkfPose
from aeris.mapping.deltas import DeltaTracker
from aeris.mapping.projection import BandGrid, Cell2D, cell_center_world, project_band
from aeris.mapping.raycast import warm_up
from aeris.mapping.voxel import MappingConfig, VoxelMap
from aeris.perception.depth.projection import CameraIntrinsics, depth_to_points_camera
from aeris.safety.shield import CollisionShield, ShieldConfig
from aeris.safety.supervisor import SafetySupervisor
from aeris.simulation.bridge.client import GzBridgeClient
from aeris.simulation.groundtruth.service import GroundTruthService
from aeris.vehicle.interface import VehicleEndpoint, VelocitySetpoint

_TICK_PERIOD_S = 0.1
_ROTATE_YAW_RATE_RADPS = 0.6
_ROTATE_DURATION_S = 2.0


@dataclass(frozen=True, slots=True)
class ExplorationEpisodeResult:
    final_grid: BandGrid
    gt_trajectory: tuple[tuple[float, Vec3, float], ...]  # (t_sim_s, position_world, yaw_rad)
    n_subgoals_chosen: int
    n_rotate_scans: int
    shield_intervention_rate: float
    wall_time_s: float
    reason: str = ""
    # GT-altitude validity of ``gt_trajectory`` against the episode's own
    # altitude band (``z_lo_m``/``z_hi_m``); ``None`` only when the episode
    # never reached its control loop (preflight/arm failure).
    altitude_validity: AltitudeValidity | None = None


def _yaw_from_orientation(q: Quaternion) -> float:
    heading = q.rotate(Vec3(1.0, 0.0, 0.0))
    return math.atan2(heading.y, heading.x)


def _rotation_matrix(q: Quaternion) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


@dataclass(slots=True)
class _DepthSource:
    """Identical role to :class:`aeris.evaluation.mapping_episode._DepthSource`
    (frame's own ``t_sim_s``, vectorized extrinsics) -- kept as its own
    small copy rather than importing that module's private class across an
    unrelated experiment boundary, matching that module's own precedent."""

    endpoint: str
    intrinsics: CameraIntrinsics
    t_body_optical: Transform
    depth_stride: int = 8
    _client: GzBridgeClient | None = None
    _rotation: np.ndarray = field(init=False)
    _translation: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self._rotation = _rotation_matrix(self.t_body_optical.rotation)
        t = self.t_body_optical.translation
        self._translation = np.array([t.x, t.y, t.z])

    def __enter__(self) -> _DepthSource:
        self._client = GzBridgeClient(self.endpoint, ["depth"], conflate=True, rcv_timeout_ms=50)
        return self

    def __exit__(self, *exc: object) -> None:
        if self._client is not None:
            self._client.close()

    def latest_points_body(self) -> tuple[list[Vec3], float | None]:
        assert self._client is not None
        msg = self._client.recv()
        if msg is None:
            return [], None
        payload = msg["payload"]
        buf: array[float] = array("f")
        buf.frombytes(payload["data"])
        if len(buf) != payload["width"] * payload["height"]:
            return [], None
        points_optical = depth_to_points_camera(buf, self.intrinsics, stride=self.depth_stride)
        if not points_optical:
            return [], msg["t_sim_s"]
        optical_arr = np.array(points_optical)
        body_arr = optical_arr @ self._rotation.T + self._translation
        points_body = [Vec3(x, y, z) for x, y, z in body_arr.tolist()]
        return points_body, msg["t_sim_s"]


def _path_to_waypoints(grid: BandGrid, path_cells: tuple[Cell2D, ...], *, z_m: float) -> list[Vec3]:
    return [cell_center_world(grid, cell, z_m=z_m) for cell in path_cells]


async def run_exploration_episode(
    *,
    supervisor: SafetySupervisor,
    endpoint: VehicleEndpoint,
    bridge_endpoint: str,
    camera_intrinsics: CameraIntrinsics,
    t_body_optical: Transform,
    gt_service: GroundTruthService,
    gt_model_name: str,
    strategy: ExplorationStrategy,
    hover_altitude_m: float,
    z_lo_m: float,
    z_hi_m: float,
    x_range_m: tuple[float, float],
    y_range_m: tuple[float, float],
    timeout_s: float,
    mapping_config: MappingConfig | None = None,
    inflation_m: float = 0.3,
    unknown_cost_multiplier: float = 1.0,
    arrival_radius_m: float = 0.4,
    # Not "how often exploration decisions should happen" (arrival and a
    # significant map change already trigger a replan as soon as either
    # is real) -- this is a stuck-safety-net only. Confirmed live
    # (Phase 12) that 10.0 was far too aggressive for that role: in a
    # densely cluttered world the collision shield can throttle a
    # chosen subgoal's approach so heavily (80-94% intervention rate,
    # measured) that the vehicle is still making genuine, if slow,
    # progress toward it well past 10s -- a short timeout abandoned that
    # progress and forced a fresh (often equally-throttled) decision
    # roughly every 10s, which live data showed tracked almost exactly
    # with elapsed/10 rather than with genuine arrivals. 45.0 gives a
    # real hop time to complete even under heavy throttling while still
    # catching an agent that's truly stuck (oscillating, not just slow).
    max_replan_interval_s: float = 45.0,
    replan_map_change_cells: int = 20,
    shield_config: ShieldConfig | None = None,
    cruise_speed_mps: float = 1.0,
    altitude_kp_per_s: float = 1.0,
    altitude_max_vz_mps: float = 0.5,
    max_altitude_excursion_s: float = DEFAULT_MAX_EXCURSION_S,
    clock: Clock,
) -> ExplorationEpisodeResult:
    warm_up()
    config = mapping_config or MappingConfig()
    voxel_map = VoxelMap(config=config)
    ekf_source = Px4EkfPose(max_gap_s=2.0)
    follower = PathFollower(cruise_speed_mps=cruise_speed_mps, arrival_radius_m=arrival_radius_m)
    altitude_hold = AltitudeHold(
        target_z_m=hover_altitude_m, kp_per_s=altitude_kp_per_s, max_vz_mps=altitude_max_vz_mps
    )
    shield = CollisionShield(config=shield_config or ShieldConfig())
    delta_tracker = DeltaTracker()

    gt_trajectory: list[tuple[float, Vec3, float]] = []
    n_subgoals_chosen = 0
    n_rotate_scans = 0
    reason = ""
    t_start = time.monotonic()

    current_path: list[Vec3] = []
    waypoint_index = 0
    rotate_until_s: float | None = None
    t_last_replan_s = 0.0

    gc.disable()
    try:
        with _DepthSource(bridge_endpoint, camera_intrinsics, t_body_optical) as depth_source:
            try:
                await supervisor.connect(endpoint)
                if not await supervisor.preflight_check():
                    return ExplorationEpisodeResult(
                        project_band(
                            voxel_map,
                            z_lo_m=z_lo_m,
                            z_hi_m=z_hi_m,
                            x_range_m=x_range_m,
                            y_range_m=y_range_m,
                        ),
                        (),
                        0,
                        0,
                        0.0,
                        time.monotonic() - t_start,
                        "preflight_check failed",
                    )

                arm_result = await supervisor.arm_and_takeoff(hover_altitude_m)
                if not arm_result.ok:
                    return ExplorationEpisodeResult(
                        project_band(
                            voxel_map,
                            z_lo_m=z_lo_m,
                            z_hi_m=z_hi_m,
                            x_range_m=x_range_m,
                            y_range_m=y_range_m,
                        ),
                        (),
                        0,
                        0,
                        0.0,
                        time.monotonic() - t_start,
                        "arm_and_takeoff rejected",
                    )

                deadline = clock.now() + 30.0
                reached_altitude = False
                while clock.now() < deadline:
                    state = await supervisor.get_vehicle_state()
                    if state.pose_odom.z >= hover_altitude_m - 0.5:
                        reached_altitude = True
                        break
                    await asyncio.sleep(0.5)
                if not reached_altitude:
                    reason = "never reached hover altitude"

                await supervisor.confirm_airborne()
                await supervisor.start_autonomy()

                grid = project_band(
                    voxel_map,
                    z_lo_m=z_lo_m,
                    z_hi_m=z_hi_m,
                    x_range_m=x_range_m,
                    y_range_m=y_range_m,
                    free_rule="any_observed",
                )
                t_sim_s_start: float | None = None
                deadline = clock.now() + timeout_s
                while clock.now() < deadline:
                    state = await supervisor.get_vehicle_state()
                    ekf_source.update(state)
                    if t_sim_s_start is None:
                        t_sim_s_start = state.t_sim_s
                    # Replan/rotation timing uses PX4's own sim clock, not
                    # a synthetic per-tick counter -- this loop's real
                    # per-iteration cost (GT-pose subprocess poll, depth
                    # processing, A*, project_band) is well over the
                    # nominal 100ms tick (Phase 11 measured ~500-700ms for
                    # this same GT-polling pattern), so a counter
                    # incremented by a fixed 0.1 per iteration drifts far
                    # behind real/sim time -- a 10s replan interval or a
                    # 2s rotation burst measured against that counter
                    # would actually take many times longer in real time
                    # than intended, badly distorting the exploration
                    # cadence. state.t_sim_s tracks real elapsed time
                    # regardless of how long any given tick takes.
                    elapsed = state.t_sim_s - t_sim_s_start

                    points_body, depth_t_sim_s = depth_source.latest_points_body()
                    if points_body and depth_t_sim_s is not None:
                        ekf_estimate = ekf_source.pose_at(depth_t_sim_s)
                        if ekf_estimate is not None:
                            origin = ekf_estimate.pose_m_b.translation
                            voxel_map.integrate_points(
                                origin, [ekf_estimate.pose_m_b.apply(p) for p in points_body]
                            )
                            gc.collect()

                            gt_pos_world, gt_quat = gt_service.get_pose(gt_model_name)
                            gt_trajectory.append(
                                (depth_t_sim_s, gt_pos_world, _yaw_from_orientation(gt_quat))
                            )

                    grid = project_band(
                        voxel_map,
                        z_lo_m=z_lo_m,
                        z_hi_m=z_hi_m,
                        x_range_m=x_range_m,
                        y_range_m=y_range_m,
                        inflation_m=inflation_m,
                        free_rule="any_observed",
                    )
                    delta = delta_tracker.diff(elapsed, grid)

                    agent_pos = ekf_source.pose_at(state.t_sim_s)
                    agent_position_m = (
                        agent_pos.pose_m_b.translation if agent_pos is not None else state.pose_odom
                    )
                    agent_yaw_rad = (
                        _yaw_from_orientation(agent_pos.pose_m_b.rotation)
                        if agent_pos is not None
                        else 0.0
                    )
                    pose = AgentPose(position_m=agent_position_m, yaw_rad=agent_yaw_rad)

                    at_final_waypoint = waypoint_index >= len(current_path)
                    reached_current_waypoint = (
                        not at_final_waypoint
                        and (agent_position_m - current_path[waypoint_index]).norm()
                        <= arrival_radius_m
                    )
                    if reached_current_waypoint:
                        waypoint_index += 1
                        at_final_waypoint = waypoint_index >= len(current_path)

                    if rotate_until_s is not None:
                        # Mid-rotation: hold the scan for its own fixed
                        # duration, not the path-following triggers below
                        # -- current_path is empty while rotating, which
                        # would otherwise make at_final_waypoint trivially
                        # True on every tick and re-trigger a replan
                        # attempt every 0.1s instead of letting the scan run.
                        should_replan = elapsed >= rotate_until_s
                    else:
                        should_replan = (
                            at_final_waypoint
                            or len(delta.changed) >= replan_map_change_cells
                            or (elapsed - t_last_replan_s) >= max_replan_interval_s
                        )

                    if should_replan:
                        t_last_replan_s = elapsed
                        rotate_until_s = None
                        subgoal = strategy.select_subgoal(grid, pose, elapsed)
                        if subgoal is None:
                            current_path = []
                            waypoint_index = 0
                            rotate_until_s = elapsed + _ROTATE_DURATION_S
                            n_rotate_scans += 1
                        else:
                            start_cell = (
                                math.floor(agent_position_m.x / grid.resolution_m),
                                math.floor(agent_position_m.y / grid.resolution_m),
                            )
                            plan = astar(
                                grid,
                                start_cell,
                                subgoal.cell,
                                unknown_cost_multiplier=unknown_cost_multiplier,
                            )
                            if plan.reason == "ok" and plan.path_cells:
                                current_path = _path_to_waypoints(
                                    grid, plan.path_cells, z_m=hover_altitude_m
                                )
                                waypoint_index = 1 if len(current_path) > 1 else 0
                                n_subgoals_chosen += 1
                            else:
                                current_path = []
                                waypoint_index = 0
                                rotate_until_s = elapsed + _ROTATE_DURATION_S
                                n_rotate_scans += 1

                    if rotate_until_s is not None:
                        v_desired = Vec3(0.0, 0.0, 0.0)
                        yaw_rate = _ROTATE_YAW_RATE_RADPS
                    else:
                        target = current_path[min(waypoint_index, len(current_path) - 1)]
                        v_desired = follower.compute_velocity(agent_position_m, target)
                        yaw_rate = 0.0

                    v_shielded, _intervened = shield.project(
                        v_desired,
                        orientation_odom_body=state.orientation_odom,
                        points_body=points_body,
                    )
                    # Explicit altitude hold on every command, rotate
                    # scans included -- applied after the shield, which
                    # only ever constrains the horizontal component, so
                    # the shield's guarantee is untouched and its
                    # intervention metric still measures horizontal
                    # throttling only. Uses PX4's own (EKF) altitude,
                    # never GT (spec §17.4).
                    v_cmd = altitude_hold.apply(v_shielded, state.pose_odom.z)
                    sp = VelocitySetpoint(velocity=v_cmd, yaw_rate_radps=yaw_rate, frame="odom")
                    await supervisor.submit_setpoint(sp)
                    supervisor.autonomy_heartbeat()
                    await supervisor.tick()

                    await asyncio.sleep(_TICK_PERIOD_S)
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
    finally:
        gc.collect()
        gc.enable()

    return ExplorationEpisodeResult(
        final_grid=grid,
        gt_trajectory=tuple(gt_trajectory),
        n_subgoals_chosen=n_subgoals_chosen,
        n_rotate_scans=n_rotate_scans,
        shield_intervention_rate=shield.intervention_rate,
        wall_time_s=time.monotonic() - t_start,
        reason=reason,
        altitude_validity=check_altitude_band(
            gt_trajectory,
            z_lo_m=z_lo_m,
            z_hi_m=z_hi_m,
            max_excursion_s=max_altitude_excursion_s,
        ),
    )

"""Runs one live "hover-and-yaw sweep in a known world" episode (spec §51
Phase 11's own integration-test line) and builds three parallel
:class:`~aeris.mapping.voxel.VoxelMap`\\ s from the *same* depth stream --
one per pose-source condition (GT / PX4 EKF / noisy-EKF) -- so the
map-accuracy experiment's "GT pose vs EKF pose vs noisy pose" comparison
(spec's own implementation task) needs only one real flight per world, not
three.

Lives beside :mod:`aeris.evaluation.avoidance_episode` (Phase 10's
identical "one live episode, one result" pattern) for the same reason:
this module owns simulation/IPC concerns (Sensor Bridge, GT polling,
control-loop timing) that :mod:`aeris.mapping`/:mod:`aeris.localization`
deliberately don't.

The GT pose is read only to (a) build the ``map_gt`` comparison condition
and (b) record a ground-truth trajectory for ATE/RPE -- never fed to the
``map_ekf``/``map_noisy`` conditions, which only ever see PX4's own
estimate, matching spec §17.4's provenance rule. Unlike
:mod:`aeris.mapping`/:mod:`aeris.localization` themselves, this module is
allowed to import :mod:`aeris.simulation.groundtruth` (spec §14.3 contract
1 only forbids it for the reusable packages, not evaluator-side glue).
"""

from __future__ import annotations

import asyncio
import gc
import random
import time
from array import array
from dataclasses import dataclass, field

import numpy as np

from aeris.core.clock import Clock
from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import ZERO, Vec3
from aeris.core.types import Provenance
from aeris.localization.noisy import NoisyPoseSource
from aeris.localization.pose_source import PoseEstimate
from aeris.localization.px4_ekf import Px4EkfPose
from aeris.mapping.raycast import warm_up
from aeris.mapping.voxel import MappingConfig, VoxelMap
from aeris.perception.depth.projection import CameraIntrinsics, depth_to_points_camera
from aeris.safety.supervisor import SafetySupervisor
from aeris.simulation.bridge.client import GzBridgeClient
from aeris.simulation.groundtruth.service import GroundTruthService
from aeris.vehicle.interface import VehicleEndpoint, VelocitySetpoint

_TICK_PERIOD_S = 0.1


@dataclass(frozen=True, slots=True)
class MappingEpisodeResult:
    map_gt: VoxelMap
    map_ekf: VoxelMap
    map_noisy: VoxelMap
    positions_odom: tuple[Vec3, ...]  # PX4 EKF's own reported trajectory
    positions_noisy: tuple[Vec3, ...]  # NoisyPoseSource's trajectory (odom/M frame)
    positions_gt_world: tuple[Vec3, ...]  # GT world-frame trajectory
    frame_latencies_s: tuple[float, ...]  # per-frame map_ekf integration time (the gated metric)
    n_frames: int
    reason: str = ""


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
    """Polls the Sensor Bridge for the latest depth frame and back-projects
    it into body-frame points (identical role to
    :class:`aeris.evaluation.avoidance_episode._DepthSource`; kept as its
    own small copy here rather than importing that module's private class
    across an unrelated experiment boundary).

    ``t_body_optical.apply(p)`` per point, called once per pixel in a
    plain Python loop (~3,500 points/frame at ``depth_stride=8``),
    measured live at 13-26ms/frame -- comfortably blowing this phase's
    50ms map-update-latency budget on its own, before any actual mapping
    work. Precomputing the fixed transform's rotation as a matrix once
    and applying it to every point in one vectorized NumPy multiply
    (~2-4ms/frame, measured) removes that cost; Phase 10's identical
    per-point pattern in ``avoidance_episode.py`` has the same headroom
    but wasn't hit by as tight a gate there, and is out of this phase's
    scope to touch.
    """

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
        """Returns ``(points, t_sim_s)`` -- the frame's *own* capture
        timestamp, not the caller's current tick time. Using the tick's
        own timestamp to query a pose source for this frame's points was
        a real, live-diagnosed bug (see this module's docstring / Phase 11
        report): while the ZMQ CONFLATE queue always hands back its
        latest frame, "latest" can trail the current tick by tens to
        hundreds of milliseconds (queueing, backprojection, other
        per-tick I/O ahead of this call) -- during the sweep's continuous
        yaw rotation, even ~100ms of mismatch rotates the effective
        camera bearing by several degrees, enough to place a detection a
        pillar-radius or more away from the pillar's true position at
        typical experiment ranges. Callers must resolve pose *at this
        timestamp*, not at "now"."""
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


def _gt_pose(
    gt_service: GroundTruthService, model_name: str, spawn_world: Vec3
) -> tuple[Vec3, Vec3, Quaternion]:
    """One GT poll, returning ``(pos_world, pos_odom, orientation)``.

    ``pos_odom = pos_world - spawn_world`` (V1: ``T_M_O = I``) -- the same
    conversion Phase 10 established live (``pose_odom`` is arm-relative,
    not world-origin-relative), with no rotational offset between ``O``
    and ``W`` (also confirmed live).
    """
    pos_world, quat = gt_service.get_pose(model_name)
    return pos_world, pos_world - spawn_world, quat


async def run_mapping_episode(
    *,
    supervisor: SafetySupervisor,
    endpoint: VehicleEndpoint,
    bridge_endpoint: str,
    camera_intrinsics: CameraIntrinsics,
    t_body_optical: Transform,
    gt_service: GroundTruthService,
    gt_model_name: str,
    spawn_world: Vec3,
    hover_altitude_m: float,
    sweep_duration_s: float,
    yaw_rate_radps: float,
    mapping_config: MappingConfig | None = None,
    noise_seed: int = 20261001,
    noisy_walk_step_m: float = 0.01,
    noisy_noise_std_m: float = 0.05,
    clock: Clock,
) -> MappingEpisodeResult:
    # One-time Numba JIT compile/cache-load (~0.5s cold) -- done here,
    # before the timed control loop even connects to the vehicle, per
    # raycast.warm_up()'s own docstring.
    warm_up()

    config = mapping_config or MappingConfig()
    map_gt = VoxelMap(config=config)
    map_ekf = VoxelMap(config=config)
    map_noisy = VoxelMap(config=config)

    # max_gap_s widened from the buffer's own 100ms default (spec §18.3's
    # figure, sized for a ~10-30Hz pipeline): this loop's real per-tick
    # period -- a synchronous GT-pose subprocess poll plus a real map
    # integration -- measured live at up to ~700ms, so the tight default
    # made pose_at() spuriously return None for a depth frame's timestamp
    # just one tick old (see Px4EkfPose's own docstring).
    ekf_source = Px4EkfPose(max_gap_s=2.0)
    noisy_source = NoisyPoseSource(
        base=ekf_source,
        rng=random.Random(noise_seed),
        walk_step_m=noisy_walk_step_m,
        noise_std_m=noisy_noise_std_m,
    )

    positions_odom: list[Vec3] = []
    positions_noisy: list[Vec3] = []
    positions_gt_world: list[Vec3] = []
    frame_latencies: list[float] = []
    reason = ""

    # gc disabled for the whole sweep, re-enabled unconditionally below --
    # confirmed live (Phase 11) that periodic ~40ms GC pauses, not the
    # mapping code itself, were what pushed this phase's first several
    # live latency-gate measurements over the 50ms budget. A `gc.collect()`
    # right after each timed integration call (still outside the timed
    # window) keeps memory bounded across a long sweep without ever
    # letting a pause land inside the measurement.
    gc.disable()
    try:
        with _DepthSource(bridge_endpoint, camera_intrinsics, t_body_optical) as depth_source:
            try:
                await supervisor.connect(endpoint)
                if not await supervisor.preflight_check():
                    return MappingEpisodeResult(
                        map_gt, map_ekf, map_noisy, (), (), (), (), 0, "preflight_check failed"
                    )

                arm_result = await supervisor.arm_and_takeoff(hover_altitude_m)
                if not arm_result.ok:
                    return MappingEpisodeResult(
                        map_gt, map_ekf, map_noisy, (), (), (), (), 0, "arm_and_takeoff rejected"
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

                elapsed = 0.0
                deadline = clock.now() + sweep_duration_s
                while clock.now() < deadline:
                    state = await supervisor.get_vehicle_state()
                    ekf_source.update(state)

                    # Depth read *before* the GT poll, and every pose
                    # queried at the depth frame's *own* t_sim_s -- not
                    # this tick's state.t_sim_s. Live-diagnosed bug (Phase
                    # 11 report): querying "now" for a frame that (thanks
                    # to ZMQ CONFLATE queueing, backprojection, and this
                    # same loop's own per-tick work) was actually captured
                    # a tick or more ago rotated the effective camera
                    # bearing by several degrees during the sweep's
                    # continuous yaw -- enough to place every single
                    # detection a pillar-radius or more from the pillar's
                    # true position (0% precision, every world, every
                    # pose-source condition, before this fix). Recorded
                    # trajectory positions are likewise taken at this same
                    # depth-frame instant (not every tick), so
                    # positions_odom/positions_gt_world/positions_noisy
                    # stay index-paired for ATE/RPE.
                    points_body, depth_t_sim_s = depth_source.latest_points_body()
                    if points_body and depth_t_sim_s is not None:
                        gt_pos_world, gt_pos_m, gt_quat = _gt_pose(
                            gt_service, gt_model_name, spawn_world
                        )

                        gt_estimate = PoseEstimate(
                            pose_m_b=Transform(gt_quat, gt_pos_m),
                            covariance_diag=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                            provenance=Provenance.ORACLE,
                            source_name="gt",
                        )
                        ekf_estimate = ekf_source.pose_at(depth_t_sim_s)
                        noisy_estimate = noisy_source.pose_at(depth_t_sim_s)
                        if ekf_estimate is None:
                            # Can't align this frame to an EKF pose (gap
                            # too large) -- skip it entirely rather than
                            # recording a GT position with no paired
                            # estimate.
                            await asyncio.sleep(_TICK_PERIOD_S)
                            elapsed += _TICK_PERIOD_S
                            continue
                        positions_gt_world.append(gt_pos_world)
                        positions_odom.append(ekf_estimate.pose_m_b.translation)

                        origin_gt = gt_estimate.pose_m_b.translation
                        map_gt.integrate_points(
                            origin_gt, [gt_estimate.pose_m_b.apply(p) for p in points_body]
                        )

                        t0 = time.perf_counter()
                        origin_ekf = ekf_estimate.pose_m_b.translation
                        map_ekf.integrate_points(
                            origin_ekf, [ekf_estimate.pose_m_b.apply(p) for p in points_body]
                        )
                        frame_latencies.append(time.perf_counter() - t0)
                        gc.collect()

                        if noisy_estimate is not None:
                            origin_noisy = noisy_estimate.pose_m_b.translation
                            positions_noisy.append(origin_noisy)
                            map_noisy.integrate_points(
                                origin_noisy,
                                [noisy_estimate.pose_m_b.apply(p) for p in points_body],
                            )

                    sp = VelocitySetpoint(
                        velocity=ZERO, yaw_rate_radps=yaw_rate_radps, frame="body"
                    )
                    await supervisor.submit_setpoint(sp)
                    supervisor.autonomy_heartbeat()
                    await supervisor.tick()

                    await asyncio.sleep(_TICK_PERIOD_S)
                    elapsed += _TICK_PERIOD_S
                if not reason and elapsed < sweep_duration_s:
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

    return MappingEpisodeResult(
        map_gt=map_gt,
        map_ekf=map_ekf,
        map_noisy=map_noisy,
        positions_odom=tuple(positions_odom),
        positions_noisy=tuple(positions_noisy),
        positions_gt_world=tuple(positions_gt_world),
        frame_latencies_s=tuple(frame_latencies),
        n_frames=len(frame_latencies),
        reason=reason,
    )

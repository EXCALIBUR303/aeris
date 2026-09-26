"""``run_episode`` (spec §38.4): wraps one Tier-H mission execution into a
complete, replay-recorded, metriced episode.

Runs a background telemetry-sampling task concurrently with
``MissionExecutive.run()`` (rather than modifying the executive itself)
so ``/vehicle/state``/``/gt/pose`` samples land in the MCAP file at a
steady rate independent of the mission's own waypoint-tracking loop
timing, and so metrics are computed from *exactly* the same samples that
get recorded -- which is what makes "re-reading the MCAP reproduces the
metrics exactly" (spec's own Phase 7 validation-gate item 2) true by
construction, not by coincidence.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from pathlib import Path

from aeris.autonomy.mission.executive import MissionExecutive, MissionResult
from aeris.autonomy.mission.spec import MissionSpec
from aeris.core.clock import Clock
from aeris.core.frames.vector import Vec3
from aeris.core.logging import get_logger
from aeris.core.types import RunStatus
from aeris.evaluation.metrics.collision import (
    GroundTruthGeometry,
    default_world_geometry,
    has_collision,
    minimum_clearance_m,
)
from aeris.evaluation.metrics.nav import control_smoothness, distance_travelled_m
from aeris.replay import channels
from aeris.replay.recorder import ReplayRecorder
from aeris.safety.supervisor import SafetySupervisor
from aeris.simulation.groundtruth import GroundTruthService
from aeris.vehicle.interface import VehicleEndpoint

_logger = get_logger(component="evaluation.episode")

_SAMPLE_PERIOD_S = 0.2


@dataclass(frozen=True, slots=True)
class EpisodeMetrics:
    distance_travelled_m: float
    flight_time_s: float
    control_smoothness: float
    min_clearance_m: float
    collided: bool
    max_waypoint_error_m: float
    return_to_base_success: bool


@dataclass(frozen=True, slots=True)
class EpisodeResult:
    status: RunStatus
    reason: str
    mission_result: MissionResult | None
    metrics: EpisodeMetrics | None
    replay_path: Path
    sample_count: int


async def run_episode(
    supervisor: SafetySupervisor,
    spec: MissionSpec,
    endpoint: VehicleEndpoint,
    *,
    clock: Clock,
    replay_path: Path,
    vehicle_model_name: str = "x500_0",
    vehicle_radius_m: float = 0.5,
    geometry: GroundTruthGeometry | None = None,
    ground_truth: GroundTruthService | None = None,
) -> EpisodeResult:
    """Run one mission attempt, recording it and computing its metrics.

    Never raises for an ordinary episode failure (rejected command,
    timeout) -- reported in the returned :class:`EpisodeResult`. Spec
    §38.4's "record as invalid, re-run once" retry policy is the caller's
    job (:mod:`aeris.experiments.runner`), since retrying needs a fresh
    ``SimulationLauncher``/connection, which this function doesn't own.
    """
    geometry = geometry or default_world_geometry()
    ground_truth = ground_truth or GroundTruthService()
    executive = MissionExecutive(supervisor, spec, clock=clock)

    positions: list[Vec3] = []
    velocities: list[Vec3] = []

    recorder = ReplayRecorder(replay_path)
    stop_sampling = asyncio.Event()

    async def sample_loop() -> None:
        while not stop_sampling.is_set():
            try:
                state = await supervisor.get_vehicle_state()
            except Exception as exc:  # vehicle briefly unreachable -- keep sampling
                _logger.warning("episode.sample_failed", error=str(exc))
                await asyncio.sleep(_SAMPLE_PERIOD_S)
                continue

            positions.append(state.pose_odom)
            velocities.append(state.velocity_odom_mps)
            recorder.write(
                channels.VEHICLE_STATE,
                state.t_sim_s,
                {
                    "pos": [state.pose_odom.x, state.pose_odom.y, state.pose_odom.z],
                    "vel": [
                        state.velocity_odom_mps.x,
                        state.velocity_odom_mps.y,
                        state.velocity_odom_mps.z,
                    ],
                    "armed": state.armed,
                    "flight_mode": state.flight_mode.value,
                    "landed_state": state.landed_state.value,
                },
            )
            try:
                gt_pos, _gt_orient = ground_truth.get_pose(vehicle_model_name)
                recorder.write(
                    channels.GT_POSE, state.t_sim_s, {"pos": [gt_pos.x, gt_pos.y, gt_pos.z]}
                )
            except Exception:  # GT read is best-effort -- never fail the episode over it
                pass
            await asyncio.sleep(_SAMPLE_PERIOD_S)

    sampler_task = asyncio.create_task(sample_loop())
    try:
        recorder.write(
            channels.MISSION_EVENTS, clock.now(), {"event": "mission_started", "name": spec.name}
        )
        mission_result = await executive.run(endpoint)
        recorder.write(
            channels.MISSION_EVENTS,
            clock.now(),
            {
                "event": "mission_finished",
                "state": mission_result.state.value,
                "reason": mission_result.reason,
            },
        )
        for event in supervisor.events():
            recorder.write(
                channels.SAFETY_EVENTS,
                event.t_sim_s,
                {"kind": event.kind.value, "message": event.message},
            )
    finally:
        stop_sampling.set()
        await sampler_task
        recorder.close()

    if not mission_result.ok:
        return EpisodeResult(
            RunStatus.FAILED,
            mission_result.reason,
            mission_result,
            None,
            replay_path,
            len(positions),
        )

    metrics = EpisodeMetrics(
        distance_travelled_m=distance_travelled_m(positions),
        flight_time_s=mission_result.wall_time_s,
        control_smoothness=control_smoothness(velocities, _SAMPLE_PERIOD_S),
        min_clearance_m=minimum_clearance_m(positions, geometry) if positions else float("inf"),
        collided=has_collision(positions, geometry, vehicle_radius_m) if positions else False,
        max_waypoint_error_m=mission_result.max_waypoint_error_m,
        return_to_base_success=(
            mission_result.return_outcome.arrived if mission_result.return_outcome else False
        ),
    )
    return EpisodeResult(
        RunStatus.COMPLETED, "ok", mission_result, metrics, replay_path, len(positions)
    )

from __future__ import annotations

from pathlib import Path

import pytest
from tests.fixtures.vehicle_fakes import FakeVehicle, make_state

from aeris.autonomy.mission.spec import MissionSpec
from aeris.core.clock import WallClock
from aeris.core.frames.vector import Vec3
from aeris.core.types import RunStatus
from aeris.evaluation.episode import run_episode
from aeris.evaluation.metrics.nav import distance_travelled_m
from aeris.replay import channels
from aeris.replay.reader import ReplayReader
from aeris.safety.envelope import Envelope
from aeris.safety.supervisor import SafetySupervisor
from aeris.vehicle.interface import VehicleEndpoint


class _InstantArrivalVehicle(FakeVehicle):
    """Snaps position to whatever was last commanded -- makes the mission
    converge in a handful of ticks instead of needing real flight
    dynamics, matching the pattern already used for
    ``tests/unit/autonomy/test_executive.py``."""

    async def set_position_target(self, sp: object) -> None:  # type: ignore[override]
        await super().set_position_target(sp)  # type: ignore[arg-type]
        target = sp.position_odom  # type: ignore[attr-defined]
        self.state = make_state(pose_odom=target, home_odom=Vec3(0, 0, 0))


class _StubGroundTruth:
    """No live gz in a unit test -- ``run_episode`` treats a GT read
    failure as best-effort and continues without it."""

    def get_pose(self, model_name: str) -> tuple[Vec3, object]:
        raise RuntimeError("no live gz in this unit test")


@pytest.fixture
def envelope() -> Envelope:
    return Envelope.model_validate(
        {
            "v_xy_max_mps": 3.0,
            "vz_max_mps": 1.5,
            "yaw_rate_max_radps": 1.0,
            "accel_max_mps2": 2.0,
            "altitude_floor_m": 0.3,
            "altitude_ceiling_m": 20.0,
            "geofence_radius_m": 30.0,
        }
    )


@pytest.fixture
def spec() -> MissionSpec:
    return MissionSpec.model_validate(
        {
            "name": "test_episode",
            "takeoff_altitude_m": 3.0,
            "time_budget_s": 60.0,
            "waypoint_timeout_s": 30.0,
            "waypoints": [
                {"x": 5.0, "y": 0.0, "z": 3.0, "acceptance_radius_m": 0.5, "dwell_s": 0.0},
                {"x": 5.0, "y": 5.0, "z": 3.0, "acceptance_radius_m": 0.5, "dwell_s": 0.0},
            ],
        }
    )


async def test_episode_completes_and_produces_metrics(
    tmp_path: Path, spec: MissionSpec, envelope: Envelope
) -> None:
    vehicle = _InstantArrivalVehicle(make_state(pose_odom=Vec3(0, 0, 3.0), home_odom=Vec3(0, 0, 0)))
    supervisor = SafetySupervisor(vehicle, envelope=envelope, clock=WallClock())
    replay_path = tmp_path / "episode.mcap"

    result = await run_episode(
        supervisor,
        spec,
        VehicleEndpoint(host="127.0.0.1", port=14540),
        clock=WallClock(),
        replay_path=replay_path,
        ground_truth=_StubGroundTruth(),  # type: ignore[arg-type]
    )

    assert result.status == RunStatus.COMPLETED, result.reason
    assert result.metrics is not None
    assert result.metrics.max_waypoint_error_m <= 0.5
    assert result.metrics.return_to_base_success
    assert replay_path.is_file()
    assert result.sample_count > 0


async def test_episode_failure_is_reported_not_raised(
    tmp_path: Path, spec: MissionSpec, envelope: Envelope
) -> None:
    vehicle = FakeVehicle(make_state(pose_odom=Vec3(0, 0, 0), ekf_flags_raw=0))  # unhealthy EKF
    supervisor = SafetySupervisor(vehicle, envelope=envelope, clock=WallClock())
    replay_path = tmp_path / "episode.mcap"

    result = await run_episode(
        supervisor,
        spec,
        VehicleEndpoint(host="127.0.0.1", port=14540),
        clock=WallClock(),
        replay_path=replay_path,
        ground_truth=_StubGroundTruth(),  # type: ignore[arg-type]
    )

    assert result.status == RunStatus.FAILED
    assert result.metrics is None
    assert "preflight" in result.reason


async def test_rereading_the_mcap_reproduces_distance_travelled_exactly(
    tmp_path: Path, spec: MissionSpec, envelope: Envelope
) -> None:
    """spec §51 Phase 7 validation-gate item 2: "Re-reading the MCAP
    reproduces the metrics exactly." True by construction (metrics are
    computed from the same samples that get recorded) -- this test proves
    it, not just asserts it."""
    vehicle = _InstantArrivalVehicle(make_state(pose_odom=Vec3(0, 0, 3.0), home_odom=Vec3(0, 0, 0)))
    supervisor = SafetySupervisor(vehicle, envelope=envelope, clock=WallClock())
    replay_path = tmp_path / "episode.mcap"

    result = await run_episode(
        supervisor,
        spec,
        VehicleEndpoint(host="127.0.0.1", port=14540),
        clock=WallClock(),
        replay_path=replay_path,
        ground_truth=_StubGroundTruth(),  # type: ignore[arg-type]
    )
    assert result.status == RunStatus.COMPLETED
    assert result.metrics is not None

    with ReplayReader(replay_path) as reader:
        vehicle_state_samples = reader.read_channel(channels.VEHICLE_STATE)
    recomputed_positions = [Vec3(*sample["pos"]) for sample in vehicle_state_samples]
    recomputed_distance = distance_travelled_m(recomputed_positions)

    assert recomputed_distance == pytest.approx(result.metrics.distance_travelled_m)
    assert len(vehicle_state_samples) == result.sample_count


async def test_mission_events_channel_is_recorded(
    tmp_path: Path, spec: MissionSpec, envelope: Envelope
) -> None:
    vehicle = _InstantArrivalVehicle(make_state(pose_odom=Vec3(0, 0, 3.0), home_odom=Vec3(0, 0, 0)))
    supervisor = SafetySupervisor(vehicle, envelope=envelope, clock=WallClock())
    replay_path = tmp_path / "episode.mcap"

    await run_episode(
        supervisor,
        spec,
        VehicleEndpoint(host="127.0.0.1", port=14540),
        clock=WallClock(),
        replay_path=replay_path,
        ground_truth=_StubGroundTruth(),  # type: ignore[arg-type]
    )

    with ReplayReader(replay_path) as reader:
        events = reader.read_channel(channels.MISSION_EVENTS)
    event_names = [e["event"] for e in events]
    assert event_names == ["mission_started", "mission_finished"]

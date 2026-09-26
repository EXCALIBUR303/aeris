from __future__ import annotations

import pytest
from tests.fixtures.vehicle_fakes import FakeVehicle, make_state

from aeris.autonomy.mission.executive import MissionExecutive
from aeris.autonomy.mission.spec import MissionSpec
from aeris.autonomy.mission.states import MissionState, is_terminal
from aeris.core.clock import ManualClock
from aeris.core.frames.vector import Vec3
from aeris.safety.envelope import Envelope
from aeris.safety.supervisor import SafetySupervisor
from aeris.vehicle.interface import VehicleEndpoint


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
            "name": "test_square",
            "takeoff_altitude_m": 3.0,
            "time_budget_s": 60.0,
            "waypoint_timeout_s": 30.0,
            "waypoints": [
                {"x": 5.0, "y": 0.0, "z": 3.0, "acceptance_radius_m": 0.5, "dwell_s": 0.0},
                {"x": 5.0, "y": 5.0, "z": 3.0, "acceptance_radius_m": 0.5, "dwell_s": 0.0},
            ],
        }
    )


@pytest.fixture
def vehicle() -> FakeVehicle:
    return FakeVehicle(make_state(pose_odom=Vec3(0, 0, 3.0)))


@pytest.fixture
def supervisor(vehicle: FakeVehicle, envelope: Envelope) -> SafetySupervisor:
    return SafetySupervisor(vehicle, envelope=envelope, clock=ManualClock())


@pytest.fixture
def executive(supervisor: SafetySupervisor, spec: MissionSpec) -> MissionExecutive:
    return MissionExecutive(supervisor, spec, clock=ManualClock())


# --- abort in every state (spec §51 Phase 6's literal test requirement) --------------


@pytest.mark.parametrize("state", list(MissionState))
def test_abort_from_every_state(executive: MissionExecutive, state: MissionState) -> None:
    executive._state = state  # reach into internals: pure state-machine test
    executive.abort()
    if is_terminal(state):
        assert executive.state == state  # already terminal -- no-op
    else:
        assert executive.state == MissionState.ABORTED
        assert executive.aborted


def test_abort_is_idempotent(executive: MissionExecutive) -> None:
    executive._state = MissionState.EXECUTING
    executive.abort()
    executive.abort()  # calling it again on an already-ABORTED executive must not raise
    assert executive.state == MissionState.ABORTED


# --- full happy-path run() against a fake vehicle -------------------------------------


class _InstantArrivalVehicle(FakeVehicle):
    """A FakeVehicle whose position snaps to whatever was last commanded --
    makes ``MissionExecutive.run()`` converge in a handful of ticks instead
    of needing real flight dynamics, for a fast, deterministic unit test."""

    async def set_position_target(self, sp: object) -> None:  # type: ignore[override]
        await super().set_position_target(sp)  # type: ignore[arg-type]
        target = sp.position_odom  # type: ignore[attr-defined]
        self.state = make_state(pose_odom=target, home_odom=Vec3(0, 0, 0))


async def test_full_mission_completes(spec: MissionSpec, envelope: Envelope) -> None:
    vehicle = _InstantArrivalVehicle(make_state(pose_odom=Vec3(0, 0, 3.0), home_odom=Vec3(0, 0, 0)))
    supervisor = SafetySupervisor(vehicle, envelope=envelope, clock=ManualClock())
    executive = MissionExecutive(supervisor, spec, clock=ManualClock())

    result = await executive.run(VehicleEndpoint(host="127.0.0.1", port=14540))

    assert result.ok, result.reason
    assert executive.state == MissionState.COMPLETE
    assert len(result.waypoints) == len(spec.waypoints)
    assert all(w.arrived for w in result.waypoints)
    assert all(
        w.error_m <= w0.acceptance_radius_m
        for w, w0 in zip(result.waypoints, spec.waypoints, strict=True)
    )
    assert result.return_outcome is not None
    assert result.return_outcome.arrived


async def test_mission_fails_cleanly_when_preflight_check_fails(
    spec: MissionSpec, envelope: Envelope
) -> None:
    vehicle = FakeVehicle(make_state(pose_odom=Vec3(0, 0, 0), ekf_flags_raw=0))
    supervisor = SafetySupervisor(vehicle, envelope=envelope, clock=ManualClock())
    executive = MissionExecutive(supervisor, spec, clock=ManualClock())

    result = await executive.run(VehicleEndpoint(host="127.0.0.1", port=14540))

    assert not result.ok
    assert executive.state == MissionState.ABORTED
    assert "preflight" in result.reason

from __future__ import annotations

import pytest
from tests.fixtures.vehicle_fakes import FakeVehicle, make_state

from aeris.core.clock import ManualClock
from aeris.core.errors import UnsafeStateError
from aeris.core.frames.vector import Vec3
from aeris.safety.envelope import Envelope
from aeris.safety.events import SafetyEventKind
from aeris.safety.supervisor import _TRANSITIONS, SafetySupervisor, SupervisorState
from aeris.vehicle.interface import (
    CommandResult,
    CommandResultCode,
    VehicleEndpoint,
    VelocitySetpoint,
)


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
def clock() -> ManualClock:
    return ManualClock()


@pytest.fixture
def vehicle() -> FakeVehicle:
    return FakeVehicle(make_state(pose_odom=Vec3(0, 0, 5.0)))


@pytest.fixture
def supervisor(vehicle: FakeVehicle, envelope: Envelope, clock: ManualClock) -> SafetySupervisor:
    return SafetySupervisor(
        vehicle,
        envelope=envelope,
        clock=clock,
        autonomy_tick_timeout_s=2.0,
        telemetry_stale_timeout_s=3.0,
        emergency_land_altitude_m=1.0,
    )


async def _fly_to_autonomy_active(supervisor: SafetySupervisor) -> None:
    await supervisor.connect(VehicleEndpoint(host="127.0.0.1", port=14540))
    assert await supervisor.preflight_check()
    result = await supervisor.arm_and_takeoff(5.0)
    assert result.ok
    await supervisor.confirm_airborne()
    await supervisor.start_autonomy()


# --- table-driven state machine (exhaustive) ---------------------------------------


ALL_STATES = [
    SupervisorState.DISCONNECTED,
    SupervisorState.CONNECTED,
    SupervisorState.PREFLIGHT_OK,
    SupervisorState.ARMING,
    SupervisorState.TAKING_OFF,
    SupervisorState.AIRBORNE_MANUAL_HOLD,
    SupervisorState.AUTONOMY_ACTIVE,
    SupervisorState.INTERVENTION,
    SupervisorState.LANDING,
    SupervisorState.LANDED,
    SupervisorState.DISARMED,
    SupervisorState.FAILSAFE,
]


@pytest.mark.parametrize("from_state", ALL_STATES)
@pytest.mark.parametrize("to_state", ALL_STATES)
def test_transition_table_matches_allowed_set_exactly(
    supervisor: SafetySupervisor, from_state: str, to_state: str
) -> None:
    supervisor._state = from_state  # reach into internals: pure state-machine test
    allowed = to_state in _TRANSITIONS[from_state]
    if allowed:
        supervisor._transition(to_state)
        assert supervisor.state == to_state
    else:
        with pytest.raises(UnsafeStateError):
            supervisor._transition(to_state)
        assert supervisor.state == from_state  # rejected transition doesn't mutate state


def test_any_state_can_reach_failsafe(supervisor: SafetySupervisor) -> None:
    for state in ALL_STATES:
        supervisor._state = state
        supervisor._transition(SupervisorState.FAILSAFE)
        assert supervisor.state == SupervisorState.FAILSAFE


# --- happy path --------------------------------------------------------------------


async def test_full_happy_path_sequence(supervisor: SafetySupervisor, vehicle: FakeVehicle) -> None:
    await _fly_to_autonomy_active(supervisor)
    assert supervisor.state == SupervisorState.AUTONOMY_ACTIVE

    result = await supervisor.submit_setpoint(VelocitySetpoint(velocity=Vec3(1.0, 0.0, 0.0)))
    assert result.ok

    stop_result = await supervisor.stop_autonomy()
    assert stop_result.ok
    assert supervisor.state == SupervisorState.AIRBORNE_MANUAL_HOLD

    land_result = await supervisor.land()
    assert land_result.ok
    assert supervisor.state == SupervisorState.LANDING

    supervisor.confirm_landed()
    assert supervisor.state == SupervisorState.LANDED

    disarm_result = await supervisor.disarm()
    assert disarm_result.ok
    assert supervisor.state == SupervisorState.DISARMED

    call_names = [name for name, _args in vehicle.calls]
    assert call_names == [
        "connect",
        "start_offboard",  # arm_and_takeoff: offboard climb, started before arming
        "arm",
        "hold",  # confirm_airborne: explicitly commands Hold
        "start_offboard",  # start_autonomy: re-engages OFFBOARD (see its docstring)
        "set_velocity_target",
        "hold",  # stop_autonomy: explicitly commands Hold again
        "land",
        "disarm",
    ]


async def test_operation_out_of_state_raises(supervisor: SafetySupervisor) -> None:
    with pytest.raises(UnsafeStateError):
        await supervisor.confirm_airborne()  # never armed/took off
    with pytest.raises(UnsafeStateError):
        await supervisor.start_autonomy()  # never airborne


# --- preflight / arm / takeoff rejection paths --------------------------------------


async def test_preflight_check_fails_when_ekf_unhealthy(
    supervisor: SafetySupervisor, vehicle: FakeVehicle
) -> None:
    vehicle.state = make_state(ekf_flags_raw=0)
    await supervisor.connect(VehicleEndpoint(host="127.0.0.1", port=14540))
    ok = await supervisor.preflight_check()
    assert not ok
    assert supervisor.state == SupervisorState.CONNECTED  # did not advance


async def test_offboard_climb_start_rejected_falls_back_to_preflight_ok(
    supervisor: SafetySupervisor, vehicle: FakeVehicle
) -> None:
    vehicle.results["start_offboard"] = CommandResult(CommandResultCode.DENIED)
    await supervisor.connect(VehicleEndpoint(host="127.0.0.1", port=14540))
    await supervisor.preflight_check()
    result = await supervisor.arm_and_takeoff(5.0)
    assert not result.ok
    assert supervisor.state == SupervisorState.PREFLIGHT_OK
    assert not any(name == "arm" for name, _args in vehicle.calls)


async def test_arm_rejected_after_offboard_climb_start_falls_back_and_stops_offboard(
    supervisor: SafetySupervisor, vehicle: FakeVehicle
) -> None:
    vehicle.results["arm"] = CommandResult(CommandResultCode.TEMPORARILY_REJECTED)
    await supervisor.connect(VehicleEndpoint(host="127.0.0.1", port=14540))
    await supervisor.preflight_check()
    result = await supervisor.arm_and_takeoff(5.0)
    assert not result.ok
    assert supervisor.state == SupervisorState.PREFLIGHT_OK
    assert any(name == "stop_offboard" for name, _args in vehicle.calls)


async def test_start_autonomy_reengages_offboard_and_fails_state_change_on_rejection(
    supervisor: SafetySupervisor, vehicle: FakeVehicle
) -> None:
    await supervisor.connect(VehicleEndpoint(host="127.0.0.1", port=14540))
    await supervisor.preflight_check()
    await supervisor.arm_and_takeoff(5.0)
    await supervisor.confirm_airborne()

    vehicle.results["start_offboard"] = CommandResult(CommandResultCode.DENIED)
    result = await supervisor.start_autonomy()
    assert not result.ok
    assert supervisor.state == SupervisorState.AIRBORNE_MANUAL_HOLD  # did not advance


# --- S2: setpoints only accepted in AUTONOMY_ACTIVE ----------------------------------


async def test_setpoint_rejected_outside_autonomy_active(
    supervisor: SafetySupervisor, vehicle: FakeVehicle
) -> None:
    await _fly_to_autonomy_active(supervisor)
    await supervisor.stop_autonomy()
    assert supervisor.state == SupervisorState.AIRBORNE_MANUAL_HOLD

    result = await supervisor.submit_setpoint(VelocitySetpoint(velocity=Vec3(1.0, 0.0, 0.0)))
    assert not result.ok
    assert not any(name == "set_velocity_target" for name, _args in vehicle.calls)


# --- S1: validator wired into submit_setpoint ----------------------------------------


async def test_setpoint_rejected_by_validator_is_not_forwarded(
    supervisor: SafetySupervisor, vehicle: FakeVehicle, envelope: Envelope
) -> None:
    await _fly_to_autonomy_active(supervisor)
    before = len(vehicle.calls)

    bad = VelocitySetpoint(velocity=Vec3(envelope.v_xy_max_mps + 10.0, 0.0, 0.0))
    result = await supervisor.submit_setpoint(bad)
    assert not result.ok
    assert len(vehicle.calls) == before  # nothing new sent to the vehicle

    rejected_events = [e for e in supervisor.events() if e.kind == SafetyEventKind.COMMAND_REJECTED]
    assert rejected_events


# --- S4: watchdogs / intervention / recovery ------------------------------------------


async def test_tick_intervenes_on_stalled_autonomy_tick(
    supervisor: SafetySupervisor, vehicle: FakeVehicle, clock: ManualClock
) -> None:
    await _fly_to_autonomy_active(supervisor)
    clock.advance(10.0)  # well past autonomy_tick_timeout_s=2.0, no heartbeat() called
    await supervisor.tick()
    assert supervisor.state == SupervisorState.INTERVENTION
    assert any(name == "hold" for name, _args in vehicle.calls)


async def test_tick_intervenes_on_stale_telemetry(
    supervisor: SafetySupervisor, vehicle: FakeVehicle, clock: ManualClock
) -> None:
    await _fly_to_autonomy_active(supervisor)
    supervisor.autonomy_heartbeat()
    vehicle.state = make_state(pose_odom=Vec3(0, 0, 5.0), heartbeat_age_s=10.0)
    await supervisor.tick()
    assert supervisor.state == SupervisorState.INTERVENTION


async def test_tick_intervenes_on_unhealthy_ekf(
    supervisor: SafetySupervisor, vehicle: FakeVehicle, clock: ManualClock
) -> None:
    await _fly_to_autonomy_active(supervisor)
    supervisor.autonomy_heartbeat()
    vehicle.state = make_state(pose_odom=Vec3(0, 0, 5.0), ekf_flags_raw=0)
    await supervisor.tick()
    assert supervisor.state == SupervisorState.INTERVENTION


async def test_tick_recovers_from_intervention_once_fault_clears(
    supervisor: SafetySupervisor, vehicle: FakeVehicle, clock: ManualClock
) -> None:
    await _fly_to_autonomy_active(supervisor)
    clock.advance(10.0)
    await supervisor.tick()
    assert supervisor.state == SupervisorState.INTERVENTION

    supervisor.autonomy_heartbeat()
    vehicle.state = make_state(pose_odom=Vec3(0, 0, 5.0), heartbeat_age_s=0.1)
    await supervisor.tick()
    assert supervisor.state == SupervisorState.AUTONOMY_ACTIVE


async def test_tick_is_a_noop_before_airborne(
    supervisor: SafetySupervisor, vehicle: FakeVehicle
) -> None:
    await supervisor.connect(VehicleEndpoint(host="127.0.0.1", port=14540))
    await supervisor.tick()  # DISCONNECTED->CONNECTED only; must not raise or touch vehicle
    assert supervisor.state == SupervisorState.CONNECTED


# --- emergency stop ------------------------------------------------------------------


async def test_emergency_stop_holds_when_above_land_threshold(
    supervisor: SafetySupervisor, vehicle: FakeVehicle
) -> None:
    await _fly_to_autonomy_active(supervisor)
    vehicle.state = make_state(pose_odom=Vec3(0, 0, 5.0))  # above emergency_land_altitude_m=1.0
    result = await supervisor.emergency_stop_simulation()
    assert result.ok
    assert supervisor.aborted
    assert supervisor.state == SupervisorState.FAILSAFE
    assert any(name == "hold" for name, _args in vehicle.calls)
    assert not any(name == "land" for name, _args in vehicle.calls)


async def test_emergency_stop_lands_when_below_land_threshold(
    supervisor: SafetySupervisor, vehicle: FakeVehicle
) -> None:
    await _fly_to_autonomy_active(supervisor)
    vehicle.state = make_state(pose_odom=Vec3(0, 0, 0.5))  # below emergency_land_altitude_m=1.0
    result = await supervisor.emergency_stop_simulation()
    assert result.ok
    assert supervisor.aborted
    assert supervisor.state == SupervisorState.LANDING
    assert any(name == "land" for name, _args in vehicle.calls)


# --- events ----------------------------------------------------------------------------


async def test_events_are_recorded(supervisor: SafetySupervisor) -> None:
    await _fly_to_autonomy_active(supervisor)
    kinds = {e.kind for e in supervisor.events()}
    assert SafetyEventKind.STATE_TRANSITION in kinds

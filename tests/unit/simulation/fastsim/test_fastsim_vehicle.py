"""FastSimVehicle implements VehicleInterface *semantics*: the real
SafetySupervisor (with the real S1 validator) can arm, offboard-take-off,
hand over to autonomy, fly a velocity command, land and disarm it."""

from __future__ import annotations

import asyncio

import numpy as np
import pytest

from aeris.core.clock import ManualClock
from aeris.core.frames.vector import Vec3
from aeris.safety.envelope import load_envelope
from aeris.safety.supervisor import SafetySupervisor, SupervisorState
from aeris.simulation.fastsim.batch import CameraRig, FastSimBatch, PoseNoiseParams
from aeris.simulation.fastsim.dynamics import DynamicsParams
from aeris.simulation.fastsim.vehicle import FastSimVehicle
from aeris.simulation.fastsim.world import WorldBank, build_world
from aeris.simulation.worlds.spec import Bounds, SpawnPose, WorldFamily, WorldSpec, WorldSplit
from aeris.vehicle.interface import FlightMode, LandedState, VehicleEndpoint, VelocitySetpoint


def _vehicle(clock: ManualClock) -> FastSimVehicle:
    spec = WorldSpec(
        name="fsv",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TRAIN,
        seed=0,
        bounds=Bounds(min_x=-20, min_y=-20, max_x=20, max_y=20, max_z=5),
        altitude_band_m=(0.3, 3.0),
        spawn_poses=(SpawnPose(x=0.0, y=0.0, z=0.1),),
    )
    sim = FastSimBatch(
        bank=WorldBank.from_worlds([build_world(spec, resolution_m=0.2)]),
        rig=CameraRig.load(stride=16),
        dyn_params=[DynamicsParams(tau_xy_s=0.44, delay_xy_s=0.28, accel_max_xy_mps2=4.0)],
        pose_noise=PoseNoiseParams(walk_step_m=0.0, noise_std_m=0.0, yaw_noise_std_rad=0.0),
    )
    sim.reset_envs(
        np.array([0]), world=np.array([0]), pos=np.array([[0.0, 0.0, 0.24]]), yaw=np.zeros(1)
    )
    return FastSimVehicle(sim, clock=clock)


async def _run(clock: ManualClock, sup: SafetySupervisor, seconds: float) -> None:
    for _ in range(round(seconds / 0.1)):
        clock.advance(0.1)
        sup.autonomy_heartbeat()
        await sup.tick()


def test_safety_supervisor_flies_a_full_cycle_on_fastsim() -> None:
    async def scenario() -> None:
        clock = ManualClock()
        vehicle = _vehicle(clock)
        sup = SafetySupervisor(
            vehicle, envelope=load_envelope("configs/vehicle/safety.yaml"), clock=clock
        )
        await sup.connect(VehicleEndpoint(host="127.0.0.1", port=14540))
        assert await sup.preflight_check()
        assert (await sup.arm_and_takeoff(2.0)).ok
        await _run(clock, sup, 4.0)
        s = await sup.get_vehicle_state()
        assert s.landed_state is LandedState.IN_AIR and s.pose_odom.z > 1.5

        await sup.confirm_airborne()
        await sup.start_autonomy()
        assert sup.state is SupervisorState.AUTONOMY_ACTIVE
        assert (await sup.submit_setpoint(VelocitySetpoint(velocity=Vec3(1.0, 0.0, 0.0)))).ok
        await _run(clock, sup, 5.0)
        s = await sup.get_vehicle_state()
        # ~5 s at the identified dynamics: settled to ~1 m/s, ~4.3 m travelled.
        assert s.velocity_odom_mps.x == pytest.approx(1.0, abs=0.02)
        assert s.pose_odom.x == pytest.approx(5.0 - 0.28 - 0.44, abs=0.3)

        await sup.stop_autonomy()
        await sup.land()
        await _run(clock, sup, 5.0)
        s = await sup.get_vehicle_state()
        assert s.landed_state is LandedState.ON_GROUND
        sup.confirm_landed()
        assert (await sup.disarm()).ok
        assert not (await sup.get_vehicle_state()).armed

    asyncio.run(scenario())


def test_velocity_targets_are_ignored_outside_offboard_mode() -> None:
    async def scenario() -> None:
        clock = ManualClock()
        v = _vehicle(clock)
        await v.start_offboard(VelocitySetpoint(velocity=Vec3(0.0, 0.0, 0.7)))
        await v.arm()
        clock.advance(4.0)
        await v.hold()
        await v.set_velocity_target(VelocitySetpoint(velocity=Vec3(1.0, 0.0, 0.0)))
        x0 = (await v.get_vehicle_state()).pose_odom.x
        clock.advance(3.0)
        s = await v.get_vehicle_state()
        assert s.flight_mode is FlightMode.HOLD
        assert s.pose_odom.x == pytest.approx(x0, abs=1e-6)  # stored, not consumed (PX4 semantics)

    asyncio.run(scenario())


def test_odom_origin_is_the_arm_point_and_disarm_in_the_air_is_refused() -> None:
    async def scenario() -> None:
        clock = ManualClock()
        v = _vehicle(clock)
        await v.start_offboard(VelocitySetpoint(velocity=Vec3(0.0, 0.0, 0.7)))
        await v.arm()
        s = await v.get_vehicle_state()
        assert abs(s.pose_odom.x) < 1e-9 and abs(s.pose_odom.z) < 1e-9
        clock.advance(3.0)
        assert not (await v.disarm()).ok

    asyncio.run(scenario())

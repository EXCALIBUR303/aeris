"""``FastSimVehicle``: one FastSim vehicle behind the same
:class:`~aeris.vehicle.interface.VehicleInterface` + ``CommandPort``
protocols :class:`~aeris.vehicle.px4_mavlink.adapter.Px4MavlinkAdapter`
implements (spec §51 Phase 13 task 2), so Tier-agnostic code --
:class:`~aeris.safety.supervisor.SafetySupervisor`, the mission executive,
episode runners -- can drive FastSim unmodified.

**Semantics copied from Tier H, not invented:**

- Offboard setpoints are only *consumed* while the mode is OFFBOARD
  (PX4's ``mavlink_receiver`` guards on ``nav_state``; Phase 5 found this
  live) -- a velocity target sent in HOLD is stored but ignored.
- ``start_offboard(initial)`` then ``arm()`` climbs (the supervisor's
  Phase 5 "offboard takeoff"); ``hold()`` holds position; ``land()``
  descends at 0.7 m/s and reports ON_GROUND at touchdown.
- The odom origin is the vehicle's pose at arming (Phase 10's live
  finding), and ``pose_odom`` is the *estimate* (truth + the fitted
  pose-error model), exactly what an EKF would report.

**Physics** is :class:`~aeris.simulation.fastsim.batch.FastSimBatch` with
N=1 (identified horizontal + yaw dynamics, collision). Vertical motion is
*not* identified this phase (every FastSim task holds altitude): it is a
plain first-order lag with the horizontal time constant, used only to get
through takeoff/landing, and is labeled as such.

**Time** follows an injected :class:`~aeris.core.clock.Clock`: every
protocol call first advances the physics to ``clock.now()``, in
``dt_sim`` steps. With a ``ManualClock`` a test controls time exactly;
with a ``WallClock`` it runs at real-time speed.
"""

from __future__ import annotations

import math
from collections.abc import AsyncIterator

import numpy as np

from aeris.core.clock import Clock
from aeris.core.constants import EKF_HEALTHY_REQUIRED_FLAGS
from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import ZERO, Vec3
from aeris.simulation.fastsim.batch import FastSimBatch
from aeris.simulation.fastsim.dynamics import DT_SIM_S
from aeris.vehicle.interface import (
    BatterySimState,
    CommandResult,
    CommandResultCode,
    EkfFlags,
    FlightMode,
    GpsFixType,
    GpsStatus,
    LandedState,
    LinkStatus,
    PositionSetpoint,
    Setpoint,
    VehicleEndpoint,
    VehicleState,
    VelocitySetpoint,
    Waypoint,
)

_OK = CommandResult(CommandResultCode.ACCEPTED)
_LAND_SPEED_MPS = 0.7


class FastSimVehicle:
    def __init__(
        self, sim: FastSimBatch, *, clock: Clock, ground_contact_z_m: float = 0.24
    ) -> None:
        if sim.n != 1:
            raise ValueError("FastSimVehicle drives exactly one FastSim env")
        self.sim = sim
        self.clock = clock
        self.ground_contact_z = ground_contact_z_m  # base_link height when landed
        self.connected = False
        self.armed = False
        self.mode = FlightMode.HOLD
        self._t_last = clock.now()
        self._t_sim = 0.0
        self._vz = 0.0
        self._cmd = np.zeros(4)  # odom-frame vx, vy, vz, yaw rate
        self._hold_xy: np.ndarray | None = None
        self._home_world = sim.dyn.state.pos[0].copy()

    # -- physics ----------------------------------------------------------------------
    def _advance(self) -> None:
        now = self.clock.now()
        steps = int((now - self._t_last) / DT_SIM_S)
        if steps <= 0:
            return
        self._t_last += steps * DT_SIM_S
        s = self.sim.dyn.state
        tau = float(self.sim.dyn.tau_xy[0])
        for _ in range(steps):
            cmd = self._effective_command()
            if self.armed and not self._on_ground_now():
                self.sim.dyn.step(cmd[None, [0, 1, 3]])
            elif self.armed and cmd[2] > 0.0:
                self.sim.dyn.step(np.zeros((1, 3)))
            vz_cmd = cmd[2] if self.armed else 0.0
            self._vz += (vz_cmd - self._vz) * (DT_SIM_S / tau)
            s.pos[0, 2] = max(self.ground_contact_z, s.pos[0, 2] + self._vz * DT_SIM_S)
            if self._on_ground_now():
                self._vz = max(self._vz, 0.0)
                s.vel[0] = 0.0
                s.yaw_rate[0] = 0.0
            self._t_sim += DT_SIM_S

    def _on_ground_now(self) -> bool:
        return bool(self.sim.dyn.state.pos[0, 2] <= self.ground_contact_z + 1e-6)

    def _effective_command(self) -> np.ndarray:
        if not self.armed:
            return np.zeros(4)
        if self.mode is FlightMode.OFFBOARD:
            return self._cmd
        if self.mode is FlightMode.LAND:
            return np.array([0.0, 0.0, -_LAND_SPEED_MPS, 0.0])
        return np.zeros(4)  # HOLD: stop and hover (position hold ~ zero velocity here)

    # -- VehicleInterface -------------------------------------------------------------
    async def connect(self, endpoint: VehicleEndpoint) -> None:
        self.connected = True

    async def disconnect(self) -> None:
        self.connected = False

    async def get_vehicle_state(self) -> VehicleState:
        self._advance()
        s = self.sim.dyn.state
        xy, yaw = self.sim.estimated_pose()
        home = self._home_world
        vx, vy = float(s.vel[0, 0]), float(s.vel[0, 1])
        airborne = not self._on_ground_now()
        landed = (
            LandedState.IN_AIR
            if airborne and self.mode is not FlightMode.LAND
            else (LandedState.LANDING if airborne else LandedState.ON_GROUND)
        )
        return VehicleState(
            t_sim_s=self._t_sim,
            armed=self.armed,
            flight_mode=self.mode,
            landed_state=landed,
            pose_odom=Vec3(
                float(xy[0, 0] - home[0]),
                float(xy[0, 1] - home[1]),
                float(s.pos[0, 2] - home[2]),
            ),
            orientation_odom=Quaternion.from_yaw(float(yaw[0])),
            velocity_odom_mps=Vec3(vx, vy, self._vz),
            angular_velocity_body_radps=Vec3(0.0, 0.0, float(s.yaw_rate[0])),
            home_odom=ZERO,
            gps=GpsStatus(fix_type=GpsFixType.FIX_3D, satellites_visible=12, eph_m=0.3, epv_m=0.5),
            battery_sim=BatterySimState(remaining_fraction=1.0, voltage_v=16.0),
            ekf_flags=EkfFlags(
                raw_flags=EKF_HEALTHY_REQUIRED_FLAGS,
                pos_horiz_accuracy_m=0.1,
                pos_vert_accuracy_m=0.1,
            ),
            link=LinkStatus(connected=self.connected, last_heartbeat_age_s=0.0),
        )

    async def subscribe_telemetry(self, rate_hz: float) -> AsyncIterator[VehicleState]:
        raise NotImplementedError("FastSim is stepped by its clock; poll get_vehicle_state()")
        yield await self.get_vehicle_state()  # pragma: no cover

    def command_port(self) -> FastSimVehicle:
        return self

    # -- CommandPort ------------------------------------------------------------------
    async def arm(self) -> CommandResult:
        self._advance()
        if not self.armed:
            self._home_world = self.sim.dyn.state.pos[0].copy()  # odom origin = arm point
        self.armed = True
        return _OK

    async def disarm(self) -> CommandResult:
        self._advance()
        if not self._on_ground_now():
            return CommandResult(CommandResultCode.DENIED, "refusing to disarm in the air")
        self.armed = False
        return _OK

    async def takeoff(self, altitude_m: float) -> CommandResult:
        return CommandResult(CommandResultCode.UNSUPPORTED, "use offboard takeoff (Phase 5)")

    async def land(self) -> CommandResult:
        self._advance()
        self.mode = FlightMode.LAND
        return _OK

    async def hold(self) -> CommandResult:
        self._advance()
        self.mode = FlightMode.HOLD
        return _OK

    async def return_to_launch(self) -> CommandResult:
        return CommandResult(CommandResultCode.UNSUPPORTED, "RTL is not modeled in FastSim")

    async def start_offboard(self, initial: Setpoint) -> CommandResult:
        self._advance()
        self._store(initial)
        self.mode = FlightMode.OFFBOARD
        return _OK

    async def stop_offboard(self) -> CommandResult:
        self._advance()
        self.mode = FlightMode.HOLD
        return _OK

    async def set_position_target(self, sp: PositionSetpoint) -> None:
        raise NotImplementedError(
            "FastSim tasks command velocity; position targets are not modeled"
        )

    async def set_velocity_target(self, sp: VelocitySetpoint) -> None:
        self._advance()
        self._store(sp)

    async def goto_waypoint(self, wp: Waypoint) -> CommandResult:
        return CommandResult(CommandResultCode.UNSUPPORTED, "missions are not modeled in FastSim")

    async def emergency_stop_simulation(self) -> CommandResult:
        self.armed = False
        self.mode = FlightMode.HOLD
        return _OK

    def _store(self, sp: Setpoint) -> None:
        if not isinstance(sp, VelocitySetpoint):
            raise NotImplementedError("FastSim accepts velocity setpoints only")
        v = sp.velocity
        if sp.frame == "body":
            yaw = float(self.sim.dyn.state.yaw[0])
            c, s = math.cos(yaw), math.sin(yaw)
            v = Vec3(c * v.x - s * v.y, s * v.x + c * v.y, v.z)
        self._cmd = np.array([v.x, v.y, v.z, sp.yaw_rate_radps or 0.0])

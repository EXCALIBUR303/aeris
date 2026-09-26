"""``MissionExecutive`` — a deterministic state machine driving one
``SafetySupervisor`` through a full waypoint mission (spec §51 Phase 6).

``aeris.autonomy`` may depend only on ``aeris.vehicle.interface`` types
and ``aeris.safety`` (spec §14.3 contract 2) — never a vehicle adapter
implementation directly. This module never imports
``aeris.vehicle.px4_mavlink`` (import-linter contract).
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field

from aeris.autonomy.mission.spec import MissionSpec, WaypointSpec
from aeris.autonomy.mission.states import MissionState, allowed_transitions, is_terminal
from aeris.core.clock import Clock
from aeris.core.errors import UnsafeStateError
from aeris.core.frames.vector import Vec3
from aeris.core.logging import get_logger
from aeris.safety.supervisor import SafetySupervisor
from aeris.vehicle.interface import PositionSetpoint, VehicleEndpoint

_logger = get_logger(component="autonomy.mission.executive")

_TICK_PERIOD_S = 0.1


@dataclass(frozen=True, slots=True)
class WaypointOutcome:
    index: int
    target: Vec3
    achieved: Vec3
    error_m: float
    arrived: bool


@dataclass(frozen=True, slots=True)
class MissionResult:
    state: MissionState
    waypoints: tuple[WaypointOutcome, ...] = field(default_factory=tuple)
    return_outcome: WaypointOutcome | None = None
    reason: str = ""
    wall_time_s: float = 0.0

    @property
    def ok(self) -> bool:
        return self.state == MissionState.COMPLETE

    @property
    def max_waypoint_error_m(self) -> float:
        if not self.waypoints:
            return 0.0
        return max(w.error_m for w in self.waypoints)


class MissionExecutive:
    """Drives one ``SafetySupervisor`` through IDLE -> ... -> COMPLETE (or
    ABORTED/FAILSAFE) for a given :class:`MissionSpec`."""

    def __init__(self, supervisor: SafetySupervisor, spec: MissionSpec, *, clock: Clock) -> None:
        self._supervisor = supervisor
        self._spec = spec
        self._clock = clock
        self._state = MissionState.IDLE
        self._aborted = False

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def aborted(self) -> bool:
        return self._aborted

    def _transition(self, new_state: MissionState) -> None:
        allowed = allowed_transitions(self._state)
        if new_state not in allowed:
            raise UnsafeStateError(
                f"illegal mission transition {self._state!r} -> {new_state!r} "
                f"(allowed: {sorted(allowed)})"
            )
        old = self._state
        self._state = new_state
        _logger.info("mission.transition", frm=old.value, to=new_state.value)

    def abort(self) -> None:
        """Callable from any non-terminal state (spec §51 Phase 6: "abort
        path"; tested as "abort in every state"). A no-op once terminal."""
        if is_terminal(self._state):
            return
        self._aborted = True
        self._transition(MissionState.ABORTED)

    async def run(self, endpoint: VehicleEndpoint) -> MissionResult:
        """Run the full mission end to end. Never raises for an ordinary
        mission failure (rejected command, timeout, abort) -- that's
        reported in the returned :class:`MissionResult`."""
        t0 = self._clock.now()
        try:
            self._transition(MissionState.PREFLIGHT)
            await self._supervisor.connect(endpoint)
            if not await self._supervisor.preflight_check():
                self.abort()
                return self._result(reason="preflight_check failed", t0=t0)

            self._transition(MissionState.TAKEOFF)
            takeoff_result = await self._supervisor.arm_and_takeoff(self._spec.takeoff_altitude_m)
            if not takeoff_result.ok:
                self.abort()
                return self._result(
                    reason=f"arm_and_takeoff rejected: {takeoff_result.message}", t0=t0
                )

            if not await self._wait_for_altitude(self._spec.takeoff_altitude_m):
                await self._supervisor.emergency_stop_simulation()
                self._transition(MissionState.FAILSAFE)
                return self._result(reason="never reached takeoff altitude", t0=t0)
            await self._supervisor.confirm_airborne()

            start_result = await self._supervisor.start_autonomy()
            if not start_result.ok:
                await self._supervisor.emergency_stop_simulation()
                self._transition(MissionState.FAILSAFE)
                return self._result(
                    reason=f"start_autonomy rejected: {start_result.message}", t0=t0
                )

            home = (await self._supervisor.get_vehicle_state()).pose_odom

            self._transition(MissionState.EXECUTING)
            t_mission_start_s = self._clock.now()
            outcomes: list[WaypointOutcome] = []
            for i, wp in enumerate(self._spec.waypoints):
                if self._aborted:
                    break
                if self._clock.now() - t_mission_start_s > self._spec.time_budget_s:
                    _logger.info("mission.time_budget_exceeded", waypoint_index=i)
                    break
                outcomes.append(await self._fly_to_waypoint(i, wp))

            self._transition(MissionState.RETURN)
            return_outcome = None
            if not self._aborted:
                # dwell_s=0.0: about to land right after, no reason to loiter.
                return_wp = WaypointSpec(x=home.x, y=home.y, z=home.z, dwell_s=0.0)
                return_outcome = await self._fly_to_waypoint(len(self._spec.waypoints), return_wp)

            await self._supervisor.stop_autonomy()
            self._transition(MissionState.LAND)
            land_result = await self._supervisor.land()
            if not land_result.ok:
                self.abort()
                return self._result(
                    outcomes, return_outcome, reason=f"land rejected: {land_result.message}", t0=t0
                )
            if not await self._wait_for_landed():
                self.abort()
                return self._result(outcomes, return_outcome, reason="never landed", t0=t0)
            self._supervisor.confirm_landed()
            await self._supervisor.disarm()

            self._transition(MissionState.COMPLETE)
            return self._result(outcomes, return_outcome, reason="ok", t0=t0)
        finally:
            await self._supervisor.disconnect()

    def _result(
        self,
        outcomes: list[WaypointOutcome] | None = None,
        return_outcome: WaypointOutcome | None = None,
        *,
        reason: str,
        t0: float,
    ) -> MissionResult:
        return MissionResult(
            state=self._state,
            waypoints=tuple(outcomes or ()),
            return_outcome=return_outcome,
            reason=reason,
            wall_time_s=self._clock.now() - t0,
        )

    async def _fly_to_waypoint(self, index: int, wp: WaypointSpec) -> WaypointOutcome:
        target = wp.position
        sp = PositionSetpoint(position_odom=target, yaw_rad=wp.yaw_rad)
        deadline = self._clock.now() + self._spec.waypoint_timeout_s
        arrived_since: float | None = None
        last_state = await self._supervisor.get_vehicle_state()
        while self._clock.now() < deadline:
            if self._aborted:
                break
            result = await self._supervisor.submit_setpoint(sp)
            if not result.ok:
                break
            self._supervisor.autonomy_heartbeat()
            await self._supervisor.tick()
            last_state = await self._supervisor.get_vehicle_state()
            error = (last_state.pose_odom - target).norm()
            if error <= wp.acceptance_radius_m:
                if arrived_since is None:
                    arrived_since = self._clock.now()
                if self._clock.now() - arrived_since >= wp.dwell_s:
                    return WaypointOutcome(index, target, last_state.pose_odom, error, True)
            else:
                arrived_since = None
            await asyncio.sleep(_TICK_PERIOD_S)
        error = (last_state.pose_odom - target).norm()
        return WaypointOutcome(index, target, last_state.pose_odom, error, False)

    async def _wait_for_altitude(
        self, altitude_m: float, *, timeout_s: float = 30.0, tolerance_m: float = 0.5
    ) -> bool:
        deadline = self._clock.now() + timeout_s
        while self._clock.now() < deadline:
            state = await self._supervisor.get_vehicle_state()
            if state.pose_odom.z >= altitude_m - tolerance_m:
                return True
            await asyncio.sleep(0.5)
        return False

    async def _wait_for_landed(self, *, timeout_s: float = 30.0) -> bool:
        deadline = self._clock.now() + timeout_s
        while self._clock.now() < deadline:
            state = await self._supervisor.get_vehicle_state()
            if state.landed_state.value == "on_ground":
                return True
            await asyncio.sleep(0.5)
        return False

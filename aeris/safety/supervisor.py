"""``SafetySupervisor`` — the sole holder of a ``CommandPort`` (spec §16.1
rule 3, §16.3's state machine, §14.3 contract 3).

Nothing outside this module (and, transitively, outside ``aeris.safety``)
should ever call ``VehicleInterface.command_port()`` --
``tests/unit/safety/test_command_port_contract.py`` scans for that.
"""

from __future__ import annotations

from aeris.core.clock import Clock
from aeris.core.constants import (
    AERIS_SAFETY_SUPERVISOR_RATE_HZ,
    EKF_HEALTHY_FORBIDDEN_FLAGS,
    EKF_HEALTHY_REQUIRED_FLAGS,
)
from aeris.core.errors import UnsafeStateError
from aeris.core.frames.vector import Vec3
from aeris.core.logging import get_logger
from aeris.safety.envelope import Envelope
from aeris.safety.events import SafetyEvent, SafetyEventKind
from aeris.safety.validator import CommandValidator
from aeris.safety.watchdog import StalenessWatchdog
from aeris.vehicle.interface import (
    CommandPort,
    CommandResult,
    CommandResultCode,
    EkfFlags,
    PositionSetpoint,
    Setpoint,
    VehicleEndpoint,
    VehicleInterface,
    VehicleState,
    VelocitySetpoint,
)

_logger = get_logger(component="safety.supervisor")

_TICK_PERIOD_S = 1.0 / AERIS_SAFETY_SUPERVISOR_RATE_HZ


class SupervisorState:
    """Spec §16.3's state machine, as string constants (not a StrEnum, so
    they read identically in code and in event logs without a ``.value``)."""

    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    PREFLIGHT_OK = "preflight_ok"
    ARMING = "arming"
    TAKING_OFF = "taking_off"
    AIRBORNE_MANUAL_HOLD = "airborne_manual_hold"
    AUTONOMY_ACTIVE = "autonomy_active"
    INTERVENTION = "intervention"
    LANDING = "landing"
    LANDED = "landed"
    DISARMED = "disarmed"
    FAILSAFE = "failsafe"


# The literal spec §16.3 diagram:
#   DISCONNECTED -> CONNECTED -> PREFLIGHT_OK -> ARMING -> TAKING_OFF
#     -> AIRBORNE_MANUAL_HOLD <-> AUTONOMY_ACTIVE -> [INTERVENTION -> back]
#     -> LANDING -> LANDED -> DISARMED
#   any -> FAILSAFE -> LANDING/HOLD
# Plus the pragmatic edges every real run needs: a rejected arm/takeoff
# falls back to PREFLIGHT_OK/ARMING rather than straight to FAILSAFE (a
# rejection is not an emergency), and DISCONNECTED is reachable from any
# not-yet-airborne state (an operator/test can always abort a preflight).
_TRANSITIONS: dict[str, frozenset[str]] = {
    SupervisorState.DISCONNECTED: frozenset({SupervisorState.CONNECTED}),
    SupervisorState.CONNECTED: frozenset(
        {SupervisorState.PREFLIGHT_OK, SupervisorState.DISCONNECTED}
    ),
    SupervisorState.PREFLIGHT_OK: frozenset({SupervisorState.ARMING, SupervisorState.DISCONNECTED}),
    SupervisorState.ARMING: frozenset(
        {SupervisorState.TAKING_OFF, SupervisorState.PREFLIGHT_OK, SupervisorState.DISCONNECTED}
    ),
    SupervisorState.TAKING_OFF: frozenset(
        {SupervisorState.AIRBORNE_MANUAL_HOLD, SupervisorState.ARMING}
    ),
    SupervisorState.AIRBORNE_MANUAL_HOLD: frozenset(
        {SupervisorState.AUTONOMY_ACTIVE, SupervisorState.LANDING}
    ),
    SupervisorState.AUTONOMY_ACTIVE: frozenset(
        {
            SupervisorState.AIRBORNE_MANUAL_HOLD,
            SupervisorState.INTERVENTION,
            SupervisorState.LANDING,
        }
    ),
    SupervisorState.INTERVENTION: frozenset(
        {SupervisorState.AUTONOMY_ACTIVE, SupervisorState.AIRBORNE_MANUAL_HOLD}
    ),
    SupervisorState.LANDING: frozenset({SupervisorState.LANDED}),
    SupervisorState.LANDED: frozenset({SupervisorState.DISARMED}),
    SupervisorState.DISARMED: frozenset({SupervisorState.DISCONNECTED}),
    SupervisorState.FAILSAFE: frozenset(
        {SupervisorState.LANDING, SupervisorState.AIRBORNE_MANUAL_HOLD}
    ),
}
# "any -> FAILSAFE" (spec §16.3): added to every state's allowed set below,
# rather than repeated in the table above.
for _state, _targets in _TRANSITIONS.items():
    _TRANSITIONS[_state] = _targets | {SupervisorState.FAILSAFE}


def _ekf_healthy(flags: EkfFlags) -> bool:
    raw = flags.raw_flags
    return (
        raw & EKF_HEALTHY_REQUIRED_FLAGS
    ) == EKF_HEALTHY_REQUIRED_FLAGS and raw & EKF_HEALTHY_FORBIDDEN_FLAGS == 0


class SafetySupervisor:
    """Spec §16: the only path to the vehicle. Owns S1 (validator), S2
    (state gating, here), and S4 (watchdogs)."""

    def __init__(
        self,
        vehicle: VehicleInterface,
        *,
        envelope: Envelope,
        clock: Clock,
        autonomy_tick_timeout_s: float = 2.0,
        telemetry_stale_timeout_s: float = 3.0,
        emergency_land_altitude_m: float = 1.0,
        takeoff_climb_speed_mps: float = 0.7,
    ) -> None:
        self._vehicle = vehicle
        # The ONLY call to command_port() anywhere outside a test fixture.
        self._command_port: CommandPort = vehicle.command_port()
        self._envelope = envelope
        self._clock = clock
        self._validator = CommandValidator(envelope)
        self._tick_watchdog = StalenessWatchdog("autonomy_tick", autonomy_tick_timeout_s, clock)
        self._telemetry_stale_timeout_s = telemetry_stale_timeout_s
        self._emergency_land_altitude_m = emergency_land_altitude_m
        # Verified live in Phase 5 (see arm_and_takeoff's docstring): 0.7
        # m/s reliably produces a clean climb within the envelope's
        # vz_max_mps without tripping S1 on the very first setpoint.
        self._takeoff_climb_speed_mps = takeoff_climb_speed_mps

        self._state = SupervisorState.DISCONNECTED
        self._events: list[SafetyEvent] = []
        self._aborted = False

    # --- introspection -----------------------------------------------------------

    @property
    def state(self) -> str:
        return self._state

    @property
    def envelope(self) -> Envelope:
        return self._envelope

    @property
    def aborted(self) -> bool:
        return self._aborted

    def events(self) -> tuple[SafetyEvent, ...]:
        return tuple(self._events)

    # --- state machine -------------------------------------------------------------

    def _transition(self, new_state: str, *, reason: str = "") -> None:
        allowed = _TRANSITIONS.get(self._state, frozenset())
        if new_state not in allowed:
            raise UnsafeStateError(
                f"illegal supervisor transition {self._state!r} -> {new_state!r} "
                f"(allowed: {sorted(allowed)})"
            )
        old_state = self._state
        self._state = new_state
        self._log_event(
            SafetyEventKind.STATE_TRANSITION,
            f"{old_state} -> {new_state}",
            {"from": old_state, "to": new_state, "reason": reason},
        )

    def _require_state(self, *allowed: str) -> None:
        if self._state not in allowed:
            raise UnsafeStateError(
                f"operation requires state in {sorted(allowed)}, currently {self._state!r}"
            )

    def _log_event(
        self, kind: SafetyEventKind, message: str, detail: dict[str, object] | None = None
    ) -> None:
        event = SafetyEvent(
            t_sim_s=self._clock.now(), kind=kind, message=message, detail=detail or {}
        )
        self._events.append(event)
        _logger.info("safety.event", kind=kind.value, message=message, **(detail or {}))

    # --- lifecycle -------------------------------------------------------------------

    async def connect(self, endpoint: VehicleEndpoint) -> None:
        self._require_state(SupervisorState.DISCONNECTED)
        await self._vehicle.connect(endpoint)
        self._transition(SupervisorState.CONNECTED)

    async def disconnect(self) -> None:
        await self._vehicle.disconnect()
        self._state = SupervisorState.DISCONNECTED

    async def get_vehicle_state(self) -> VehicleState:
        """Read-only telemetry passthrough -- unlike ``CommandPort``, safe
        for any caller (autonomy, scripts, tests) to use directly."""
        return await self._vehicle.get_vehicle_state()

    async def preflight_check(self) -> bool:
        """S2's EKF health gate, checked once before arming (spec §16.2)."""
        self._require_state(SupervisorState.CONNECTED)
        state = await self._vehicle.get_vehicle_state()
        healthy = _ekf_healthy(state.ekf_flags) and state.link.connected
        if not healthy:
            self._log_event(
                SafetyEventKind.COMMAND_REJECTED,
                "preflight check failed: EKF unhealthy or link not connected",
                {"ekf_flags": state.ekf_flags.raw_flags, "link_connected": state.link.connected},
            )
            return False
        self._transition(SupervisorState.PREFLIGHT_OK)
        return True

    async def arm_and_takeoff(self, altitude_m: float) -> CommandResult:
        """Arm and begin climbing toward ``altitude_m``.

        Deliberately does **not** use ``CommandPort.takeoff()`` (PX4's own
        ``MAV_CMD_NAV_TAKEOFF`` / ``AUTO_TAKEOFF`` mode). Phase 5 found,
        reading the pinned PX4 checkout
        (``MulticopterPositionControl.cpp``'s ``not_taken_off`` branch and
        ``MulticopterLandDetector.cpp``'s low-throttle ground-contact
        check), that ``AUTO_TAKEOFF`` on a cold-started SITL instance can
        get stuck in a self-reinforcing low-thrust/ground-contact loop --
        confirmed live: the vehicle oscillates within a few cm of the
        ground and auto-disarms after ``COM_DISARM_PRFLT`` (~10s) without
        ever genuinely leaving it. Arming and immediately entering
        OFFBOARD with a modest, constant upward velocity setpoint --
        the same pattern real companion-computer-driven flight
        (MAVSDK/ROS2 "offboard takeoff" tutorials) uses -- was verified
        live (twice, reproducibly, clean monotonic climbs to >2m within
        10s) to reliably produce a real climb instead. See the Phase 5
        report for the raw traces.

        The caller polls altitude/`landed_state` and calls
        :meth:`confirm_airborne` once satisfied -- unchanged from before;
        ``altitude_m`` is accepted for interface symmetry/documentation,
        the actual arrival check is still the caller's job.
        """
        self._require_state(SupervisorState.PREFLIGHT_OK)
        self._transition(SupervisorState.ARMING)

        climb_sp = VelocitySetpoint(velocity=Vec3(0.0, 0.0, self._takeoff_climb_speed_mps))
        offboard_result = await self._command_port.start_offboard(climb_sp)
        if not offboard_result.ok:
            self._transition(
                SupervisorState.PREFLIGHT_OK, reason="pre-arm offboard-climb start rejected"
            )
            return offboard_result

        arm_result = await self._command_port.arm()
        if not arm_result.ok:
            await self._command_port.stop_offboard()
            self._transition(SupervisorState.PREFLIGHT_OK, reason="arm rejected")
            return arm_result

        self._transition(SupervisorState.TAKING_OFF)
        return arm_result

    async def confirm_airborne(self) -> CommandResult:
        """Call once the caller has observed the vehicle actually left the
        ground (e.g. polled ``VehicleState.landed_state``/altitude) after
        :meth:`arm_and_takeoff` -- kept as an explicit step rather than
        folded into ``arm_and_takeoff`` so the caller controls how it
        decides "airborne" (this phase found PX4's own takeoff timing is
        not always trustworthy on its own -- see the Phase 4 report).

        Explicitly commands Hold, rather than assuming PX4's takeoff
        sequence already leaves it there, so ``AIRBORNE_MANUAL_HOLD``
        actually matches what the vehicle is doing."""
        self._require_state(SupervisorState.TAKING_OFF)
        result = await self._command_port.hold()
        self._transition(SupervisorState.AIRBORNE_MANUAL_HOLD)
        return result

    async def start_autonomy(self, initial: Setpoint | None = None) -> CommandResult:
        """Hand control to autonomy -- re-engages OFFBOARD mode.

        PX4 only consumes ``SET_POSITION_TARGET_LOCAL_NED`` while its own
        ``nav_state`` is ``OFFBOARD`` (verified in Phase 4:
        ``mavlink_receiver.cpp``'s handler literally guards the publish on
        it). :meth:`confirm_airborne`/:meth:`stop_autonomy` explicitly
        command Hold, which switches PX4 *out* of OFFBOARD -- without this
        re-engaging it, every subsequent :meth:`submit_setpoint` would be
        accepted by the validator and streamed over MAVLink, but silently
        ignored by PX4 (confirmed live in Phase 5: the vehicle stayed
        essentially motionless at hover with the target 5m away, until
        this fix). Defaults to a zero-velocity setpoint (hold in place via
        OFFBOARD, rather than PX4's own Hold mode) if the caller doesn't
        have a real first command ready yet.
        """
        self._require_state(SupervisorState.AIRBORNE_MANUAL_HOLD)
        if initial is None:
            initial = VelocitySetpoint(velocity=Vec3(0.0, 0.0, 0.0))
        result = await self._command_port.start_offboard(initial)
        if not result.ok:
            return result
        self._tick_watchdog.mark()
        self._validator.reset_rate_limit_history()
        self._transition(SupervisorState.AUTONOMY_ACTIVE)
        return result

    async def stop_autonomy(self) -> CommandResult:
        """Explicitly commands Hold -- without this, PX4 would keep
        executing the last-streamed offboard setpoint indefinitely, which
        would make ``AIRBORNE_MANUAL_HOLD`` a lie."""
        self._require_state(SupervisorState.AUTONOMY_ACTIVE, SupervisorState.INTERVENTION)
        result = await self._command_port.hold()
        self._transition(SupervisorState.AIRBORNE_MANUAL_HOLD)
        return result

    def autonomy_heartbeat(self) -> None:
        """The autonomy loop calls this every tick to prove it's alive (S4)."""
        self._tick_watchdog.mark()

    async def land(self) -> CommandResult:
        self._require_state(
            SupervisorState.AIRBORNE_MANUAL_HOLD,
            SupervisorState.AUTONOMY_ACTIVE,
            SupervisorState.FAILSAFE,
        )
        self._transition(SupervisorState.LANDING)
        return await self._command_port.land()

    def confirm_landed(self) -> None:
        self._require_state(SupervisorState.LANDING)
        self._transition(SupervisorState.LANDED)

    async def disarm(self) -> CommandResult:
        self._require_state(SupervisorState.LANDED)
        result = await self._command_port.disarm()
        if result.ok:
            self._transition(SupervisorState.DISARMED)
        return result

    # --- commanded setpoints (S1 + S2) ------------------------------------------------

    async def submit_setpoint(self, sp: Setpoint) -> CommandResult:
        """The only way autonomy code (a future ``aeris.autonomy``, or
        ``scripts/fly_box.py`` standing in for it now) reaches the vehicle
        with a position/velocity command -- gated by S2 (only in
        ``AUTONOMY_ACTIVE``) then S1 (:class:`~aeris.safety.validator.CommandValidator`)."""
        if self._state != SupervisorState.AUTONOMY_ACTIVE:
            reason = f"setpoint rejected: supervisor state is {self._state!r}, not AUTONOMY_ACTIVE"
            self._log_event(SafetyEventKind.COMMAND_REJECTED, reason)
            return CommandResult(CommandResultCode.DENIED, reason)

        state = await self._vehicle.get_vehicle_state()
        result = self._validator.validate(sp, state=state, now_s=self._clock.now())
        if not result.accepted:
            self._log_event(SafetyEventKind.COMMAND_REJECTED, result.reason, {"setpoint": repr(sp)})
            return CommandResult(CommandResultCode.DENIED, result.reason)

        if isinstance(sp, VelocitySetpoint):
            await self._command_port.set_velocity_target(sp)
        elif isinstance(sp, PositionSetpoint):
            await self._command_port.set_position_target(sp)
        else:
            raise TypeError(f"unknown setpoint type: {type(sp).__name__}")
        return CommandResult(CommandResultCode.ACCEPTED)

    # --- watchdogs / periodic tick (S4) -----------------------------------------------

    async def tick(self) -> None:
        """Run S2/S4 checks once. Call at ``AERIS_SAFETY_SUPERVISOR_RATE_HZ``
        (20 Hz, spec §13.4) while airborne."""
        if self._state not in (
            SupervisorState.AIRBORNE_MANUAL_HOLD,
            SupervisorState.AUTONOMY_ACTIVE,
            SupervisorState.INTERVENTION,
        ):
            return

        state = await self._vehicle.get_vehicle_state()
        fault_reason = self._first_fault(state)

        if fault_reason is not None:
            if self._state == SupervisorState.AUTONOMY_ACTIVE:
                self._transition(SupervisorState.INTERVENTION, reason=fault_reason)
                self._log_event(SafetyEventKind.INTERVENTION_STARTED, fault_reason)
                await self._command_port.hold()
            elif self._state == SupervisorState.INTERVENTION:
                pass  # already intervening; nothing new to do
        elif self._state == SupervisorState.INTERVENTION:
            # Fault cleared -- spec §16.3's "[INTERVENTION -> back]".
            self._transition(SupervisorState.AUTONOMY_ACTIVE, reason="fault cleared")
            self._log_event(SafetyEventKind.INTERVENTION_CLEARED, "all watchdogs clear")

    def _first_fault(self, state: VehicleState) -> str | None:
        if self._state == SupervisorState.AUTONOMY_ACTIVE and self._tick_watchdog.is_stale:
            self._log_event(
                SafetyEventKind.WATCHDOG_TRIP,
                f"autonomy tick stale ({self._tick_watchdog.age_s:.2f}s > "
                f"{self._tick_watchdog.timeout_s}s)",
            )
            return "autonomy_tick_stale"
        if state.link.last_heartbeat_age_s > self._telemetry_stale_timeout_s:
            self._log_event(
                SafetyEventKind.WATCHDOG_TRIP,
                f"telemetry stale ({state.link.last_heartbeat_age_s:.2f}s > "
                f"{self._telemetry_stale_timeout_s}s)",
            )
            return "telemetry_stale"
        if not _ekf_healthy(state.ekf_flags):
            self._log_event(
                SafetyEventKind.WATCHDOG_TRIP,
                f"EKF unhealthy (flags={state.ekf_flags.raw_flags})",
            )
            return "ekf_unhealthy"
        return None

    # --- emergency stop (spec §16.5) --------------------------------------------------

    async def emergency_stop_simulation(self) -> CommandResult:
        """Spec §16.5: "(1) switch PX4 to Hold (or Land if airborne below a
        threshold); (2) stop the autonomy loop; (3) mark the run ABORTED."

        Step 3 is a local flag (:attr:`aborted`) rather than a real
        run-manifest update -- that infrastructure is Phase 38's; this
        exposes the hook it will call.
        """
        state = await self._vehicle.get_vehicle_state()
        if state.pose_odom.z < self._emergency_land_altitude_m:
            action = "land"
            result = await self._command_port.land()
        else:
            action = "hold"
            result = await self._command_port.hold()

        self._aborted = True
        # Follow the spec §16.3 diagram literally: "any -> FAILSAFE ->
        # LANDING/HOLD". FAILSAFE itself stands in for the "HOLD" outcome
        # (hold() was already called above); "land" additionally steps
        # FAILSAFE -> LANDING to match what was actually commanded.
        if self._state != SupervisorState.FAILSAFE:
            self._transition(
                SupervisorState.FAILSAFE, reason=f"emergency_stop_simulation ({action})"
            )
        if action == "land":
            self._transition(
                SupervisorState.LANDING, reason="emergency_stop_simulation follow-through"
            )
        self._log_event(
            SafetyEventKind.EMERGENCY_STOP,
            "emergency_stop_simulation invoked",
            {"altitude_m": state.pose_odom.z, "action": action},
        )
        return result

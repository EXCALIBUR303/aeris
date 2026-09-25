# AERIS — Flight safety layer

## Architecture (spec §16)

`aeris.safety.SafetySupervisor` is the **only** path to the vehicle (spec
§16.1 rule 3, §14.3 contract 3) — it is the only code that ever calls
`VehicleInterface.command_port()`, enforced by
[`tests/unit/safety/test_command_port_contract.py`](../tests/unit/safety/test_command_port_contract.py),
which scans `aeris/` and `scripts/` for that one call site.

| Layer | Module | What it does |
|---|---|---|
| S1 — Command validation | `aeris.safety.validator` | `CommandValidator`: NaN/Inf rejection, bounds (`‖v_xy‖`, `\|v_z\|`, `\|yaw_rate\|`), acceleration/rate-of-change limits, altitude floor/ceiling, geofence (rejects only a command that would move *further* outward — a recovery move back inward is always allowed). |
| — | `aeris.safety.envelope` | `Envelope`: the S1 bounds + geofence, a Pydantic config loaded from `configs/vehicle/safety.yaml`. `assert_strictly_inside_px4_geofence()` checks AERIS's own fence is actually smaller than PX4's S0 one, rather than leaving that relationship as an unverified comment. |
| S2 — State gating | `aeris.safety.supervisor` | `SafetySupervisor`'s state machine (below) — setpoints are only accepted in `AUTONOMY_ACTIVE`; an EKF-health check gates `preflight_check()`. |
| S4 — Watchdogs | `aeris.safety.watchdog` | `StalenessWatchdog`: a generic "has `mark()` been called recently enough" primitive, driving both the autonomy-tick watchdog and (via `VehicleState.link.last_heartbeat_age_s`) telemetry staleness. |
| Events | `aeris.safety.events` | `SafetyEvent`/`SafetyEventKind` — logged via `aeris.core.logging` and kept in the supervisor's own in-memory log; the data shape Phase 7's `EventBus`/replay will consume, built now so that phase doesn't retrofit it. |

## State machine (spec §16.3)

```
DISCONNECTED -> CONNECTED -> PREFLIGHT_OK -> ARMING -> TAKING_OFF -> AIRBORNE_MANUAL_HOLD
      <-> AUTONOMY_ACTIVE -> [INTERVENTION -> back] -> LANDING -> LANDED -> DISARMED
any -> FAILSAFE -> LANDING / AIRBORNE_MANUAL_HOLD ("HOLD")
```

Implemented as an explicit `{state: allowed_next_states}` table in
`aeris/safety/supervisor.py`, exhaustively tested (every state × every
state, 144 cases) in `tests/unit/safety/test_supervisor.py`. A few
pragmatic edges beyond the literal diagram: a rejected `arm()`/offboard-climb-start
falls back to `PREFLIGHT_OK` (a rejection isn't an emergency); any
not-yet-airborne state can return to `DISCONNECTED`.

## Two live findings that changed the design

**PX4's `AUTO_TAKEOFF` mode can get stuck on a cold-started SITL
instance.** Reading `MulticopterPositionControl.cpp` (`not_taken_off`
branch) and `MulticopterLandDetector.cpp` in the pinned checkout: PX4
substitutes an empty setpoint plus a fixed downward acceleration until
the vehicle has genuinely left the ground, and a cold instance's
hover-thrust estimate isn't converged yet, so the low-throttle/
ground-contact loop can repeat until `COM_DISARM_PRFLT` (~10s) auto-disarms
— confirmed live, reproducibly. **Fix:** `SafetySupervisor.arm_and_takeoff()`
does not use `CommandPort.takeoff()` (PX4's `MAV_CMD_NAV_TAKEOFF`) at all —
it starts OFFBOARD with a modest constant upward velocity setpoint
*before* arming (verified order matters), the same pattern real
companion-computer-driven flight (MAVSDK/ROS2 "offboard takeoff"
tutorials) uses. Verified live, repeatedly, with clean monotonic climbs.

**PX4 only consumes offboard setpoints while `nav_state == OFFBOARD`.**
`confirm_airborne()`/`stop_autonomy()` explicitly command Hold (so
`AIRBORNE_MANUAL_HOLD` actually matches what the vehicle is doing) — but
Hold switches PX4 *out* of OFFBOARD. Without re-engaging it,
`submit_setpoint()` would validate and stream a setpoint that PX4 quietly
ignores. **Fix:** `start_autonomy()` calls
`CommandPort.start_offboard()` again before transitioning to
`AUTONOMY_ACTIVE`. Confirmed live: without this fix the vehicle sat
motionless at hover with the target 5m away; with it, it reaches the
target in ~2.5s and holds within ~5cm.

A third, unrelated bug surfaced by finally exercising `apply_params()`
live for the first time (Phase 3 never had): PX4 silently drops any
`PARAM_SET` whose wire `MAV_PARAM_TYPE` doesn't exactly match the
parameter's own declared type — `aeris.simulation.launcher.params` always
sent `REAL32` regardless. Fixed by reading each parameter's real type
first (also absorbs PX4's parameter subsystem not responding until
`Mavlink::boot_complete()` fires, observed to sometimes take its own
~20s sim-time fallback rather than the explicit startup-script call).

## Using it

```python
from aeris.core.clock import WallClock
from aeris.safety.envelope import load_envelope
from aeris.safety.supervisor import SafetySupervisor
from aeris.vehicle.interface import PositionSetpoint, VehicleEndpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

envelope = load_envelope("configs/vehicle/safety.yaml")
adapter = Px4MavlinkAdapter()
supervisor = SafetySupervisor(adapter, envelope=envelope, clock=WallClock())

await supervisor.connect(VehicleEndpoint(host="127.0.0.1", port=14540))
assert await supervisor.preflight_check()
await supervisor.arm_and_takeoff(altitude_m=3.0)
# ... poll altitude yourself, then:
await supervisor.confirm_airborne()
await supervisor.start_autonomy()

result = await supervisor.submit_setpoint(PositionSetpoint(position_odom=...))

# call periodically (20 Hz, spec §13.4) while airborne:
supervisor.autonomy_heartbeat()  # from the autonomy loop
await supervisor.tick()          # S2/S4 checks; auto-intervenes to Hold on a fault
```

Or via the CLI-adjacent script:

```bash
uv run python scripts/fly_box.py --runs 20 --altitude-m 3.0 --side-length-m 5.0
```

## Validated results (Phase 5)

See `docs/phase_reports/phase-5.md` for the actual 20-run numbers and the
fault-injection results — this file describes the mechanism; the phase
report has the data.

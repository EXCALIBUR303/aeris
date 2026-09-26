# AERIS — Waypoint missions

## Architecture (spec §51 Phase 6)

`aeris.autonomy.mission.MissionExecutive` is the first thing in
`aeris.autonomy` — the autonomy engine layer (spec §13.2): pure Python,
runs on an injected `Clock`, no transport code. It drives one
`aeris.safety.SafetySupervisor` through a deterministic state machine
end to end. Per spec §14.3 contract 2, `aeris.autonomy` may depend only
on `aeris.vehicle.interface` types and `aeris.safety` — never a vehicle
adapter directly (import-linter enforced, same restriction `aeris.safety`
itself has).

| Module | Responsibility |
|---|---|
| `aeris.autonomy.mission.spec` | `MissionSpec`/`WaypointSpec` (Pydantic) — a named waypoint list, takeoff altitude, time budget, per-waypoint arrival radius + dwell. Loaded from `configs/missions/*.yaml`. |
| `aeris.autonomy.mission.states` | `MissionState` + an explicit `{state: allowed_next_states}` table, exhaustively tested (every state × every state). |
| `aeris.autonomy.mission.executive` | `MissionExecutive` — the state machine itself, waypoint-arrival hysteresis, and `MissionResult`/`WaypointOutcome`. |

## State machine

```
IDLE -> PREFLIGHT -> TAKEOFF -> EXECUTING -> RETURN -> LAND -> COMPLETE
any non-terminal state -> ABORTED
any state -> FAILSAFE
```

Patterned after spec §32.2's fuller search-and-rescue executive
("IDLE -> PREFLIGHT -> TAKEOFF -> SEARCH ... -> RETURN -> LAND ->
COMPLETE") with `SEARCH`/`INVESTIGATE` collapsed into a single
`EXECUTING` state that visits the mission's waypoint list in order — the
exploration-strategy plug-in point Phase 21 needs isn't required until a
mission has targets to search for.

## Waypoint arrival: radius + dwell

A waypoint counts as arrived only once the vehicle has stayed within
`acceptance_radius_m` continuously for `dwell_s` seconds — not on the
first instant it happens to be close enough (spec's own "Known risks:
arrival-detection jitter" / "Failure/rollback: hysteresis on arrival").
Getting knocked back outside the radius resets the dwell timer. Verified
live: this produces materially tighter, more consistent arrival errors
(0.15–0.19m across 15 live mission runs) than a single-sample check would.

## Using it

```bash
uv run aeris mission run configs/missions/square.yaml
```

Or programmatically:

```python
from aeris.autonomy.mission.executive import MissionExecutive
from aeris.autonomy.mission.spec import load_mission_spec
from aeris.core.clock import WallClock
from aeris.safety.envelope import load_envelope
from aeris.safety.supervisor import SafetySupervisor
from aeris.vehicle.interface import VehicleEndpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

spec = load_mission_spec("configs/missions/square.yaml")
envelope = load_envelope("configs/vehicle/safety.yaml")
adapter = Px4MavlinkAdapter()
supervisor = SafetySupervisor(adapter, envelope=envelope, clock=WallClock())
executive = MissionExecutive(supervisor, spec, clock=WallClock())

result = await executive.run(VehicleEndpoint(host="127.0.0.1", port=14540))
print(result.ok, result.state, result.max_waypoint_error_m)
```

## Templates

`configs/missions/`: `square.yaml` (4 waypoints, the spec's literal
expected-output example), `triangle.yaml` (3 waypoints), `line.yaml` (a
simple out-and-back). All at 3m altitude, sized to the envelope Phase 5
already validated live.

## Validated results (Phase 6)

See `docs/phase_reports/phase-6.md` for the actual 15-run numbers — this
file describes the mechanism; the phase report has the data.

# AERIS — Vehicle interface, telemetry, and frames

## Transport (ADR-0003, finalized Phase 4)

pymavlink is AERIS's sole V1 vehicle transport. MAVSDK was evaluated first
(spec's original plan) but its `asyncio.Mavsdk(Configuration(...))`
construction segfaults deterministically — confirmed in Phase 1, re-tested
in Phase 4 against mavsdk 4.0.0 (the latest release at the time; no newer
point release exists), reproduced both from a plain sync context and a
proper `asyncio.run()` context. No matching upstream issue was found. See
[`aeris/vehicle/px4_mavlink/__init__.py`](../aeris/vehicle/px4_mavlink/__init__.py)
for the full writeup.

## Modules

| Module | Responsibility |
|---|---|
| `vehicle/interface.py` | Transport-agnostic `VehicleInterface`/`CommandPort` protocols, `VehicleState` and friends. No PX4/MAVLink import — import-linter enforces this. |
| `vehicle/hardware_guard.py` | `assert_endpoint_allowed()` — refuses any non-loopback endpoint unless explicitly marked `simulated=True` (spec §16.6). |
| `vehicle/px4_mavlink/adapter.py` | `Px4MavlinkAdapter` — the V1 implementation. Single background reader thread owns `recv_match()`; every other method reads from lock-protected caches. |
| `vehicle/px4_mavlink/modes.py` | Decodes PX4's `custom_mode` bitfield (transcribed from `px4_custom_mode.h`) into `FlightMode`. |
| `vehicle/px4_mavlink/setpoints.py` | `encode_setpoint()` — the one place (besides `core.frames.conventions` itself) allowed to call the ENU/FLU→NED/FRD conversions; import-linter enforces this too. |
| `vehicle/px4_mavlink/state.py` | Builds a `VehicleState` snapshot from the adapter's raw MAVLink message cache. |
| `core/frames/` | `Vec3`, `Quaternion` (Hamilton convention), `Transform`, and `conventions.py`'s fixed ENU↔NED / FLU↔FRD / camera-optical↔body rotations (spec §19). Zero dependencies beyond the stdlib — `aeris.core` stays dependency-minimal. |

## Frames (spec §19)

AERIS internals are ENU (world/odom/map) + FLU (body), exclusively. NED/FRD
conversion is confined to the PX4 adapter boundary — nothing outside
`aeris.core.frames.conventions` and `aeris.vehicle.px4_mavlink.setpoints`
is allowed to touch a NED or FRD value (import-linter contract).

- `enu_to_ned_vec` / `ned_to_enu_vec`: `(x_n, y_n, z_n) = (y_e, x_e, -z_e)` — an involution.
- `flu_to_frd_vec` / `frd_to_flu_vec`: negate Y and Z — also an involution.
- `attitude_enu_body_to_ned_frd` / its inverse: compose the two fixed 180° rotation quaternions around the orientation.
- `enu_yaw_to_ned_yaw` / `ned_yaw_to_enu_yaw`: `psi_NED = pi/2 - psi_ENU`.
- `camera_optical_to_body_rotation()`: fixed quaternion for a forward-facing camera with no extra mount rotation (optical +z → body +x).
- `map_enu_to_threejs()`: `(x_V, y_V, z_V) = (x_M, z_M, -y_M)`, for the Phase 27+ frontend.

All of this is covered by 42 property-based unit tests (fixed-seed random
vectors/quaternions, round-trip and involution checks, known values, the
spec §19.4-mandated cardinal-yaw and known-camera-ray tests) in
`tests/unit/core/frames/`.

## Using it

```bash
uv run aeris sim up --profile headless_x500 &
uv run aeris vehicle monitor                 # live state in ENU, 5 Hz
uv run aeris vehicle monitor --rate 20 --port 14541   # instance 1
```

Or programmatically:

```python
from aeris.vehicle.interface import VehicleEndpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

adapter = Px4MavlinkAdapter()
await adapter.connect(VehicleEndpoint(host="127.0.0.1", port=14540))
state = await adapter.get_vehicle_state()   # pose_odom/velocity_odom_mps are ENU
await adapter.disconnect()
```

## The offboard heartbeat requirement

PX4's `health_and_arming_checks` refuses to arm on a link with no live GCS
heartbeat ("Preflight Fail: No connection to the GCS"), found by reading
`px4.log` in Phase 4 live testing. `Px4MavlinkAdapter` sends its own 1 Hz
`HEARTBEAT` (`MAV_TYPE_GCS`) from `connect()` until `disconnect()` for
exactly this reason — a receive-only client never satisfies this check, no
matter how long you wait.

## Known limitation: the setpoint echo needs real flight

PX4's `MulticopterPositionControl` (`not_taken_off` branch) substitutes an
empty, all-NaN setpoint (plus a fixed downward acceleration, "to make sure
there's no thrust") for whatever the offboard link sends, until the vehicle
has genuinely reached `TakeoffState::flight`. This means
`POSITION_TARGET_LOCAL_NED` — PX4's own setpoint echo — only ever reflects
a commanded value once the vehicle is actually airborne; it cannot be used
to verify the wire encoding while grounded. Reliable arm→takeoff→hover
sequencing is Phase 5's `SafetySupervisor`, not this phase's adapter — see
`tests/integration/test_vehicle_frames_live.py`'s
`test_commanded_enu_east_velocity_arrives_at_px4_as_ned_north_east_vy`
(currently `xfail`, not skipped, so Phase 5 can un-xfail it once real
takeoff sequencing exists).

## Validated results (Phase 4)

See `docs/phase_reports/phase-4.md` for the live-SITL telemetry rate/latency
numbers and the arming/heartbeat/frame findings — this file describes the
mechanism; the phase report has the numbers.

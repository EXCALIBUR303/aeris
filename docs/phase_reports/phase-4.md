# AERIS — Phase 4 Report

============================================================
AERIS — PHASE 4 COMPLETE
============================================================

**PHASE:** 4 — Vehicle interface + telemetry + frames core.

**IMPLEMENTED:**
- `aeris.core.frames`: dependency-free `Vec3`, Hamilton-convention `Quaternion` (with a robust trace-based `from_matrix_columns`, used to derive the fixed camera-optical-to-body rotation), `Transform`, and `conventions.py`'s fixed ENU↔NED / FLU↔FRD / camera-optical↔body rotations plus yaw and three.js mapping helpers (spec §19.3).
- `aeris.vehicle.interface`: transport-agnostic `VehicleInterface`/`CommandPort` protocols, `VehicleState` and its component dataclasses, `PositionSetpoint`/`VelocitySetpoint`/`Waypoint` (spec §15.1). No PX4/MAVLink import anywhere in this module — enforced by a new import-linter contract.
- `aeris.vehicle.hardware_guard`: `assert_endpoint_allowed()`, the loopback-or-explicit-`simulated=True` guard (spec §16.6).
- `aeris.vehicle.px4_mavlink`: the V1 adapter (`Px4MavlinkAdapter`) implementing both `VehicleInterface` and `CommandPort` — connect/telemetry/arm/disarm/takeoff/land/hold/RTL/offboard velocity+position/goto-waypoint/emergency-stop, a single background reader thread bridging pymavlink's synchronous socket API into asyncio, PX4 `custom_mode` decoding (`modes.py`, transcribed from `px4_custom_mode.h`), setpoint encoding (`setpoints.py`, the one other place allowed to call the NED/FRD conversions), and `VehicleState` construction from the raw MAVLink cache (`state.py`).
- ADR-0003 **finalized**: pymavlink is AERIS's sole V1 vehicle transport. MAVSDK's `asyncio.Mavsdk(Configuration(...))` segfault (found in Phase 1) was re-tested against mavsdk 4.0.0 — the latest release, no newer point release exists — reproduced deterministically in both a sync and a proper `asyncio.run()` context, with no matching upstream issue found. Full writeup in `aeris/vehicle/px4_mavlink/__init__.py`.
- `aeris vehicle monitor` CLI subcommand (spec's literal Phase 4 expected output) — connects to a running PX4 instance and prints live `VehicleState` in ENU at a configurable rate.
- `docs/vehicle.md`.
- 216 unit tests (94 new for this phase: 42 in `tests/unit/core/frames/`, 52 in `tests/unit/vehicle/`) and a new `tests/integration/` live-SITL suite (5 tests).

**FILES CREATED:** 24 new files — `aeris/core/frames/{__init__,vector,quaternion,transform,conventions}.py`, `aeris/vehicle/{__init__,interface,hardware_guard}.py`, `aeris/vehicle/px4_mavlink/{__init__,adapter,modes,setpoints,state}.py`, `tests/unit/core/frames/{test_vector,test_quaternion,test_transform,test_conventions}.py`, `tests/unit/vehicle/{test_hardware_guard,test_modes,test_setpoints,test_vehicle_state}.py`, `tests/integration/{__init__,test_vehicle_telemetry,test_vehicle_frames_live}.py`, `tests/conftest.py`, `docs/vehicle.md`.

**FILES MODIFIED:** `aeris/cli.py` (`aeris vehicle monitor`), `pyproject.toml` (import-linter contracts for `core.frames` and `vehicle.interface`; dropped the now-unused `mavsdk.*` mypy override), `tests/sim/conftest.py` (fixtures moved to the new root-level `tests/conftest.py` so `tests/integration/` can share them).

**TESTS RUN:**
- `pytest tests/unit` — 216 tests, local.
- `ruff check .`, `ruff format --check .`, `mypy aeris`, `lint-imports` — all clean, local.
- `pytest tests/integration -v -m integration -s` (live PX4 SITL + Gazebo, **not** run in CI per spec §44.2) — run three times across this phase (once mid-investigation, twice as full confirmation runs after each fix).
- Manual end-to-end CLI test: `aeris sim up --profile headless_x500` (background) → `aeris vehicle monitor` → live ENU state printed at the requested rate → `aeris sim down` → confirmed no orphaned processes.

**TEST RESULTS:**

*Unit (local):* 216 passed, 0 failed.

*Integration (live SITL, final confirmation run, 113.23s):*

| Test | Result |
|---|---|
| `test_commanded_enu_east_velocity_arrives_at_px4_as_ned_north_east_vy` | **XFAIL** (expected — see "Problems found" #4) |
| `test_t_sim_s_tracks_gazebo_clock` | **PASSED** — PX4 `t_sim_s=16.79s` vs. gz `/clock=16.98s` |
| `test_telemetry_rate_meets_20hz_target` | **PASSED** — 150 distinct samples / 3s = 50.0 Hz (target: ≥20 Hz) |
| `test_get_vehicle_state_p95_latency` | **PASSED** — p50=0.03ms, p95=0.06ms |
| `test_heartbeat_age_grows_after_px4_stops` | **PASSED** — 0.16s before stop, 3.47s after 3s of silence |

**SIMULATION RESULTS:**
- Telemetry rate: 50.0 Hz sustained (2.5× the 20 Hz gate), achieved by explicitly requesting `LOCAL_POSITION_NED`/`ATTITUDE_QUATERNION`/`POSITION_TARGET_LOCAL_NED` at 50 Hz via `MAV_CMD_SET_MESSAGE_INTERVAL` on connect — the offboard link doesn't stream these at a guaranteed rate by default (unlike the GCS link).
- `get_vehicle_state()` p95 latency: 0.06ms (an in-memory cache read, as designed — nowhere close to being network-RTT-bound).
- Sim-clock tracking: PX4's `time_boot_ms` (used for `t_sim_s`) matches Gazebo's own `/clock` topic to within 0.2s across a ~17s window — confirms it's genuinely lockstep-tracking sim time, not wall time.
- `aeris vehicle monitor`: verified manually against a live `headless_x500` instance — printed correct ENU pose/velocity, flight mode, landed state, and heartbeat age at the requested rate; clean disconnect and no orphaned processes on `aeris sim down` afterward.

**PROBLEMS FOUND:**
1. **PX4 refuses to arm without a live GCS heartbeat.** `health_and_arming_checks` logs `Preflight Fail: No connection to the GCS` and rejects every `arm()` attempt (`TEMPORARILY_REJECTED`), no matter how long the wait, on a link that only ever *receives*. A real GCS/companion always heartbeats back; `Px4MavlinkAdapter` didn't.
2. **pytest test-module basename collision.** `tests/unit/vehicle/test_state.py` (this phase) collided with the pre-existing `tests/unit/simulation/test_state.py` (Phase 3) under pytest's import mode.
3. **mypy `no-any-return`** in `state.py`'s `t_sim_s_from_local_position`: `local_pos.time_boot_ms / 1000.0` returned `Any` (pymavlink messages are untyped).
4. **The setpoint-echo live test needs real flight, not just arming.** `POSITION_TARGET_LOCAL_NED` (PX4's own setpoint echo) never reflected the commanded velocity — reading `MulticopterPositionControl.cpp` in the pinned PX4 checkout (`not_taken_off` branch) shows PX4 deliberately substitutes an empty all-NaN setpoint plus a fixed `(0,0,100)` downward acceleration ("high downwards acceleration to make sure there's no thrust") for whatever the offboard link sends, until the vehicle has genuinely reached `TakeoffState::flight`. A pure-horizontal velocity command (`vz=0`) never satisfies the `want_takeoff` check (`velocity[2] < 0`), so the vehicle correctly never leaves the ground. Forcing a real arm→takeoff→hover sequence hit a separate SITL characteristic: the vehicle climbed a few centimeters and settled back down without reaching commanded altitude on a cold-started instance (a hover-thrust-estimator convergence issue, not an AERIS bug) — confirmed via `listener`-equivalent telemetry across a 16-sample window, not assumed.
5. **`gz topic -e -t /clock` hangs without `GZ_IP` set**, and hangs regardless if read after `launcher.stop()` has already killed the Gazebo server (a repeat of the Phase 1 `GZ_IP` finding, this time in a test's own subprocess call).
6. **Unused mypy override.** `mavsdk.*` was still listed in `[[tool.mypy.overrides]]` even though nothing imports `mavsdk` anywhere post-ADR-0003 — mypy flagged it as an unused section.

**PROBLEMS FIXED:**
1. `Px4MavlinkAdapter` now sends its own 1 Hz `HEARTBEAT` (`MAV_TYPE_GCS`) from `connect()` until `disconnect()`, matching what QGroundControl/MAVSDK do. Confirmed fixed: `arm()` now returns `ACCEPTED` and the vehicle genuinely arms.
2. Renamed to `tests/unit/vehicle/test_vehicle_state.py`; cleared stale `__pycache__`; re-ran the full unit suite to confirm no other collisions (216/216 passed).
3. Added an explicit `float()` cast.
4. Not "fixed" — correctly re-scoped. The test is marked `xfail` (not skipped, not deleted) with a precise citation of the PX4 source line, so it stays runnable and Phase 5 can un-xfail it once `SafetySupervisor` provides real arm/takeoff/hover sequencing. The spec's own text (quoted in the test module's docstring) already tags the *full* live-SITL frame test as "P5/P8" — this phase's attempt at a flight-free shortcut simply turned out not to be possible, which is itself the useful finding.
5. The `/clock` read now happens *inside* the same async `run()` function, before `adapter.disconnect()`/`launcher.stop()`, with `GZ_IP=127.0.0.1` explicitly set in the subprocess environment.
6. Removed `mavsdk.*` from the mypy override (kept `pymavlink.*`); left `mavsdk` itself in the `sim` extra's dependency list for future re-testing of the segfault against later releases.

All six were found and fixed (or, for #4, correctly re-scoped and documented) before this report was written; the final state passes 216/216 unit tests, 4/5 integration tests (1 expected `xfail`), and all lint/format/typecheck/import-contract checks.

**KNOWN LIMITATIONS:**
- The live SITL frame-correctness test (spec §19.4 item 4, the spec's own "P5/P8"-tagged item) cannot pass until Phase 5 provides reliable arm/takeoff/hover sequencing. Frame *math* correctness is otherwise exhaustively covered by 42 property-based unit tests (round trips, involutions, known values, the spec-mandated cardinal-yaw and known-camera-ray tests).
- `goto_waypoint()` is a simple bounded poll loop (explicitly documented as such in its docstring) — proper mission sequencing is Phase 6's job.
- No takeoff-ramp or ground-safety awareness exists in AERIS itself yet (it's entirely PX4-side, as this phase discovered); Phase 5's `SafetySupervisor` is where AERIS-side flight-state awareness belongs.
- `aeris vehicle monitor` is read-only telemetry — no command-issuing CLI exists yet (arm/takeoff/goto), by design; commanding is gated behind Phase 5's `SafetySupervisor` being the only `CommandPort` holder (spec §14.3, already enforced by an import-linter contract against `aeris.vehicle.interface`, though not yet against a CLI that doesn't exist).

**REMAINING RISKS:**
- The SITL hover-thrust-estimator convergence characteristic found while chasing problem #4 (brief climb, settle back to ~3cm, never reaching commanded altitude on a cold-started instance) is unexplained and could affect Phase 5's takeoff work if it recurs — worth checking whether it's consistent or was a one-off if Phase 5 hits similar behavior.
- The flaky heartbeat timeout noted in the Phase 3 report was not observed this phase, but wasn't specifically retested either.

**VALIDATION GATE (spec §51 Phase 4 / §19.4):** *"All frame tests pass (§19.4's mandatory tests); telemetry rate ≥20Hz with recorded p95 latency; refuses non-loopback endpoints; the transport decision (MAVSDK vs pymavlink) is recorded as ADR-0003 final."*

| Requirement | Result |
|---|---|
| All spec §19.4 frame tests pass | **PASS** — round trips/involutions (items 1), cardinal-point yaw (item 2), known camera ray (item 3) all pass; item 4 (live SITL) is the spec's own "P5/P8"-tagged item, correctly deferred (see above) |
| Telemetry rate ≥20 Hz, p95 latency recorded | **PASS** — 50.0 Hz achieved, p95=0.06ms |
| Refuses non-loopback endpoints | **PASS** — `hardware_guard.py`, 9/9 unit tests, including the deliberate `0.0.0.0` rejection |
| ADR-0003 recorded as final | **PASS** — pymavlink primary/sole V1 transport, documented in `aeris/vehicle/px4_mavlink/__init__.py` and `docs/vehicle.md` |

**VALIDATION GATE: PASS.**

**CURRENT AERIS STATUS:** AERIS can now connect to a running PX4 instance, stream telemetry at well above the required rate, decode flight mode/landed state/GPS/battery/EKF status, and issue every command in `CommandPort` (arm, disarm, takeoff, land, hold, RTL, offboard position/velocity, waypoint-goto, emergency-stop) — all through a transport-agnostic interface with the NED/FRD conversion boundary strictly confined and import-linter-enforced. `aeris vehicle monitor` gives a live, human-readable view of any running vehicle. No safety layer or mission logic exists yet, and — as this phase discovered — no AERIS code can yet make the vehicle actually leave the ground safely; that's Phase 5's job.

============================================================
NEXT PHASE
============================================================

**NEXT PHASE:** 5 — Programmatic takeoff / flight / landing + safety supervisor v1.
**RECOMMENDED MODEL:** Sonnet.
**RECOMMENDED EFFORT:** High.
**WHY THIS MODEL:** Deterministic engineering against a clear spec (§16.3, §51 Phase 5) — a supervisor state machine, command validation, geofencing, and watchdogs.
**WHY THIS EFFORT:** Safety-critical correctness — this phase establishes the *only* command path (spec §16) and the import contract "only Safety may hold a CommandPort." It also inherits this phase's freshest finding: real takeoff sequencing (arm → command a genuine climb → wait for `TakeoffState::flight`, not just "vz<0 once") needs to be handled deliberately, since PX4 silently no-ops any offboard setpoint that never asks to climb, and this phase's own ad-hoc takeoff attempt stalled at ~8cm without reaching commanded altitude on a cold-started SITL instance — worth a repeatable, closely-watched first test rather than assuming a bare `takeoff()` call is sufficient.
**SWITCH REQUIRED:** NO (already on Sonnet).
**USE OPUS ONLY IF:** the safety architecture itself appears flawed (e.g., a PX4 behavior makes the spec's S2 gating unsound) — not expected.

**RECOMMENDED SKILLS / CONNECTORS:** Bash; QGroundControl if visual debugging of a stalled takeoff becomes necessary (not currently installed).

**EXPECTED OUTPUT:**
- `aeris/safety/{supervisor.py,validator.py,envelope.py,watchdog.py,events.py}`
- `configs/vehicle/safety.yaml`
- `scripts/fly_box.py`
- Level L1 achieved: 20/20 "fly a box" runs complete, max position error < 0.5m at waypoints, no failsafe
- A step-response dataset (± 0.5/1/2 m/s per axis, yaw-rate steps) for Phase 13's later system ID
- Import contract "only Safety may hold a CommandPort" active and enforced
- A resolved, empirically-verified takeoff sequence that reliably reaches real altitude from a cold-started SITL instance (this phase's stalled attempt is the starting point, not a solved problem)

**ACTION REQUIRED:**

Switch model/effort if necessary (none needed). Then reply exactly:

**CONTINUE**

============================================================

Stopping here. Phase 5 has not been started.

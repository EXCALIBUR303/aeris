# AERIS — PX4 reference

## What is PX4? What is SITL?

PX4 Autopilot is the open-source flight-control firmware AERIS uses for
all attitude/rate/position stabilization, state estimation (EKF2), flight
modes, and failsafes. AERIS never reimplements any of this (spec §16.1).

SITL ("Software In The Loop") is PX4 compiled for the host machine's own
CPU, talking to a physics simulator (Gazebo) instead of real sensors and
motors, over the same interfaces it would use on real hardware.

## Pin

See `configs/versions.lock.yaml` for the exact pinned tag, commit, and the
Phase 1 compatibility patches (`third_party/patches/px4/`) needed to build
it on this class of Mac. PX4 is external and pinned (ADR-0002) — it lives
outside this repo (`~/aeris-deps/PX4-Autopilot` by default; see
"Resolving PX4's location" below).

## Resolving PX4's location (`aeris.simulation.px4_paths`)

Spec §8.3 precedence, implemented in `resolve_px4_dir()`:

1. the `AERIS_PX4_DIR` environment variable
2. `px4_dir:` in `configs/local.yaml` (gitignored — copy from
   `configs/local.example.yaml`)
3. the default, `~/aeris-deps/PX4-Autopilot`

`Px4Layout` (also in `px4_paths.py`) resolves every path AERIS needs from
there: the built binary, its default working directory (`rootfs/`), the
Gazebo worlds/models directories, the built Gazebo plugins, and the
Gazebo server config. `Px4Layout.ensure_built()` raises `Px4NotFoundError`
with an actionable message if PX4 isn't checked out or built — **AERIS
never builds PX4 automatically** (spec §8.3); building is the one-time
manual step documented in `docs/mac-setup.md`.

## How AERIS launches PX4 SITL + Gazebo (`aeris.simulation.launcher`)

Read directly from `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim` in the
pinned checkout (Phase 3) rather than assumed from documentation, because
the actual mechanism has details worth getting right:

1. **AERIS launches Gazebo itself**: `gz sim --verbose=N -r -s
   <world>.sdf`, with `PX4_GZ_STANDALONE=1` telling PX4 not to launch its
   own copy. This is a deliberate architecture choice (spec §17.2) — AERIS
   owns the Gazebo server's process lifecycle for tracked-PID shutdown,
   rather than trusting PX4 to manage a process it spawned as a detached
   background job.
2. **AERIS waits for the world to be ready**, using the *exact* check PX4
   itself uses internally: `gz service -i --service
   "/world/<world>/scene/info"` containing `"Service providers"`. Not a
   different, invented check — the same one, so "ready" means the same
   thing to AERIS as it does to PX4.
3. **AERIS launches the built `px4` binary.** PX4 handles spawning its own
   vehicle model into the already-running world and starting its internal
   `gz_bridge` module — that part is PX4's job (`gz service -s
   ".../create"` with an inline `<include>` of the model's SDF), not
   reimplemented in AERIS.
4. **AERIS waits for a MAVLink heartbeat**, then a healthy
   `ESTIMATOR_STATUS` (EKF2 alive and reporting; see
   `aeris.simulation.launcher.readiness` for exactly what "healthy" means
   at this phase versus the stricter convergence gate later phases will
   want before arming).

PX4's own `main.cpp` `chdir()`s to `<binary_dir>/rootfs` by default
regardless of the invoking process's working directory (verified by
reading `platforms/posix/src/px4/common/main.cpp` in Phase 3) — AERIS's
launcher relies on this rather than passing an explicit `-w`.

## MAVLink ports

See `configs/vehicle/px4_sitl.yaml` for the full reference table and
`aeris.simulation.launcher.ports.instance_ports()` for the tested
implementation — both transcribed directly from
`ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`. The port AERIS's own
`aeris.vehicle` adapters connect to (Phase 4+) is the **offboard** link:
local `14580 + instance`, remote `14540 + instance` — this is what
`wait_for_px4_heartbeat()` listens on.

## Parameters

`aeris.simulation.launcher.params` parses a simple `NAME VALUE` text
format (AERIS's own, documented in `configs/vehicle/px4_params/`'s files —
not a reverse-engineering of QGC's `.params` export format, which wasn't
worth matching for Phase 3's needs) and applies it over MAVLink
`PARAM_SET`, verifying PX4's `PARAM_VALUE` ack. See
`configs/vehicle/px4_params/sitl_base.params` for AERIS's baseline
(`COM_RC_IN_MODE=4`, "disable manual control," since SITL has no RC
transmitter — the standard recommendation for a companion-computer-only
setup).

## Known open issues carried from Phase 1

- **MAVSDK-Python 4.0.0's native binding segfaults** on `Mavsdk(Configuration(...))`
  construction on this machine. `aeris.simulation.launcher.readiness` uses
  **pymavlink** exclusively for exactly this reason — it's validated
  working. Phase 4 (Vehicle interface) owns re-evaluating MAVSDK.
- **An MAV_CMD_COMPONENT_ARM_DISARM attempt in Phase 1 was rejected**
  (`MAV_RESULT_TEMPORARILY_REJECTED`) for an undiagnosed reason.
  `COM_RC_IN_MODE=4` (above) is a plausible, not-yet-confirmed contributing
  fix, applied in Phase 3's baseline params either way since it's
  independently well-justified. Phase 5 owns diagnosing arming properly.

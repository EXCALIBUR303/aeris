# ADR-0003: MAVSDK-Python as the primary vehicle transport, pymavlink secondary

**Status:** Accepted (Phase 0), **at risk** — see Phase 1 finding below
**Spec reference:** AERIS_TECHNICAL_SPEC.md §8.5, §15

## Context

PX4 documents MAVSDK as its recommended offboard API, with pymavlink as a
lower-level widely-used alternative. AERIS needs one transport-agnostic
`VehicleInterface` behind which the actual wire protocol is an
implementation detail (autonomy code never sees it).

## Decision

`Px4MavsdkAdapter` is the primary implementation; `Px4PymavlinkAdapter` is
secondary, used for messages MAVSDK lacks. `Ros2Px4Adapter` is reserved for
the Linux fallback only (see ADR-0005). Phase 4 runs a measured comparison
(telemetry rate, setpoint latency, capability coverage) before finalizing.

## Consequences — updated by the Phase 1 finding

MAVSDK-Python 4.0.0 turned out to be a near-total rewrite: a native
ctypes-loaded `libcmavsdk.dylib` instead of the old gRPC + `mavsdk_server`
subprocess model, with a changed construction API. On this machine,
`Mavsdk(Configuration(...))` **segfaults** on construction (reproduced
3x). `pymavlink` was validated instead: full heartbeat/telemetry/command-ack
round trip against running PX4 SITL. Phase 4 must re-test MAVSDK against a
newer point release before committing to it as primary per this ADR; if the
crash persists, this ADR should be revised to make pymavlink primary. See
`docs/phase_reports/phase-1.md` and `configs/versions.lock.yaml` →
`mavsdk_python` for the full repro.

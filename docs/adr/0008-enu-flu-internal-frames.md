# ADR-0008: ENU/FLU internal frames; NED/FRD only inside the PX4 adapter

**Status:** Accepted (Phase 0)
**Spec reference:** AERIS_TECHNICAL_SPEC.md §19

## Context

PX4 and MAVLink use NED (position) / FRD (body) conventions. Gazebo,
REP-103, and most robotics/mapping tooling use ENU / FLU. AERIS's mapping,
perception, and learning code has no reason to think in NED.

## Decision

All AERIS-internal frames are ENU (world/odometry/map) and FLU (body).
NED/FRD conversions happen exactly once, at the PX4 vehicle adapter
boundary (`aeris/vehicle/px4_mavsdk` or `px4_mavlink`) — nowhere else in
the codebase performs this conversion. `aeris.core.frames` (Phase 4) is the
single place the fixed rotation matrices and yaw-offset formulas live, with
property-based round-trip tests.

## Consequences

Every non-vehicle AERIS package (mapping, perception, learning, frontend
via the ENU→three.js mapping) works in one consistent, natural frame.
Requires care in Phase 4/5 to get the one conversion boundary exactly
right, including a live SITL test (command +x ENU velocity, verify PX4
reports the matching NED velocity) — a frame bug there would silently
corrupt everything downstream.

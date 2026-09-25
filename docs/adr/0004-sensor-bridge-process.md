# ADR-0004: Separate Sensor Bridge process for Gazebo Python bindings

**Status:** Accepted (Phase 0)
**Spec reference:** AERIS_TECHNICAL_SPEC.md §7.2, §18.4

## Context

Gazebo's Python bindings (`gz.transport13`, `gz.msgs10`) are built against
whatever Python Homebrew's `gz-harmonic` formula links against, which is
independent of — and likely a different version from — the Python
environment AERIS's own ML/autonomy code runs in.

## Decision

A dedicated `aeris-sensor-bridge` process runs in the Gazebo-bindings
Python environment, subscribes to sensor and clock topics via
`gz.transport`, and forwards them to the AERIS core process over local IPC
(ZeroMQ + msgpack). Ground truth is published on a separate topic
namespace that the agent-side client never subscribes to, enforced by an
allowlist and a test (spec §17.4).

## Consequences

Removes any need to force AERIS's ML stack and Gazebo's bindings onto the
same Python version — each keeps its own, natural dependency set. Adds one
extra process and IPC hop, budgeted for in the control-loop timing table
(spec §13.4) and implemented starting Phase 8.

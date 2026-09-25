# ADR-0002: PX4 as a pinned external dependency; no firmware modifications

**Status:** Accepted (Phase 0), implemented (Phase 1)
**Spec reference:** AERIS_TECHNICAL_SPEC.md §8.3

## Context

PX4-Autopilot is a multi-gigabyte repository with many submodules. AERIS
needs a specific version of it to build, but should not vendor it, and
should not accumulate ad-hoc behavior changes to flight code over time.

## Decision

PX4 lives outside the AERIS repo (default `~/aeris-deps/PX4-Autopilot`,
configurable via `AERIS_PX4_DIR`). The exact pin (tag + commit SHA, plus
Gazebo/MAVSDK/QGC versions) is recorded in `configs/versions.lock.yaml`.
Configuration goes through PX4 parameters, environment variables, and
custom worlds/models — never firmware source edits — except for the
minimal, documented, build-system-only patches under
`third_party/patches/px4/`, applied via `git apply` and never touching
flight-control, estimation, or simulation *behavior*.

## Consequences

Phase 1 needed 9 such patches (see `third_party/patches/px4/README.md`) to
get the pinned version building against the current macOS toolchain — all
narrowly scoped, verified to `git apply --check` cleanly, and documented
with the exact symptom each one fixes. Any other machine setting up AERIS
must apply the same patches; this is intentional and inspectable, rather
than a live, drifting fork of PX4.

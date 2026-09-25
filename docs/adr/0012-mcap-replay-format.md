# ADR-0012: MCAP replay format with detail levels; replay = playback

**Status:** Accepted (Phase 0)
**Spec reference:** AERIS_TECHNICAL_SPEC.md §39

## Context

Every evaluated mission needs to be replayable for debugging, evidence,
and demonstration, without either requiring a ROS installation or storing
unbounded raw sensor streams by default.

## Decision

MCAP (an open, self-describing log container; pure-Python reader/writer,
no ROS dependency) is the replay format, with configurable channel detail
levels L0 (summary) through L4 (full raw sensors, debug-only). Replay is
**pure data playback**, keyed by `t_sim_s`, never re-simulation — so it is
exact by construction, unlike re-running a seed (which is only
statistically reproducible per ADR/spec §40's determinism classes).

## Consequences

Replay Studio (Phase 30) can be built without any ROS/Foxglove dependency,
though Foxglove Studio can still open the same files for independent
debugging. Ground-truth channels are flagged `privileged` in the schema and
shown in Replay Studio only behind an explicit, labeled toggle (spec §46.5,
§48).

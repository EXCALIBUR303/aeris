# AERIS product & experience audit

Audited 2026-10-26 against `b55ca2f` before implementation.

## Product and people

AERIS is a research platform for simulated drone autonomy, mapping, exploration, reproducible experiments, and PPO. Researchers need to understand where a vehicle went, what it observed, how much it explored, and whether an experiment is valid. Portfolio visitors need a safe introduction with no simulator installation.

## Stack and current UI

Actual implementation reaches Phase 14: Python 3.12, uv, Pydantic, NumPy/Numba, PyTorch/Gymnasium, pymavlink, MCAP; optional PX4 SITL and Gazebo Harmonic. CLI workflows include simulation, vehicle telemetry, missions, world generation and training. Inspected README, technical spec, pyproject, CI, architecture, ADR-0013, WorldSpec, provenance enums, configs and Phase 14 report. README and architecture summary still describe Phase 2 and are stale. There is no existing frontend or backend application. UI/API phases 25–26 are planned.

## Experience problems

- Hierarchy: research outputs need domain knowledge; no mission summary or guided entry.
- Responsiveness: no browser, mobile or embeddable experience.
- Feedback: no linked playback, map/timeline synchronization or browser loading/error/empty states.
- Accessibility: no browser keyboard/focus semantics, plot alternatives, reduced motion or contrast verification.
- Performance: no UI baseline. A full 3D simulator would impose unnecessary GPU/loading costs for a portfolio.
- Integrity: illustrative examples must remain distinct from measured scientific evidence, including simulation tier and provenance.

## Unfinished work

Real server/WebSocket transport, recorded-run API, sensor imagery, MCAP conversion, mission command authorization and full Control Center are future work. Higher roadmap phases (learned navigation/exploration, detection, SAR, language missions) remain unimplemented. PX4/Gazebo tests need external services. Browser tests and embedding docs were absent. Known phase-14 limitations include no checkpoint retention, trusted-local pickle environment snapshots and an observation hash that omits episode-time scaling. These are not UI fixes and remain untouched.

## This change

Separate Next.js showcase: explicitly synthetic mission playback, linked 2D map/metrics/events, comparisons, scenarios, exports, themes and keyboard controls. It is not a connected flight console or validated experiment. No Python logic or API contracts change. See `frontend/QA.md` for evidence and remaining limits.

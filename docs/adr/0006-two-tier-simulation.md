# ADR-0006: Two-tier simulation — FastSim for learning, PX4+Gazebo for evaluation

**Status:** Accepted (Phase 0)
**Spec reference:** AERIS_TECHNICAL_SPEC.md §17

## Context

PX4 SITL + Gazebo Harmonic (Tier H) runs at best a few times real-time,
one vehicle at a time, on this hardware — nowhere near enough throughput
for PPO training (which needs thousands of steps/second). Training
directly in Tier H is infeasible; skipping high-fidelity simulation
entirely would invalidate every learned-autonomy result.

## Decision

Two tiers: **Tier F (AERIS FastSim)** — a vectorized, geometry-based
simulator with dynamics/sensor models system-identified from Tier H step
responses — is used for RL training and fast ablations. **Tier H (PX4 SITL
+ Gazebo)** is used for classical baselines and **all headline
evaluation**; a learned-autonomy result is only reported as an AERIS result
if it was evaluated in Tier H. Both tiers are generated from one
`WorldSpec` (ADR-0007), and the gap between them is itself measured (RQ4).

## Consequences

Makes RL training on this hardware feasible while keeping every reported
claim honest about which tier produced it (every result, chart, and
manifest is tagged `Tier: F` or `Tier: H`). Adds real engineering cost:
Phase 13 must build and validate FastSim's fidelity (parity tests,
randomization) before any learning phase can be trusted.

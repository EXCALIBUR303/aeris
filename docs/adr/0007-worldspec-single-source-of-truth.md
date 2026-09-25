# ADR-0007: WorldSpec as the single source of world truth for both tiers

**Status:** Accepted (Phase 0)
**Spec reference:** AERIS_TECHNICAL_SPEC.md §9.3

## Context

With two simulation tiers (ADR-0006) plus an evaluator that needs ground
truth, there's a real risk of authoring world geometry three times and
having it drift out of sync — silently invalidating fidelity claims.

## Decision

A single typed, seed-reproducible `WorldSpec` (bounds, obstacles, targets,
spawn poses, family, split) is rendered by three independent renderers:
`SdfRenderer` (Gazebo `.sdf`), `FastSimRenderer` (voxel/primitive geometry
for training), and `GroundTruthRenderer` (evaluator-only occupancy/labels).
A consistency test checks the SDF-rendered and FastSim-rendered occupancy
agree within voxel resolution.

## Consequences

World generation logic is written once (Phase 9) and consumed by every
later phase. The consistency test becomes a required gate for Phase 9 and
a standing regression check any time a generator changes.

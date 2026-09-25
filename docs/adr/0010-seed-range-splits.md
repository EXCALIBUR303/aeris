# ADR-0010: Seed-range splits + held-out world family + split guards

**Status:** Accepted (Phase 0)
**Spec reference:** AERIS_TECHNICAL_SPEC.md §33.2

## Context

Generalization claims ("the model generalizes") are only valid if training,
validation, and test worlds are genuinely disjoint, and if there's a
held-out world *family* (not just held-out seeds of a seen family) to test
true out-of-distribution generalization.

## Decision

Fixed, disjoint seed ranges: train `[0, 10000)`, val `[10000, 10200)`,
test-ID `[20000, 20100)`, test-OOD `[30000, 30100)` (family F4, "collapsed
structure," never used in training, tuning, or model selection). Ranges and
a hash of generated test `WorldSpec`s are frozen in
`configs/worlds/splits.lock`. Training and tuning entry points programmatically
refuse to generate worlds from ranges they aren't authorized for, and this
is unit-tested (Phase 9).

## Consequences

A generalization claim in a phase report can be mechanically audited: scan
every training/tuning manifest for a config hash and confirm no test-range
seed appears (Phase 34's final audit does exactly this). Splits are frozen
once set — changing them later requires a documented spec change (§57.3),
not a quiet edit.

# ADR-0015: Hierarchical learned exploration via masked subgoal actions

**Status:** Accepted (Phase 0), to be implemented in Phase 19
**Spec reference:** AERIS_TECHNICAL_SPEC.md §26.2

## Context

Learned exploration needs an action space that is both (a) expressive
enough for the learned policy to meaningfully outperform classical frontier
methods (RQ1), and (b) safe by construction — no learned component should
ever be able to command something the deterministic safety layer can't
shield.

## Decision

The exploration policy outputs a discrete, egocentric, **masked** subgoal
(12 bearings × 2 ranges + a rotate-in-place scan = 25 actions), snapped to
the nearest reachable known-free map cell. Action masking uses only the
agent's own map (legitimate information — no privileged data). The chosen
subgoal is executed by the same A* planner, path follower, and collision
shield used by the classical baselines — the policy never outputs raw
velocity or lower-level commands for exploration.

## Consequences

Guarantees the learned exploration policy is exactly as safe as the
classical baselines it's compared against (same shield, same follower),
which is required for RQ1 to be a fair comparison. Keeps the learned
component's job narrowly scoped to "where to go next," which is also the
right level of abstraction for the partial-observability/memory research
question (RQ2).

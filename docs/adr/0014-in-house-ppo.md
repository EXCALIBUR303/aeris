# ADR-0014: In-house PPO (CleanRL-style), with an SB3 dev-only sanity reference

**Status:** Accepted (Phase 0), to be implemented in Phase 14
**Spec reference:** AERIS_TECHNICAL_SPEC.md §27

## Context

AERIS's RL results need to be explainable in detail (spec §56, viva
requirement: "What is PPO? What are actor and critic? What is GAE?") and
scientifically auditable — every design choice (truncation-vs-termination
bootstrapping, advantage normalization, clipping) needs to be inspectable
and testable, not hidden behind a framework's internals.

## Decision

PPO is implemented in-house in PyTorch (~600–900 lines), following the
well-documented "37 implementation details of PPO" conventions
(CleanRL-style), with the mathematical test suite in spec §27.4 as a hard
requirement (GAE against hand-computed references, truncation-bootstrap
correctness, clip-region gradient checks, a memory-only known-answer task).
Stable-Baselines3 is a **dev-only** dependency used solely as a sanity
check against a reference implementation on one toy task — never shipped
as, or relied on for, an actual AERIS result.

## Consequences

Every PPO design decision is inspectable, citable, and independently
tested — the basis for the project's academic explainability requirement.
Costs more implementation and testing effort up front (Phase 14 is one of
the higher-effort phases) than adopting an existing RL library wholesale.

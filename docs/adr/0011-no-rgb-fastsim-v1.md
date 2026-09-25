# ADR-0011: No RGB in FastSim V1; RQ3 gated on a feasible render path

**Status:** Accepted (Phase 0)
**Spec reference:** AERIS_TECHNICAL_SPEC.md §5, §17.1

## Context

Photorealistic (or even simple rasterized) RGB rendering at training
throughput is likely infeasible on this hardware within FastSim's
vectorized-NumPy/Numba design, while depth and LiDAR are naturally handled
by raycasting the same `WorldSpec` geometry FastSim already needs.

## Decision

FastSim (Tier F) provides raycast depth and LiDAR with a fitted noise
model, but **no RGB** in V1. RQ3 (does depth beat RGB for local
navigation?) is explicitly gated: Phase 16 measures whether a rendered-RGB
training path is actually feasible (Gazebo RGB throughput, or a FastSim
flat-shaded rasterizer) and makes a go/no-go call with numbers. If it's a
no-go, RQ3 is reported as "not executed," with the reason — never silently
dropped or faked.

## Consequences

Keeps FastSim's core design (depth/LiDAR raycasting) simple and fast.
Accepts that RQ3 may end up unanswered in V1; that's an acceptable,
honestly-reported outcome per spec §57 rather than a reason to fabricate an
RGB training capability the hardware can't support.

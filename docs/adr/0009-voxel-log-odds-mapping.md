# ADR-0009: 3D voxel log-odds + 2D altitude-band projection for V1 mapping

**Status:** Accepted (Phase 0)
**Spec reference:** AERIS_TECHNICAL_SPEC.md §21

## Context

AERIS needs a spatial representation for planning, exploration, and
learned-policy observations. Full 3D multi-altitude exploration, semantic
voxels, and octree-based representations (OctoMap-style) are all viable but
add complexity V1's fixed-altitude-band mission profile doesn't need.

## Decision

A sparse, block-hashed 3D voxel log-odds occupancy grid (0.15–0.25 m
resolution, Numba-accelerated ray integration) is the ground representation.
The drone explores within a fixed altitude band; a 2D projection of that
band (occupied if any voxel in-band is occupied, inflated by vehicle
radius) feeds A*, frontier detection, and learned-policy map crops.

## Consequences

Simple, fast enough on CPU, and sufficient for every V1 mission profile.
Explicitly deferred: full 3D multi-altitude exploration, semantic voxels,
and octree structures — revisit only if a later phase's research question
genuinely needs them (no maintained, native Apple-Silicon octree binding
was found to justify the dependency as of Phase 0).

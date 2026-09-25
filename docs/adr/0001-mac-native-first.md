# ADR-0001: Mac-native first; isolate Linux-only components

**Status:** Accepted (Phase 0), validated (Phase 1)
**Spec reference:** AERIS_TECHNICAL_SPEC.md §7, §12

## Context

PX4 officially supports native macOS development with Gazebo Harmonic on
Apple Silicon, but several capabilities (multi-vehicle Gazebo simulation,
the official ROS 2 workflow) are documented Linux-only. The development
machine is a single, fanless MacBook Air.

## Decision

Build and run the AERIS core (autonomy, learning, backend, frontend) and
PX4 SITL + Gazebo natively on macOS. Only move a component off the Mac
(Docker container, Ubuntu ARM64 VM, or remote Linux) when it hits a
documented Linux-only requirement, and only that component — never migrate
the whole project.

## Consequences

Phase 1 proved this works, including the highest-risk item (headless
rendering sensors — depth camera, LiDAR). Phase 33 (multi-drone) will need
a Linux fallback for Gazebo multi-vehicle simulation; everything else stays
native. Keeps the development loop fast (no VM/container overhead) for the
vast majority of phases.

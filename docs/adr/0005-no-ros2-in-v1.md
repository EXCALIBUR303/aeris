# ADR-0005: No ROS 2 dependency in V1; adapter seam reserved

**Status:** Accepted (Phase 0)
**Spec reference:** AERIS_TECHNICAL_SPEC.md §11

## Context

PX4's official ROS 2 workflow targets Ubuntu 24.04 + ROS 2 Jazyy (or 22.04
+ Humble) — not macOS. ROS 2 itself lists macOS below Tier 1 for Jazzy.

## Decision

Nothing under `aeris/` imports `rclpy` in V1. `VehicleInterface`,
`SensorSource`, `MapPublisher`, and `PoseSource` are defined as protocols
specifically so a future `aeris/adapters/ros2/` package — used only in the
Linux fallback (ADR-0001) — can implement them later without touching
autonomy code, if a phase genuinely needs a ROS-only capability (lower
latency `px4_ros_com`, ROS-centric SLAM, external tool interop) and
justifies the added Linux dependency in its phase report.

## Consequences

Keeps V1 fully Mac-native. Deliberately defers `px4_ros_com`'s lower
latency and any ROS-centric SLAM tooling (spec §23) unless a later phase
proves it's actually needed.

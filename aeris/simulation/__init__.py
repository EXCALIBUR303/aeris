"""aeris.simulation — PX4 SITL + Gazebo Harmonic process orchestration (Tier H).

Spec §17.2. PX4-Autopilot itself is an external, pinned dependency (spec
§8.3, ADR-0002) — nothing here vendors or modifies PX4 source; this package
only launches the already-built binaries and Gazebo, and talks to them over
their normal external interfaces (env vars, gz CLI, MAVLink).
"""

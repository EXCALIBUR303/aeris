# AERIS — Phase 0 Report

============================================================
AERIS — PHASE 0 COMPLETE
============================================================

**PHASE:** 0 — Architecture and research specification.

**IMPLEMENTED:**
The authoritative spec, `AERIS_TECHNICAL_SPEC.md` (v1.0.0, 3,172 lines).
- Covers all 58 required sections and a 35-phase roadmap (Phases 0–34).
- Every phase has all 19 required fields (Objective, Why it exists, Dependencies,
  Files/modules, Implementation tasks, Research considerations, Tests, Validation
  gate, Expected output, Known risks, Failure/rollback, Recommended model, Recommended
  effort, Why this model, Why this effort, Recommended skills/connectors, Opus
  escalation conditions, When not to use Opus, Stop condition).
- Appendices: A (environment snapshot), B (sources), C (15 ADRs), D (viva question
  map), E (glossary).

**FILES CREATED:**
- `~/Claude/aeris/AERIS_TECHNICAL_SPEC.md`
- Project memory entry (`aeris-project.md` in the assistant's persistent memory)

**FILES MODIFIED:** none (memory index only).

**TESTS RUN:** Structural check of the spec (section count, phase count, field
completeness per phase).

**TEST RESULTS:** PASS — 58/58 sections, 35/35 phases, all 19 fields present in
every phase.

**SIMULATION RESULTS:** None (not authorized in Phase 0).

**EXPERIMENT RESULTS:** None.

**PROBLEMS FOUND (research + machine inspection):**
1. Open PX4 build bug on macOS: PX4/PX4-Autopilot#27026 (April 2026) — gstreamer
   linking, Qt5 plugin prefix, and a `-Wdouble-promotion` error building the Gazebo
   modules on macOS/Apple Silicon.
2. Conflicting PX4 docs: the Gazebo overview page says "not available for
   Windows or macOS" while the macOS setup page says Gazebo Harmonic is supported
   and CI-tested on Apple Silicon runners. The macOS setup page is treated as
   authoritative (newer, dev-team-supported).
3. Multi-vehicle Gazebo simulation and PX4's official ROS 2 workflow (Ubuntu
   24.04 + ROS 2 Jazzy) are Linux-only.
4. PX4's built-in Collision Prevention (`CP_DIST`) only works in Position mode,
   not Offboard — AERIS needs its own collision shield.
5. Machine constraints: fanless MacBook Air (Apple M5, 16 GB RAM, throttles under
   sustained load); macOS 27.2 (newer than any documented PX4/Gazebo
   configuration); `cmake`/`ninja` not installed; default `python3` is 3.14
   (too new for parts of the ML stack; `python3.11` available as fallback).

**PROBLEMS FIXED (in the design):**
- Two-tier simulation (FastSim for RL training, PX4 SITL + Gazebo for all
  headline evaluation) to work around infeasible training throughput in
  high-fidelity sim on this hardware.
- Separate Sensor Bridge process to isolate the Homebrew-Python Gazebo bindings
  from the AERIS ML Python environment.
- Deterministic collision shield in the Safety layer (Offboard mode has no
  native PX4 collision protection).
- Ground-truth/agent-observation separation enforced by import contracts,
  provenance tagging, and a privileged evaluator-only channel.

**KNOWN LIMITATIONS:** Every "ASSUMPTION" in the spec is unverified until its
named phase runs. Highest-impact: whether Gazebo's depth camera and LiDAR
produce data in headless mode on this Mac (spec §9.4) — the first thing Phase 1
must test.

**REMAINING RISKS:** PX4 build on macOS 27.2/clang 21; rendering-sensor support
headless; PyTorch MPS availability; Tier-H evaluation wall-clock cost (est.
12–30 h for the flagship RQ1 evaluation); thermal throttling.

**VALIDATION GATE:** PASS.

**CURRENT AERIS STATUS:** Architecture specified. No code exists; no autonomy
level reached.

============================================================
NEXT PHASE
============================================================

**NEXT PHASE:** 1 — Mac / PX4 / Gazebo compatibility spike.
**RECOMMENDED MODEL:** Sonnet.
**RECOMMENDED EFFORT:** High.
**SWITCH REQUIRED:** YES (Opus → Sonnet).
**USE OPUS ONLY IF:** depth camera and LiDAR cannot run headless on any option
(native, Docker, or VM), forcing a redesign of the sensor strategy.

**ACTION REQUIRED:** User replied **CONTINUE** (and switched to Sonnet 5) on
2026-09-25. Phase 1 authorized.

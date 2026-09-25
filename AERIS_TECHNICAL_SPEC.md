# AERIS — Autonomous Drone Intelligence Research Platform

**Perceive. Navigate. Explore. Decide.**

## AERIS_TECHNICAL_SPEC.md — Authoritative Technical Specification

| Field | Value |
|---|---|
| Document status | **AUTHORITATIVE** (Phase 0 output) |
| Spec version | 1.0.0 |
| Created | 2026-09-25 |
| Phase | 0 — Architecture + research specification |
| Author model / effort | Opus 5.5 / High |
| Change control | Changes to this document require an explicit spec-change note in the phase report of the phase that makes them (§57.3). Phases may *refine* sections; they may not silently contradict them. |

### How to read this document

- **VERIFIED** means the claim was checked against current authoritative documentation or the actual machine during Phase 0 (sources are listed in Appendix B).
- **ASSUMPTION** means the claim is believed but unproven. Every assumption names the phase that must validate it.
- **DECISION** means an architectural choice. Each one has a short justification and appears in the ADR index (Appendix C).
- **MUST / MUST NOT / SHOULD / MAY** follow RFC 2119 meaning.
- When code and this document disagree, the code is wrong, unless a phase report records an approved spec change.

---

## Table of contents

1. Executive summary
2. Project vision
3. Scope / non-scope
4. Research objectives
5. Research questions
6. Testable hypotheses
7. Mac hardware / environment strategy
8. PX4 strategy
9. Gazebo strategy
10. QGroundControl strategy
11. ROS 2 strategy
12. Linux fallback strategy
13. Complete system architecture
14. Repository architecture
15. Vehicle interface
16. Flight safety architecture
17. Simulation architecture
18. Sensor architecture
19. Coordinate-frame conventions
20. Perception
21. Mapping
22. Localization
23. SLAM roadmap
24. Classical navigation
25. Classical exploration
26. RL role
27. PPO methodology
28. Memory
29. Learned navigation
30. Learned exploration
31. Target detection
32. Search-and-rescue mission
33. Generalization
34. GPS-denied navigation
35. Moving-target tracking
36. Language missions
37. Multi-drone extension
38. Experiment architecture
39. Replay architecture
40. Reproducibility
41. Metrics
42. Ablations
43. Failure modes
44. Testing
45. Backend
46. Frontend
47. Professional UI/UX strategy
48. Design system
49. Visualization strategy
50. Skills / connectors / tool strategy
51. Complete phase roadmap (per-phase specifications)
52. Model recommendation for every phase
53. Effort recommendation for every phase
54. Validation gate for every phase
55. Opus escalation conditions
56. Computational requirements
57. Documentation strategy
58. Final audit criteria

Appendices: A — Phase 0 environment snapshot · B — Sources · C — ADR index · D — Viva question map · E — Glossary

---

## 1. Executive summary

AERIS is a research platform for **higher-level autonomy of a simulated quadrotor**. PX4 Autopilot is the flight controller. Gazebo (Harmonic) provides physics and sensor simulation. PX4 keeps full responsibility for stabilization, attitude and rate control, and its own failsafes. AERIS adds everything above that layer: vehicle abstraction, safety supervision, perception, mapping, planning, exploration, learned decision-making, mission execution, experiments, replay, and a professional control-center UI.

The **flagship demonstration** is a simulated search-and-rescue mission. The drone takes off, explores an unknown environment without collisions, maps it, detects defined mission targets, estimates where they are, avoids re-searching covered space, and returns to land. The mission produces a replayable report.

The **flagship research question** is whether a learned, memory-equipped, hierarchical exploration policy trained with PPO beats a strong classical frontier-exploration baseline. The comparison is made under identical sensing, identical safety layers and identical evaluation, in held-out environments. The answer is **not assumed**. A negative result is a valid, publishable outcome.

Five findings from Phase 0 shape the architecture:

1. **PX4 officially supports native macOS development with Gazebo Harmonic** on Apple Silicon (VERIFIED, PX4 main docs). The Gazebo build on macOS has an **open build bug**: PX4 issue #27026, April 2026, covering gstreamer linking, the Qt5 plugin prefix and a `-Wdouble-promotion` error. Phase 1 must prove the stack builds on *this* Mac before anything else starts.
2. **Several PX4 + Gazebo capabilities are Linux-only**: multi-vehicle Gazebo simulation (VERIFIED) and the official ROS 2 workflow (Ubuntu 24.04 + ROS 2 Jazzy, VERIFIED). AERIS V1 therefore **MUST NOT depend on ROS 2**. Multi-drone work (Phase 33) needs a Linux fallback.
3. **The development machine is a fanless MacBook Air (Apple M5, 16 GB RAM)**. PX4 SITL + Gazebo runs at most a few times faster than real time, one vehicle at a time. That rules out RL training directly in SITL. **DECISION (ADR-006):** AERIS uses a *two-tier simulation architecture*:
   - a fast, vectorized, geometry-based **AERIS FastSim** for learning;
   - **PX4 SITL + Gazebo** as the high-fidelity environment for validation and evaluation.

   Both tiers are generated from a single `WorldSpec`, and both feed the same observation and action contracts. The sim-to-sim transfer gap is itself measured (RQ4).
4. **Camera, depth and LiDAR data reach AERIS through Gazebo Transport, not MAVLink.** On macOS the Gazebo Python bindings are tied to Homebrew's Python, which will likely differ from the ML environment's Python. **DECISION (ADR-004):** a separate **Sensor Bridge process** isolates that dependency.
5. **Ground truth and agent observations are separated architecturally.** Mechanisms: package boundaries enforced by import contracts, provenance-tagged observation fields, and an evaluator-only ground-truth channel. They are not just a convention.

**Primary vehicle transport (DECISION, ADR-003):** MAVSDK-Python, the PX4-recommended API, speaking MAVLink over UDP. It sits behind a transport-agnostic `VehicleInterface`. A pymavlink adapter is the fallback, and a ROS 2 adapter is reserved for Linux.

**Roadmap:** 35 phases (0–34), each gated by an objective validation gate. Opus 5.5 is used where methodology decides whether results are valid: RL environment, PPO, learned autonomy, SAR design, GPS-denied, UI design, final audit. Sonnet does most engineering. Haiku handles low-risk chores.

---

## 2. Project vision

AERIS should grow step by step into an internal-grade autonomy research platform where every capability:

- **builds on a validated lower capability** (the progression in §2.1);
- **is measurable** (the metrics in §41);
- **is honest about what is learned and what is engineered** (the table in §26.3);
- **can be replayed** (§39).

### 2.1 Capability progression → autonomy levels → phases

| Level | Capability | Phase(s) | Validated by |
|---|---|---|---|
| L0 | PX4 SITL smoke test | 1, 3 | vehicle spawns, telemetry flows |
| L1 | Programmatic flight | 4, 5 | scripted takeoff/hover/land, 20 repeats |
| L2 | Waypoint navigation | 6 | waypoint error & completion metrics |
| L3 | Autonomous takeoff/mission/landing | 6 | full mission state machine runs |
| L4 | Sensor acquisition | 8 | depth/LiDAR/RGB frames with correct frames & timestamps |
| L5 | Classical obstacle avoidance | 10 | collision rate on obstacle suites |
| L6 | Mapping | 11 | map accuracy vs ground truth |
| L7 | Classical exploration | 12 | coverage curves vs random baseline |
| L8 | Learned navigation | 17 | held-out collision/success vs classical |
| L9 | Learned exploration | 18–19 | coverage vs frontier (RQ1) |
| L10 | Visual target detection | 20 | precision/recall/distance |
| L11 | Search-and-rescue | 21 | SAR metrics suite |
| L12 | Generalization | 22 | OOD world-family results |
| L13 | GPS-denied navigation | 23 | ATE/RPE + mission metrics without GPS |
| L14 | Moving-target tracking | 24 | track RMSE, FOV retention |
| L15 | Language-conditioned missions | 32 (optional) | compliance metrics vs unconditioned |
| L16 | Multi-drone | 33 (optional) | team coverage vs single-drone |

### 2.2 What AERIS is not

- It is not a thin wrapper around PX4 missions, a waypoint script, or a dashboard of simulated numbers.
- It is not a low-level controller research project. AERIS does not replace PX4's attitude or rate loops (§16).
- It is not flight-ready software. Physical deployment is out of scope (§3.2).

---

## 3. Scope / non-scope

### 3.1 In scope (V1 = Phases 0–31)

- A single simulated multirotor: PX4 x500 family in Gazebo Harmonic, PX4 SITL, on macOS Apple Silicon.
- High-level commanding: position, velocity and yaw setpoints through PX4 Offboard mode, and PX4-native takeoff and land.
- A deterministic safety supervisor and collision shield, independent of any learned component.
- Sensor pipeline: forward depth camera, 2D LiDAR, RGB camera, IMU and GPS via PX4 telemetry, and PX4 EKF2 state estimate.
- Mapping: 3D voxel log-odds occupancy with a 2D projection for planning.
- Classical baselines: waypoint following, reactive avoidance, A*, frontier exploration, random exploration.
- AERIS FastSim, a training simulator, plus a Gymnasium-compatible environment.
- An in-house PPO implementation (MLP and GRU variants) with mathematical tests.
- Learned local navigation and learned hierarchical exploration.
- Target detection of *defined simulated target classes*, and a search-and-rescue mission.
- Generalization experiments on procedurally generated, split world families.
- GPS-denied navigation with explicitly defined available information.
- Moving-target tracking against scripted simulated targets.
- Experiment tracking, a replay system, a FastAPI backend, and the React control center.

### 3.2 Non-scope (explicit)

| Excluded | Reason |
|---|---|
| Physical drone flight, HITL, real hardware connections | Safety. Simulation is not validated for hardware. The code MUST refuse non-loopback vehicle endpoints (§16.6). |
| Learned motor / attitude / rate control | Safety architecture rule (§16.1). Would need a separate research justification and validation phase that is not planned in V1. |
| Replacing PX4 EKF2 or controllers | PX4's responsibility. |
| Modifying PX4 firmware source | Not needed. PX4 is treated as a pinned external dependency (§8.3). |
| Gazebo Classic, jMAVSim as primary simulator | Obsolete or less capable. Gazebo Harmonic is PX4's current simulator. |
| ROS 2 as a V1 dependency | Officially Linux-only for PX4 (§11). |
| Large foundation models in perception V1 | Principle: small, testable components first (§20). |
| Photorealistic RGB training at scale on the Mac | Rendering throughput is infeasible on this hardware (§56). RGB-heavy RQs are gated (RQ3). |
| Cloud experiment services (W&B etc.) | Local-first and free. Unneeded accounts. |

### 3.3 Optional / advanced (only after V1 audit gates)

- Phase 32: language-conditioned missions.
- Phase 33: multi-drone (requires the Linux fallback).
- A full SLAM stack (§23).
- Learned obstacle avoidance beyond the shielded local policy.

---

## 4. Research objectives

- **O1 — Correct autonomy stack.** Build a layered autonomy stack on top of PX4 with clean separation of flight control, safety and decision-making, and show that it completes missions safely in SITL.
- **O2 — Strong baselines.** Implement classical navigation and exploration baselines strong enough that beating them means something.
- **O3 — Learned exploration.** Train a learned, partially observable exploration policy and compare it against the baselines under controlled, fair conditions.
- **O4 — Transfer.** Quantify the sim-to-sim gap between the training simulator (FastSim) and high-fidelity SITL, and test whether randomization reduces it.
- **O5 — Mission-level evaluation.** Evaluate search-and-rescue performance end to end: exploration + perception + mission logic.
- **O6 — Robustness.** Measure generalization to unseen world families, and degradation under GPS denial.
- **O7 — Reproducible science.** Every reported number traces to a run manifest (code SHA, config, seeds, versions) and can be regenerated, within the determinism limits stated in §40.

---

## 5. Research questions

The prompt's candidate questions are refined here so that each one is **answerable with a controlled comparison on this hardware**. Each RQ names its independent variable, controls, primary metric and phase.

**RQ1 (primary) — Learned vs classical exploration.**
Under identical sensing (forward depth + 2D LiDAR + PX4 EKF2 pose), an identical safety shield, an identical low-level planner and follower, and an identical time budget: does a PPO-trained hierarchical exploration policy reach higher **coverage at fixed time** in **unseen** procedurally generated environments than (a) nearest-frontier and (b) utility-based frontier exploration, evaluated in **PX4 SITL + Gazebo**?
- IV: exploration strategy.
- Controls: sensors, shield, planner, speed limits, worlds and starts (paired).
- Primary metric: C(180 s).
- Phases 12, 19, 22.

**RQ2 — Memory vs explicit map.**
In a 2×2 ablation (GRU memory yes/no × egocentric-map input yes/no), how much does recurrence reduce the revisited-space ratio and improve coverage? Does its benefit disappear when an explicit map is supplied?
- Primary: revisit ratio R, C(180 s).
- Phases 18, 19.

**RQ3 — Depth vs RGB (feasibility-gated).**
For learned local navigation, does depth input yield a lower collision rate than RGB input?
- *Gate:* only runs if a rendered-RGB training path at adequate throughput exists (remote Linux/GPU rendering, or a validated FastSim RGB rasterizer). Otherwise it is **reported as not executed**, with the reason.
- Phase 17.

**RQ4 — Sim-to-sim transfer.**
How much does policy performance drop from FastSim to PX4 SITL + Gazebo? Does dynamics and sensor randomization in FastSim, identified from SITL (Phase 13), shrink the gap?
- Primary: ΔC(180 s) and Δcollision rate, FastSim vs SITL, on identical WorldSpecs.
- Phases 13, 17, 19.

**RQ5 — Generalization.**
How does performance degrade from in-distribution test worlds to a **held-out world family** never seen in training or validation? Does the learned-vs-frontier ranking hold?
- Phase 22.

**RQ6 — Target-aware search.**
In SAR missions, does adding target-detection reward and observation channels reduce **time-to-first-target** and raise **target recall at T** without lowering coverage, compared with coverage-only exploration plus the same detector?
- Phase 21.

**RQ7 (advanced) — GPS denial.**
When GPS is removed and position comes only from explicitly defined non-GPS sources (optical flow + rangefinder via PX4 EKF2, and/or AERIS depth odometry), how much do localization error (ATE/RPE) and mission metrics degrade versus the GPS-aided baseline?
- Phase 23.

Questions from the prompt that were **deferred or merged**, with reasons:

- "Does explicit mapping improve exploration efficiency?" is answered by RQ2's map-input factor and the classical frontier baseline.
- "Can RL improve high-level exploration over classical strategies?" is RQ1.
- "How robust is navigation when GPS becomes unavailable?" is RQ7.
- "Can semantic perception improve target-search efficiency?" is RQ6. Semantic segmentation itself is optional.

---

## 6. Testable hypotheses

The hypotheses are written as H0/H1 pairs with **pre-registered** decision rules. Thresholds are fixed now and may only be changed in a pre-registration update *before* test-world evaluation (§38.5).

**Statistical protocol (applies to all):**
- Evaluation is paired: every method runs on the same (world seed, start pose) pairs.
- Learned methods use ≥ 5 independent training seeds. Classical methods use ≥ 3 repeated runs per pair in SITL to absorb simulator nondeterminism.
- Reported statistics: mean, IQM, and 95 % **paired bootstrap CI** of the difference (10 000 resamples).
- Multiple comparisons within one RQ use the Holm–Bonferroni correction.
- "Meaningful" means the CI excludes 0 **and** the point estimate exceeds the stated minimum effect.

| ID | H0 (null) | H1 (alternative) | Primary metric | Min. meaningful effect | Phase |
|---|---|---|---|---|---|
| H0.1 (sanity) | Frontier ≤ random | Frontier > random coverage | C(180 s) | +10 pp | 12 |
| H1 | Learned-GRU ≤ best frontier | Learned-GRU > best frontier on in-distribution test worlds | C(180 s) | +5 pp | 19 |
| H1b | Learned collision rate ≥ frontier + 2 pp | Learned is not less safe (non-inferiority) | episode collision rate | margin 2 pp | 19 |
| H2a | GRU does not reduce revisit ratio (no map input) | GRU reduces R | R | −0.05 absolute | 18 |
| H2b | GRU benefit with map = GRU benefit without map | Benefit shrinks when the map is provided (interaction) | ΔC interaction | 3 pp | 18 |
| H3 | Depth collision rate ≥ RGB | Depth < RGB | collision rate | −5 pp | 17 (gated) |
| H4 | Randomization does not reduce transfer gap | Gap shrinks | |ΔC| FastSim→SITL | 3 pp | 19 |
| H5 | OOD drop of learned ≤ OOD drop of frontier | Learned drops more (a *risk* hypothesis, tested honestly either way) | ΔC ID→OOD | 5 pp | 22 |
| H6 | Target-aware ≥ coverage-only in time-to-first-target | Target-aware reaches first target faster | TTFT (median) | −15 % | 21 |
| H7 | GPS-denied ATE ≤ threshold | Characterization, not superiority: report ATE, RPE, mission-success drop | ATE RMSE | descriptive | 23 |

Any outcome, including H0 not rejected, is reported plainly (§57, "No fake success").

---

## 7. Mac hardware / environment strategy

### 7.1 Recorded machine (VERIFIED, Phase 0 inspection, 2026-09-25)

| Item | Value |
|---|---|
| Model | MacBook Air, `Mac17,3` (**fanless**: sustained loads thermally throttle) |
| Chip | Apple M5, arm64 |
| CPU | 10 cores (4 performance "Super" + 6 efficiency) |
| GPU | 8-core Apple GPU, Metal 4 |
| RAM | 16 GB unified |
| macOS | 27.2 (build 26B5091g) |
| Disk free | ~169 GiB of 460 GiB |
| Homebrew | `/opt/homebrew` (native arm64 prefix) |
| Python | default `python3` = 3.14.7 (Homebrew); `python3.11` = 3.11.16 available |
| PyTorch | not installed |
| Xcode CLT | installed (`/Library/Developer/CommandLineTools`), Apple clang 21.0.0 |
| CMake / Ninja | **not installed** |
| Git | 2.54.0 (Apple) |
| Gazebo (`gz`) | not installed |
| QGroundControl | not installed |
| Other | Docker 29.8.0 (Docker Desktop), `uv` installed, Node 26.9.0 / npm, Ollama, `gh` authenticated (account EXCALIBUR303) |

### 7.2 Strategy

- **DECISION (ADR-001): Mac-native first.** PX4 SITL + Gazebo Harmonic run natively (PX4 officially supports this, with Apple Silicon CI). Linux is used only where a capability is documented Linux-only or fails natively after Phase 1 diagnosis (§12).
- **Two Python environments (DECISION, ADR-004):**
  1. **AERIS environment:** `uv`-managed venv on **Python 3.12**. ASSUMPTION, verify in Phase 2: 3.12 has arm64 wheels for torch, mavsdk, numba, opencv-python-headless and fastapi. 3.14 is too new for part of the ML stack (known from prior projects on this machine), and 3.11 is a fallback.
  2. **Gazebo-bindings environment:** whatever Python Homebrew's `gz-harmonic` Python bindings (`gz.transport13`, `gz.msgs10`) are built against. The **Sensor Bridge** process runs there and forwards data to AERIS over local IPC (§18.4). This removes any need to force-match Python versions.
- **PX4's own `.venv`** (created by `Tools/setup/macos.sh`) is used only for building PX4. AERIS never imports from it.
- **PyTorch backend policy:**
  - `cpu` is the default for RL (small MLP/GRU networks, where CPU is typically faster than MPS because of kernel-launch overhead; ASSUMPTION, benchmarked in Phase 14).
  - `mps` is used for CNN training and inference in perception when a benchmark shows a speedup.
  - The device is always configuration-driven and recorded in manifests.
  - CUDA code paths are never assumed.
  - Known risk: PyTorch MPS availability regressions have been reported on new macOS releases (pytorch#167679, #177819). The machine runs macOS 27.2, newer than any documented configuration, so CPU fallback is mandatory and automatic.
- **Thermals:** the Air throttles under sustained load. Training runs log steps-per-second over time so throttling is visible. FULL-tier training is scheduled in ≤ 3 h blocks on mains power. SITL GUI and training MUST NOT run at the same time.
- **Shell requirement from PX4 docs (VERIFIED):** `ulimit -S -n 2048` in `~/.zshrc`, because the macOS default of 256 open files breaks the PX4 build.
- **Node:** there is a known IPv6 stall on this Mac for npm. Frontend phases MUST set `NODE_OPTIONS=--no-network-family-autoselection` and use npm, not pnpm (recorded project knowledge).

### 7.3 Environment facts that must be re-verified in Phase 1

- PX4 builds `px4_sitl` with the Gazebo modules on macOS 27.2 / clang 21, given the open issue #27026.
- `gz sim` server runs natively. The GUI either runs or has a documented workaround (render-engine flag, XQuartz, separate server/GUI processes).
- **Rendering sensors (depth camera, GPU LiDAR) produce data in server/headless mode on macOS.** This is the single highest-impact unknown (§9.4).
- The Gazebo Python bindings are importable, and their Python version is recorded.
- MAVSDK-Python has an arm64 macOS wheel with a bundled `mavsdk_server`. The PyPI package name is verified; upstream docs reference both `mavsdk` and `mavsdk-grpc`.

---

## 8. PX4 strategy

### 8.1 Role

PX4 is the **flight controller**. It owns:
- state estimation (EKF2);
- position, velocity, attitude and rate control;
- the mixer;
- arming checks;
- flight modes;
- failsafes;
- geofence;
- takeoff and land;
- MAVLink telemetry.

AERIS commands PX4 through **Offboard mode** setpoints (position / velocity / yaw) and PX4 actions (arm, takeoff, land, hold, return).

### 8.2 Facts (VERIFIED, PX4 docs `main`, 2026-09)

- The latest stable release is **PX4 v1.17.0** (announced 2026-05-18). v1.16 was the previous release.
- macOS setup:
  1. `xcode-select --install`
  2. Homebrew
  3. `ulimit -S -n 2048`
  4. `git clone https://github.com/PX4/PX4-Autopilot.git` and `git submodule update --init --recursive --force`
  5. `./Tools/setup/macos.sh --sim-tools` (installs Gazebo Harmonic and XQuartz; creates `.venv`)
  6. `source .venv/bin/activate`
  7. Smoke test: `make px4_sitl gz_x500`.
- "PX4 CI exercises this setup on Apple Silicon runners only."
- Offboard mode requires a continuous proof-of-life setpoint stream of **≥ 2 Hz**. Loss triggers the failsafe after `COM_OF_LOSS_T` with action `COM_OBL_RC_ACT`. Setpoints are in **NED** (local), with attitude and rates in **FRD**.
- PX4 **Collision Prevention** (`CP_DIST`) works **only in Position mode** (acceleration-based), not in Offboard. AERIS must therefore supply its own collision shield for offboard control (§16.4).
- SITL is **lockstep** with Gazebo. `PX4_SIM_SPEED_FACTOR` scales sim speed.

### 8.3 Dependency management (DECISION, ADR-002)

- PX4 is **external**. It is not vendored or submoduled in the AERIS repo, because it is multi-GB with submodules.
- Location comes from `AERIS_PX4_DIR` (default `~/aeris-deps/PX4-Autopilot`), set in `configs/local.yaml` (gitignored) or the environment.
- The pin is recorded in `configs/versions.lock.yaml`: PX4 tag + commit SHA, Gazebo version, MAVSDK version, QGC version. At startup a version check warns on mismatch, and experiment manifests record actual versions.
- **Pin selection procedure (Phase 1):**
  1. Try the **v1.17.x** release tag first (stable).
  2. If the macOS Gazebo build fails on the tag but succeeds on `main`, pin a specific `main` SHA and document why.
  3. If both fail, apply the *minimum* local build fix, as a patch file under `third_party/patches/px4/` with a README (build-system fixes only, never flight code) or as an upstream-PR reference. Otherwise go to the Linux fallback (§12).
- **No PX4 firmware source modifications.** Configuration goes through:
  - PX4 parameters, via a parameter profile file (`configs/vehicle/px4_params/*.params`) applied at launch;
  - PX4 environment variables (`PX4_GZ_*`, `PX4_SIM_*`, `PX4_SYS_AUTOSTART`);
  - custom worlds and models via `GZ_SIM_RESOURCE_PATH` and **standalone mode** (`PX4_GZ_STANDALONE=1`, where AERIS launches Gazebo itself with its own world).

### 8.4 Vehicle selection

- **Primary: x500 family (DECISION).** It is PX4's reference Gazebo quadrotor and ships sensor variants (VERIFIED):
  - `gz_x500` (base);
  - `gz_x500_depth` (forward OAK-D-like depth camera);
  - `gz_x500_mono_cam`, `gz_x500_mono_cam_down`;
  - `gz_x500_lidar_2d` (Hokuyo UTM-30LX-like, 0.1–30 m, 270°);
  - `gz_x500_lidar_front`, `gz_x500_lidar_down` (Lightware LW20/C 1-D);
  - `gz_x500_flow` (optical flow);
  - `gz_x500_vision` (external-vision odometry);
  - `gz_x500_gimbal`.
- **AERIS vehicle `aeris_x500`** (Phase 8): a model composed *outside the PX4 tree* that includes the x500 base plus forward depth camera, 2D LiDAR and a forward RGB camera. It is registered via `GZ_SIM_RESOURCE_PATH` and launched with the x500 airframe (`PX4_SYS_AUTOSTART` for the x500, recorded in Phase 3).

  If the composed model cannot be loaded without touching PX4 sources, fall back to the stock variant that best matches the phase (`x500_depth` for perception/mapping, `x500_lidar_2d` for avoidance), and document the reduced sensor set.

### 8.5 Communication options (VERIFIED, PX4 "Robotics / Offboard APIs" page)

| Option | Status per PX4 docs | AERIS use |
|---|---|---|
| **MAVSDK** (MAVLink, C++/Python) | Recommended, broad OS support | **Primary adapter** (`Px4MavsdkAdapter`). asyncio, bundled `mavsdk_server`. |
| pymavlink (raw MAVLink) | Lower-level, widely used | **Secondary adapter** for messages MAVSDK lacks (e.g., `OBSTACLE_DISTANCE`, `ODOMETRY` injection for external vision, custom parameter handling). |
| ROS 2 via uXRCE-DDS | Linux-only, lower latency, deep uORB access | **Reserved** `Ros2Px4Adapter`, Linux fallback only (§11). |
| MAVROS / ROS 1 | ROS 1 EOL May 2025 | Not used. |
| DroneKit | Unmaintained | Not used. |

---

## 9. Gazebo strategy

### 9.1 Facts (VERIFIED, 2026-09)

- PX4 `main` docs list **Gazebo Harmonic** as the macOS target, installed by `macos.sh --sim-tools`.
- The PX4 Gazebo overview page still says the simulator is "not available for Windows or macOS". The mac setup page contradicts it. **The mac page is newer and states "supported by the PX4 dev team"**, but the contradiction is recorded as a risk.
- Gazebo's own Harmonic macOS page is stale: it lists Big Sur / Monterey binaries via `brew tap osrf/simulation; brew install gz-harmonic`.
- PX4 env vars:
  - `HEADLESS=1`
  - `PX4_GZ_WORLD`
  - `PX4_GZ_MODEL_POSE="x,y,z,roll,pitch,yaw"`
  - `PX4_GZ_STANDALONE=1`
  - `PX4_SIM_SPEED_FACTOR`
  - `PX4_SIM_MODEL` / `PX4_SYS_AUTOSTART`
- Stock worlds: `default`, `aruco`, `baylands`, `lawn`, `windy`, `walls`, `ridge`, `rover`, `moving_platform`.
- **Multi-vehicle Gazebo simulation is "only supported on Linux".**
- Community-reported mac/VM issue: GUI crashes, worked around with `--render-engine ogre` / `PX4_GZ_SIM_RENDER_ENGINE=ogre`. ASSUMPTION: this env var is still honored; verify in Phase 1.

### 9.2 Operating modes

| Mode | When | Notes |
|---|---|---|
| **Server-only (headless)**, primary | All automated runs, evaluation, CI-like sim tests | Lowest resources. Rendering sensors still need a render engine; see §9.4. |
| Server + GUI | Visual debugging only | On macOS the Gazebo GUI and server are expected to run as separate processes (ASSUMPTION; verify). XQuartz may be required. |
| AERIS-managed standalone | Custom AERIS worlds | AERIS starts `gz sim -s <world.sdf>` first, then PX4 with `PX4_GZ_STANDALONE=1`. Keeps worlds out of the PX4 tree. |

### 9.3 World pipeline (DECISION, ADR-007: single source of truth)

`WorldSpec` (a typed, seed-reproducible description: bounds, primitives, walls, rooms, targets, spawn poses, lighting, family, split) is rendered by:
- `SdfRenderer` → Gazebo `.sdf` world (plus model includes);
- `FastSimRenderer` → voxel / primitive geometry for AERIS FastSim;
- `GroundTruthRenderer` → evaluator-only occupancy, free-space masks, target lists.

No world geometry is authored twice. A consistency test checks that the SDF-rendered and FastSim-rendered occupancy agree to within voxel resolution.

### 9.4 Highest-impact unknown: rendering sensors on macOS

Depth cameras, RGB cameras and `gpu_lidar` in Gazebo Harmonic depend on the rendering engine (ogre2 by default). Phase 1 MUST test `gz_x500_depth` and `gz_x500_lidar_2d` in server mode and record:
- message rate;
- image and scan validity;
- real-time factor.

Outcomes and responses:

| Outcome | Response |
|---|---|
| Works at ≥ 10 Hz depth / ≥ 10 Hz scan with RTF ≥ 0.5 | Proceed Mac-native. |
| Works only with `ogre` (v1) engine | Proceed. Record the engine in the version lock. |
| Rendering sensors fail natively, non-rendering sim works | Primary: run the Gazebo server in an **Ubuntu 24.04 arm64 Docker container or VM with software rendering (EGL/llvmpipe)** for sensor phases (§12). Parallel measure: *geometry-based* depth and LiDAR computed by raycasting the `WorldSpec` in AERIS (the FastSim sensor model), clearly labeled `SIM-GEOMETRIC` and never presented as rendered Gazebo sensors. |
| Nothing runs natively | Linux fallback for the simulator. The AERIS core stays Mac-native (§12). |

---

## 10. QGroundControl strategy

- **Facts (VERIFIED):** QGroundControl stable **5.1**. macOS 13+; universal binary (Apple Silicon native), signed and notarized DMG.
- **Role:** independent **debug and inspection tool**, not part of AERIS's control path.
  - Uses: parameters, flight-mode and arming diagnostics, the MAVLink inspector, log download (ULog), and geofence visualization.
  - AERIS never depends on QGC being open.
- **Coexistence:** PX4 SITL exposes separate MAVLink UDP endpoints for the GCS and for offboard APIs, so QGC and MAVSDK can connect at the same time. The exact ports are recorded in Phase 3 in `configs/vehicle/px4_sitl.yaml`. They are never hardcoded (§36 of prompt).
- **Rule:** during automated experiments QGC SHOULD be closed or in read-only use. A human changing modes or parameters in QGC during an experiment invalidates that run, and the run manifest records `gcs_connected: true/false` from heartbeat detection.
- **Validation in Phase 1:** QGC connects to SITL, shows the vehicle, parameters and mode, and can arm/takeoff/land manually once.

---

## 11. ROS 2 strategy

- **Facts (VERIFIED):** PX4's supported ROS 2 platform is **ROS 2 Jazzy on Ubuntu 24.04** (Humble on 22.04 as an alternative), via the uXRCE-DDS client (PX4) and agent (companion). PX4's ROS 2 guide does not mention macOS. For ROS 2 itself, Tier-1 platforms for Jazzy are Ubuntu 24.04 and Windows; macOS is not Tier 1.
- **DECISION (ADR-005):** AERIS V1 has **no ROS 2 dependency**. Nothing in `aeris/` imports `rclpy`.
- **Adapter seam:** `VehicleInterface`, `SensorSource`, `MapPublisher` and `PoseSource` are protocols. A future `aeris/adapters/ros2/` package, running only in the Linux fallback, may implement them to:
  - use `px4_ros_com` / `px4_msgs` for lower-latency vehicle I/O;
  - integrate ROS-centric SLAM (e.g., RTAB-Map, Cartographer) behind `PoseSource` (§23);
  - interoperate with external ROS tools.
- **Trigger for adding ROS 2:** only when a phase needs a ROS-only capability *and* its phase report justifies the Linux dependency.

---

## 12. Linux fallback strategy

Principle: **isolate, don't migrate.** The AERIS core (autonomy, learning, backend, frontend) stays Mac-native. Only the component that cannot run natively moves.

| Option | Setup | Use for | Limits |
|---|---|---|---|
| **A. Native headless** | Gazebo server only, no GUI | Default. GUI problems don't block anything. | Needs rendering sensors to work headless (§9.4) |
| **B1. Docker (Ubuntu 24.04 arm64)** | Docker Desktop is already installed. Container runs `gz sim -s` + PX4 SITL; UDP/gz-transport exposed to the host. | Linux-only sim features (**multi-vehicle**), rendering-sensor fallback via software rendering | No GPU in containers on macOS: software rendering is slow. Gazebo discovery across the container boundary needs configuration (`GZ_IP`, `GZ_PARTITION`, host networking caveats on macOS). |
| **B2. Ubuntu 24.04 ARM64 VM (UTM or Parallels)** | Full desktop VM | ROS 2 Jazzy experiments, Gazebo GUI if native GUI fails | Limited virtual GPU (Gazebo crashes in Mac VMs are reported). RAM contention on a 16 GB machine: give the VM ≤ 6 GB. |
| **C. Remote Linux (+ NVIDIA)** | SSH machine (user-provided) | Large-scale training, rendered-RGB training (RQ3), many-parallel SITL evaluation | Costs money or needs access. Not assumed available. |
| **D. Make the feature optional** | — | Anything whose fallback cost exceeds its research value | Reported as not executed |

**Selection rule:** a phase that hits a Linux-only need MUST state which option it recommends and why, and MUST NOT silently move the whole environment. Transport code (MAVSDK UDP, gz-transport, the IPC bridge) is written to be host-agnostic: all endpoints come from configuration.


---

## 13. Complete system architecture

### 13.1 Refinements to the conceptual architecture

The prompt's five-layer diagram is kept, with seven justified changes:

1. **The Safety layer is lifted out of "Vehicle Interface" into its own mandatory choke point.** Nothing reaches the vehicle adapter except through `SafetySupervisor`. It is the only holder of the command handle.
2. **A Privileged Ground-Truth channel is added beside the stack, not inside it.** Only the evaluator, the world generator and the offline label generator may read it.
3. **A Sensor Bridge process sits between Gazebo and AERIS** (ADR-004) for Python-version isolation and crash isolation.
4. **AERIS FastSim is a second backend** for the same Vehicle/Sensor interfaces (ADR-006). It is used for learning.
5. **Experiment/Replay recording is a cross-cutting service** fed by an event bus, not bolted onto the UI.
6. **Clock abstraction:** all autonomy runs on *simulation time* (Gazebo `/clock`, lockstep with PX4), never wall time. This keeps speed-factor changes and pauses correct.
7. **"Fleet" is removed from V1 UI areas** until Phase 33 exists (principle: only sections with genuine functionality).

### 13.2 Layered architecture

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ AERIS CONTROL CENTER  (React/TS/Vite)                                        │
│ Mission Control · Live Flight · Perception · World Map · Autonomy ·          │
│ Training Lab · Experiment Lab · Replay Studio · System   (Fleet: Phase 33)   │
└───────────────────────────────▲───────────────┬──────────────────────────────┘
                   WebSocket (throttled streams) │ REST (commands, queries)
┌───────────────────────────────┴───────────────▼──────────────────────────────┐
│ AERIS SERVER  (FastAPI, localhost)                                           │
│ SimulationService · MissionService · TelemetryService · MapService ·         │
│ ModelService · TrainingService · ExperimentService · ReplayService ·         │
│ SystemService          (no arbitrary command execution — §45.4)              │
└───────────────────────────────▲───────────────┬──────────────────────────────┘
                     in-process API / event bus  │
┌───────────────────────────────┴───────────────▼──────────────────────────────┐
│ AUTONOMY ENGINE  (pure Python; runs on sim clock; no transport code)         │
│  Mission Executive (state machine) ─► Exploration (frontier | learned) ─►    │
│  Planning (A*/path follower) ─► Local Navigation (classical | learned) ─►     │
│  HighLevelAction                                                             │
│  Perception (depth→points, detector) · Localization (PoseSource) ·           │
│  Mapping (voxel log-odds) · World Model (map+targets+mission memory)         │
└───────────────────────────────▲───────────────┬──────────────────────────────┘
          AgentObservation (provenance-checked)  │ HighLevelAction
┌───────────────────────────────┴───────────────▼──────────────────────────────┐
│ SAFETY LAYER  (deterministic; the ONLY path to the vehicle)                  │
│  Command validator · Envelope/geofence · Collision shield · Watchdog ·        │
│  Mode/arming gate · Intervention logger                                      │
└───────────────────────────────▲───────────────┬──────────────────────────────┘
                                 │               │ Validated setpoints
┌───────────────────────────────┴───────────────▼──────────────────────────────┐
│ VEHICLE INTERFACE LAYER  (protocols + adapters)                              │
│  VehicleInterface ◄─ Px4MavsdkAdapter | Px4PymavlinkAdapter |                │
│                      FastSimVehicle | (Ros2Px4Adapter, Linux only)           │
│  SensorSource     ◄─ GzBridgeClient (IPC) | FastSimSensors                   │
│  ClockSource      ◄─ GzClock | FastSimClock                                  │
└──────────┬──────────────────────────┬────────────────────────────────────────┘
   MAVLink/UDP (MAVSDK)          local IPC (ZeroMQ, msgpack)
┌──────────▼───────────┐   ┌──────────▼─────────────────────────────────────────┐
│ PX4 SITL (lockstep)  │   │ SENSOR BRIDGE process (Homebrew-Python gz bindings)│
│ EKF2, controllers,   │   │ gz.transport subscribe: depth, rgb, scan,          │
│ failsafes, geofence  │   │ camera_info, /clock, [GT pose → privileged chan]   │
└──────────┬───────────┘   └──────────▲─────────────────────────────────────────┘
           │ gz_bridge (PX4 module)    │ gz-transport
┌──────────▼───────────────────────────┴───────────────────────────────────────┐
│ GAZEBO HARMONIC (server; GUI optional)                                       │
│ aeris_x500 · physics · WorldSpec-generated world · targets ·                 │
│ depth/RGB/LiDAR/IMU/GPS/baro/contact sensors                                 │
└──────────────────────────────────────────────────────────────────────────────┘

 Side channel (evaluator-only):  GroundTruthService ◄─ gz pose/contacts, WorldSpec
 Cross-cutting:  EventBus → Recorder (MCAP) → Replay/Experiment stores
 Debug tool:     QGroundControl ◄─ MAVLink (GCS port) — never in control path
```

### 13.3 Runtime process model

| Process | Python / runtime | Responsibility | Crash policy |
|---|---|---|---|
| `gz sim -s` | native (Homebrew) | physics, sensors | SimulationService detects the crash and marks the run invalid |
| `px4` (SITL) | native | flight control | same |
| `aeris-sensor-bridge` | Homebrew Python (gz bindings) | gz → IPC forwarding, clock | restart allowed; stale-data watchdog in the Safety layer |
| `aeris-core` | AERIS venv (3.12) | autonomy loop + safety + vehicle adapter + recorder | on exception, Safety fallback = PX4 Hold/Land via a separate watchdog path |
| `aeris-server` | AERIS venv | FastAPI; may host `aeris-core` in-process for interactive use | — |
| `aeris-train` | AERIS venv | FastSim vectorized training (no PX4/Gazebo) | checkpoint/resume |
| frontend dev server | Node | UI | — |

### 13.4 Control loop timing (DECISION)

| Loop | Rate (sim time) | Owner |
|---|---|---|
| PX4 inner loops | PX4-internal | PX4 |
| Setpoint stream to PX4 | 20 Hz (≥ 2 Hz required; 10× margin) | Vehicle adapter, fed by the supervisor's latest validated command |
| Safety supervisor + shield | 20 Hz | Safety layer |
| Local navigation / learned local policy | 10 Hz | Autonomy |
| Exploration decision (hierarchical) | event-driven, or ≤ 1 Hz | Autonomy |
| Mapping update | on each depth/scan frame (≤ 15 Hz), budgeted | Autonomy |
| Mission executive | 5 Hz + events | Autonomy |
| UI telemetry push | 10 Hz state, 1–2 Hz map deltas | Server |

---

## 14. Repository architecture

### 14.1 Refined structure

Directories are created **only in the phase that first needs them**. The phase that creates each one is listed in brackets.

```
aeris/                                   (repo root; ~/Claude/aeris)
├── AERIS_TECHNICAL_SPEC.md              [P0]
├── README.md                            [P2]
├── pyproject.toml / uv.lock             [P2]  (package `aeris`, extras: sim, learn, server)
├── Makefile                             [P2]  (thin: setup, test, lint, sim-smoke …)
├── .gitignore / .editorconfig / .pre-commit-config.yaml   [P2]
├── importlinter.ini (or [tool.importlinter])               [P2]  dependency contracts (§14.3)
├── configs/
│   ├── versions.lock.yaml               [P1]  frozen known-good versions
│   ├── local.example.yaml               [P2]  AERIS_PX4_DIR etc. (local.yaml gitignored)
│   ├── simulation/                      [P3]  launch profiles (headless, gui, speed factor)
│   ├── vehicle/                         [P3]  px4_sitl.yaml (ports), px4_params/*.params, safety.yaml
│   ├── worlds/                          [P9]  WorldSpec family generators' parameters, splits.lock
│   ├── missions/                        [P6]
│   ├── learning/                        [P13] env/ppo configs (smoke/dev/full)
│   └── experiments/                     [P7]  experiment definitions + preregistration/
├── aeris/
│   ├── core/                            [P2]  types, units, clock protocol, config loader, logging, errors
│   │   └── frames/                      [P4]  transforms, conventions (§19)
│   ├── vehicle/
│   │   ├── interface.py                 [P4]  VehicleInterface protocol + state dataclasses
│   │   ├── px4_mavsdk/                  [P4]
│   │   ├── px4_mavlink/                 [P4/P10 as needed]
│   │   └── telemetry/                   [P4]
│   ├── safety/                          [P5]  supervisor, envelope, watchdog, shield [P10]
│   ├── simulation/
│   │   ├── launcher/                    [P3]  process manager for gz + px4 (whitelisted profiles)
│   │   ├── bridge/                      [P8]  sensor bridge (runs under gz Python) + IPC schema
│   │   ├── worlds/                      [P9]  WorldSpec, generators, SDF renderer
│   │   ├── models/                      [P8]  aeris_x500 SDF, target models
│   │   ├── groundtruth/                 [P7]  PRIVILEGED — evaluator-only (§17.4)
│   │   └── fastsim/                     [P13] vectorized training simulator
│   ├── perception/
│   │   ├── depth/ lidar/                [P8/P16]
│   │   └── detection/                   [P20]
│   ├── localization/                    [P11/P23] PoseSource implementations
│   ├── mapping/                         [P11] voxel log-odds, 2D projection, frontiers
│   ├── autonomy/
│   │   ├── mission/                     [P6]  executive state machine, mission specs
│   │   ├── navigation/                  [P10] path follower, reactive avoidance
│   │   ├── planning/                    [P12] A*
│   │   ├── exploration/                 [P12] random, frontier; learned wrapper [P19]
│   │   └── tracking/                    [P24]
│   ├── learning/
│   │   ├── envs/                        [P13] Gymnasium envs over FastSim
│   │   ├── spaces/                      [P13] ObservationBuilder / ActionDecoder (shared train+deploy)
│   │   ├── ppo/                         [P14] rollout buffer, GAE, losses, trainer
│   │   └── networks/                    [P14] encoders, GRU core, heads
│   ├── evaluation/                      [P7]  metrics, episode runner, statistics
│   ├── experiments/                     [P7]  run manifest, registry, preregistration
│   └── replay/                          [P7]  MCAP recorder/reader, detail levels
├── backend/app/                         [P25] api/ websocket/ services/ schemas/
├── frontend/                            [P27] src/{app,features,components,visualization,charts,services,styles,tokens}
├── design/                              [P26] design-system spec, tokens source (JSON), UI specs
├── scripts/                             [P1+] one-off CLIs (thin wrappers over package code)
├── tests/                               [P2]  unit/ sim/ integration/ eval/ backend/ (frontend tests live in frontend/)
├── third_party/patches/                 [P1 only if needed]
├── docs/                                [P1+] mac-setup.md, px4.md, …, adr/, phase_reports/
├── results/   models/   replays/        [P7]  gitignored outputs (only small, curated summaries committed under docs/)
```

**Changes from the prompt's tree:**
- `safety/` is promoted to a top-level package.
- `simulation/groundtruth/` is isolated.
- `simulation/fastsim/` is added.
- `learning/spaces/` is added, so observation preprocessing is shared between training and deployment.
- `core/frames/` is added.
- `perception/rgb` and `perception/fusion` are dropped until a phase needs them.
- `mapping/semantic` and `mapping/voxel` merge into `mapping/`.
- `buffers/` and `training/` merge into `ppo/`.
- `design/` is added for the design system.

### 14.2 Code standards (binding)

- Python 3.12, fully typed, checked with `mypy --strict` on `aeris/core`, `aeris/vehicle`, `aeris/safety`, `aeris/learning/ppo`, `aeris/evaluation`, and relaxed elsewhere. Lint and format with `ruff`.
- Dataclasses (`frozen=True, slots=True`) for internal data. Pydantic v2 for configs and API schemas.
- **Units** are SI internally: m, s, rad, m/s. Field names carry units when ambiguous (`yaw_rad`, `t_sim_s`).
- No hardcoded ports, paths or magic numbers. Constants live in configs or are named in `aeris/core/constants.py` with a source comment.
- No `except Exception: pass`. Exceptions are typed (`aeris.core.errors`), and structured logging (`structlog` or stdlib JSON logging, decided in P2) carries `run_id`, `t_sim`.
- File size guideline: ≤ 400 lines per module; review if exceeded.
- No global mutable state. Dependencies are injected through constructors.

### 14.3 Enforced dependency contracts (import-linter, CI-checked from P2)

1. `aeris.learning`, `aeris.autonomy`, `aeris.perception`, `aeris.mapping` and `aeris.localization` **MUST NOT import** `aeris.simulation.groundtruth`.
2. `aeris.autonomy` **MUST NOT import** any vehicle adapter implementation. It may use only `aeris.vehicle.interface` types and emits `HighLevelAction`.
3. Only `aeris.safety` may call command methods on a `VehicleInterface`. Enforced by exposing a `CommandPort` that only the supervisor receives, plus a test that scans for command-method calls.
4. `aeris.*` **MUST NOT import** `backend` or `frontend`. `backend` may import `aeris`.
5. Nothing under `aeris/` imports `rclpy` (ROS 2) or `gz.*`. `gz.*` is imported only in `aeris/simulation/bridge/_gz_process.py`, which runs in the bridge environment.

---

## 15. Vehicle interface

### 15.1 Design

`VehicleInterface` is an **async protocol**. Adapters implement it, and autonomy code never sees the transport. For FastSim, a synchronous stepping façade wraps the same semantics (§17.3).

```python
class VehicleInterface(Protocol):
    # lifecycle
    async def connect(self, endpoint: VehicleEndpoint) -> None: ...
    async def disconnect(self) -> None: ...
    # state (all return agent-legitimate estimates, never simulator ground truth)
    async def get_vehicle_state(self) -> VehicleState: ...        # snapshot
    def subscribe_telemetry(self, rate_hz: float) -> AsyncIterator[VehicleState]: ...
    # command port — handed ONLY to SafetySupervisor
    def command_port(self) -> CommandPort: ...

class CommandPort(Protocol):
    async def arm(self) -> CommandResult: ...
    async def disarm(self) -> CommandResult: ...
    async def takeoff(self, altitude_m: float) -> CommandResult: ...
    async def land(self) -> CommandResult: ...
    async def hold(self) -> CommandResult: ...
    async def return_to_launch(self) -> CommandResult: ...
    async def start_offboard(self, initial: Setpoint) -> CommandResult: ...
    async def stop_offboard(self) -> CommandResult: ...
    async def set_position_target(self, sp: PositionSetpoint) -> None: ...   # ENU/odom frame in AERIS types
    async def set_velocity_target(self, sp: VelocitySetpoint) -> None: ...   # ENU odom or FLU body frame
    async def goto_waypoint(self, wp: Waypoint) -> CommandResult: ...        # convenience = position setpoint + arrival monitor
    async def emergency_stop_simulation(self) -> CommandResult: ...          # §16.5
```

`VehicleState` is a frozen dataclass:
- `t_sim_s`
- `armed`, `flight_mode: FlightMode`, `landed_state`
- `pose_odom: Pose` (ENU, from PX4 EKF2), `velocity_odom_mps`, `angular_velocity_body_radps`
- `home_odom`
- `gps: GpsStatus` (fix type, satellites, eph/epv)
- `battery_sim: BatterySimState`, labeled *simulated drain model, not physical energy*
- `ekf_flags` (estimator health)
- `link: LinkStatus` (last heartbeat age)
- `source: Provenance.ESTIMATE`

The `get_pose`, `get_velocity`, `get_flight_mode` and `get_battery_simulation_state` accessors from the prompt are convenience properties on `VehicleState`, which avoids many round-trips.

### 15.2 Adapters

| Adapter | Transport | Phase | Notes |
|---|---|---|---|
| `Px4MavsdkAdapter` | MAVSDK-Python → `mavsdk_server` (gRPC) → MAVLink UDP | 4 | Primary. Does the ENU↔NED and FLU↔FRD conversion at the boundary. |
| `Px4PymavlinkAdapter` | pymavlink UDP | 4 (minimal) / 10+ | For messages MAVSDK lacks. May share the vehicle connection on a separate MAVLink port. |
| `FastSimVehicle` | in-process | 13 | Same state and command semantics. The velocity-tracking dynamics model comes from system identification. |
| `Ros2Px4Adapter` | uXRCE-DDS | Linux fallback only | Not in V1. |
| `ReplayVehicle` | MCAP file | 30 | Read-only; for replay-driven tests. |

### 15.3 Transport decision process (Phase 4)

MAVSDK is the default (ADR-003), but Phase 4 runs a measured check before freezing it:
- telemetry rate achieved for position/velocity/attitude;
- setpoint round-trip latency (command timestamp → PX4 `LOCAL_POSITION_NED` response);
- reconnect behavior;
- whether MAVSDK Offboard supports every setpoint type AERIS needs (position+yaw, velocity NED+yaw, body velocity+yaw rate).

If MAVSDK fails a requirement, that capability is routed through pymavlink. The fix is not to rewrite the interface.

### 15.4 Timestamps

Every state sample carries `t_sim_s`, derived from PX4's MAVLink `time_boot_ms`/`time_usec`, which in lockstep SITL tracks simulation time (ASSUMPTION; verify in P4 against gz `/clock`), and `t_recv_wall_s`. Autonomy logic uses only `t_sim_s`.

---

## 16. Flight safety architecture

### 16.1 Hard rules

1. **No learned component outputs anything below `HighLevelAction`**, which is one of:
   - body/odom velocity + yaw rate;
   - a position/waypoint target;
   - a subgoal for the planner.

   Motor, thrust, attitude and body-rate outputs are *not representable* in the action types.
2. **PX4 keeps stabilization, estimation and its own failsafes.** AERIS never disables PX4 failsafes to make an experiment pass.
3. **All commands pass through `SafetySupervisor`.** It alone holds the `CommandPort` (§14.3 contract 3).
4. **The safety layer is deterministic, tested and never learned.** Its interventions are logged and reported as metrics. A learned policy's result is never shown without its shield-intervention rate.
5. **Real hardware is out of scope** (§16.6).

### 16.2 Layers

| Layer | Implementation | Examples |
|---|---|---|
| S0 — PX4 native | PX4 params (profile file) | geofence `GF_*` (outer envelope), offboard loss `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` (→ Hold, then Land), low-battery sim failsafe, max tilt / velocity limits (`MPC_*`), RC-loss handling configured for SITL without RC |
| S1 — Command validation | `aeris.safety.validator` | NaN/Inf rejection; frame checks; bounds (‖v_xy‖ ≤ v_max, \|v_z\| ≤ vz_max, \|ψ̇\| ≤ ψ̇_max); acceleration / rate-of-change limits; altitude floor/ceiling; AERIS geofence (strictly inside PX4's) |
| S2 — State gating | `aeris.safety.supervisor` | Commands only in allowed states (armed ∧ OFFBOARD ∧ mission ACTIVE); reject during takeoff/land transitions; EKF health gate (PX4 estimator flags) |
| S3 — Collision shield | `aeris.safety.shield` (Phase 10) | From *agent-observed* depth/scan: limits velocity toward obstacles so stopping distance < clearance − margin; emergency brake to hover |
| S4 — Watchdogs | `aeris.safety.watchdog` | Autonomy tick overdue > T_a → hover setpoint; sensor data stale > T_s → brake + hover; link heartbeat lost → PX4 S0 handles; mission time budget exceeded → RTL/Land |

### 16.3 Supervisor state machine

```
DISCONNECTED → CONNECTED → PREFLIGHT_OK → ARMING → TAKING_OFF → AIRBORNE_MANUAL_HOLD
      ↔ AUTONOMY_ACTIVE (offboard) → [INTERVENTION → back] → LANDING → LANDED → DISARMED
      any → FAILSAFE (PX4-reported or AERIS-triggered) → LANDING/HOLD
```

Transitions are events on the bus and are recorded in the replay.

### 16.4 Collision shield specification (Phase 10)

The shield uses the agent's own sensing:
- the depth point cloud and LiDAR scan, projected into body-frame horizontal sectors (e.g., 72 × 5°, similar in spirit to PX4 Collision Prevention);
- plus a vertical clearance check.

For a commanded horizontal velocity v, with d_i the clearance in sector i, a_brake the configured deceleration and τ the latency allowance, the permitted speed component toward sector i is

  v_i,max = max(0, √(2·a_brake·max(0, d_i − d_safe)) − a_brake·τ),

and the command is projected so that for every sector i, (v · û_i) ≤ v_i,max. Unknown sectors (no data, outside the FOV) are handled by a configured policy. The default policy is **conservative**: lateral and backward motion is limited to the speed at which the vehicle can stop within its last-seen clearance, and forward-only flight is preferred because the depth camera is forward-facing.

- **Intervention metric:** fraction of control ticks where ‖v_cmd − v_shielded‖ > ε.
- **Tests:** property-based tests confirm that no shielded command moves toward an obstacle closer than d_safe; there are also stopping-distance checks against the FastSim dynamics.

### 16.5 Emergency stop (simulation)

`emergency_stop_simulation()` runs these steps in order:
1. switch PX4 to Hold (or Land if airborne below a threshold);
2. stop the autonomy loop;
3. mark the run `ABORTED`.

A separate `SimulationService.pause()` (Gazebo pause) and `reset()` exist for sim control. MAVLink flight termination is available in SITL only as an explicit, logged debug action.

### 16.6 Hardware guard

`VehicleEndpoint` accepts only loopback or configured-simulator hosts (`127.0.0.1`, `::1`, or an explicitly configured Linux-fallback sim host tagged `simulated: true`). At connect time, the adapter verifies it is talking to a SITL instance (autopilot version / SITL indicators, determined in Phase 4) and refuses otherwise. There is no code path or documentation for real-vehicle flight.

---

## 17. Simulation architecture

### 17.1 Two tiers (ADR-006)

| Aspect | Tier H: PX4 SITL + Gazebo Harmonic | Tier F: AERIS FastSim |
|---|---|---|
| Purpose | Integration, classical baselines, **all headline evaluation** | RL training, fast ablations, unit-level autonomy tests |
| Dynamics | Full multirotor physics + PX4 controllers + EKF2 | Identified closed-loop model: velocity-setpoint → velocity response (first/second-order + delay + limits), fitted to Tier H step responses |
| Sensors | Rendered depth/RGB, GPU LiDAR, IMU/GPS via PX4 | Raycast depth and LiDAR on `WorldSpec` geometry with a noise model; **no RGB** in V1 (ADR-011) |
| Pose | PX4 EKF2 estimate (agent) + gz GT (evaluator) | GT + noise model mimicking EKF2 error statistics measured in Tier H (agent), GT (evaluator) |
| Throughput | ~real-time × speed factor (measured P1/P3); 1 vehicle on macOS | Target ≥ 2 000 env-steps/s aggregate on this Mac (gate in P13) |
| Determinism | Statistical (D1) | Bitwise on CPU given seed (D0) |

**Validity rule:** a claim about learned autonomy is only reported as an AERIS result if it was **evaluated in Tier H**. FastSim-only numbers are labeled "FastSim" everywhere.

### 17.2 Simulation services

- `SimulationLauncher` starts gz server (+GUI) and PX4 from **named profiles** in `configs/simulation/*.yaml`: world path, model, spawn pose, speed factor, headless flag, render engine. It waits for readiness (gz topics up, PX4 heartbeat, EKF converged), exposes `reset()` (kill and relaunch, or world reset where it works cleanly; decided empirically in P3), and captures logs to the run directory.
- **Reset strategy:** full PX4 + gz restart between evaluation episodes is the correct default, because it guarantees a clean EKF and failsafe state. It is slower. A faster "respawn + PX4 reboot" is allowed only if P3 shows state is fully clean.

### 17.3 FastSim design (Phase 13)

- Vectorized NumPy with Numba JIT for raycasting (Amanatides–Woo voxel traversal). N environments per process, M processes. `gymnasium.vector` compatible.
- The world is a voxel occupancy volume (e.g., 0.1–0.2 m) rendered from `WorldSpec`. Obstacles are static in V1. Moving targets (P24) are handled analytically.
- The vehicle is a point mass with radius r_v and a velocity-tracking model v̇ = (v_sp_limited − v)/τ_v with first-order delay d, acceleration limits and yaw-rate tracking. Parameters are **identified from Tier H logs** (P13 system ID), with randomization ranges set around the identified values.
- Collision: the vehicle sphere intersects an occupied voxel → termination + penalty. Tier H collision detection uses gz contact sensors or GT clearance, per §41.
- **Parity tests** use the same `WorldSpec` and the same open-loop command sequence in both tiers. The pipeline is accepted when the trajectory RMSE and the depth-image statistics are within the thresholds set in P13.

### 17.4 Ground-truth separation (enforced)

- `aeris.simulation.groundtruth.GroundTruthService` receives gz model poses, contact events and the `WorldSpec`. It is constructed only by `aeris.evaluation`, dataset-labeling scripts and the world generator.
- The Sensor Bridge publishes GT on a **separate IPC topic namespace** (`gt/*`). The agent-side `GzBridgeClient` does not subscribe to it. That is enforced by a channel allowlist and a test.
- Every `AgentObservation` field carries a `Provenance` (`SENSOR`, `ESTIMATE`, `MISSION_INPUT`, `ORACLE`). `ObservationBuilder` raises an error on `ORACLE` unless the run config sets `oracle_baseline: true`. In that case the run is tagged `ORACLE` in its manifest, the UI and every report.
- **Asymmetric actor-critic exception:** a *critic* may receive privileged state during training only (it is not used at deployment), but only as a declared variant (`critic_privileged: true`) and never by default (§27.8).

---

## 18. Sensor architecture

### 18.1 Sensor inventory

| Sensor | Source (Tier H) | Path to AERIS | Rate (target) | Agent-legitimate? | Phase |
|---|---|---|---|---|---|
| IMU, baro, mag | Gazebo → PX4 | Consumed by PX4 EKF2; raw via MAVLink `HIGHRES_IMU` if needed | PX4 | Yes | 4 |
| GPS | Gazebo → PX4 | PX4 EKF2; status via MAVLink | 5–10 Hz | Yes (unless GPS-denied condition) | 4 |
| Vehicle state estimate | PX4 EKF2 | MAVSDK telemetry | 20–50 Hz | Yes (`ESTIMATE`) | 4 |
| Forward depth camera | gz depth_camera (x500_depth: OAK-D-like) | Sensor Bridge → IPC | 10–15 Hz | Yes | 8 |
| Forward RGB camera | gz camera | Sensor Bridge → IPC (downsampled/JPEG) | 5–15 Hz | Yes | 8 |
| 2D LiDAR | gz gpu_lidar (Hokuyo-like, 270°, 0.1–30 m) | Sensor Bridge → IPC | 10 Hz | Yes | 8 |
| Downward rangefinder | gz (LW20-like) → PX4 distance_sensor | MAVLink `DISTANCE_SENSOR` | 10–20 Hz | Yes | 23 |
| Optical flow | gz (x500_flow) → PX4 | PX4 EKF2 fusion | — | Yes | 23 |
| Collision/contact | gz contact sensor | Sensor Bridge **GT channel** | event | **No**: evaluator-only | 7/9 |
| Model GT pose | gz pose topics | Sensor Bridge **GT channel** | 50 Hz | **No**: evaluator-only | 7 |
| Sim clock | gz `/clock` | Sensor Bridge | per step | Yes (time only) | 8 |

The collision signal the *agent* may use is only what it can sense (proximity from depth/LiDAR) plus PX4 state anomalies. Contact sensors exist for evaluation.

### 18.2 Sensor frame data model

```python
@dataclass(frozen=True, slots=True)
class SensorFrame:
    sensor_id: str            # e.g. "front_depth"
    frame_id: str             # e.g. "front_depth_optical"
    t_sim_s: float            # capture time (gz header stamp)
    seq: int
    provenance: Provenance    # SENSOR
    payload: DepthImage | RgbImage | LaserScan | ...
```

Intrinsics (`CameraInfo`) and extrinsics (`T_base_sensor`) come from configuration that is **generated from the vehicle SDF** (single source), and they are verified at runtime against gz `camera_info`.

### 18.3 Time alignment

Each depth frame is associated with the vehicle pose **interpolated at the frame's `t_sim_s`** from a pose buffer (≥ 2 s history). Frames whose pose cannot be interpolated (gap > 100 ms) are dropped and counted.

### 18.4 Sensor Bridge IPC (ADR-004)

- **Transport:** ZeroMQ PUB/SUB over `ipc://` or `tcp://127.0.0.1:<configured>`. Messages are msgpack with a versioned schema. Images are raw `uint16`/`float32` depth, and RGB can be JPEG-compressed optionally. ZeroMQ is chosen because both Python environments can install it easily and it supports drop-oldest high-water marks.
- **Back-pressure:** a subscriber `CONFLATE`/HWM setting keeps only the latest frame per sensor for control. The recorder uses a separate, lossless (configurable) subscription.
- **Health:** per-topic rate and age are published at 1 Hz. The safety watchdog consumes age.
- **Fallback:** if the bridge Python can install the AERIS package with its dependencies, it may run in-process. The architecture still keeps the IPC seam.

---

## 19. Coordinate-frame conventions

This is **critical**. It is implemented in `aeris/core/frames` and unit-tested in Phase 4. Any change is a spec change.

### 19.1 Frames

| Frame | Symbol | Convention | Origin | Who may use it |
|---|---|---|---|---|
| Gazebo world | `W` | ENU (x East, y North, z Up) | gz world origin | **Evaluator / GT only** |
| PX4 local | `L` | **NED** | PX4 EKF2 local origin (≈ home at arming) | Vehicle adapter only |
| AERIS odometry | `O` | **ENU**; the ENU re-expression of `L` | same origin as `L` | Agent (estimate) |
| AERIS map | `M` | ENU | V1: `M ≡ O` (T_M_O = I, stored explicitly). Becomes ≠ I only with loop-closing localization (§23). | Agent |
| Body (AERIS) | `B` | **FLU** (x Forward, y Left, z Up) | vehicle CoM | Agent |
| Body (PX4) | `B_frd` | **FRD** | vehicle CoM | Vehicle adapter only |
| Camera optical | `C_opt` | z forward, x right, y down (REP-103 optical) | camera center | Perception |
| LiDAR | `S_lidar` | FLU-aligned sensor frame | LiDAR center | Perception |
| Three.js scene | `V` | Y-up right-handed (x = E, y = Up, z = −N) | = `M` origin | Frontend only |

**DECISION (ADR-008):** AERIS internals use **ENU/FLU** (REP-103, matching Gazebo, and more natural for mapping). NED/FRD appear **only** inside the PX4 adapter. The conversion happens once, at that boundary.

### 19.2 Notation

- `T_A_B` ∈ SE(3) maps coordinates expressed in frame B into frame A: `p_A = T_A_B · p_B`.
- Composition: `T_A_C = T_A_B · T_B_C`.
- Rotations are stored as unit quaternions `(w, x, y, z)` in code, normalized after every composition, with a Hamilton product.
- Angles are in radians internally. Yaw ψ is measured from the frame's x-axis, positive counter-clockwise in ENU. Displays use degrees.

### 19.3 Fixed conversions

- **ENU ↔ NED (vectors):** `R_NED_ENU = [[0,1,0],[1,0,0],[0,0,−1]]` (an involution, so it is its own inverse). `(x_n, y_n, z_n) = (y_e, x_e, −z_e)`.
- **FLU ↔ FRD:** `R_FRD_FLU = diag(1, −1, −1)` (also an involution).
- **Attitude:** `R_L_Bfrd = R_NED_ENU · R_O_B · R_FLU_FRD`, where R_FLU_FRD = R_FRD_FLU.
- **Yaw:** `ψ_NED = wrap(π/2 − ψ_ENU)`. PX4 yaw 0 = North; AERIS yaw 0 = East.
- **Camera optical in body:** `R_B_Copt = [[0,0,1],[−1,0,0],[0,−1,0]]`, so optical +z → body +x, optical +x → body −y, optical +y → body −z. The full `T_B_C = (R_B_Copt · R_mount, t_mount)` comes from the SDF sensor pose.
- **Three.js:** `(x_V, y_V, z_V) = (x_M, z_M, −y_M)`.
- **Evaluator alignment:** `T_W_O` is computed once per episode from the GT spawn pose and PX4's reported EKF origin, then refined by least squares over the first N seconds of hover. It is used **only** to compare agent estimates to GT (localization error), never by the agent.

### 19.4 Required tests (Phase 4 and later)

- Round trips: ENU→NED→ENU and FLU→FRD→FLU are identity (property-based, random vectors and quaternions).
- Yaw conversion at the cardinal points: ENU yaw 0 (East) ↔ NED yaw π/2; ENU π/2 (North) ↔ NED 0.
- Known camera ray: a point 1 m in front of the camera (optical (0,0,1)) with the vehicle at ψ_ENU = π/2 maps to +y (North) in `O`.
- Depth → map: a synthetic depth image of a plane at known distance projects to the expected voxel set.
- A live SITL test (P5/P8) commands +x ENU velocity and checks that PX4 NED velocity is +y (north component 0, east component +), and that gz GT displacement is along world East after alignment.
- A frame-graph test ensures every published `frame_id` has a path to `M`.

---

## 20. Perception

### 20.1 Progression

1. **Geometric perception (P8, P10, P11):** depth → point cloud (pinhole back-projection with `CameraInfo`, range clipping, invalid-pixel masking); LiDAR scan → points. Obstacle and free space come from **geometry, not learning**. This is deterministic and testable.
2. **Learned encoders for RL (P17–P19):** small CNN over downsampled depth (e.g., 64×48 or 80×60) or 1-D conv over LiDAR rays, trained end-to-end with the PPO objective. They are part of the policy network, not a separate perception product.
3. **Target detection (P20):** a staged detector for defined target classes (§31).
4. **Optional:** semantic segmentation, and an RGB learned encoder (RQ3, gated).

### 20.2 Rules

- No large foundation models in V1 perception. A later phase may propose one only with a research justification and measured latency on this Mac.
- Perception preprocessing (resize, normalization, clipping) lives in `aeris.learning.spaces` / `aeris.perception.*`. It is imported by **both** training and deployment. Duplication is a defect.
- Every perception component reports inference latency p50/p95 on this Mac (CPU and MPS where relevant).

---

## 21. Mapping

### 21.1 Representation (DECISION, ADR-009)

- **3D voxel log-odds occupancy** in `M`:
  - resolution 0.15–0.25 m (configured; default 0.2 m);
  - sparse block hashing (e.g., 8³ blocks in a dict of NumPy arrays) with Numba ray integration;
  - clamped log-odds (l_min, l_max);
  - inverse sensor model with l_occ, l_free and a max-range free-space policy (rays with no return mark free up to a configured range; not treated as "occupied at max").
- **2D projection for planning and exploration in V1:** the vehicle explores within a fixed **altitude band** [z_lo, z_hi]. A 2D cell is occupied if any voxel in the band (inflated by vehicle radius + margin) is occupied, free if all are observed free, and unknown otherwise. The 2D grid feeds A*, frontier detection and the learned-policy egocentric crops.
- **Explored set** E(t): 2D cells observed (free or occupied) by the agent's own sensors.

Why this choice: it is correct, simple and fast enough on the CPU. Full 3D exploration (multi-altitude) is an extension, not V1. OctoMap-style octrees are not needed at these scales. There is also no maintained, native Apple-Silicon Python binding that justifies the dependency (recorded as an ASSUMPTION; revisit if the map scale grows).

### 21.2 Pipeline

```
Depth/LiDAR SensorFrame ─► points in sensor frame ─► T_M_S(t) = T_M_O · T_O_B(t) · T_B_S
   ─► ray integration (free along ray, occupied at hit) ─► voxel map ─► 2D band projection
   ─► frontier extraction / inflation / planner costmap ─► map deltas (for UI/replay)
```

### 21.3 Map evaluation (evaluator-only)

Evaluation compares against the `WorldSpec` GT occupancy **within observed regions**:
- occupied-cell precision and recall;
- free-space false-occupied rate;
- map coverage;
- with GT pose vs with EKF pose (drift contribution).

### 21.4 Semantic map

Detections become target hypotheses (position + covariance + class + confidence) held in the **World Model**, not as voxels (P20/P21). Semantic voxels are optional.

---

## 22. Localization

Three levels are **named distinctly in code, UI and reports** (the prompt's "don't call ground truth SLAM" rule):

| Level | Name in AERIS | Source | Agent-legitimate |
|---|---|---|---|
| L-GT | `GroundTruthPose` | gz model pose | **No**: evaluator / oracle baseline only |
| L-EST-GPS | `Px4EkfPose` (GPS-aided) | PX4 EKF2 fusing simulated IMU/GPS/baro/mag | **Yes** (default V1). It is an estimate with realistic sim noise, *not* ground truth. |
| L-EST-NOGPS | `Px4EkfPose` (flow/vision-aided), `DepthOdometryPose` | PX4 EKF2 without GPS + optical flow/rangefinder, and/or AERIS depth odometry fed as external vision | Yes (P23) |
| L-SLAM | `SlamPose` | map-based pose + loop closure | Yes (only if actually implemented; §23) |

- `PoseSource` protocol: `pose_at(t_sim) -> PoseEstimate(T_M_B, covariance, provenance, source_name)`.
- **Noisy-pose experiments** (the step before GPS-denied): a `NoisyPoseSource` wrapper adds controlled drift (random walk) and noise to `Px4EkfPose`. It is used to measure mapping and exploration sensitivity. This is labeled as synthetic degradation.
- **Localization error metrics** (§41): ATE and RPE against GT after the `T_W_O` alignment (§19.3).

---

## 23. SLAM roadmap

| Stage | What | Claim allowed |
|---|---|---|
| 0 | Mapping with `Px4EkfPose` (GPS-aided) | "Mapping with estimated (GPS-aided) pose", **not SLAM** |
| 1 | Mapping with `NoisyPoseSource` | "Mapping under synthetic pose drift" |
| 2 | GPS-denied: EKF2 + flow/rangefinder; AERIS depth/scan odometry (ICP / correlative scan matching) → PX4 external-vision input | "Odometry-based localization (no loop closure)" |
| 3 (optional) | 2D LiDAR pose-graph SLAM: scan matching + loop closure + graph optimization (Mac-native, Python + SciPy/`gtsam` if an arm64 wheel exists, otherwise a small custom Gauss–Newton) | "SLAM", **only** if loop closures are detected, the map/pose are jointly optimized, and ATE improvement over stage 2 is measured |
| 3-alt | ROS-centric SLAM (RTAB-Map / Cartographer / ORB-SLAM3) via the Linux fallback + ROS 2 adapter | Same criteria |

SLAM is **not a V1 gate**. The GPS-denied phase (P23) decides between stage 2 and optional stage 3 based on RQ7 needs.

---

## 24. Classical navigation

Components (only those that serve the RQs):

| Component | Purpose | Phase |
|---|---|---|
| Waypoint follower (PX4 position setpoints + arrival criteria) | L2/L3 missions | 6 |
| Path follower (carrot / pure-pursuit on a polyline → velocity setpoints with speed and curvature limits) | Executes planner paths; shared by classical and learned-exploration pipelines | 10 |
| Reactive avoidance (sector-based, VFH-style histogram of depth/LiDAR clearance → steering) | Classical local-navigation baseline for learned local navigation (RQ3 / L8) | 10 |
| 2D A* on inflated occupancy (8-connected, octile heuristic, unknown-cell cost configurable) | Global planning in known or partially known maps; used by frontier and learned-subgoal execution | 12 |
| 3D A* on voxels | Only if a phase needs multi-altitude planning (not V1) | deferred |
| RRT / RRT* | **Not implemented** unless a continuous-space planning RQ appears (grid planning suffices for these worlds) | deferred |

Measured metrics: collision rate, minimum clearance, path efficiency, mission completion, command smoothness, planning latency (§41).

---

## 25. Classical exploration

| Baseline | Definition | Role |
|---|---|---|
| **Random** | Uniformly random reachable subgoal in the same egocentric action space as the learned policy, executed by the same planner and follower | Lower bound; sanity check for H0.1 |
| **Nearest frontier** (Yamauchi 1997) | Frontier cells = free cells adjacent to unknown. Cluster (connected components ≥ min size) and go to the nearest reachable cluster centroid (A* path length). | Standard classical baseline |
| **Utility frontier** | Score = expected information gain (unknown cells within sensor footprint at the candidate, from a raycast estimate) / (path length + λ·turn cost). Greedy with replanning on arrival or on a significant map change. | **Strong** classical baseline for RQ1. Parameters tuned on **validation** worlds only, with the same tuning budget recorded. |

Fairness rules (RQ1):
- identical sensors, map, planner, follower, shield, speed limits and time budget;
- baselines get a tuning budget (grid search on validation worlds) that is documented alongside the learned policy's hyperparameter budget.

---

## 26. RL role

### 26.1 Where RL is used, and why

| Candidate | Use RL? | Justification |
|---|---|---|
| Stabilization / attitude / motor control | **No** | PX4's job. Safety rule. |
| Collision safety | **No** | Deterministic shield (§16). Safety must be verifiable. |
| Global path planning in a known map | **No** | A* is optimal and verifiable. |
| **Exploration strategy under partial observability** | **Yes (primary)** | Classical frontier methods are greedy and myopic. Learning could exploit structural regularities of environment families (corridors, rooms). This is a genuine open question (RQ1/RQ2). |
| **Local navigation from raw depth/LiDAR to a goal** | **Yes (secondary)** | Tests learned reactive navigation against classical reactive avoidance under partial observability (L8, RQ3). It is also a simpler stepping stone for debugging the PPO pipeline before exploration. |
| **Target-search strategy** | **Yes (extension of exploration)** | RQ6. |
| High-level mission decisions (e.g., when to return) | **No (V1)** | A deterministic budget rule is safer and explainable. Possible later ablation. |

### 26.2 Action abstraction (hierarchical)

- **Learned local navigation (P17):** action = body-frame horizontal velocity + yaw rate `(v_x, v_y, ψ̇)` at 10 Hz. Altitude is held by the altitude-hold setpoint. Continuous, bounded, shielded.
- **Learned exploration (P19):** action = **subgoal** in an egocentric polar set (DECISION default: 12 bearings × 2 ranges = 24 options + 1 "rotate-in-place scan" = 25 discrete actions). A subgoal is snapped to the nearest reachable known-free cell. The shared planner + follower + shield execute it until arrival, blockage or a timeout of k seconds. Decision frequency is event-driven (≈ 0.2–1 Hz). This keeps learning at the level where it can add value and keeps safety deterministic.

### 26.3 What is learned vs deterministic (viva table; kept current in docs)

| Component | Learned? |
|---|---|
| PX4 flight control, EKF2 | No (PX4) |
| Safety supervisor, shield, watchdogs | No |
| Mapping, frontier detection, A*, path follower | No |
| Exploration subgoal selection (learned variant) | **Yes (PPO)** |
| Local velocity policy (learned variant) | **Yes (PPO)** |
| Target detector (stage ≥ 2) | **Yes (supervised)** |
| Target localization / association | No (geometric + clustering) |
| Mission executive | No (state machine) |
| Language parser (optional) | Maybe (constrained LLM, schema-validated) |

---

## 27. PPO methodology

### 27.1 Implementation policy

- **In-house PPO in PyTorch** (`aeris/learning/ppo`), a readable implementation of about 600–900 lines. It follows the well-documented "37 implementation details" conventions (CleanRL-style), cited in `docs/reinforcement-learning.md`. There is no hidden framework dependency.
- **Reference sanity check:** on one toy task, AERIS PPO's learning curve is compared against a reference implementation (e.g., Stable-Baselines3, installed as a *dev-only* dependency). The purpose is to catch gross bugs, not to match exactly.

### 27.2 Components

| Component | Specification |
|---|---|
| Observation encoder | Per modality: depth → small CNN (3 conv layers, e.g., 32-64-64 channels, stride 2, ReLU) → 256; LiDAR rays → 1-D conv or MLP → 128; egocentric map crops (C×64×64: free / occupied / unknown / visited / [frontier] / [detections]) → CNN → 256; low-dim state (goal vector, velocity, heading sin/cos, time remaining) → MLP → 64. Concatenate → trunk MLP 256. |
| Memory core | Optional GRU (hidden 256) after the trunk (§28). |
| Actor head | Continuous: mean via linear layer; **state-independent log-std** parameter (init −0.5), diagonal Gaussian; actions clipped to bounds in `ActionDecoder`, with log-prob computed on the *unclipped* sample (documented CleanRL convention). Discrete: categorical logits over 25 subgoals, with **action masking** for unreachable subgoals (masked logits = −1e9). Masks are computed from the agent's own map (legitimate information). |
| Critic head | Linear → scalar V(s). Shared trunk by default. Separate-network variant as an ablation if value interference is suspected. |
| Initialization | Orthogonal; gain √2 for hidden layers, 0.01 for the policy output, 1.0 for the value output. |
| Rollout storage | Tensors [T, N, …] for obs, actions, log-probs, rewards, dones (terminated), truncateds, values, action masks, GRU h₀ per chunk. |
| Returns / GAE | See §27.3. |
| Loss | L = L_clip + c_v·L_V − c_e·H[π] |
| Optimizer | Adam (lr 3e-4, eps 1e-5); linear LR annealing (configurable) |
| Gradient clipping | global norm 0.5 |
| Batching | Feed-forward: shuffle all T·N samples into M minibatches. Recurrent: minibatches **of whole env sequences** (split N envs into M groups), BPTT over the full rollout length T with stored initial hidden states and masking at dones. |
| Epochs | K = 4 (default), with optional early stop when approx-KL > 1.5·target_kl (target 0.02, logged) |
| Advantage normalization | Per minibatch (mean 0, std 1) |
| Value loss | Clipped value loss optional (default off; ablation noted); c_v = 0.5 |
| Entropy | c_e = 0.01 (discrete), 0.0–0.005 (continuous), configurable and annealable |
| Observation normalization | Running mean/var for low-dim vectors (saved in the checkpoint; frozen at eval); images scaled by fixed physical ranges (depth / max_range), not running statistics |
| Reward scaling | Optional return-based reward scaling (running std of discounted return), stored in the checkpoint |
| Checkpointing | Every K updates + best-on-validation: model, optimizer, normalizers, RNG states (python/numpy/torch), global step, config hash, git SHA, env-version hash |
| Resume | Bitwise-identical continuation on CPU (tested) |
| Evaluation | Deterministic policy (mean / argmax) **and** stochastic policy reported. Evaluation environments are separate from training environments and use validation-split worlds during training. |

Default hyperparameters (DEV config; FULL values are tuned on validation worlds with a recorded budget): γ = 0.99, λ = 0.95, ε = 0.2, T = 128 (local nav) / 64 decisions (exploration), N = 16–64 envs, M = 4, K = 4.

### 27.3 Mathematics (implemented and tested exactly)

- **Termination vs truncation** (Gymnasium semantics): `terminated` = true terminal (collision, mission success). `truncated` = time limit or other artificial cut.
- **TD residual:** δ_t = r_t + γ·(1 − term_t)·V̂_{t+1} − V(s_t), where V̂_{t+1} = V(s_{t+1}) normally, **and on truncation V̂_{t+1} = V(s_final^{(t)})**, the value of the true final observation before auto-reset (the bootstrapping-on-truncation rule).
- **GAE:** Â_t = δ_t + γλ·(1 − done_t)·Â_{t+1}, where done_t = term_t ∨ trunc_t, so the recursion does not cross episode boundaries.
- **Returns (value targets):** R_t = Â_t + V(s_t).
- **Ratio:** ρ_t = exp(log π_θ(a_t|s_t) − log π_θ_old(a_t|s_t)).
- **Clipped surrogate:** L_clip = −E[min(ρ_t·Â_t, clip(ρ_t, 1−ε, 1+ε)·Â_t)].
- **Diagnostics logged per update:**
  - approx-KL, estimated as E[(ρ − 1) − log ρ];
  - clip fraction;
  - entropy;
  - value loss;
  - explained variance 1 − Var[R − V]/Var[R];
  - gradient norm;
  - SPS (steps per second);
  - learning rate.

### 27.4 Mandatory PPO tests (Phase 14)

1. GAE matches a hand-computed reference on a 5-step trajectory with a mid-episode termination and a truncation, for γ, λ ∈ {(0.99, 0.95), (1, 1), (0.9, 0)}.
2. Truncation bootstrapping uses the final-observation value, not the reset observation.
3. With θ = θ_old, ρ ≡ 1, clip fraction = 0 and L_clip = −E[Â].
4. Clip behavior: for positive Â and ρ > 1 + ε the gradient w.r.t. log π is zero; the mirror case holds for negative Â.
5. Gaussian log-prob and entropy match `torch.distributions` closed forms. Masked categorical assigns zero probability to masked actions and its entropy ignores them.
6. Rollout buffer shapes, and recurrent minibatch sequence integrity (no cross-env mixing).
7. **Known-answer learning tests** (SMOKE, < 2 min CPU): (a) a contextual bandit converges to the optimal arm; (b) a 1-D point-goal task reaches ≥ 95 % success; (c) a memory task (cue at t = 0, answer at t = T) that **only the GRU variant solves**, which proves the memory path works.
8. Determinism: same seed → identical first-N-update losses on CPU.
9. Checkpoint → resume → identical subsequent losses.
10. NaN guard: detection trips on injected NaN and the trainer halts with diagnostics.

### 27.5 Rewards (defined per task, versioned, unit-tested)

The reward for each task lives in `aeris/learning/envs/rewards.py` as a pure function over `(prev_state, action, state, info)`. Each has tests with hand-built transitions. Reward-version strings are recorded in manifests.

- **Local navigation:** r = +Δ(geodesic distance to goal decrease) · w_p − w_t − w_c·𝟙[collision] − w_s·𝟙[shield intervention] + R_goal·𝟙[success]. Geodesic distance is precomputed on the FastSim GT map *for reward only*. This is legitimate: reward is a training signal and is not an observation. It is recorded as a privileged-reward note, and the agent never observes it.
- **Exploration:** r = w_e·(new explored cells in the agent's map, in m²) − w_t·(decision duration) − w_c·𝟙[collision] − w_s·(shield interventions during subgoal execution) − w_f·𝟙[subgoal failed]. Plus, in SAR, + R_det·(newly confirmed true targets). For SAR, true/false is judged by evaluator matching during *training* in FastSim, which is again reward-only privileged information.
- **Reward-hacking watchlist** (checked by trajectory audits in P15/P19): spinning in place to farm small gains, oscillating subgoals, hugging walls to exploit sensor-model artifacts, exploiting map noise to "discover" phantom cells, stalling near the time limit, and farming false-positive detections.

### 27.6 Environment contract (P13)

- Gymnasium `Env` API; vectorized via `gymnasium.vector` (async sub-processes) or a native batched FastSim.
- `ObservationBuilder` and `ActionDecoder` live in `aeris.learning.spaces` and are shared with Tier H deployment (`LearnedExplorer` / `LearnedLocalNav` wrappers). A test verifies identical outputs for identical inputs across both call sites.
- The observation spec is serialized into every checkpoint. Deployment refuses a checkpoint whose spec mismatches the builder.

### 27.7 Training feasibility (targets; measured in P13–P15)

| Task | Steps (DEV) | Steps (FULL) | Est. wall time on M5 Air (≥ 2 k SPS) |
|---|---|---|---|
| Local nav (MLP) | 2 M | 10–20 M | 20 min / 1.5–3 h |
| Exploration (decisions; each ≈ 10–50 low-level ticks) | 0.2 M decisions | 2–5 M decisions | 1–2 h / 8–25 h (split across sessions; resume) |

If FULL budgets prove infeasible, the options in order are: reduce map crop size; lower the decision rate; use remote Linux (Option C); or report reduced-budget results *as such*.

### 27.8 Allowed variants (declared, never silent)

- `critic_privileged`: asymmetric actor-critic. Allowed as a declared variant, because the critic is discarded at deployment.
- `oracle_baseline`: the actor receives GT. Allowed **only** as an upper-bound baseline, labeled `ORACLE`.

---

## 28. Memory

- **Architecture:** encoder → GRU(256) → heads. The hidden state is carried across steps within an episode.
- **Reset:** `h_t ← h_t · (1 − done_{t−1})` applied at the input of every step. In rollouts, the initial hidden state of each chunk is stored. In training, sequences are replayed from those stored states with the same masking.
- **Tests:**
  - (a) *leak test*: the output after a done equals the output of a freshly initialized agent on the same observation;
  - (b) the memory known-answer task (§27.4 #7c);
  - (c) hidden-state shape and device checks;
  - (d) deployment wrapper resets on episode/mission start and after any safety takeover that invalidates the context (configurable; logged).
- **Comparison (RQ2):** {MLP, GRU} × {map input, no map input}. The model-capacity difference is controlled by matching parameter counts (±10 %) via trunk width.

---

## 29. Learned navigation

- **Task (P17): PointGoal navigation in cluttered worlds at a fixed altitude band.**
  - Observation: forward depth (downsampled) and/or LiDAR rays, goal vector in `B` (computed from `Px4EkfPose`-equivalent estimate + goal in `O`), body velocity, previous action.
  - Action: `(v_x, v_y, ψ̇)` bounded.
  - Shield active during training (shielded RL) and evaluation. A "no-shield" ablation runs in FastSim only.
- **Baselines:** reactive VFH-style avoidance + goal attraction; A* on the agent's online map + path follower.
- **Metrics:** success rate (reach within r_goal before T), SPL (success weighted by path length: SPL = (1/N)·Σ S_i·ℓ_i / max(p_i, ℓ_i)), collision rate, minimum clearance, shield-intervention rate, smoothness.
- **Tier H evaluation** on validation (during development) and test worlds (final).

---

## 30. Learned exploration

- **Task (P19):** maximize explored area within T = 180 s (configurable) in unknown worlds, starting from a random valid start pose.
- **Observations (agent-legitimate):**
  - egocentric map crops from the agent's own 2D map, at two scales (e.g., 64×64 at 0.2 m and 64×64 at 0.8 m), heading-aligned;
  - channels: free / occupied / unknown / visited-trail / [frontier mask: ablation factor] / [detections: SAR];
  - optional forward-depth encoding;
  - low-dim: time remaining, current speed, fraction explored (by the agent's own map);
  - GRU memory (variant).
- **Actions:** 25-way masked subgoal (§26.2).
- **Executed by:** the same planner + follower + shield as the frontier baselines, which gives a fair comparison.
- **Evaluation metrics (§41):** C(t) curve, C(60/120/180 s), time to 50/80 % coverage, distance travelled, revisit ratio R, collision rate, shield-intervention rate, decision latency.
- **Fairness:** same compute-independent budget per episode (sim time). The learned policy's inference latency is measured and must not exceed the decision period, otherwise the result is reported with that caveat.

---

## 31. Target detection

- **Target classes are defined by AERIS worlds**, and nothing more is implied:
  - `marker_red` (red emergency marker panel);
  - `aruco_tag` (fiducial, for calibration);
  - `casualty_mannequin` (a simple humanoid mannequin mesh, introduced in stage 3).

  Reports say "detected `casualty_mannequin`", never "survivor", unless the mission definition maps a class to a mission label and the UI shows that mapping.
- **Staged progression:**
  1. **Stage 1 — Known marker, classical:** HSV color segmentation for `marker_red`, and OpenCV ArUco detection (PX4 ships an `aruco` world). This validates the camera → geometry → world-position chain.
  2. **Stage 2 — Learned detector for 2–3 classes:** a small license-compatible detector, e.g., torchvision SSDLite-MobileNetV3 or FasterRCNN-MobileNet (BSD). **Ultralytics YOLO is excluded by default because of the AGPL-3.0 license**; it may be used only after an explicit license decision by the user. Training data is auto-labeled from Gazebo renders using GT poses and model bounding volumes (privileged data used offline for labels only). Train/val/test images come from disjoint world seeds.
  3. **Stage 3 — Occlusion, distance and lighting variation; unseen worlds.**
- **Localization of detections:** bounding-box center pixel → depth (median over the box region from the aligned depth image) → 3D point in `C_opt` → `M` via T_M_C(t). A target hypothesis is created, then **associated** with existing hypotheses (gated nearest-neighbour, Mahalanobis gate). A hypothesis is **confirmed** after ≥ k detections within a window.
- **Metrics:**
  - per frame: precision, recall, AP@0.5 (offline test set);
  - per mission: target recall, reports precision, false-positive rate per minute;
  - localization error (m) of confirmed targets;
  - detection distance distribution;
  - latency;
  - **mission impact** (SAR metrics with a detector vs with an oracle detector, the latter labeled ORACLE).

---

## 32. Search-and-rescue mission

### 32.1 Formal mission definition

Given:
- a `WorldSpec` with search volume V_s, launch pose p₀ and N ∈ [3, 8] targets of defined classes;
- a time budget T_max;
- a battery-simulation budget.

The drone MUST:
1. preflight-check, arm and take off to z_search;
2. explore V_s;
3. avoid collisions;
4. detect, localize and confirm targets;
5. avoid redundant coverage;
6. return to launch and land when (all N found, if N is known to the mission) ∨ (return-time estimate t_rtl + margin ≥ remaining time) ∨ (battery-sim threshold) ∨ (operator abort);
7. produce a mission report.

**Knowledge settings:** N-known (the mission states the target count) and N-unknown (the drone must decide when to stop). Both are reported separately.

### 32.2 Mission executive (deterministic state machine)

States: `IDLE → PREFLIGHT → TAKEOFF → SEARCH (explore) ⇄ INVESTIGATE (approach & confirm hypothesis, bounded time) → RETURN → LAND → COMPLETE`. Also `ABORTED` and `FAILSAFE` from any state.

The exploration strategy (random / frontier / learned) is a **plug-in** inside SEARCH. Everything else is identical across compared methods.

### 32.3 Metrics

- Targets found (confirmed within r_match = 2.0 m of a GT target; Hungarian matching).
- Target recall N_found/N and report precision.
- Time to first target (TTFT) and time to all targets (TTA; censored at T_max, analysed with survival-style reporting).
- Coverage C(T) and distance.
- Collisions.
- Return success (landed within r_home = 1.5 m of launch, disarmed).
- **Mission success** = no collision ∧ return success ∧ recall ≥ required fraction (mission-defined; default 1.0 for N-known).

### 32.4 Mission report and replay

Automatically generated at the end of each mission:
- JSON + Markdown summary;
- trajectory plot;
- coverage curve;
- target table (est. vs GT, error, time found);
- safety events;
- replay file link.

---

## 33. Generalization

### 33.1 World families (procedural `WorldSpec` generators)

| Family | Description | Split usage |
|---|---|---|
| F1 Rubble field | Open outdoor area, scattered blocks/debris, varying density | train / val / test |
| F2 Office | Rooms + corridors + doors (graph-based floorplan generator) | train / val / test |
| F3 Warehouse | Shelf aisles, pallets, open bays | train / val / test |
| **F4 Collapsed structure** | Irregular partially blocked rooms, tilted slabs within the altitude band, narrow passages | **OOD test only**, never used for training, tuning or model selection |

### 33.2 Splits (DECISION, ADR-010)

- Seed ranges: train `[0, 10 000)`, val `[10 000, 10 200)`, test-ID `[20 000, 20 100)`, test-OOD (F4) `[30 000, 30 100)`.
- The ranges and a hash of generated test WorldSpecs are frozen in `configs/worlds/splits.lock`.
- The **training entry point refuses** to generate worlds from val/test ranges. The **tuning entry point refuses** test ranges. Both are unit-tested.
- Test worlds are evaluated only after pre-registration (§38.5).

### 33.3 Perturbation axes (tested separately, one factor at a time, then combined)

- Obstacle density.
- Layout family.
- Target positions.
- Start poses.
- Lighting (RGB/detector only).
- Depth noise σ and dropout.
- LiDAR noise.
- Pose drift (NoisyPoseSource).
- GPS degradation (eph inflation, dropouts).
- Wind (Gazebo `windy` world plugin; Tier H only).

---

## 34. GPS-denied navigation

### 34.1 Condition definition (exact)

- In Tier H, GPS is disabled at the PX4 level through EKF2 GPS fusion control (the exact parameter names, e.g., `EKF2_GPS_CTRL`, are verified in P23 for the pinned PX4 version) and/or by removing the GPS sensor from the model.
- **The agent may use:** IMU (via PX4), barometer, downward rangefinder, optical flow (x500_flow sensors), forward depth, LiDAR, and its own odometry estimates.
- **The agent must not use:** gz GT pose (enforced by §17.4), or any GPS-derived quantity.
- **`x500_vision`'s external-vision odometry is ground-truth-derived in simulation.** If used, it is labeled `ORACLE-VIO` (with injected noise) and serves only as an upper bound. It is never reported as GPS-denied capability.

### 34.2 Pipelines compared

| ID | Localization | Label |
|---|---|---|
| G0 | EKF2 with GPS | baseline |
| G1 | EKF2 with optical flow + rangefinder (PX4-native GPS-denied path) | GPS-denied |
| G2 | G1 + AERIS depth/LiDAR odometry (ICP / scan matching) sent to PX4 as external vision (`ODOMETRY` / `VISION_POSITION_ESTIMATE` via pymavlink) | GPS-denied |
| G3 (optional) | G2 + loop-closure SLAM (§23 stage 3) | GPS-denied + SLAM |
| GO | ORACLE-VIO | upper bound |

### 34.3 Metrics

- ATE RMSE and RPE (per 10 m travelled) vs GT.
- Mission metrics (C(T), SAR recall, return success).
- Collision rate.
- Divergence events (EKF resets or innovation failures).

The learned policies are evaluated **unchanged** (trained with the pose-noise model) and, optionally, fine-tuned with drift randomization. Both are reported.

---

## 35. Moving-target tracking

- **Pipeline:** RGB detection → association → **constant-velocity Kalman filter** in `M` (state [p, v], process noise q, measurement from depth-localized detections) → short-horizon prediction → pursuit setpoint with **stand-off distance** and altitude hold. The pursuit setpoint is converted to velocity commands via the follower and passes through the Safety layer. Tracking commands never bypass the shield or the envelope.
- **Targets:** scripted Gazebo movers (a velocity-controlled model on waypoints; speed ≤ 0.6 × drone max speed). A matching analytic mover exists in FastSim for tests.
- **Metrics:**
  - track continuity (fraction of time the target is tracked);
  - target position RMSE of the KF vs GT;
  - FOV retention;
  - stand-off error;
  - minimum separation (safety);
  - reacquisition time after occlusion.
- **Model/effort:** Sonnet/High. The design is classical estimation + control. Escalate to Opus only if learned tracking is proposed.

---

## 36. Language missions (optional, advanced; Phase 32)

- **Architecture:** natural language → `MissionParser` → **typed `MissionSpec`** (Pydantic; the same type the UI mission builder produces) → validation (schema + semantic checks against the mission map's named regions) → Mission Executive.
- **Region names** (e.g., "east wing") come from **mission-provided annotations** (operator knowledge, like a floor plan), not from GT the agent could not have. Allowed `MissionSpec` constructs:
  - region ordering/priority (`search_order`, `priority_regions`);
  - target classes of interest;
  - stop conditions;
  - return rules;
  - contingencies (`on_blocked: replan_alternative_route`).
- **Parser options:**
  - (a) a deterministic grammar for a controlled English subset (baseline);
  - (b) a local LLM via Ollama (already installed on this machine), constrained to JSON schema output, with validation and a mandatory confirmation step in the UI.

  An LLM output can **never** produce commands outside `MissionSpec`, and it can never bypass Safety.
- **Research test:** does the conditioned mission change behaviour as instructed? Compliance metrics:
  - fraction of coverage in priority regions within the first X s;
  - the order in which regions were first entered;
  - task success.

  These are measured against the unconditioned mission on the same worlds. Parser accuracy is measured on a held-out instruction set.
- Only proceeds after the V1 SAR mission passes its gate (P21) and the Phase 31 integration audit.

---

## 37. Multi-drone extension (optional; Phase 33)

- **Platform constraint:** multi-vehicle Gazebo simulation is **Linux-only** (VERIFIED). Options:
  - Tier H via **Option B1 Docker** or **Option C remote Linux**;
  - FastSim multi-agent on the Mac (FastSim natively supports multiple point-mass agents).
- **Architecture:** per-drone AERIS core instances (each with its own Safety supervisor and vehicle adapter, using PX4 instance ports) + a **Team Coordinator**:
  - frontier/region task allocation: a greedy auction or Hungarian assignment as the baseline; learned allocation only if an RQ justifies it;
  - a shared map via map-delta exchange over a simulated comms channel with configurable bandwidth, latency and dropout;
  - inter-drone separation as a hard safety constraint (deconfliction in the Safety layer using broadcast estimates).
- **RQ (if executed):** team coverage vs N and communication degradation; the effect of allocation strategy.
- **Model/effort:** Opus / Extra High (methodology + safety of multi-agent interaction).

---

## 38. Experiment architecture

### 38.1 Concepts

- **Config:** Pydantic-validated YAML. Tiers: `smoke` (CI, < 5 min), `dev` (≤ 1 h), `full` (hours). Config composition is an explicit `extends:` list plus CLI overrides (`key=value`). The resolved config is frozen, hashed (SHA-256 of canonical JSON) and saved.
- **Run:** one execution (training or evaluation). Directory `results/runs/<YYYYMMDD-HHMMSS>_<name>_<cfghash8>/` containing:
  - `manifest.json`
  - `config.resolved.yaml`
  - `metrics.jsonl`
  - `events.jsonl`
  - `eval/` (per-episode JSONL)
  - `logs/` (px4, gz, bridge, aeris)
  - `replays/` (MCAP)
  - checkpoints under `models/<run_id>/`
- **Experiment:** a declared matrix (methods × seeds × world splits × conditions) in `configs/experiments/<exp>.yaml`, expanded into runs by `aeris.experiments.runner`, plus an `analysis` spec (metrics, statistical tests, plots). Results aggregate into `results/experiments/<exp_id>/summary.{json,md}` and figures generated **from the stored data only**.
- **Registry:** `results/index.sqlite`, rebuildable by scanning run directories. The filesystem is the source of truth.

### 38.2 Manifest (mandatory fields)

- `run_id`, `experiment_id`, `timestamp_utc`
- `git_sha` + dirty flag (dirty runs are tagged and excluded from final reporting)
- `aeris_version`, `config_hash`
- Task, algorithm/method, reward version, observation-spec hash
- Seeds (master + derived)
- World split + seeds + WorldSpec hashes
- Sensor configuration
- Tier (H/F)
- Device (cpu/mps)
- Software versions (Python, torch, numpy, mavsdk, PX4 tag/SHA, Gazebo version, QGC if connected, macOS)
- Hardware snapshot
- Training steps / wall time / SPS profile
- Checkpoint path + hash
- Provenance flags (`ORACLE`, `critic_privileged`, `gcs_connected`, `dirty`)
- Status (`completed | failed | aborted | invalid`) + reason

### 38.3 Tracking

- `metrics.jsonl` is the source of truth; the backend and UI read it.
- A TensorBoard writer is optional (dev convenience; `tensorboard` is a dev extra).
- No cloud trackers.

### 38.4 Evaluation runner

- `EpisodeRunner(tier, method, world, start, seed)` produces an `EpisodeResult` (per-episode metrics + safety events + replay path).
- Tier H runs sequentially (single vehicle) with a full restart between episodes (§17.2). Tier F runs vectorized.
- A failed or invalid episode (sim crash, EKF failure at spawn) is **recorded as invalid, with its cause**, re-run once, and reported in counts. It is never silently dropped.

### 38.5 Pre-registration

Before any **test-split** evaluation, `configs/experiments/preregistration/<exp>.md` is committed. It records:
- hypotheses;
- metrics;
- statistical tests;
- the number of seeds and episodes;
- exclusion rules;
- the checkpoint-selection rule (chosen on validation only).

The runner refuses test-split evaluation without a committed pre-registration whose hash is stored in the manifest.

---

## 39. Replay architecture

### 39.1 Format (DECISION, ADR-012)

- **MCAP** container (an open robotics log format; pure-Python `mcap` reader/writer; no ROS required) with JSON-schema'd or msgpack channels. Foxglove Studio can open MCAP files for independent debugging (optional tool).
- PX4's own **ULog** (`.ulg`) from SITL is linked in the manifest, not duplicated.

### 39.2 Channels

| Channel | Content | Detail level |
|---|---|---|
| `/mission/events` | state transitions, target confirmations, aborts | L0+ |
| `/vehicle/state` | VehicleState (estimate) | L1 (10 Hz) |
| `/autonomy/action` | HighLevelAction + policy info (action probabilities / value / chosen subgoal) | L1 |
| `/safety/events`, `/safety/intervention` | validator rejections, shield deltas | L1 |
| `/gt/pose`, `/gt/contacts` | ground truth (**flagged privileged**; displayed only in evaluation/replay GT overlays) | L1 |
| `/map/deltas` + `/map/keyframe` (every 10 s) | changed 2D cells; voxel keyframes optional | L2 |
| `/perception/detections` | bounding boxes + hypotheses | L2 |
| `/plan/path`, `/plan/frontiers` | planned path, frontier clusters | L2 |
| `/sensors/depth_small`, `/sensors/rgb_jpeg` | downsampled frames at ≤ 2 Hz | L3 |
| `/sensors/*_full` | full-rate raw | L4 (debug only; off by default) |

- **Detail levels:** L0 = summary-only (a few KB); L1 default for evaluation (≈ 1–5 MB/min); L2 default for flagship demos; L3/L4 opt-in.
- **Replay determinism:** replay is **data playback**, not re-simulation, so it is exact by construction. Re-simulation (re-running a seed) is a separate, statistical operation (§40).
- **Sync:** every message is keyed by `t_sim_s`. The Replay Studio clock is a single sim-time cursor, and channels are sampled by "latest ≤ cursor". A desync test checks that the cursor at time t shows the state, map and detections that were logged at ≤ t.

---

## 40. Reproducibility

- **Seeding:** a master seed feeds `numpy.random.SeedSequence`, spawning children for world generation, each env, torch, python `random`, and the detector's data split. All seeds are recorded.
- **Determinism classes:**
  - **D0** (bitwise): FastSim + PPO on CPU with `torch.use_deterministic_algorithms(True)`, single-threaded per process where needed. Tested.
  - **D1** (statistical): Tier H. Gazebo physics + PX4 lockstep is deterministic in principle, but thread scheduling, UDP timing, EKF initialization and process startup introduce variation. AERIS **documents and measures** this: repeated identical episodes → variance of metrics (P5/P12). It never claims bitwise reproducibility.
  - **D2**: MPS training. Not guaranteed deterministic; recorded as such.
- **Environment pinning:** `uv.lock`, `package-lock.json`, `configs/versions.lock.yaml`. A `scripts/env_report.py` output is stored in each manifest.
- **Regeneration:** every figure in docs/reports is produced by a script from run data (`scripts/figures/…`), and the figure's caption cites run/experiment IDs.

---

## 41. Metrics

All metrics are implemented in `aeris/evaluation/metrics/`, unit-tested against hand-computed cases, and documented here. Ground-truth-based metrics are computed **by the evaluator only**.

| Metric | Definition | Notes |
|---|---|---|
| Mission success | Boolean predicate defined by the mission spec (§32.3) | |
| Collision (episode) | 𝟙[∃t: contact event with non-ground geometry ∨ d_GT(t) < r_v] | d_GT = GT clearance of vehicle center to nearest occupied geometry; r_v = vehicle collision radius |
| Collision rate | episodes with collision / episodes; also collisions per km | |
| Minimum clearance | d_min = min_t d_GT(t) | |
| Distance travelled | D = Σ‖p_{t+1} − p_t‖ (GT positions, 10 Hz) | |
| Flight time | t_land − t_takeoff | |
| Path efficiency (nav) | η = ℓ*/D, with ℓ* = shortest collision-free path length (A* on GT map, inflated) | ≤ 1 |
| SPL | (1/N)·Σ S_i·ℓ*_i / max(D_i, ℓ*_i) | standard |
| Coverage | C(t) = \|E_GT(t) ∩ F\| / \|F\| | F = reachable free cells within search volume/altitude band (GT). E_GT(t) = cells that fell inside the agent's sensor footprint up to t, computed by the evaluator via raycasting from **GT poses** (measures what was actually seen, independent of agent map errors). The agent-map-based coverage is reported separately. |
| Time to coverage threshold | T_c(x) = min{t : C(t) ≥ x}, censored at T_max | x ∈ {0.5, 0.8} |
| Revisit ratio | R = (entries into already-visited 1 m cells after leaving them for > 10 s) / (total cell entries) | parameters fixed here |
| Map coverage / accuracy | see §21.3 | |
| Target recall / precision | Hungarian matching of confirmed reports to GT targets within r_match | |
| TTFT, TTA | first / last confirmation time of matched targets | censored |
| Localization error | ATE = √((1/N)·Σ‖p̂_t − p_t‖²) after `T_W_O` alignment; RPE over Δ = 10 m segments | |
| Control smoothness | J = (1/T)·Σ‖(v_cmd,t − v_cmd,t−1)/Δt‖²·Δt | mean squared commanded acceleration |
| Shield intervention rate | fraction of ticks with ‖v_cmd − v_shield‖ > 0.05 m/s | reported with every learned-policy result |
| Inference latency | p50/p95 ms per policy/perception call, on the recorded device | |
| Command latency | t(PX4 state reflects setpoint change) − t(setpoint issued), from step-response tests | |
| Energy proxy | (a) flight time; (b) ∫‖a_cmd‖² dt | **Proxies only**. PX4 SITL battery is a simple drain model, not physical energy. Labeled as such. |
| Return-to-base success | landed ∧ disarmed ∧ ‖p_land − p₀‖ ≤ r_home | |

Forbidden: aggregate "intelligence" or "autonomy" scores, and any metric without a formal definition here or in a documented spec update.

---

## 42. Ablations

| ID | Ablation | Question | Tier |
|---|---|---|---|
| A1 | GRU vs MLP (param-matched) | RQ2 memory | F, H |
| A2 | Map input vs no map input (depth-only) | RQ2 explicit map | F, H |
| A3 | Frontier channel in observation vs none | Does the learned policy just re-learn frontier following? | F, H |
| A4 | Shield on vs off during training (eval always on) | Shielded-RL effect | F |
| A5 | Dynamics/sensor randomization on/off | RQ4 transfer | F→H |
| A6 | Reward terms (−shield penalty, −revisit shaping if added) | Reward-shaping sensitivity | F |
| A7 | Depth vs LiDAR vs both (local nav) | Sensor modality | F, H |
| A8 | Depth vs RGB | RQ3 (gated) | requires render path |
| A9 | Detector: classical vs learned vs oracle | Perception's mission impact | H |
| A10 | Target-aware vs coverage-only exploration | RQ6 | H |
| A11 | Pose: EKF(GPS) vs NoisyPose vs GPS-denied G1/G2 | RQ7 | H |
| A12 | Critic privileged vs symmetric | Training efficiency (declared variant) | F |

Each ablation lists its fixed factors in the experiment config. Ablation results are only compared within the same tier.

---

## 43. Failure modes

Diagnose **layer by layer**, bottom-up: build → sim → PX4 → transport → frames → perception → mapping → planning → learning → UI. Redesign is the last resort and needs a phase-report justification.

| Failure | Symptoms | First diagnostic steps | Typical fixes / fallback |
|---|---|---|---|
| PX4 fails to build | make errors in gz modules | Check `ulimit -n`; activate PX4 `.venv`; compare against issue #27026 (gstreamer link, Qt5 prefix, `-Wdouble-promotion`); check Homebrew formula versions; try release tag vs main | Pin a working SHA; minimal build-system patch in `third_party/patches` (§8.3); Option B1 |
| Gazebo fails to render on macOS | GUI crash, black window | Run server-only; `--render-engine ogre`; XQuartz logout/login; check separate server/GUI processes | Headless (Option A); GUI in VM (B2) only if needed |
| Rendering sensors produce no data | depth/scan topics silent in headless | `gz topic -l/-e`; check engine; RTF | §9.4 decision table |
| SITL does not launch | PX4 waits for gz / times out | Launch order; `PX4_GZ_STANDALONE`; `GZ_SIM_RESOURCE_PATH`; ports in use | Launcher readiness checks; kill stale processes (tracked PIDs only) |
| Vehicle cannot arm | preflight failures | QGC/MAVLink `STATUSTEXT`; EKF not converged; sensor calibration params in SITL | Wait for EKF; correct param profile; never disable arming checks globally without a logged justification |
| Vehicle unstable | oscillation, flips | Check that AERIS setpoints are not conflicting (single writer), setpoint jumps, frame conversion (yaw!), speed factor too high for CPU (RTF drop) | Rate limits; lower speed factor; verify frames tests |
| Telemetry connection fails | MAVSDK connect hangs | Ports (GCS vs offboard), `mavsdk_server` process, firewall prompts | Configured endpoints; pymavlink probe |
| Coordinate-frame mismatch | map mirrored/rotated, drone flies wrong way | Run frame tests; live +x ENU command test (§19.4); plot GT vs estimate | Fix at the single adapter boundary only |
| Mapping drift / smear | doubled walls | Timestamp alignment (pose interpolation), extrinsics, EKF drift vs GT pose ablation | Fix alignment; tune inverse sensor model |
| Planner collision | path through obstacles | Inflation radius, unknown-cell policy, stale map, follower corner-cutting | Correct inflation; replanning triggers; shield catches residual |
| RL not learning | flat return | Known-answer tests; reward scale; obs normalization; action bounds; entropy collapse; LR; check that the env step is correct via a scripted-policy return | Methodical checklist in `docs/reinforcement-learning.md`; escalate conceptual issues to Opus |
| Reward hacking | high return, low C(T) | Trajectory audits (watchlist §27.5); compare reward vs evaluator metrics | Fix reward; re-version; never keep a hacked result |
| Agent circles / refuses to explore | low coverage, high revisit | Action-mask bugs; subgoal failure penalty too high; GRU reset bug | Inspect action distributions and masks in replay |
| Agent exploits simulator | FastSim-only success | Compare Tier F vs H; check for collision-detection gaps, map-edge effects | Fix FastSim; parity tests |
| Policy overfits one world | train ≫ val | World diversity; seed ranges; early stopping on val | More families; randomization |
| Detector false positives | phantom targets | Confusion analysis; confirmation-k; class balance | Retrain; raise the confirmation threshold |
| Memory state leaks | behaviour depends on the previous episode | Leak test (§28) | Fix reset masking |
| NaN loss | NaNs in grads | NaN guard; log-std bounds; reward/obs extremes; LR | Clip; fix inputs |
| PyTorch MPS issue | `mps` unavailable / wrong results | Compare to CPU on a fixed batch | Fall back to CPU (config) |
| Insufficient RAM | swap, OOM | Activity Monitor; run the components separately | Headless; fewer envs; no GUI; Option C |
| Simulation too slow | RTF < 0.5 | Disable GUI; lower sensor rates/resolution; speed factor 1 | Accept slower eval; batch overnight |
| WebSocket overload | UI lag, memory growth | Per-client queue depth; message rates | Drop-oldest queues; throttle; delta encoding |
| Replay desync | overlays misaligned | Timestamp source (sim vs wall); cursor sampling test | Enforce `t_sim_s` everywhere |

---

## 44. Testing

### 44.1 Test taxonomy and markers

| Marker | Needs | Runs where | Examples |
|---|---|---|---|
| `unit` | nothing external | every commit (pre-commit subset + CI) | frames, vehicle-state parsing, map updates, reward functions, buffers, GAE, PPO clipping, metrics, config validation, import contracts |
| `sim` | PX4 SITL + Gazebo on this Mac | local, on demand (`make test-sim`) | PX4 startup, Gazebo startup, vehicle spawn, takeoff, landing, reset, sensor availability & rates |
| `integration` | SITL + AERIS processes | local | AERIS→vehicle interface, vehicle→PX4, PX4→Gazebo (GT confirms motion), sensor→perception, map→planner, planner→vehicle, policy→safety layer |
| `eval` | SITL / FastSim, long | scheduled/manual | mission success, collision, coverage, detection, generalization suites |
| `backend` | FastAPI TestClient | CI | API, WebSocket, schemas (OpenAPI snapshot) |
| `frontend` | Node | CI | Vitest (state, API integration with mocked server), Playwright (critical interactions, responsive layouts) + in-app browser verification |
| `e2e` | full stack | local | start simulation → launch mission → fly → receive telemetry → complete → record replay → display result |

### 44.2 CI

- GitHub Actions (the repo is private; `gh` is authenticated on this machine):
  - lint + type-check + `unit` + `backend` + `frontend` unit tests + import-linter contracts;
  - SMOKE learning tests on CPU.
- SITL tests are **not** in hosted CI by default; they are recorded in phase reports with logs.
- An optional Linux CI job using PX4's container images may be added later (Option B1 parity).

### 44.3 Test rules

- Every bug fix adds a regression test at the lowest layer that reproduces it.
- Tests never depend on wall-clock sleeps for correctness in sim tests. They wait on sim-time conditions with timeouts.
- **Mocks** are allowed only in `unit`/`backend`/`frontend` tests and are centralized (`tests/fixtures/`, `frontend/src/mocks/`). They never ship in production code paths (§46.5).

---

## 45. Backend

### 45.1 Stack

- FastAPI + Uvicorn and Pydantic v2 schemas.
- The server runs in the AERIS venv and binds to `127.0.0.1` by default. The host and port come from config.
- `aeris-core` is hosted in-process for interactive sessions through a `CoreSupervisor`. Batch experiments run from the CLI without the server.

### 45.2 Services

| Service | Responsibility |
|---|---|
| SimulationService | Start, stop or reset **named profiles only**. Reports status, RTF and process health. |
| MissionService | Validates and starts `MissionSpec`s, handles abort, and exposes mission state. |
| TelemetryService | Fans out VehicleState, safety events and sensor health (throttled). |
| MapService | Serves the current 2D map (PNG/RLE) and deltas. |
| ModelService | Lists checkpoints with their manifests and observation-spec compatibility. |
| TrainingService | Starts and stops FastSim training from named configs, reports status, and streams metrics from `metrics.jsonl`. |
| ExperimentService | Lists experiments and runs, returns summaries, and serves generated figures and data. |
| ReplayService | Lists replays, streams MCAP channels by time window, and returns metadata. |
| SystemService | Health, versions, hardware snapshot and dependency checks. |

### 45.3 REST API (refined)

| Method | Path | Notes |
|---|---|---|
| GET | `/api/health` | liveness |
| GET | `/api/system` | versions, hardware, dependency status, version-lock mismatches |
| GET | `/api/simulation/profiles` | available launch profiles |
| POST | `/api/simulation/start` | body: `{profile_id, world_id?}`. `world_id` must exist in the generated-world registry. |
| POST | `/api/simulation/stop` · `/api/simulation/reset` · `/api/simulation/pause` | |
| GET | `/api/simulation/status` | |
| GET | `/api/vehicles` | V1 returns one. Kept as a list for Phase 33. |
| GET | `/api/vehicles/{id}/state` | latest VehicleState |
| POST | `/api/missions` | create from `MissionSpec` (validated) |
| POST | `/api/missions/{id}/start` · `/api/missions/{id}/abort` | abort → Safety emergency path |
| GET | `/api/missions/{id}` | state, targets, metrics so far |
| GET | `/api/maps/current` · `/api/maps/{run_id}/keyframes` | |
| GET | `/api/models` · `/api/models/{id}` | |
| POST | `/api/training` · `/api/training/{id}/stop` | named configs only |
| GET | `/api/training/{id}` | status |
| GET | `/api/experiments` · `/api/experiments/{id}` · `/api/experiments/{id}/runs` | |
| GET | `/api/replays` · `/api/replays/{id}` · `/api/replays/{id}/channels?from=&to=&channels=` | windowed MCAP reads |

### 45.4 WebSocket channels

| Path | Rate | Content |
|---|---|---|
| `/ws/telemetry` | 10 Hz | VehicleState, supervisor state, link and sensor health |
| `/ws/mission` | event + 2 Hz | mission state, target hypotheses, safety events |
| `/ws/map` | 1–2 Hz | 2D map deltas (RLE) + planned path + frontiers |
| `/ws/perception` | ≤ 5 Hz | detection overlays + optional JPEG thumbnails (opt-in) |
| `/ws/training/{id}` | on write | new metrics rows |

Each connection has a **bounded per-client queue**. The drop-oldest policy applies only to state streams. Events are never dropped: the client can request a resync, and messages carry sequence numbers. Protocol: JSON with `{"type", "seq", "t_sim_s", "payload"}`. The schema version is negotiated at connect.

### 45.5 Security rules

- **No arbitrary command execution.** Subprocesses are launched only from whitelisted, config-defined profiles, with arguments built from validated enums and IDs. Commands are never built from strings supplied by the client.
- Localhost only by default. If exposed on a LAN (not planned), a token and CORS allowlist are required.
- File paths in requests are opaque IDs resolved server-side and cannot traverse outside the configured roots.
- Schema-validated input everywhere, and error responses do not leak stack traces.

---

## 46. Frontend

### 46.1 Stack (DECISION, ADR-013; final confirmation in Phase 26)

| Concern | Choice | Why |
|---|---|---|
| Framework | React + TypeScript + Vite | Matches the prompt and is industry standard |
| Styling | CSS Modules plus **design tokens as CSS custom properties**, generated from `design/tokens/*.json` | Full control over a bespoke design language. **Bootstrap 5 is not used** because its visual defaults conflict with the engineering aesthetic and it adds no needed capability. |
| Accessible primitives | Radix UI primitives (unstyled) | Keyboard and ARIA correctness without imposing a visual style |
| Server state | TanStack Query | Caching and invalidation for REST |
| Client/UI state | Zustand (small stores per feature) | Minimal and testable |
| Realtime | Typed WebSocket client with seq tracking and resync | §45.4 |
| API types | Generated from the backend OpenAPI (`openapi-typescript`) | No hand-duplicated schemas |
| 3D | three.js via React Three Fiber (+ drei, used selectively) | Mission 3D view |
| Realtime charts | uPlot | Fast time series for telemetry |
| Analytics charts | Observable Plot (fallback: visx) | Scientific plots: CI bands, facets, log scales. Recharts lacks good CI/facet support, and Plotly is heavy. |
| Motion | CSS transitions; Motion only for panel and layout transitions; `prefers-reduced-motion` respected | Purposeful, minimal motion |
| Tests | Vitest + Testing Library; Playwright for e2e and responsive checks | |

### 46.2 Areas (only with genuine functionality)

| Area | First real data (phase) | Content |
|---|---|---|
| Mission Control (flagship) | 28 | 3D world, drone camera, live map, flight state, mission panel, safety events |
| Live Flight | 27 | Detailed telemetry, setpoints vs response, modes, EKF health, link |
| Perception | 28 | RGB/depth streams, detections, confidence, latency |
| World Map | 28 | 2D occupancy, coverage, frontiers, paths, targets; GT overlay only in eval/replay |
| Autonomy | 28 | Policy/planner introspection: chosen subgoal, action distribution, value estimate, shield interventions |
| Training Lab | 29 | Training runs, live metrics, diagnostics (KL, clip fraction, EV) |
| Experiment Lab | 29 | Experiment matrices, results with CIs, pre-registration status |
| Replay Studio | 30 | Timeline playback of MCAP |
| System | 27 | Versions, dependency health, sim profiles, logs |
| Fleet | 33 only | Not built unless Phase 33 is executed |

### 46.3 Mission Control layout (refined from the prompt)

```
┌────┬──────────────────────────────────────────────────┬──────────────────────┐
│NAV │ 3D WORLD (R3F)                                   │ FLIGHT STATE         │
│rail│  vehicle pose, trajectory (actual: white),       │ Mode · Armed · Link  │
│    │  planned path (magenta), frontiers, targets,     │ Alt · Vel · Hdg      │
│    │  sensor FOV frustum, map voxels (LOD)            │ Pose source (EST)    │
│    │  [camera: follow | orbit | top]                  │ Battery-sim (proxy)  │
│    ├───────────────────────┬──────────────────────────┤──────────────────────│
│    │ DRONE CAMERA          │ LIVE MAP (2D, north-up)  │ MISSION              │
│    │ RGB | DEPTH toggle    │ coverage, frontiers,     │ State: SEARCH        │
│    │ detection overlay     │ path, targets, scale bar │ Targets 2 / 5 (N-known)│
│    │ age / fps indicator   │                          │ Coverage 41.2 %      │
│    │                       │                          │ Elapsed / budget     │
│    │                       │                          │ Safety: 3 interventions│
├────┴───────────────────────┴──────────────────────────┴──────────────────────┤
│ EVENT TIMELINE (mission + safety events on sim-time axis)        [ABORT]     │
└──────────────────────────────────────────────────────────────────────────────┘
```

The simulation and mission are the visual focus. The abort control is always visible, uses a distinct shape and color, and requires confirmation, since it is a simulation command.

### 46.4 Performance budgets

- 60 fps target for 3D at ≤ 200 k rendered voxel instances (instanced meshes, LOD).
- Telemetry UI updates at ≤ 10 Hz.
- Main-thread long tasks under 50 ms. Map decoding runs in a Web Worker.

### 46.5 Real-data rule enforcement

- Mocks live only in `frontend/src/mocks/` (MSW handlers) and are enabled only by `VITE_AERIS_MOCKS=1` in dev.
- A **CI check fails the build if production bundles reference the mocks module**.
- When mocks are active, a visible "MOCK DATA" banner appears in the UI.
- Before Phase 31 completes, a repo-wide audit confirms no fake telemetry, trajectories, curves, results or detections remain.

---

## 47. Professional UI/UX strategy

- **Design authority:** the AERIS design system (Phase 26) is designed deliberately for this product. Generic generators (template dashboards, v0/Lovable-style builders) are **not** used as design authority.
- **References:** avionics and ground-control conventions (glass-cockpit color semantics, FMS magenta for the planned route, caution/warning hierarchy), plus scientific-visualization practice.
- **Principles:**
  1. **Mission first.** The spatial view and mission state dominate. Chrome recedes.
  2. **State is unambiguous.** Every critical state has text, a shape or icon, and color. Color is never the only signal.
  3. **Provenance is visible.** EST (estimate), GT (ground truth, evaluation only), ORACLE and MOCK are always labeled. The UI never shows GT in a way that implies the agent knew it.
  4. **Stale data is obvious.** Every live value has an age indicator. Stale values are dimmed and labeled STALE. Missing data is never interpolated in the UI.
  5. **Density with hierarchy.** An engineering-dense layout with a clear typographic hierarchy and tabular numerals.
  6. **Restraint.** No glassmorphism, neon, glow, decorative gradients, oversized rounded cards, pill overload, gratuitous icons or ornamental animation.
  7. **Accessibility.** WCAG 2.2 AA contrast. Full keyboard operation, visible focus and screen-reader labels for controls and state.
- **UX flows to design in Phase 26:**
  - start simulation → preflight → start mission → monitor → abort / complete → report;
  - open a replay → scrub → inspect a decision;
  - compare experiment results;
  - monitor training.

---

## 48. Design system

Specified fully in Phase 26 (`design/`). Phase 0 fixes these constraints:

| Element | Constraint (Phase 0) |
|---|---|
| Typography | A humanist/grotesk UI sans + monospace with tabular numerals for telemetry, e.g., IBM Plex Sans + IBM Plex Mono (open license). A compact scale for dense UI (e.g., 11/12/13/14/16/20/24/32 px) with fixed line heights. Numbers are always tabular. Units are set in a smaller, lower-contrast style. |
| Spacing | 4 px base unit; scale 4/8/12/16/24/32/48. |
| Grid | App shell: 56 px icon rail (expandable to 200 px), fluid center, 320 px right inspector. 12-column inner grid, 16 px gutters. Supported widths from 1280 px up; responsive behavior below 1280 px is a stacked "monitoring" layout (not a full control layout); the phone layout is read-only monitoring. |
| Surfaces | 4 elevation levels expressed by **luminance steps and 1 px borders**, not heavy shadows. Radius 2–4 px. |
| Themes | Dark "instrument" theme (default for operations) + light theme (reports/print). Both are token-driven. |
| Semantic colors (avionics-derived) | **Warning** (red): immediate action. **Caution** (amber): awareness. **Normal/engaged** (green). **Advisory/selected** (cyan). **Planned route** (magenta, the FMS convention). **Actual track** (neutral white/near-white). **Ground truth** (dashed, desaturated, always labeled "GT"). |
| Flight-state encoding | DISARMED (neutral, ○), ARMED-ON-GROUND (amber, ◐, "props live"), AIRBORNE (green, ●), OFFBOARD (cyan tag "OFFB"), FAILSAFE (red, ▲ + text). Always text + glyph + color. |
| Mission-state encoding | IDLE, PREFLIGHT, TAKEOFF, SEARCH, INVESTIGATE, RETURN, LAND, COMPLETE, ABORTED, FAILSAFE, each with a label and glyph. Colors map to the semantic set (e.g., ABORTED = warning). |
| Control hierarchy | Primary (one per view), secondary, quiet. **Destructive or safety-critical** controls (abort, e-stop) use a distinct shape and color, confirmation, and are never adjacent to routine controls. |
| Charts | Axis titles with units. Labeled ticks. Legends or direct labels. Tooltips with exact values and units. CI bands at 30 % opacity. Seeds shown as thin lines when N ≤ 10. A categorical palette validated for color-vision deficiency. |
| Map conventions | North-up by default (heading-up optional). Scale bar. Occupancy: occupied = high-contrast foreground, free = base surface, unknown = subtle hatch. Coverage = cyan tint. Frontiers = amber cells. Planned = magenta. Actual track = white. Targets: detected = filled triangle + confidence; GT = hollow diamond (eval/replay only). Collision markers = red ✕ with timestamp. |
| Telemetry conventions | SI units. Fixed decimals per quantity (e.g., altitude 0.1 m, velocity 0.01 m/s, heading 1°). Sign always shown for rates. Age indicator. `—` for no data (never 0). |
| Icons | One outline icon set at 1.5 px stroke (e.g., Lucide), 16/20 px. Icons support labels; they do not replace them. |
| Focus states | 2 px focus ring in the advisory color with ≥ 3:1 contrast against adjacent colors. Never removed. |
| Loading states | Skeletons for layout-stable panels. An explicit "waiting for simulation / telemetry" state with elapsed time. No indefinite spinners without text. |
| Empty states | Explain what is missing and the next action (e.g., "No runs yet. Start a training config from Training Lab."). No decorative illustrations. |
| Tokens | Single source `design/tokens/*.json`, compiled to CSS variables and TS constants (colors are shared with R3F materials and chart palettes). |

---

## 49. Visualization strategy

| Visualization | Where | Data source | Encoding notes |
|---|---|---|---|
| 3D drone pose + trajectory | Mission Control, Replay | VehicleState (EST); GT in eval | Vehicle glyph with heading; actual track white; planned magenta; FOV frustum |
| Occupancy / coverage map (2D) | World Map, Mission Control | MapService deltas | See map conventions |
| Voxel map (3D) | Mission Control (toggle) | voxel keyframes | Instanced cubes, height-tinted, LOD |
| Target detections | Perception, maps | detections + hypotheses | Bounding boxes on camera; triangles on map; confidence numeric |
| Sensor FOV | 3D, 2D | camera intrinsics + extrinsics | Frustum outline, never filled |
| Collision markers | maps, timeline | GT contact events (eval) | red ✕, labeled GT |
| Telemetry time series | Live Flight | telemetry stream | uPlot; setpoint vs actual overlays |
| Training curves | Training Lab | metrics.jsonl | Return, success, C(T) vs env steps; diagnostics panel |
| Coverage curves C(t) | Experiment Lab, reports | eval JSONL | Mean ± 95 % CI band per method; per-seed thin lines |
| Mission success / collision rate | Experiment Lab | eval | Dot + CI whisker plots (not bar charts of means alone) |
| Target recall, TTFT | Experiment Lab | eval | ECDF / survival curves for censored times |
| Seed variance | Experiment Lab | per-seed results | Strip plots + IQM |
| Transfer gap | Experiment Lab | paired Tier F vs H | Paired slope plots |

Rules: every chart shows axis, units, labels, legend, tooltip and scale, and cites its run/experiment IDs. Nothing is fabricated. Figures in docs are generated by scripts from stored data (§40).

---

## 50. Skills / connectors / tool strategy

Verified as available in this Claude Code environment during Phase 0 (2026-09-25). Availability must be re-checked at the start of each phase.

| Capability | Status | Use in AERIS |
|---|---|---|
| Bash / Read / Edit / Write | available | all phases |
| WebSearch / WebFetch | available | doc verification at phase start (PX4/Gazebo/MAVSDK versions) |
| Built-in browser (Claude_Browser: preview_start, read_page, screenshots, resize) | available | Phases 27–31: verify UI, responsive layouts, dark/light |
| `gh` CLI | authenticated (EXCALIBUR303) | Phase 2: private repo creation (user confirmation required), CI |
| GitHub plugin connector | **requires authentication** (not needed; `gh` suffices) | — |
| Design skills: `design-token`, `color-system`, `typography-scale`, `spacing-system`, `layout-grid`, `dark-mode-design`, `data-visualization`, `dataviz`, `accessibility-audit`, `motion-system`, `component-spec`, `critique-*`, `hallmark`, `web-design-guidelines`, `design:design-system`, `design:accessibility-review` | available | Phase 26 (design system), 27–30 (reviews). Used as checklists and aids; the AERIS spec remains the design authority. |
| Figma MCP | tools present; the Figma plugin connector **requires authentication** | Optional in Phase 26 if the user authorizes it; not required |
| `engineering:architecture`, `engineering:testing-strategy`, `code-review`, `security-review` skills | available | Phase 2 (testing strategy), 25 (backend security review), 31/34 (audits) |
| Artifacts (claude.ai page) | available | Optional: shareable phase reports or design-system preview (Phase 26) |
| Ollama (local) + Ollama MCP | installed | Phase 32 language parser option (local, free) |
| Hugging Face MCP | authenticated (SIDCODER) | Phase 20: look up small, license-compatible detection models if needed |
| Docker Desktop | installed | Option B1 Linux fallback (Phases 1, 9, 33 if needed) |
| iOS Simulator, Vercel, Supabase, Railway, Canva, Gamma, etc. | available/irrelevant | **Not used**: no material benefit |

Rules:
- Use a tool only when it materially improves the result.
- Never invent connectors.
- If a phase would benefit from an unauthenticated connector, state that and continue without it.
- Subagents or workflows are used only when the user explicitly asks.

---

## 51. Complete phase roadmap

### 51.0 Conventions for all phases

- **Phase protocol (binding):** see the prompt's "Absolute Phase Boundary". At the start of every phase:
  1. read this spec;
  2. inspect the repo;
  3. read the previous phase report (`docs/phase_reports/phase-NN.md`);
  4. re-check available tools;
  5. confirm scope;
  6. confirm model and effort.

  During the phase: implement only authorized work, then test, fix, run the gate and record results. At the end: write the report (the prompt's §53 format, saved to `docs/phase_reports/`) and **STOP**. The only authorization to proceed is the user replying **CONTINUE**.
- **Standard stop condition (applies to every phase):** the validation gate has been executed with genuine, recorded results (PASS or FAIL). The phase report is written. There are no uncommitted partial changes outside the phase scope. The next phase has not been started.
  - On **FAIL**: stop and report blockers, with a proposed remediation. Do not start the next phase.
- **Standard "when not to use Opus":** build or toolchain errors, missing imports, CSS/layout issues, a single broken endpoint, test syntax errors, renames, formatting, dependency version bumps, ordinary integration bugs with a clear stack trace.
- **Effort levels available** in this environment: low / medium / high / xhigh ("Extra High") / max. "Max" is not planned for any phase.
- **Roadmap changes vs the prompt's draft** (with reasons):
  - *Experiment/metrics/replay-recording core* moves from Phase 24 to **Phase 7**, because Phases 9–23 all produce experiments and need manifests and recording from the start.
  - *FastSim + system identification* is added to the RL-environment phase (**Phase 13**), required by ADR-006.
  - *Sensor pipeline* (Phase 8) comes before *Worlds* (Phase 9), because world generation needs the validated sensor frames and model extrinsics for WorldSpec consistency tests.
  - *Moving-target tracking* is assigned **Sonnet/High** (classical KF + pursuit; §35).
  - The numbering stays 0–34: some phases shift by one because the experiment core moved.
  - Backend/UI phases (25–31) stay late on purpose, since QGC + CLI + MCAP/Foxglove cover debugging until then.

### Phase index

| # | Phase | Model | Effort |
|---|---|---|---|
| 0 | Architecture + research specification | Opus 5.5 | High |
| 1 | Mac / PX4 / Gazebo compatibility spike | Sonnet | High |
| 2 | Repository + tooling foundation | Sonnet | Medium |
| 3 | PX4 SITL integration (launcher, profiles, params) | Sonnet | High |
| 4 | Vehicle interface + telemetry + frames core | Sonnet | High |
| 5 | Programmatic takeoff / flight / landing + safety supervisor v1 | Sonnet | High |
| 6 | Waypoint missions + mission executive v1 | Sonnet | Medium |
| 7 | Experiment, metrics, ground-truth & replay-recording core | Sonnet | High |
| 8 | Sensor pipeline (Sensor Bridge, aeris_x500, time alignment) | Sonnet | High |
| 9 | WorldSpec + procedural worlds + obstacle scenarios | Sonnet | Medium |
| 10 | Classical obstacle avoidance + collision shield | Sonnet | High |
| 11 | Mapping (voxel log-odds, 2D projection) + frame validation | Sonnet | High |
| 12 | Classical planning + exploration baselines | Sonnet | High |
| 13 | RL environment: system ID + FastSim + Gymnasium env | Opus 5.5 | High |
| 14 | PPO implementation + mathematical tests | Opus 5.5 | High |
| 15 | PPO experiments / debugging on navigation curriculum | Sonnet | High |
| 16 | Perception: depth / LiDAR / RGB processing + encoders | Sonnet | High |
| 17 | Learned local navigation | Opus 5.5 | High |
| 18 | Memory-based autonomy (GRU, RQ2) | Opus 5.5 | High |
| 19 | Learned exploration (RQ1, RQ4) | Opus 5.5 | High |
| 20 | Target detection | Sonnet | High |
| 21 | Search-and-rescue mission (RQ6) | Opus 5.5 | High |
| 22 | Generalization experiments (RQ5) | Sonnet | High |
| 23 | GPS-denied navigation (RQ7) | Opus 5.5 | Extra High |
| 24 | Moving-target tracking | Sonnet | High |
| 25 | FastAPI / WebSocket platform | Sonnet | High |
| 26 | Professional product / UI design system | Opus 5.5 | High |
| 27 | React foundation + Live Flight + System | Sonnet | High |
| 28 | Mission visualization (Mission Control, Map, Perception, Autonomy) | Sonnet | High |
| 29 | Training / experiment analytics | Sonnet | High |
| 30 | Replay Studio | Sonnet | High |
| 31 | Full system integration + E2E + real-data audit | Sonnet | High |
| 32 | Language-conditioned missions (optional) | Opus 5.5 | Extra High |
| 33 | Multi-drone intelligence (optional) | Opus 5.5 | Extra High |
| 34 | Final autonomy / scientific / system audit | Opus 5.5 | Extra High |

---

### Phase 0 — Architecture + research specification

- **Objective:** produce this authoritative spec from verified research and an inspection of the machine.
- **Why it exists:** a multi-phase research platform needs one source of truth, so later phases do not re-litigate architecture or invent methodology.
- **Dependencies:** none.
- **Files/modules:** `AERIS_TECHNICAL_SPEC.md`.
- **Implementation tasks:** inspect the environment; research PX4, Gazebo, QGC, ROS 2, MAVSDK and PyTorch; define architecture, RQs, hypotheses, methodology and roadmap.
- **Research considerations:** separate verified facts from assumptions; route every assumption to a validating phase.
- **Tests:** none (documentation). Internal consistency review.
- **Validation gate:** the spec covers all 58 required sections, and every phase has all 19 required fields.
- **Expected output:** this document + Phase 0 report.
- **Known risks:** fast-moving upstream docs (PX4 main); an unreleased macOS version (27.2).
- **Failure/rollback:** revise the spec.
- **Recommended model / effort:** Opus 5.5 / High.
- **Why this model:** architecture and methodology decisions set the validity of all later work.
- **Why this effort:** broad, but it is decision synthesis rather than deep proof; High is enough.
- **Skills/connectors:** WebSearch/WebFetch, Bash inspection.
- **Opus escalation:** n/a.
- **When not to use Opus:** n/a.
- **Stop condition:** standard; STOP and await CONTINUE.

### Phase 1 — Mac / PX4 / Gazebo compatibility spike

- **Objective:** prove the chosen stack runs on *this* Mac, and freeze known-good versions.
- **Why it exists:** everything depends on PX4 SITL + Gazebo working natively (§7.3, §9.4). There is an open macOS build issue (#27026).
- **Dependencies:** Phase 0.
- **Files/modules:** `docs/mac-setup.md`, `docs/compat/phase1-log.md`, `configs/versions.lock.yaml`, `scripts/compat/*.sh` (read-only probes and smoke scripts), and `third_party/patches/px4/` only if strictly required.
- **Implementation tasks:**
  1. Install prerequisites (Homebrew packages via the PX4 script; `ulimit -S -n 2048`). **Ask the user before any system-level installs or changes to `~/.zshrc`.**
  2. Clone PX4 to `~/aeris-deps/PX4-Autopilot`. Try tag v1.17.x first, then `main` if needed.
  3. Run `./Tools/setup/macos.sh --sim-tools`, which installs Gazebo Harmonic and XQuartz. XQuartz may need a logout/login; ask the user.
  4. `make px4_sitl gz_x500` (GUI attempt) and `HEADLESS=1 make px4_sitl gz_x500`.
  5. Record RTF, CPU and RAM.
  6. **Rendering-sensor test:** `gz_x500_depth` and `gz_x500_lidar_2d` in headless mode. Check `gz topic -l` and message rates, and validate one depth image and one scan (range statistics).
  7. PX4 shell: `commander takeoff` / `commander land` in SITL.
  8. Probe the Gazebo Python bindings (`python -c "import gz.transport13"` under Homebrew Python). Record that Python version.
  9. Install QGroundControl 5.x (DMG; **ask before downloading**). Connect, view telemetry, and do one manual arm/takeoff/land.
  10. Create a throwaway venv (Python 3.12) and check that MAVSDK-Python installs (verify the package name) and connects to SITL. Read telemetry only.
  11. Check PyTorch install + MPS availability (a tiny tensor op) in the same throwaway venv.
- **Research considerations:** record exact versions and SHAs; note any deviation from the PX4 docs; test the `PX4_GZ_SIM_RENDER_ENGINE=ogre` workaround only if needed.
- **Tests:** scripted smoke probes (exit codes + logs); manual QGC check recorded with a screenshot.
- **Validation gate (PASS requires all of):**
  1. PX4 SITL builds.
  2. Gazebo server runs and x500 spawns (GT pose topic shows the model).
  3. PX4 reports a heartbeat and EKF converges.
  4. Takeoff/land succeeds via the PX4 shell.
  5. MAVSDK receives telemetry.
  6. QGC connects (or a documented reason why not).
  7. **Depth and 2D LiDAR produce valid data headless** (or an explicit §9.4 fallback decision is recorded).
  8. The version lock is written.

  GUI failure alone does **not** fail the gate if headless works.
- **Expected output:** a working native stack or an evidence-backed fallback decision; the version lock; mac-setup docs.
- **Known risks:** issue #27026 build errors; XQuartz; unreleased macOS 27.2 incompatibilities; MPS availability regression; disk usage (~15–25 GB for PX4 + builds).
- **Failure/rollback:** follow §43 rows 1–3. If native fails, run the same gate in Option B1 (Docker) and recommend it, **without migrating the rest**.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** a systems integration and debugging task that needs careful execution, not research reasoning.
- **Why this effort:** many failure points; build debugging needs thoroughness.
- **Skills/connectors:** Bash; WebFetch for issue lookups; built-in browser not needed.
- **Opus escalation:** only if the fallback decision has non-obvious research consequences, e.g., rendering sensors impossible on every option, forcing a redesign of the sensor strategy.
- **When not to use Opus:** compiler/linker errors, brew issues, env vars.
- **Stop condition:** standard.

### Phase 2 — Repository + tooling foundation

- **Objective:** create the AERIS repo skeleton, tooling, CI and core utilities.
- **Why it exists:** provides quality gates (types, lint, import contracts, tests) before real code exists.
- **Dependencies:** Phase 1 PASS.
- **Files/modules:** `pyproject.toml`, `uv.lock`, `Makefile`, `.gitignore`, `.pre-commit-config.yaml`, import-linter config, `aeris/core/{config,logging,errors,types,units,clock}.py`, `tests/unit/core/*`, `.github/workflows/ci.yml`, `README.md`, `docs/architecture.md` (summary + links to spec), `docs/adr/0001…0013.md` (from Appendix C).
- **Implementation tasks:**
  1. `git init`.
  2. Create a private GitHub repo **only after user confirmation**.
  3. uv project on Python 3.12 with extras `[sim, learn, server, dev]`.
  4. ruff + mypy config.
  5. Pydantic config loader (`extends:` + overrides + hash).
  6. Structured logging.
  7. Clock protocol.
  8. Import-linter contracts (§14.3; contracts for future packages are declared now and become active as packages appear).
  9. CI.
  10. `.gitignore` covering `results/`, `models/`, `replays/`, caches, PX4 builds, node_modules, `configs/local.yaml`, `*.ulg`, Gazebo logs.
- **Research considerations:** none. This is engineering.
- **Tests:** config composition/hash tests; logging tests; contract checks pass; CI green.
- **Validation gate:** `make lint typecheck test` passes locally and in CI; import contracts are enforced (a deliberate violation in a test fixture is detected).
- **Expected output:** a clean, installable package skeleton.
- **Known risks:** Python 3.12 wheel availability; the Node IPv6 issue (only relevant later).
- **Failure/rollback:** fall back to Python 3.11 if a critical wheel is missing (recorded).
- **Recommended model / effort:** Sonnet / Medium.
- **Why this model:** routine engineering.
- **Why this effort:** well-understood tasks.
- **Skills/connectors:** `gh` CLI; `engineering:testing-strategy` (optional).
- **Opus escalation:** none expected.
- **When not to use Opus:** everything in this phase.
- **Stop condition:** standard.

### Phase 3 — PX4 SITL integration

- **Objective:** a programmatic, reproducible launcher for Gazebo + PX4 using named profiles, readiness checks, parameter profiles, logs and reset.
- **Why it exists:** experiments need hands-off, repeatable sim lifecycles (§17.2).
- **Dependencies:** Phase 2.
- **Files/modules:** `aeris/simulation/launcher/{profiles.py,process.py,readiness.py,params.py}`, `configs/simulation/*.yaml`, `configs/vehicle/px4_sitl.yaml` (ports, airframe IDs), `configs/vehicle/px4_params/sitl_base.params`, `tests/sim/test_launcher.py`, `docs/px4.md`, `docs/simulation.md`.
- **Implementation tasks:**
  1. Standalone launch sequence: `gz sim -s <world>`, then PX4 with `PX4_GZ_STANDALONE=1`, `PX4_SYS_AUTOSTART`, `PX4_GZ_MODEL_POSE`, speed factor.
  2. Readiness: gz topics present, heartbeat, EKF OK.
  3. Tracked-PID shutdown only.
  4. Log capture to the run directory.
  5. Parameter-profile application.
  6. Reset via full restart, and measure its cost.
  7. Record the MAVLink ports for GCS vs offboard.
  8. Measure RTF at speed factors 1/2/4 headless.
- **Research considerations:** measure Tier H nondeterminism (5 identical hover episodes → pose variance) to seed §40.
- **Tests:** `sim`-marked tests for start → ready → stop ×10 without leaks; reset; profile validation (`unit`).
- **Validation gate:** 10/10 consecutive start–ready–stop cycles; reset produces a clean state (EKF re-initialized, disarmed, landed); RTF table recorded; no orphan processes.
- **Expected output:** `aeris sim up --profile headless_x500` works.
- **Known risks:** port conflicts; zombie processes; slow Gazebo startup.
- **Failure/rollback:** increase readiness timeouts; per-profile port offsets.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** integration engineering.
- **Why this effort:** process lifecycle bugs are subtle.
- **Skills/connectors:** Bash.
- **Opus escalation:** none expected.
- **When not to use Opus:** process/timeout bugs.
- **Stop condition:** standard.

### Phase 4 — Vehicle interface + telemetry + frames core

- **Objective:** implement `VehicleInterface`/`CommandPort`, `Px4MavsdkAdapter`, the telemetry stream, the `aeris.core.frames` library, and the hardware guard.
- **Why it exists:** decouples autonomy from transport (§15) and establishes frame correctness (§19).
- **Dependencies:** Phase 3.
- **Files/modules:** `aeris/vehicle/interface.py`, `aeris/vehicle/px4_mavsdk/*`, `aeris/vehicle/px4_mavlink/probe.py` (minimal), `aeris/vehicle/telemetry/*`, `aeris/core/frames/*`, tests.
- **Implementation tasks:**
  1. State dataclasses.
  2. MAVSDK connection with an endpoint allowlist.
  3. Telemetry fusion into `VehicleState` at a configured rate.
  4. ENU/NED and FLU/FRD conversions confined to the adapter.
  5. Offboard setpoint streaming task (20 Hz; latest-value).
  6. Transport evaluation (§15.3): latency and rates.
  7. `t_sim_s` derivation and verification against gz `/clock`.
- **Research considerations:** document MAVSDK limitations found; decide which capabilities go to pymavlink.
- **Tests:**
  - `unit`: frames (§19.4 property tests), state parsing, allowlist.
  - `integration`: connect, stream state for 60 s at ≥ 95 % of the target rate, heartbeat-loss detection.
- **Validation gate:** all frame tests pass; telemetry rate and latency meet targets (state ≥ 20 Hz; recorded p95 latency); refuses non-loopback endpoints; the transport decision is recorded as ADR-003 final.
- **Expected output:** `aeris vehicle monitor` prints live state in ENU.
- **Known risks:** asyncio + gRPC lifecycle issues; time-base mismatch.
- **Failure/rollback:** pymavlink adapter for the failing capability.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** well-specified integration.
- **Why this effort:** frames and async are error-prone.
- **Skills/connectors:** WebFetch (MAVSDK API docs).
- **Opus escalation:** if frame conventions (§19) prove inconsistent with PX4/gz behavior in ways that require a spec change.
- **When not to use Opus:** async bugs, API misuse.
- **Stop condition:** standard.

### Phase 5 — Programmatic takeoff / flight / landing + safety supervisor v1

- **Objective:** safe programmatic flight through `SafetySupervisor` (S1, S2, S4 layers): takeoff, hover, offboard velocity/position moves, land, emergency stop.
- **Why it exists:** Level L1, and it establishes the only command path (§16).
- **Dependencies:** Phase 4.
- **Files/modules:** `aeris/safety/{supervisor.py,validator.py,envelope.py,watchdog.py,events.py}`, `configs/vehicle/safety.yaml`, `scripts/fly_box.py`, tests.
- **Implementation tasks:**
  1. Supervisor state machine (§16.3).
  2. Command validation.
  3. AERIS geofence inside the PX4 geofence params.
  4. Watchdogs (autonomy tick, stale state).
  5. Step-response logging for later system ID.
  6. "Fly a box" script.
  7. Emergency stop.
- **Research considerations:** collect velocity step responses (± 0.5/1/2 m/s on each axis, yaw-rate steps) → dataset for Phase 13 system ID; record command latency (§41).
- **Tests:**
  - `unit`: validator bounds, NaN rejection, state-machine transitions (table-driven), watchdog timing on a fake clock.
  - `sim`: 20 repeated takeoff–box–land runs; forced autonomy stall triggers hover; offboard-loss path triggers PX4 failsafe as configured.
- **Validation gate:** 20/20 box flights complete with max position error < 0.5 m at waypoints and no failsafe; every injected fault (stale telemetry, stalled tick, out-of-bounds command, geofence breach attempt) is handled as specified, 10/10 each; import contract "only safety holds CommandPort" is active.
- **Expected output:** L1 achieved; step-response dataset.
- **Known risks:** mode-switch races; conflicting writers.
- **Failure/rollback:** simplify to position setpoints only; add explicit mode-confirmation waits.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** deterministic engineering to a clear spec.
- **Why this effort:** safety-critical correctness.
- **Skills/connectors:** Bash; QGC for debugging.
- **Opus escalation:** if the safety architecture itself appears flawed (e.g., PX4 behaviors make S2 gating unsound).
- **When not to use Opus:** timing bugs, test flakiness.
- **Stop condition:** standard.

### Phase 6 — Waypoint missions + mission executive v1

- **Objective:** `MissionSpec` v1 (waypoint missions), a deterministic mission executive state machine, and an autonomous takeoff → mission → land cycle.
- **Why it exists:** Levels L2 and L3; it is also the executive skeleton that SAR later extends.
- **Dependencies:** Phase 5.
- **Files/modules:** `aeris/autonomy/mission/{spec.py,executive.py,states.py}`, `configs/missions/*.yaml`, tests.
- **Implementation tasks:** Pydantic mission spec; executive with IDLE…COMPLETE/ABORTED; waypoint arrival criteria (radius + dwell); mission time budget; abort path.
- **Research considerations:** waypoint error and path-efficiency metrics defined here (implemented in P7).
- **Tests:**
  - `unit`: executive transitions (including abort in every state); spec validation.
  - `sim`: 3 mission templates × 5 runs.
- **Validation gate:** 15/15 missions complete; all abort-from-state tests pass; arrival errors are within the configured radius.
- **Expected output:** `aeris mission run configs/missions/square.yaml`.
- **Known risks:** arrival-detection jitter.
- **Failure/rollback:** hysteresis on arrival.
- **Recommended model / effort:** Sonnet / Medium.
- **Why this model:** routine state-machine engineering.
- **Why this effort:** builds on P5.
- **Skills/connectors:** none special.
- **Opus escalation:** none.
- **When not to use Opus:** all.
- **Stop condition:** standard.

### Phase 7 — Experiment, metrics, ground-truth & replay-recording core

- **Objective:** run manifests, config hashing, run directories, registry, `GroundTruthService` (privileged), metric library v1, episode runner, MCAP recorder (L0–L2), pre-registration guard.
- **Why it exists:** every later phase produces experiments. Reproducibility and GT separation must exist before the first comparison (§38–41).
- **Dependencies:** Phase 6.
- **Files/modules:** `aeris/experiments/{manifest,registry,runner,preregistration}.py`, `aeris/evaluation/{metrics/*,episode.py,statistics.py}`, `aeris/simulation/groundtruth/*`, `aeris/replay/{recorder,reader,channels}.py`, `configs/experiments/`, tests.
- **Implementation tasks:**
  1. Manifest capture (git SHA/dirty, versions, hardware).
  2. Metrics: waypoint error, D, flight time, smoothness, collision (GT) and min clearance (needs a GT geometry source: `WorldSpec` stub or stock-world geometry until P9).
  3. Paired bootstrap CI + Holm correction utilities.
  4. MCAP recording on the event bus.
  5. GT channel via gz pose topics (for P7 this can use a tiny gz-Python helper; the full Sensor Bridge comes in P8).
- **Research considerations:** statistics correctness (bootstrap tests against known distributions); GT/agent separation contracts go live.
- **Tests:**
  - `unit`: each metric against hand-computed cases; bootstrap coverage sanity (≈ 95 % on synthetic data); manifest completeness; pre-registration guard refuses the test split.
  - `integration`: record a P6 mission and read it back.
- **Validation gate:**
  1. A P6 mission run produces a complete manifest + MCAP + per-episode metrics.
  2. Re-reading the MCAP reproduces the metrics exactly.
  3. The import contract blocks `groundtruth` from autonomy/learning (tested).
  4. The Tier H nondeterminism measurement (P3) is formalized into a D1 report.
- **Expected output:** `aeris eval run …` produces results with provenance.
- **Known risks:** MCAP schema churn; GT timing alignment.
- **Failure/rollback:** JSONL fallback recorder (same channels).
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** infrastructure engineering to a precise spec.
- **Why this effort:** correctness of statistics and provenance is foundational.
- **Skills/connectors:** `dataviz` (for the first plots, optional).
- **Opus escalation:** if the statistical protocol (§6) needs a methodological change.
- **When not to use Opus:** serialization bugs.
- **Stop condition:** standard.

### Phase 8 — Sensor pipeline

- **Objective:** Sensor Bridge process + IPC, the `aeris_x500` model (depth + LiDAR + RGB), `GzBridgeClient`, pose interpolation buffer, depth/scan → point clouds in `M`, and sensor health.
- **Why it exists:** Level L4; perception, mapping and learning all depend on correct, time-aligned sensor data (§18).
- **Dependencies:** Phase 7 (recording), Phase 1 (rendering-sensor decision).
- **Files/modules:** `aeris/simulation/bridge/{_gz_process.py,schema.py,client.py}`, `aeris/simulation/models/aeris_x500/*`, `aeris/perception/{depth,lidar}/projection.py`, `aeris/core/frames/extrinsics.py`, `configs/vehicle/sensors.yaml` (generated from SDF), tests.
- **Implementation tasks:**
  1. Bridge in Homebrew Python: subscribe to depth, RGB, scan, camera_info and `/clock`; GT on a separate namespace.
  2. ZeroMQ publish with HWM/conflate.
  3. Build the composed model via `GZ_SIM_RESOURCE_PATH` (fallback: stock variants, §8.4).
  4. Generate extrinsics from SDF.
  5. Back-projection.
  6. Pose interpolation.
  7. Per-topic health.
- **Research considerations:** measure sensor latency (capture → AERIS receipt) and rates under load; record which sensors are rendered vs geometric fallback.
- **Tests:**
  - `unit`: back-projection against synthetic planes; extrinsics from SDF; interpolation.
  - `integration`: a known box obstacle at a known GT location appears at the correct `M` position (error < 2 voxel sizes) from depth and from LiDAR; the GT channel is unreachable from the agent client (allowlist test).
- **Validation gate:** depth ≥ 10 Hz, scan ≥ 10 Hz, RGB ≥ 5 Hz sustained for 5 min; the obstacle-localization test passes at 3 distances and 4 yaw angles; zero GT leakage.
- **Expected output:** L4; a debug view (a Matplotlib or MCAP/Foxglove snapshot) of points over GT geometry.
- **Known risks:** Python-binding mismatch; rendering performance; timestamp semantics.
- **Failure/rollback:** §9.4 decision table; reduce resolution or rate.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** integration.
- **Why this effort:** multi-process, timing and frames.
- **Skills/connectors:** WebFetch (gz-transport Python docs).
- **Opus escalation:** if frame or time conventions need redesign (§19 change).
- **When not to use Opus:** ZeroMQ/serialization bugs.
- **Stop condition:** standard.

### Phase 9 — WorldSpec + procedural worlds + obstacle scenarios

- **Objective:** `WorldSpec` schema; generators for F1–F4; SDF renderer; GT occupancy renderer; obstacle-suite scenarios; `splits.lock`.
- **Why it exists:** a single source of truth for both tiers (ADR-007); train/val/test splits (ADR-010).
- **Dependencies:** Phase 8.
- **Files/modules:** `aeris/simulation/worlds/{spec.py,generators/{rubble,office,warehouse,collapsed}.py,sdf.py,occupancy.py,splits.py}`, `configs/worlds/*`, tests.
- **Implementation tasks:** primitive-based geometry (boxes, cylinders, walls, slabs); reachability-checked spawn/start sampling; target placement slots; SDF export + load test in Gazebo; GT occupancy voxelization; obstacle suites for P10 (corridor, pillar forest, dead-end, narrow gap, overhang within band).
- **Research considerations:** family diversity statistics (free-area, corridor-width, clutter distributions) documented so splits are meaningful; F4 is held out.
- **Tests:** determinism (same seed → same hash); SDF loads in Gazebo; SDF vs occupancy consistency (render GT occupancy, compare with gz raycasts at sample points); split-guard tests.
- **Validation gate:** 50 generated worlds per family load in Gazebo without errors; consistency test agreement ≥ 99 % of sampled points; all starts reachable; the split lock is committed.
- **Expected output:** `aeris worlds generate --family office --split train --n 10`.
- **Known risks:** Gazebo load time for complex worlds; mesh vs primitive fidelity.
- **Failure/rollback:** cap complexity; primitives only.
- **Recommended model / effort:** Sonnet / Medium.
- **Why this model:** procedural generation engineering.
- **Why this effort:** clear spec, moderate complexity.
- **Skills/connectors:** none special.
- **Opus escalation:** if the split methodology is found scientifically flawed.
- **When not to use Opus:** geometry bugs.
- **Stop condition:** standard.

### Phase 10 — Classical obstacle avoidance + collision shield

- **Objective:** path follower, reactive (VFH-style) avoidance, and the S3 collision shield (§16.4).
- **Why it exists:** Level L5; the shield is a prerequisite for all learned control.
- **Dependencies:** Phases 8, 9.
- **Files/modules:** `aeris/autonomy/navigation/{follower.py,reactive.py}`, `aeris/safety/shield.py`, experiment config `configs/experiments/p10_avoidance.yaml`, tests.
- **Implementation tasks:** sector clearance model from depth + scan; the shield projection; reactive goal-seeking; intervention logging; the first formal experiment (obstacle suites × methods).
- **Research considerations:** metrics: collision rate, d_min, η, smoothness, completion; compare "reactive" vs "straight-line + shield only".
- **Tests:**
  - `unit`: shield property tests (never increases approach speed beyond v_i,max; the stop-distance invariant); sector computation.
  - `sim`: obstacle suites.
- **Validation gate:** on the P9 obstacle suites (≥ 5 scenarios × 10 runs), reactive + shield achieves **0 collisions** at the configured max speed, with results recorded (CIs); shield unit properties pass. Any collision is diagnosed and fixed or explicitly accepted with a documented reason.
- **Expected output:** first formal experiment report.
- **Known risks:** blind spots outside the camera FOV; latency.
- **Failure/rollback:** lower max speed; conservative unknown-sector policy.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** classical robotics implementation.
- **Why this effort:** safety-critical.
- **Skills/connectors:** none special.
- **Opus escalation:** if the shield cannot provide guarantees with the given sensors and the safety architecture needs redesign.
- **When not to use Opus:** tuning, bugs.
- **Stop condition:** standard.

### Phase 11 — Mapping + frame validation

- **Objective:** 3D voxel log-odds map, 2D band projection, explored set, frontier extraction, map deltas, and map evaluation vs GT.
- **Why it exists:** Level L6; prerequisite for planning, exploration and learned observations (§21).
- **Dependencies:** Phases 8, 9, 10.
- **Files/modules:** `aeris/mapping/{voxel.py,raycast.py,projection.py,frontier.py,deltas.py}`, `aeris/localization/{pose_source.py,px4_ekf.py,noisy.py}`, `aeris/evaluation/metrics/map.py`, tests.
- **Implementation tasks:** Numba raycasting; inverse sensor model; clamping; the band projection; frontier clustering; `PoseSource` + `NoisyPoseSource`; the map-accuracy experiment (GT pose vs EKF pose vs noisy pose).
- **Research considerations:** quantify the map error attributable to the estimated pose; choose the resolution by measured accuracy vs cost.
- **Tests:**
  - `unit`: known-geometry integration (single ray, plane, corner); clamping; frontier detection on synthetic grids; projection rules.
  - `integration`: hover-and-yaw sweep in a known world.
- **Validation gate:** with EKF pose, occupied precision ≥ 0.9 and recall ≥ 0.8 within observed regions on 5 worlds (thresholds may be revised only with justification in the report); map update latency p95 < 50 ms per depth frame on this Mac; no mirrored or rotated artifacts (frame test).
- **Expected output:** L6; map debug views.
- **Known risks:** Numba on arm64; performance; drift.
- **Failure/rollback:** coarser resolution; downsampled rays.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** implementation of well-known algorithms.
- **Why this effort:** performance + correctness.
- **Skills/connectors:** none special.
- **Opus escalation:** if frame or localization methodology becomes ambiguous (the prompt's explicit escalation), e.g., deciding T_M_O semantics under drift.
- **When not to use Opus:** Numba/perf issues.
- **Stop condition:** standard.

### Phase 12 — Classical planning + exploration baselines

- **Objective:** 2D A*; random, nearest-frontier and utility-frontier exploration; the exploration experiment in Tier H (H0.1).
- **Why it exists:** Level L7; the strong baselines required for RQ1 (§25).
- **Dependencies:** Phase 11.
- **Files/modules:** `aeris/autonomy/planning/astar.py`, `aeris/autonomy/exploration/{random.py,frontier.py,utility.py,base.py}`, `aeris/evaluation/metrics/coverage.py`, `configs/experiments/p12_exploration.yaml`, tests.
- **Implementation tasks:**
  1. A* with inflation and unknown-cost handling.
  2. The subgoal interface shared with the future learned policy (`ExplorationStrategy.select_subgoal(map, pose, t) -> Subgoal`).
  3. The evaluator coverage raycaster (GT poses).
  4. Utility-frontier tuning on validation worlds, with the tuning budget recorded.
  5. The exploration experiment on validation worlds.
- **Research considerations:** H0.1 test; record baseline variance (D1); the pre-registration draft for RQ1.
- **Tests:**
  - `unit`: A* optimality on known grids; frontier selection; coverage metric on synthetic trajectories.
  - `sim`: exploration episodes.
- **Validation gate:** H0.1 evaluated with a pre-registered rule (frontier > random by ≥ 10 pp C(180 s) on val worlds, CI excluding 0). If it **fails**, this is a baseline bug signal: diagnose before proceeding. Planner latency p95 < 100 ms.
- **Expected output:** L7; baseline results.
- **Known risks:** slow Tier H episode throughput.
- **Failure/rollback:** shorter T_max for DEV; overnight batches.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** classical algorithms.
- **Why this effort:** baseline strength matters scientifically.
- **Skills/connectors:** `dataviz` for coverage curves.
- **Opus escalation:** if the fairness protocol vs future learned methods needs a design decision beyond §25.
- **When not to use Opus:** algorithm bugs.
- **Stop condition:** standard.

### Phase 13 — RL environment: system ID + FastSim + Gymnasium env

- **Objective:**
  - identify the closed-loop velocity dynamics from Tier H;
  - build vectorized FastSim (geometry from WorldSpec, raycast depth/LiDAR, noise and pose models);
  - Gymnasium envs for local navigation and exploration;
  - `ObservationBuilder`/`ActionDecoder` shared with deployment;
  - parity tests.
- **Why it exists:** RL at feasible cost (ADR-006). The validity of every learned result depends on this environment being fair and faithful.
- **Dependencies:** Phases 5 (step data), 9, 11, 12.
- **Files/modules:** `aeris/simulation/fastsim/{world.py,dynamics.py,sensors.py,vehicle.py,batch.py}`, `aeris/learning/envs/{local_nav.py,exploration.py,rewards.py,wrappers.py}`, `aeris/learning/spaces/*`, `scripts/sysid/*`, tests.
- **Implementation tasks:**
  1. System ID fits (τ_v, delay, limits; per axis; with CIs).
  2. FastSim vehicle implementing `VehicleInterface` semantics.
  3. Sensor raycast matching Tier H intrinsics.
  4. EKF-like pose error model fitted to Tier H (GT vs EKF).
  5. Reward functions v1 (§27.5).
  6. Provenance enforcement.
  7. Randomization ranges.
  8. Throughput benchmark.
  9. Tier F↔H parity experiments.
- **Research considerations:**
  - What fidelity is *sufficient*: parity thresholds are set here and justified.
  - Reward-only privileged information is documented.
  - Observation leakage audit.
  - Termination vs truncation semantics.
- **Tests:**
  - `unit`: dynamics against analytic step responses; raycast against brute force; reward functions with hand transitions; spaces round-trip; provenance guard raises on ORACLE fields.
  - Gymnasium `check_env`.
  - Determinism (D0).
- **Validation gate:**
  1. System-ID model predicts held-out Tier H step responses with velocity RMSE ≤ 0.15 m/s (or a justified threshold).
  2. Open-loop trajectory parity: position RMSE ≤ 0.5 m over 20 s for 10 command sequences.
  3. Depth-statistics parity (median absolute depth error ≤ 5 % on matched frames).
  4. Throughput ≥ 2 000 steps/s aggregate (local nav).
  5. Leakage audit passes.
  6. A scripted frontier policy running inside the FastSim exploration env reproduces Tier H frontier coverage within a reported gap.
- **Expected output:** trainable envs + a parity report (the first RQ4 data point).
- **Known risks:** FastSim too idealized; throughput limits.
- **Failure/rollback:** adjust the models; widen randomization; if throughput is insufficient, reduce scope (smaller crops/rays) or recommend Option C.
- **Recommended model / effort:** Opus 5.5 / High.
- **Why this model:** environment design determines scientific validity (leakage, rewards, fidelity, fairness). This matches the prompt's RL-environment assignment.
- **Why this effort:** deep but bounded; Extra High is not needed if the spec is followed.
- **Skills/connectors:** none special.
- **Opus escalation:** n/a (already Opus). Sonnet may implement sub-tasks in follow-ups only if the user switches.
- **When not to use Opus:** in follow-up sessions, pure performance optimization of the raycaster can be done by Sonnet.
- **Stop condition:** standard.

### Phase 14 — PPO implementation

- **Objective:** in-house PPO (MLP + GRU; Gaussian + masked categorical), checkpoint/resume, diagnostics, the full §27.4 test suite, and a CPU vs MPS benchmark.
- **Why it exists:** an explainable, verified learning core (§27).
- **Dependencies:** Phase 13.
- **Files/modules:** `aeris/learning/ppo/{buffer.py,gae.py,losses.py,trainer.py,checkpoint.py,diagnostics.py}`, `aeris/learning/networks/{encoders.py,recurrent.py,heads.py,distributions.py}`, `configs/learning/ppo_*.yaml`, `tests/unit/learning/*`, `docs/reinforcement-learning.md` (math + implementation mapping).
- **Implementation tasks:** everything in §27.2–27.4; the reference sanity comparison vs SB3 on a toy task (dev-only).
- **Research considerations:** correctness first; document every design choice with a citation (Schulman et al. 2017; GAE, Schulman et al. 2016; implementation-details literature).
- **Tests:** §27.4 items 1–10, all mandatory.
- **Validation gate:** all tests pass; known-answer tasks are solved within budget; the GRU-only memory task is solved by the GRU and **not** by the MLP (proves the memory path); resume is bitwise identical on CPU; SPS benchmark recorded (CPU vs MPS) with the device decision.
- **Expected output:** `aeris train --config ppo_localnav_smoke`.
- **Known risks:** subtle recurrent-batching bugs; MPS numerical differences.
- **Failure/rollback:** CPU only; feed-forward first, then GRU.
- **Recommended model / effort:** Opus 5.5 / High.
- **Why this model:** PPO mathematical correctness is an explicit Opus responsibility.
- **Why this effort:** high rigor; well-defined scope.
- **Skills/connectors:** none special.
- **Opus escalation:** n/a.
- **When not to use Opus:** tensor-shape plumbing in follow-ups could go to Sonnet if the user prefers cost savings.
- **Stop condition:** standard.

### Phase 15 — PPO experiments / debugging on a navigation curriculum

- **Objective:**
  - Train PPO in FastSim on a local-navigation curriculum: open field → sparse obstacles → dense clutter.
  - Establish a stable training recipe, seed variance and diagnostics.
  - Run the first Tier H spot check.
- **Why it exists:** de-risks learning before the research phases. It separates "PPO works" from "the RQ design works".
- **Dependencies:** Phase 14.
- **Files/modules:** `configs/learning/localnav_*.yaml`, `configs/experiments/p15_*.yaml`, `docs/reinforcement-learning.md` (training recipe section).
- **Implementation tasks:** hyperparameter sweeps on **validation** worlds (budget recorded); 5-seed runs; trajectory audits for reward hacking; deploy the best checkpoint via `LearnedLocalNav` on 10 Tier H episodes.
- **Research considerations:** learning curves with CIs; failure taxonomy; the checkpoint-selection rule.
- **Tests:** regression SMOKE training in CI; deployment wrapper parity test (same obs → same action in FastSim wrapper vs Tier H wrapper).
- **Validation gate:** ≥ 4/5 seeds reach ≥ 80 % success on FastSim val worlds (the dense stage may be lower if documented); Tier H spot check runs end to end through the shield with results recorded (no performance threshold; honesty requirement).
- **Expected output:** a training recipe and diagnostics report.
- **Known risks:** instability; reward hacking.
- **Failure/rollback:** the §43 RL checklist; simplify the task.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** experiment execution and routine debugging.
- **Why this effort:** many runs, careful analysis.
- **Skills/connectors:** `dataviz`.
- **Opus escalation:** conceptual training failure (e.g., the learning signal cannot work as designed), suspected reward hacking not fixable by tuning, suspected PPO math error.
- **When not to use Opus:** hyperparameter tuning, crashes, config errors.
- **Stop condition:** standard.

### Phase 16 — Perception: depth / LiDAR / RGB processing + encoders

- **Objective:** production depth/LiDAR preprocessing shared by train and deploy; noise and dropout models validated against Tier H; RGB pipeline (capture, downsample, exposure/lighting variants); latency benchmarks; encoder architectures finalized.
- **Why it exists:** reliable perception inputs for learned navigation, exploration and detection (§20).
- **Dependencies:** Phases 8, 13, 15.
- **Files/modules:** `aeris/perception/{depth,lidar,rgb}/*`, `aeris/learning/spaces/preprocess.py`, tests.
- **Implementation tasks:** depth hole handling; range clipping; downsampling; LiDAR ray binning; sensor noise-model fitting (FastSim vs Tier H residuals); RGB-path feasibility study for RQ3 (measure Gazebo RGB throughput; evaluate the FastSim flat-shaded rasterizer option) → **decide whether RQ3 runs**.
- **Research considerations:** the RQ3 feasibility decision is recorded with numbers.
- **Tests:** preprocessing parity (train vs deploy); noise-model statistics; latency.
- **Validation gate:** parity tests pass; FastSim depth-noise statistics match Tier H within the stated tolerance; the RQ3 go/no-go decision is documented.
- **Expected output:** a perception module with benchmarks.
- **Known risks:** RGB infeasibility.
- **Failure/rollback:** RQ3 marked "not executed", with the reason.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** implementation + measurement.
- **Why this effort:** cross-tier parity is detailed work.
- **Skills/connectors:** none special.
- **Opus escalation:** if the sensor-fusion architecture needs redesign.
- **When not to use Opus:** OpenCV/NumPy issues.
- **Stop condition:** standard.

### Phase 17 — Learned local navigation

- **Objective:** train and evaluate learned PointGoal navigation (depth / LiDAR / both; RGB if RQ3 is a go) against the classical reactive and A* baselines, in Tier H on test-ID worlds.
- **Why it exists:** Level L8; RQ3 and A7; first sim-to-sim transfer measurement for a learned policy (RQ4).
- **Dependencies:** Phases 10, 12, 15, 16.
- **Files/modules:** `aeris/autonomy/navigation/learned.py`, `configs/experiments/p17_*.yaml`, pre-registration doc, results.
- **Implementation tasks:** pre-register; train ≥ 5 seeds per modality; select checkpoints on val; evaluate in Tier F and Tier H on test-ID; analysis with CIs.
- **Research considerations:** fairness (same shield, speed limits); shield-intervention rate reported; transfer gap.
- **Tests:** deployment parity; replay audits of failures.
- **Validation gate:** the pre-registered analysis is executed completely and reported honestly (success, SPL, collisions, interventions, CIs, transfer gap). The gate is about **methodological completeness, not about the learned policy winning**.
- **Expected output:** an RQ3 (if go) / A7 / RQ4 report.
- **Known risks:** large transfer gap.
- **Failure/rollback:** report the gap; randomization iteration (a bounded budget, pre-declared).
- **Recommended model / effort:** Opus 5.5 / High.
- **Why this model:** learned-autonomy methodology and interpretation.
- **Why this effort:** careful experiment design and analysis.
- **Skills/connectors:** `dataviz`.
- **Opus escalation:** n/a.
- **When not to use Opus:** if follow-up sessions only run pre-specified evaluations, use Sonnet.
- **Stop condition:** standard.

### Phase 18 — Memory-based autonomy (GRU, RQ2)

- **Objective:** a recurrent policy with a correct reset/BPTT, and the 2×2 memory × map ablation in the exploration environment (FastSim, then Tier H on val).
- **Why it exists:** exploration is partially observable (§28). RQ2 checks whether memory helps.
- **Dependencies:** Phases 14, 15, 16. The exploration env comes from Phase 13.
- **Files/modules:** `aeris/learning/networks/recurrent.py` (hardened), `aeris/autonomy/exploration/learned.py` (deployment wrapper with hidden-state lifecycle), configs, pre-registration.
- **Implementation tasks:**
  1. Parameter-matched MLP and GRU variants.
  2. Train 4 variants with ≥ 5 seeds each.
  3. Leak tests at deployment.
  4. Evaluate on validation worlds in Tier F and Tier H.
  5. Pre-register the RQ1 final protocol for Phase 19.
- **Research considerations:** capacity confound (parameter matching); H2a and H2b; the interaction effect.
- **Tests:** §28 tests; deployment hidden-state reset after mission start and after safety takeover.
- **Validation gate:** leak and reset tests pass; the 2×2 analysis is complete, with CIs, on validation worlds; the RQ1 pre-registration is committed.
- **Expected output:** RQ2 report (validation-level); the chosen learned architecture for Phase 19, selected on validation only.
- **Known risks:** unstable recurrent training.
- **Failure/rollback:** shorter BPTT, LayerNorm-GRU, a lower learning rate.
- **Recommended model / effort:** Opus 5.5 / High.
- **Why this model:** memory methodology and confound control.
- **Why this effort:** careful experimental design.
- **Skills/connectors:** `dataviz`.
- **Opus escalation:** n/a.
- **When not to use Opus:** pure plumbing follow-ups.
- **Stop condition:** standard.

### Phase 19 — Learned exploration (RQ1, RQ4)

- **Objective:** final training of the learned hierarchical exploration policy, then the **pre-registered RQ1 evaluation** on test-ID worlds in Tier H against the random, nearest-frontier and utility-frontier baselines. Also the transfer-gap analysis (H4) and the A3/A5 ablations.
- **Why it exists:** Level L9. This is the flagship research question.
- **Dependencies:** Phases 12, 18.
- **Files/modules:** `aeris/autonomy/exploration/learned.py`, `configs/experiments/p19_rq1.yaml`, results, `docs/evaluation.md` (RQ1 section).
- **Implementation tasks:**
  1. FULL training with ≥ 5 seeds, in resumable blocks.
  2. Checkpoint selection on validation.
  3. Tier H test-ID evaluation of all methods on paired worlds and starts.
  4. Statistics per §6.
  5. Trajectory audits.
  6. Ablations.
- **Research considerations:**
  - Fairness audit: sensors, shield, planner, time budget, tuning budgets.
  - Leakage audit.
  - Report H1/H1b/H4 exactly as pre-registered.
  - Negative results are reported plainly.
- **Tests:** evaluation-pipeline tests; replay spot checks of the best, median and worst episodes per method.
- **Validation gate:** the pre-registered analysis runs to completion with all required episodes (invalid episodes counted and explained); the fairness and leakage audits pass; the report states the conclusion that the CIs support.
- **Expected output:** the RQ1 result (positive, negative or inconclusive), with replays.
- **Known risks:**
  - The learned policy underperforms. This is acceptable scientifically.
  - Compute time: Tier H evaluation may take many hours.
- **Failure/rollback:** extend the evaluation over multiple sessions. Never shrink N after seeing results.
- **Recommended model / effort:** Opus 5.5 / High.
- **Why this model:** flagship methodology, fairness and interpretation.
- **Why this effort:** high stakes, but bounded by the pre-registration.
- **Skills/connectors:** `dataviz`; Artifacts, optionally, for a shareable results page.
- **Opus escalation:** n/a.
- **When not to use Opus:** running the long evaluation batches can be delegated to Sonnet if the user switches models between sessions.
- **Stop condition:** standard.

### Phase 20 — Target detection

- **Objective:**
  - Stage 1: classical marker and ArUco detection.
  - Stage 2: a learned small detector trained on auto-labeled renders.
  - Detection-to-world localization, association and confirmation.
  - Detection metrics.
- **Why it exists:** Level L10, and a prerequisite for SAR (§31).
- **Dependencies:** Phases 9, 11, 16.
- **Files/modules:** `aeris/perception/detection/{classical.py,learned.py,localize.py,associate.py}`, `aeris/simulation/models/targets/*`, `scripts/datasets/render_labels.py` (privileged; offline), `configs/perception/*`, tests.
- **Implementation tasks:**
  1. Target models.
  2. Auto-label pipeline with disjoint world seeds per split.
  3. Train a torchvision SSDLite or FasterRCNN-mobile model (CPU/MPS; benchmark both).
  4. Localization via aligned depth.
  5. Association and confirmation.
  6. Classical vs learned vs oracle comparison.
- **Research considerations:** license check for every model and weight; dataset card; class definitions stated precisely.
- **Tests:** localization geometry on synthetic cases; association logic; label-pipeline correctness (spot-check overlays); no GT at inference (contract).
- **Validation gate:** the offline test set reports AP@0.5, precision and recall; Tier H runs report confirmed-target localization error (median ≤ 1.0 m at ≤ 10 m range, or a justified threshold) and false-positive rate; inference latency p95 fits the perception budget.
- **Expected output:** a detector with a model card and dataset card.
- **Known risks:** sim-rendering domain gaps across lighting conditions; small dataset.
- **Failure/rollback:** stay with the classical detector for SAR v1 and report it.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** a standard supervised-learning pipeline.
- **Why this effort:** data correctness and evaluation rigor.
- **Skills/connectors:** Hugging Face MCP (optional, to look up license-compatible small detectors).
- **Opus escalation:** evidence of label leakage or an invalid evaluation split.
- **When not to use Opus:** training bugs, dataloader issues.
- **Stop condition:** standard.

### Phase 21 — Search-and-rescue mission (RQ6)

- **Objective:** a full SAR mission (§32): executive extensions (SEARCH / INVESTIGATE / RETURN logic, return-time budget), target-aware exploration variant, mission reports, and the pre-registered RQ6 evaluation.
- **Why it exists:** Level L11, the flagship demonstration.
- **Dependencies:** Phases 19, 20.
- **Files/modules:** `aeris/autonomy/mission/sar.py`, `aeris/autonomy/mission/report.py`, env extension with a detection channel, configs, pre-registration.
- **Implementation tasks:**
  1. INVESTIGATE behavior with bounded time.
  2. Return-time estimator (A* path length / speed + margin).
  3. Train the target-aware policy variant.
  4. Run the SAR experiment matrix: {frontier, learned coverage-only, learned target-aware} × {classical detector, learned detector, oracle detector} × test-ID worlds; N-known and N-unknown settings.
- **Research considerations:** detector-oracle ablation separates perception effects from exploration effects; censored-time statistics for TTFT and TTA.
- **Tests:** executive unit tests (return trigger, abort); report-generation tests; E2E SAR sim test.
- **Validation gate:** the flagship demo mission completes end to end with a recorded replay and report; the RQ6 pre-registered analysis is complete; the return-success rate is reported (target ≥ 95 % for non-collision episodes; a failure is diagnosed).
- **Expected output:** L11; SAR results; the demo replay.
- **Known risks:** compounding errors across perception and exploration.
- **Failure/rollback:** isolate each component using oracle substitutes.
- **Recommended model / effort:** Opus 5.5 / High.
- **Why this model:** mission-level methodology, reward design and interpretation.
- **Why this effort:** complex integration of research components.
- **Skills/connectors:** `dataviz`; Artifacts, optionally, for the mission report.
- **Opus escalation:** n/a.
- **When not to use Opus:** report formatting, executive bugs in follow-ups.
- **Stop condition:** standard.

### Phase 22 — Generalization experiments (RQ5)

- **Objective:** the pre-registered generalization study: test-ID vs test-OOD (F4); the perturbation axes of §33.3.
- **Why it exists:** Level L12; the "the model generalizes" claim rule.
- **Dependencies:** Phases 19, 21.
- **Files/modules:** `configs/experiments/p22_*.yaml`, pre-registration, results, `docs/evaluation.md` (RQ5 section).
- **Implementation tasks:** evaluate all methods on F4 and on the one-factor perturbations; degradation curves; H5.
- **Research considerations:** no retraining or tuning on F4; report ranking stability.
- **Tests:** split-guard verification in the manifests (F4 never seen in any training or tuning manifest; automated audit).
- **Validation gate:** the automated audit proves F4 isolation; the analysis is complete and honest.
- **Expected output:** RQ5 report.
- **Known risks:** Tier H compute time.
- **Failure/rollback:** prioritize F4 over the perturbation sweeps (declared order).
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** executes a pre-specified protocol.
- **Why this effort:** large, careful experiment execution.
- **Skills/connectors:** `dataviz`.
- **Opus escalation:** if the generalization experiment is found scientifically invalid (leakage, confounds).
- **When not to use Opus:** batch execution, plotting.
- **Stop condition:** standard.

### Phase 23 — GPS-denied navigation (RQ7)

- **Objective:** define and implement the GPS-denied conditions (§34) G0–G2 (and optionally G3/GO); depth/LiDAR odometry; external-vision injection into PX4; localization and mission evaluation.
- **Why it exists:** Level L13. There is a high risk of invalid claims (leakage of GT pose), so rigor is needed.
- **Dependencies:** Phases 11, 21.
- **Files/modules:** `aeris/localization/{depth_odometry.py,scan_matching.py,ext_vision.py}`, `aeris/vehicle/px4_mavlink/ext_vision.py`, `configs/vehicle/px4_params/gps_denied_*.params`, `configs/experiments/p23_*.yaml`, `docs/localization.md` (under mapping/autonomy docs).
- **Implementation tasks:**
  1. Verify the PX4 parameters for disabling GPS fusion in the pinned version.
  2. Model variant with flow + rangefinder.
  3. Odometry implementation.
  4. EKF external-vision fusion.
  5. Drift and failure detection with a safe fallback (hover/land on estimator failure).
  6. Evaluation.
  7. Optional SLAM stage decision (§23).
- **Research considerations:**
  - An exact information-availability table is included in the report.
  - `x500_vision` is ORACLE-VIO.
  - Learned policies are evaluated unchanged and fine-tuned, both reported.
- **Tests:** odometry on synthetic scans and depth; frame chain for external vision (FLU/ENU → PX4 NED/FRD); estimator-failure fallback in sim; leakage audit (no GT, no GPS-derived quantity in any observation or estimate).
- **Validation gate:** the leakage audit passes; G0–G2 ATE/RPE and mission metrics are reported with CIs; the estimator-failure fallback triggers correctly 10/10.
- **Expected output:** RQ7 characterization report.
- **Known risks:** EKF external-vision tuning; divergence.
- **Failure/rollback:** report G1 only; mark G2 as incomplete, with a diagnosis.
- **Recommended model / effort:** Opus 5.5 / Extra High.
- **Why this model:** estimation methodology and validity of GPS-denied claims.
- **Why this effort:** the hardest correctness and validity reasoning in V1 (frames, estimator coupling, leakage).
- **Skills/connectors:** WebFetch (PX4 EKF2 docs for the pinned version).
- **Opus escalation:** n/a.
- **When not to use Opus:** parameter-file edits, plotting.
- **Stop condition:** standard.

### Phase 24 — Moving-target tracking

- **Objective:** scripted moving targets (Tier H + FastSim analytic), detection → KF tracking → prediction → stand-off pursuit through the safety layer; tracking metrics.
- **Why it exists:** Level L14 (§35).
- **Dependencies:** Phases 20, 21.
- **Files/modules:** `aeris/autonomy/tracking/{kalman.py,pursuit.py,tracker.py}`, `aeris/simulation/models/movers/*`, configs, tests.
- **Implementation tasks:** a mover plugin/controller in Gazebo; KF with gating; pursuit law with stand-off and speed limits; occlusion handling; experiments across target speeds.
- **Research considerations:** safety invariants (minimum separation) come before tracking performance.
- **Tests:** KF against analytic motion; pursuit against simulated targets in FastSim; the safety-invariant property test.
- **Validation gate:** the safety invariant holds in all Tier H episodes (min separation ≥ configured); tracking metrics are reported across ≥ 3 target speeds.
- **Expected output:** L14 report and demo replay.
- **Known risks:** detector latency causing lag.
- **Failure/rollback:** a lower target speed range, reported.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** classical estimation and control with a clear spec.
- **Why this effort:** closed-loop tuning and safety.
- **Skills/connectors:** none special.
- **Opus escalation:** if learned tracking is proposed, or tracking destabilizes the safety architecture.
- **When not to use Opus:** KF tuning.
- **Stop condition:** standard.

### Phase 25 — FastAPI / WebSocket platform

- **Objective:** the backend of §45 against real services (simulation, missions, telemetry, maps, models, training, experiments, replays).
- **Why it exists:** the UI needs a stable, typed, secure API.
- **Dependencies:** Phases 7, 21 (real data sources exist).
- **Files/modules:** `backend/app/{main.py,api/*,websocket/*,services/*,schemas/*}`, `tests/backend/*`, `docs/backend.md`, the OpenAPI snapshot.
- **Implementation tasks:**
  1. Services wrap the `aeris` package; no logic duplication.
  2. Whitelisted profiles only.
  3. WS channels with bounded queues and seq/resync.
  4. Windowed MCAP reads.
  5. Security review.
- **Research considerations:** none. This is engineering.
- **Tests:** API tests; WS load test (10 clients × 10 Hz for 10 min with bounded memory); path-traversal and injection negative tests; OpenAPI snapshot.
- **Validation gate:** all tests pass; `security-review` skill findings resolved; no endpoint can execute arbitrary commands (audit).
- **Expected output:** a running API with docs at `/docs` (localhost).
- **Known risks:** event-loop blocking by heavy map encoding.
- **Failure/rollback:** move encoding to a worker thread or process.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** backend engineering.
- **Why this effort:** concurrency and security.
- **Skills/connectors:** `security-review`, `code-review`.
- **Opus escalation:** none expected.
- **When not to use Opus:** everything.
- **Stop condition:** standard.

### Phase 26 — Professional product / UI design system

- **Objective:**
  - The AERIS design system: tokens, typography, spacing, grid, surfaces, semantic, flight- and mission-state encodings, controls, charts, maps, telemetry, icons, focus/loading/empty states.
  - Information architecture and key screen specifications (Mission Control, Live Flight, Replay Studio, Experiment Lab).
  - Accessibility plan.
  - The final frontend stack confirmation (ADR-013).
- **Why it exists:** a deliberate, non-generic product design before any React code (§47–48).
- **Dependencies:** Phase 25 (real API shapes).
- **Files/modules:** `design/{README.md,tokens/*.json,principles.md,components/*.md,screens/*.md,a11y.md}`, a static HTML token/component specimen page, and optionally a Figma file if the user authorizes that connector.
- **Implementation tasks:**
  1. Palette derivation with WCAG contrast validation (both themes).
  2. CVD-safe categorical chart palette.
  3. Type scale.
  4. Component specs.
  5. Screen wireframes, high-fidelity specifications and a clickable HTML specimen.
  6. A critique pass using the available critique skills.
- **Research considerations:** avionics color semantics; provenance labeling (EST / GT / ORACLE / MOCK).
- **Tests:** a contrast-check script over the tokens; accessibility review.
- **Validation gate:** all token pairs used for text meet AA; every critical state has a non-color encoding; screen specs cover the four key flows; the user reviews the specimen (design sign-off requested, not assumed).
- **Expected output:** a design system package + specimen.
- **Known risks:** drift into generic dashboard aesthetics.
- **Failure/rollback:** iterate against the §47 principles.
- **Recommended model / effort:** Opus 5.5 / High.
- **Why this model:** a design-authority phase (per the prompt); the quality of judgment matters.
- **Why this effort:** broad design synthesis.
- **Skills/connectors:** `design-token`, `color-system`, `typography-scale`, `spacing-system`, `layout-grid`, `dark-mode-design`, `data-visualization`/`dataviz`, `accessibility-audit`, `component-spec`, `critique-*`, `hallmark` (anti-generic check); Figma MCP only if authenticated; built-in browser to render the specimen.
- **Opus escalation:** n/a.
- **When not to use Opus:** token-file formatting.
- **Stop condition:** standard.

### Phase 27 — React foundation + Live Flight + System

- **Objective:** the frontend scaffold (Vite/TS), token pipeline, app shell and navigation, typed API/WS clients, the MSW mock layer (dev-only with a banner), the Live Flight and System areas on real data.
- **Why it exists:** the foundation for all UI areas.
- **Dependencies:** Phases 25, 26.
- **Files/modules:** `frontend/*` (per §46), CI build checks (the mock-exclusion check).
- **Implementation tasks:** OpenAPI type generation; WS client with resync; stale-data indicators; uPlot telemetry charts; responsive shell.
- **Research considerations:** none.
- **Tests:** Vitest for stores, clients and components; Playwright for navigation, keyboard and responsive layouts; built-in browser verification with screenshots.
- **Validation gate:** Live Flight shows real SITL telemetry with correct units and age indicators; the mock-exclusion CI check passes; keyboard navigation and focus are visible; Lighthouse/axe shows no critical accessibility violations.
- **Expected output:** a running control-center shell.
- **Known risks:** npm IPv6 stall (use `NODE_OPTIONS=--no-network-family-autoselection`).
- **Failure/rollback:** —
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** frontend engineering.
- **Why this effort:** architecture of the frontend matters for later phases.
- **Skills/connectors:** built-in browser (Claude_Browser), `web-design-guidelines`, `accessibility-audit`.
- **Opus escalation:** none.
- **When not to use Opus:** all.
- **Stop condition:** standard.

### Phase 28 — Mission visualization

- **Objective:** Mission Control (R3F 3D world, drone camera, live map, mission and flight panels, event timeline, abort), World Map, Perception, and Autonomy introspection, all on live data.
- **Why it exists:** the flagship UI (§46.3).
- **Dependencies:** Phase 27.
- **Files/modules:** `frontend/src/features/{mission-control,world-map,perception,autonomy}/*`, `frontend/src/visualization/*`.
- **Implementation tasks:**
  1. ENU → three.js transform (tested).
  2. Instanced voxels with LOD.
  3. Map-delta decoding in a worker.
  4. Detection overlays.
  5. FOV frustum.
  6. Planned vs actual paths.
  7. GT overlays gated to evaluation/replay contexts, with labels.
- **Research considerations:** provenance correctness in every visual.
- **Tests:** transform unit tests; a frame-rate performance test; Playwright flows; built-in browser screenshots in both themes.
- **Validation gate:** during a live SAR mission, the UI shows state, map, path, camera and detections consistent with the recorded MCAP (a spot check of 10 timestamps); ≥ 50 fps on this Mac at the target load; no GT shown in live-operation mode.
- **Expected output:** Mission Control working on real missions.
- **Known risks:** performance on the integrated GPU.
- **Failure/rollback:** reduce LOD; 2D-first fallback.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** visualization engineering.
- **Why this effort:** 3D performance + correctness.
- **Skills/connectors:** built-in browser; `data-visualization`.
- **Opus escalation:** none.
- **When not to use Opus:** all.
- **Stop condition:** standard.

### Phase 29 — Training / experiment analytics

- **Objective:** Training Lab (live metrics and diagnostics) and Experiment Lab (matrices, CI plots, pre-registration status, per-seed views, transfer-gap plots) from real results.
- **Why it exists:** research results must be inspectable and honest (§49).
- **Dependencies:** Phases 19, 21, 22, 27.
- **Files/modules:** `frontend/src/features/{training-lab,experiment-lab}/*`, `frontend/src/charts/*`.
- **Implementation tasks:** Observable Plot chart components that follow the chart conventions; tables with exact values; export of figure data (CSV) with run IDs.
- **Research considerations:** plots must match the offline analysis numerically.
- **Tests:** numeric parity between UI-displayed statistics and `summary.json`; chart accessibility (labels, tooltips, keyboard); visual checks.
- **Validation gate:** every chart displays axis, units, labels, legend, tooltip and scale, and cites its run IDs; parity with the offline analysis is exact.
- **Expected output:** the analytics UI.
- **Known risks:** large JSONL files.
- **Failure/rollback:** server-side aggregation endpoints.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** frontend + data engineering.
- **Why this effort:** correctness of scientific displays.
- **Skills/connectors:** `dataviz`, `data-visualization`, built-in browser.
- **Opus escalation:** if a displayed statistic appears methodologically wrong (it would reflect an analysis bug).
- **When not to use Opus:** chart styling.
- **Stop condition:** standard.

### Phase 30 — Replay Studio

- **Objective:** MCAP-based replay with play, pause, step, timeline, speed, camera, map, trajectory, detection overlays and mission events, plus decision inspection (policy action distribution and value at each decision).
- **Why it exists:** debugging, evidence and demonstration (§39).
- **Dependencies:** Phases 25, 28.
- **Files/modules:** `frontend/src/features/replay-studio/*`, `backend` replay windowing (extensions only).
- **Implementation tasks:** a sim-time cursor model; prefetching windows; synchronized panels; bookmarking events; comparison mode (two runs side by side, optional).
- **Research considerations:** the GT overlay toggle is clearly labeled.
- **Tests:** the desync test (§39.2); scrubbing performance; keyboard controls.
- **Validation gate:** the desync test passes on 3 replays at 3 detail levels; the flagship SAR replay plays end to end.
- **Expected output:** Replay Studio.
- **Known risks:** memory with L3 replays.
- **Failure/rollback:** stream windows; cap the image channels.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** frontend engineering.
- **Why this effort:** synchronization complexity.
- **Skills/connectors:** built-in browser.
- **Opus escalation:** none.
- **When not to use Opus:** all.
- **Stop condition:** standard.

### Phase 31 — Full system integration + E2E + real-data audit

- **Objective:** E2E flows (start simulation → mission → telemetry → completion → replay → results display); remove or verify all mocks; performance and robustness hardening; documentation brought to match the implementation.
- **Why it exists:** V1 completion (§44.1 e2e; §46.5).
- **Dependencies:** Phases 25–30.
- **Files/modules:** `tests/e2e/*`, docs updates, `scripts/audit/real_data_audit.py`.
- **Implementation tasks:** Playwright + sim E2E; a crash-recovery test (kill the bridge mid-mission → safety response); a soak test (10 consecutive missions); the real-data audit; a README quick start.
- **Research considerations:** none new; verify that the claims in the docs match the evidence.
- **Tests:** full E2E suite.
- **Validation gate:** E2E passes 5/5; the soak test finishes with 10/10 missions valid or failures diagnosed; the real-data audit finds zero production references to mocks; docs are complete per §57.
- **Expected output:** AERIS V1.
- **Known risks:** flakiness from timing.
- **Failure/rollback:** sim-time-based waits; retries only with logging.
- **Recommended model / effort:** Sonnet / High.
- **Why this model:** integration.
- **Why this effort:** breadth.
- **Skills/connectors:** built-in browser, `code-review`.
- **Opus escalation:** a major architectural flaw is revealed by integration.
- **When not to use Opus:** integration bugs.
- **Stop condition:** standard.

### Phase 32 — Language-conditioned missions (optional)

- **Objective:** NL → `MissionSpec` (grammar baseline + optional local-LLM parser via Ollama, schema-constrained, with UI confirmation) and region-conditioned mission behaviour; the compliance evaluation (§36).
- **Why it exists:** Level L15. Language must measurably change behaviour.
- **Dependencies:** Phase 31 PASS; user opt-in.
- **Files/modules:** `aeris/autonomy/mission/language/{grammar.py,llm_parser.py,validate.py}`, `configs/missions/annotated/*`, the instruction test set, experiments.
- **Implementation tasks:** region annotations in WorldSpec-derived mission maps; executive support for priorities and contingencies; parser evaluation; behaviour-compliance experiments.
- **Research considerations:** parser accuracy vs behaviour compliance; no GT semantics leak into the agent (regions are operator-provided).
- **Tests:** schema-rejection tests (the LLM cannot create commands outside the spec); adversarial instructions; compliance-metric tests.
- **Validation gate:** parser accuracy is reported on the held-out instruction set; conditioned missions show a measured compliance difference vs unconditioned (reported with CIs, whatever the sign); zero safety bypasses.
- **Expected output:** L15 report.
- **Known risks:** LLM hallucinated fields.
- **Failure/rollback:** grammar-only.
- **Recommended model / effort:** Opus 5.5 / Extra High.
- **Why this model:** language-conditioned autonomy architecture (prompt §51).
- **Why this effort:** novel integration with safety implications.
- **Skills/connectors:** Ollama MCP (local), `claude-api` skill only if the user chooses a Claude-API parser (a cost decision).
- **Opus escalation:** n/a.
- **When not to use Opus:** grammar-coverage additions.
- **Stop condition:** standard.

### Phase 33 — Multi-drone intelligence (optional)

- **Objective:** a multi-vehicle Tier H setup via the Linux fallback (B1/C) + FastSim multi-agent; team coordinator; shared map over a simulated comms channel; inter-drone deconfliction in Safety; team-exploration experiments; the Fleet UI area.
- **Why it exists:** Level L16 (§37).
- **Dependencies:** Phase 31 PASS; user opt-in; a Linux option available.
- **Files/modules:** `aeris/autonomy/team/*`, `aeris/safety/deconfliction.py`, `aeris/simulation/launcher/multi.py`, Linux container config, Fleet UI.
- **Implementation tasks:** per-instance ports and namespaces; allocation baselines; comms model; deconfliction; experiments vs N and comms degradation.
- **Research considerations:** allocation fairness; comms realism; safety proofs for separation.
- **Tests:** deconfliction property tests; multi-instance launcher tests (Linux); comms-model tests.
- **Validation gate:** zero inter-drone collisions across all Tier H team episodes; team-coverage results with CIs.
- **Expected output:** L16 report.
- **Known risks:** Docker networking for gz-transport / MAVLink on macOS; compute.
- **Failure/rollback:** FastSim-only multi-agent results, labeled as such.
- **Recommended model / effort:** Opus 5.5 / Extra High.
- **Why this model:** multi-agent methodology + safety.
- **Why this effort:** the highest system complexity.
- **Skills/connectors:** Docker (local).
- **Opus escalation:** n/a.
- **When not to use Opus:** container networking chores.
- **Stop condition:** standard.

### Phase 34 — Final autonomy / scientific / system audit

- **Objective:** an independent audit against §58: safety architecture, leakage, claims vs evidence, reproducibility (regenerate key figures), code quality, docs and viva readiness.
- **Why it exists:** "Evidence before conclusions."
- **Dependencies:** Phase 31 (and 32/33 if executed).
- **Files/modules:** `docs/audit/final_audit.md`, fix-list issues.
- **Implementation tasks:**
  1. Re-run SMOKE and one DEV experiment end to end from a clean clone.
  2. Regenerate all headline figures from stored data.
  3. Leakage and contract audits.
  4. Claims table: every claim in the README and docs → the evidence run IDs.
  5. Viva-question walkthrough (Appendix D).
- **Research considerations:** threats to validity (internal, external, construct) are written explicitly.
- **Tests:** full suite.
- **Validation gate:** all §58 criteria are assessed PASS, or FAIL with remediation; no unsupported claims remain.
- **Expected output:** the final audit report.
- **Known risks:** discovering invalid results late. This is acceptable: report and correct.
- **Failure/rollback:** retract claims and fix.
- **Recommended model / effort:** Opus 5.5 / Extra High.
- **Why this model:** final scientific judgment.
- **Why this effort:** comprehensive, adversarial review.
- **Skills/connectors:** `code-review`, `security-review`.
- **Opus escalation:** n/a.
- **When not to use Opus:** mechanical fixes found by the audit (delegate to Sonnet in a follow-up phase).
- **Stop condition:** standard; project V1 closed.

---

## 52. Model recommendation for every phase

| Model | Phases | Rationale |
|---|---|---|
| **Opus 5.5** | 0, 13, 14, 17, 18, 19, 21, 23, 26, 32, 33, 34 | Methodology decides validity: environment/reward/leakage design, PPO correctness, learned-autonomy experiment design and interpretation, SAR design, GPS-denied validity, design authority, optional advanced autonomy, final audit |
| **Sonnet** | 1–12, 15, 16, 20, 22, 24, 25, 27–31 | Engineering and integration to a precise spec, executing pre-registered experiments |
| **Haiku** | none as a primary phase model | Use *within* phases for low-risk chores: docs formatting, renames, repetitive test scaffolding, small CSS fixes, changelog updates |

Model availability observed in this environment (2026-09-25): Opus 5.5, Sonnet 5, Haiku 4.5, and Fable 5.1 are listed. The user's policy designates Opus 5.5 as the principal research/architecture model, and this spec follows that policy. Availability is re-checked at each phase start.

## 53. Effort recommendation for every phase

| Effort | Phases |
|---|---|
| Medium | 2, 6, 9 |
| High | 0, 1, 3, 4, 5, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 24, 25, 26, 27, 28, 29, 30, 31 |
| Extra High (`xhigh`) | 23, 32, 33, 34 |
| Low | none as a phase default (ad-hoc chores only) |

## 54. Validation gate for every phase

| # | Gate (summary; full text in §51) |
|---|---|
| 0 | Spec complete (58 sections; 19 fields per phase) |
| 1 | PX4 builds; gz server + x500 spawn; heartbeat/EKF; takeoff/land; MAVSDK telemetry; QGC; **headless depth + LiDAR valid** (or recorded fallback); version lock |
| 2 | lint/type/test/CI green; import contracts enforced |
| 3 | 10/10 start–ready–stop; clean reset; RTF table; no orphans |
| 4 | frame tests; telemetry ≥ 20 Hz; latency recorded; hardware guard; transport ADR final |
| 5 | 20/20 box flights < 0.5 m error; all injected faults handled 10/10 |
| 6 | 15/15 missions; abort-from-every-state tests |
| 7 | manifest + MCAP + metrics; exact re-read parity; GT contracts; D1 report |
| 8 | depth ≥ 10 Hz, scan ≥ 10 Hz, RGB ≥ 5 Hz for 5 min; obstacle localization test; zero GT leakage |
| 9 | 50 worlds/family load; ≥ 99 % SDF/occupancy agreement; reachable starts; split lock |
| 10 | 0 collisions on obstacle suites (or diagnosed); shield properties |
| 11 | map precision ≥ 0.9 / recall ≥ 0.8 (EKF pose); update p95 < 50 ms; no frame artifacts |
| 12 | H0.1 evaluated (frontier > random); planner p95 < 100 ms |
| 13 | sysid RMSE; trajectory and depth parity; ≥ 2 000 SPS; leakage audit; frontier-in-FastSim gap reported |
| 14 | all PPO tests; known-answer tasks; GRU-only memory task; bitwise resume; device decision |
| 15 | ≥ 4/5 seeds ≥ 80 % val success; Tier H spot check recorded |
| 16 | preprocessing parity; noise-model match; RQ3 go/no-go |
| 17 | pre-registered analysis complete (methodological gate) |
| 18 | leak/reset tests; 2×2 analysis; RQ1 pre-registration committed |
| 19 | pre-registered RQ1 evaluation complete; fairness + leakage audits |
| 20 | AP/precision/recall; localization error; FP rate; latency |
| 21 | flagship mission E2E with replay + report; RQ6 analysis; return-success reported |
| 22 | F4-isolation audit; RQ5 analysis complete |
| 23 | leakage audit; G0–G2 ATE/RPE + mission metrics; estimator fallback 10/10 |
| 24 | min-separation invariant in all episodes; tracking metrics at ≥ 3 speeds |
| 25 | API/WS tests; WS load test; security review resolved; no arbitrary execution |
| 26 | AA contrast; non-color encodings; 4 key flows specified; user design review requested |
| 27 | real telemetry in Live Flight; mock-exclusion check; a11y no criticals |
| 28 | UI vs MCAP consistency at 10 timestamps; ≥ 50 fps; no GT in live mode |
| 29 | chart completeness; exact parity with offline stats |
| 30 | desync test on 3 replays × 3 detail levels; flagship replay plays |
| 31 | E2E 5/5; soak 10/10 or diagnosed; zero mock references; docs complete |
| 32 | parser accuracy; compliance difference with CIs; zero safety bypasses |
| 33 | zero inter-drone collisions; team results with CIs |
| 34 | all §58 criteria assessed; no unsupported claims |

---

## 55. Opus escalation conditions

**Escalate to Opus** (switch the model, or flag in the phase report for the next session) when any of these occur:

1. RL methodology may be wrong: reward design, termination semantics, evaluation protocol.
2. PPO appears mathematically wrong: a known-answer test fails after ordinary debugging, or GAE/clipping behavior is inconsistent with the math.
3. Reward hacking is detected and a simple fix does not remove it.
4. Learned exploration collapses (e.g., entropy collapse to a degenerate subgoal, circling) and the §43 checklist does not resolve it.
5. The agent exploits simulator information (FastSim-only success, collision-detection gaps).
6. Observation leakage is found or suspected (a GT or privileged path into the actor).
7. The GPS-denied evaluation may be invalid (any GPS- or GT-derived quantity reaching the estimate or the policy).
8. The sensor-fusion architecture needs redesign.
9. A major autonomy architecture decision is questionable (safety layering, hierarchical action design).
10. SLAM/localization methodology needs redesign (T_M_O semantics, drift handling).
11. A generalization experiment may be scientifically invalid (split leakage, confounds).
12. Language-conditioned autonomy needs architectural reasoning.
13. Multi-drone coordination methodology is required.
14. A pre-registered analysis is about to be changed after data was seen. **Always escalate this.** Changes after seeing data are allowed only as clearly labeled exploratory analyses.

**Do not escalate** for: npm/pip/brew failures, import errors, CSS misalignment, a broken endpoint, test syntax errors, renames, formatting, flaky-test timing (unless it reveals a design flaw), or routine performance tuning.

---

## 56. Computational requirements

### 56.1 Budgets on the recorded machine (estimates; each is measured in the named phase)

| Workload | CPU | RAM | Notes / measured in |
|---|---|---|---|
| PX4 SITL | 0.5–1 core | ~0.3–0.5 GB | P1/P3 |
| Gazebo server, x500 base, headless | 1–2 cores | ~1 GB | P1 |
| Gazebo server + depth + RGB + LiDAR | 2–3 cores + GPU | 1.5–2.5 GB | P1/P8; rendering cost is the main unknown |
| Gazebo GUI | +1 core + GPU | +1 GB | debug only |
| QGroundControl | < 1 core | ~0.5 GB | debug only |
| AERIS core (mapping, Numba) | 1 core | 0.5–1 GB | P11 |
| Sensor Bridge | < 1 core | 0.2–0.5 GB | P8 |
| FastSim training (16–64 envs, small nets, CPU) | 4–8 cores | 1–4 GB | P13/P14 |
| Detector training (MPS or CPU) | GPU/CPU | 2–4 GB | P20 |
| Backend + frontend dev | 1 core | ~1 GB | P25+ |

**Rules:**
- On 16 GB, never run the Gazebo GUI, training and the frontend dev server at the same time.
- Tier H evaluation runs headless with QGC closed.
- Keep ≥ 40 GB of free disk for PX4 builds, datasets, checkpoints and replays. Replays default to L1.

### 56.2 Time estimates (to be measured and corrected)

- **Tier H episode (180 s sim):** about 60–180 s of wall time at an RTF of 1–3, plus ≈ 30–60 s restart overhead. An RQ1 evaluation (4 methods, 5 learned seeds, 50 paired world/start pairs) comes to roughly 400–500 episodes, or about 12–30 h split over several sessions or nights. This is feasible, but it is the dominant cost. The episode count is set in pre-registration from a power estimate based on Phase 12 variance, not chosen arbitrarily.
- **FastSim training:** §27.7.
- **Thermals:** the Air throttles. Report sustained SPS, not peak.

### 56.3 When remote compute is recommended (Option C)

Remote compute is recommended when any of these hold:
- the FULL training budget exceeds ~25 h per variant after optimization;
- RQ3 needs rendered RGB training;
- multi-drone Tier H evaluation is needed;
- the Tier H evaluation queue exceeds ~48 h.

The recommendation is always made explicitly in a phase report, with a cost/benefit note, and is never assumed.

---

## 57. Documentation strategy

### 57.1 Documents (created when the implementation exists; each describes actual behavior)

| Doc | Created | Content |
|---|---|---|
| `README.md` | P2 (quick start grows through P31) | What AERIS is, status table of levels achieved (with evidence links), quick start |
| `AERIS_TECHNICAL_SPEC.md` | P0 | This document (authoritative) |
| `docs/architecture.md` | P2 | Summary + diagrams; links to the ADRs |
| `docs/adr/NNNN-*.md` | P2 (from Appendix C) | Architecture decision records |
| `docs/mac-setup.md` | P1 | Exact, tested setup steps for this Mac |
| `docs/px4.md` | P3 | PX4 pin, params, ports, launch profiles, SITL explanation |
| `docs/simulation.md` | P3/P9/P13 | Tiers, worlds, splits, FastSim, parity results |
| `docs/autonomy.md` | P5+ | Safety layers, executive, planners, exploration |
| `docs/perception.md` | P8/P16/P20 | Sensors, frames, preprocessing, detector + model/dataset cards |
| `docs/mapping.md` | P11 | Voxel map, projection, frontiers, localization levels, SLAM status |
| `docs/reinforcement-learning.md` | P13/P14 | Env contract, rewards, PPO math → code mapping, training recipe, debugging checklist |
| `docs/experiments.md` | P7 | Manifests, configs, pre-registration, how to reproduce |
| `docs/evaluation.md` | P7+ | Metric definitions (mirrors §41), results per RQ with run IDs |
| `docs/backend.md` | P25 | API/WS reference (generated OpenAPI + narrative) |
| `docs/frontend.md` | P27 | Architecture, design-system usage, mock policy |
| `docs/viva_guide.md` | P12, then updated | Answers to the Appendix D questions, reflecting the actual implementation |
| `docs/phase_reports/phase-NN.md` | every phase | The mandatory end-of-phase report |

### 57.2 Rules

- Docs describe the implementation as it is, not aspirations.
- Planned features are marked **Planned (Phase N)**.
- Every quantitative claim cites run IDs.
- Diagrams live as text (Mermaid/ASCII) in the repo.

### 57.3 Spec change control

- A phase that changes this spec must include a "SPEC CHANGES" section in its report (section, old → new, reason).
- It must bump the spec version: minor for refinements, major for architectural changes.
- It must update the ADR index.

---

## 58. Final audit criteria (Phase 34)

| Area | Criterion |
|---|---|
| Safety architecture | No path from any learned component to below-velocity commands (type- and contract-verified). The SafetySupervisor is the sole CommandPort holder. Shield properties hold. The hardware guard is present. |
| Separation of flight control | PX4 source unmodified (or only documented build patches). No AERIS inner-loop control. |
| Observation integrity | Import contracts pass. Provenance audit of every checkpoint's observation spec. No ORACLE results presented as agent results. |
| Claims vs evidence | Every claim in the README and docs maps to run IDs. The §3 and prompt-§57 "never claim" list is checked item by item (navigation learned, RL beat classical, no-GPS, SLAM, generalization). |
| Statistical validity | Pre-registrations exist for all test-split results. Paired designs. CIs reported. Multiple-comparison correction applied. Invalid episodes are accounted for. |
| Reproducibility | A clean clone reproduces SMOKE and one DEV experiment. Headline figures regenerate from stored data. Version locks are honored. Determinism classes are stated. |
| Baselines | Classical baselines were tuned with a recorded budget comparable to the learned methods. |
| Generalization | F4 isolation proven by a manifest audit. |
| GPS-denied | Information-availability table. No GT or GPS leakage. ORACLE-VIO labeled. |
| Code quality | Lint and type checks pass. No god classes or giant files without justification. No silent exception swallowing. No dead experimental code. No unused dependencies (`deptry` or equivalent). |
| Frontend honesty | No mocks in production. All charts cite data sources. Provenance labels (EST/GT/ORACLE) are correct. |
| Accessibility | WCAG 2.2 AA for text contrast. Keyboard operation. Non-color state encodings. |
| Documentation | All §57.1 docs are present and accurate. The viva guide answers all Appendix D questions correctly for the actual system. |
| Negative results | Reported with the same prominence as positive ones. |

---

## Appendix A — Phase 0 environment snapshot (2026-09-25)

```
Model: MacBook Air (Mac17,3), Apple M5, 10 cores (4P "Super" + 6E), GPU 8-core, Metal 4, RAM 16 GB
macOS 27.2 (26B5091g), arm64; disk free ~169 GiB / 460 GiB
Homebrew /opt/homebrew (arm64)
python3 3.14.7 (brew) · python3.11 3.11.16 · torch: not installed
Xcode CLT: /Library/Developer/CommandLineTools · Apple clang 21.0.0
cmake: not installed · ninja: not installed · git 2.54.0 (Apple)
gz: not installed · QGroundControl: not installed
Docker 29.8.0 · uv present · node v26.9.0 / npm · ollama present · gh authenticated (EXCALIBUR303)
Thermal/perf warnings: none recorded at inspection time
Repository: ~/Claude/aeris did not exist before Phase 0 (created for this spec). Not a git repo yet (Phase 2).
```

Note: `cmake`/`ninja` are expected to be installed by PX4's `macos.sh` (verify in Phase 1).

## Appendix B — Sources consulted in Phase 0 (accessed 2026-09-25)

- PX4 Guide (main), macOS Development Environment: https://docs.px4.io/main/en/dev_setup/dev_env_mac.html
- PX4 Guide (main), Gazebo Simulation: https://docs.px4.io/main/en/sim_gazebo_gz/
- PX4 Guide (main), Gazebo Vehicles: https://docs.px4.io/main/en/sim_gazebo_gz/vehicles.html
- PX4 Guide (main), Multi-Vehicle Simulation with Gazebo: https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation.html
- PX4 Guide (main), ROS 2 User Guide: https://docs.px4.io/main/en/ros2/user_guide.html
- PX4 Guide (main), Robotics / offboard APIs: https://docs.px4.io/main/en/robotics/
- PX4 Guide (main), Offboard Mode: https://docs.px4.io/main/en/flight_modes/offboard.html
- PX4 Guide (main), Collision Prevention: https://docs.px4.io/main/en/computer_vision/collision_prevention.html
- PX4 v1.17 release announcement: https://px4.io/px4-autopilot-release-v1-17-what-you-need-to-know/
- PX4 issue #27026 (macOS gazebo-sim build failure, 2026-04): https://github.com/PX4/PX4-Autopilot/issues/27026
- Gazebo Harmonic macOS install: https://gazebosim.org/docs/harmonic/install_osx/
- Gazebo Transport Python support: https://gazebosim.org/api/transport/13/python.html
- QGroundControl download/install: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html
- MAVSDK-Python: https://github.com/mavlink/MAVSDK-Python
- ROS 2 REP-2000 (platform tiers): https://reps.openrobotics.org/rep-2000/ ; Jazzy release notes: https://docs.ros.org/en/kilted/Releases/Release-Jazzy-Jalisco.html
- PyTorch MPS availability issues on recent macOS: https://github.com/pytorch/pytorch/issues/167679 , https://github.com/pytorch/pytorch/issues/177819
- Community report of Gazebo GUI crash on Mac VMs + ogre workaround: https://github.com/pennaerial/monorepo/issues/505
- Methodology references (to be cited in docs): Schulman et al., "Proximal Policy Optimization Algorithms" (2017); Schulman et al., "High-Dimensional Continuous Control Using Generalized Advantage Estimation" (2016); Yamauchi, "A Frontier-Based Approach for Autonomous Exploration" (1997); Anderson et al., "On Evaluation of Embodied Navigation Agents" (SPL, 2018); Agarwal et al., "Deep RL at the Edge of the Statistical Precipice" (IQM/bootstrap, 2021); Huang et al., "The 37 Implementation Details of PPO" (2022).

## Appendix C — ADR index (to be expanded into `docs/adr/` in Phase 2)

| ADR | Decision | Section |
|---|---|---|
| ADR-001 | Mac-native first; isolate Linux-only components | §7, §12 |
| ADR-002 | PX4 as a pinned external dependency; no firmware modifications | §8.3 |
| ADR-003 | MAVSDK-Python as the primary vehicle transport behind `VehicleInterface`; pymavlink secondary | §8.5, §15 |
| ADR-004 | Separate Sensor Bridge process (Homebrew-Python gz bindings) + ZeroMQ IPC | §7.2, §18.4 |
| ADR-005 | No ROS 2 dependency in V1; adapter seam reserved | §11 |
| ADR-006 | Two-tier simulation (FastSim for learning, PX4+Gazebo for evaluation); headline claims require Tier H | §17 |
| ADR-007 | `WorldSpec` as the single source of world truth for both tiers + evaluator | §9.3 |
| ADR-008 | ENU/FLU internal frames; NED/FRD only inside the PX4 adapter | §19 |
| ADR-009 | 3D voxel log-odds + 2D altitude-band projection for V1 | §21 |
| ADR-010 | Seed-range splits + held-out world family F4 + split guards | §33 |
| ADR-011 | No RGB in FastSim V1; RQ3 gated on a feasible render path | §17.1, §5 |
| ADR-012 | MCAP replay format with detail levels; replay = playback | §39 |
| ADR-013 | React/TS/Vite + CSS-variable tokens + Radix + R3F + uPlot + Observable Plot; no Bootstrap | §46 |
| ADR-014 | In-house PPO (CleanRL-style), with an SB3 dev-only sanity reference | §27 |
| ADR-015 | Hierarchical learned exploration via masked subgoal actions executed by the shared planner + shield | §26.2 |

## Appendix D — Viva question map

| Question | Answer location (docs) | Key point AERIS must be able to show |
|---|---|---|
| What is PX4? | px4.md | Open-source flight-control stack: estimation, control, modes, failsafes |
| What is SITL? | px4.md | The PX4 firmware compiled for the host, running against a simulator in lockstep |
| What is Gazebo? | simulation.md | Physics + sensor simulator; Harmonic is the current LTS used by PX4 |
| Why use a flight controller? | autonomy.md | Stability and safety are solved, verified problems; separation of concerns |
| Why doesn't RL directly control motors? | autonomy.md, §16 | Safety, sample efficiency, verifiability; PX4 does it better |
| What is autonomous navigation? | autonomy.md | Perception → localization → mapping → planning → control loop |
| What is occupancy mapping? | mapping.md | Probabilistic log-odds grid updated by ray casting |
| What is SLAM? | mapping.md, §23 | Joint pose + map estimation with loop closure, and exactly which AERIS stage (if any) qualifies |
| What is LiDAR? | perception.md | Time-of-flight ranging; the 2D scan in AERIS |
| What is sensor fusion? | perception.md / px4.md | EKF2 fuses IMU/GPS/baro/flow; AERIS fuses depth + LiDAR in the map |
| What is partial observability? | reinforcement-learning.md | The agent sees only local sensor data, not the full state |
| Why use memory? | reinforcement-learning.md | Integrates history under partial observability (RQ2 evidence) |
| What is PPO? | reinforcement-learning.md | Clipped-surrogate policy gradient; the math in §27.3 |
| What are actor and critic? | reinforcement-learning.md | Policy vs value estimator; the shared trunk |
| What is GAE? | reinforcement-learning.md | Exponentially weighted TD residuals trading bias against variance (λ) |
| What is reward shaping? | reinforcement-learning.md | Auxiliary reward terms; risks (hacking watchlist) |
| What is exploration? | autonomy.md | Acquiring information about unknown space efficiently |
| What is frontier exploration? | autonomy.md | Go to known-free/unknown boundaries (Yamauchi) |
| What is learned exploration? | autonomy.md | A PPO policy choosing subgoals; the RQ1 result |
| What is GPS-denied navigation? | mapping.md, §34 | Localization without GNSS, with an exact availability table |
| What is generalization? | evaluation.md | Performance on unseen worlds; F4 OOD result |
| What is an ablation study? | evaluation.md | Remove or alter one factor to measure its effect (§42) |
| Which parts are learned? Which are deterministic? | autonomy.md, §26.3 | The table, kept current |

## Appendix E — Glossary (abbreviated)

| Term | Meaning |
|---|---|
| ATE / RPE | Absolute trajectory error / relative pose error |
| CI | Confidence interval (paired bootstrap, 95 %) |
| EKF2 | PX4's extended Kalman filter estimator |
| ENU / NED | East-North-Up / North-East-Down frames |
| FLU / FRD | Forward-Left-Up / Forward-Right-Down body frames |
| GT | Ground truth (simulator; evaluator-only) |
| ID / OOD | In-distribution / out-of-distribution |
| IQM | Interquartile mean |
| MCAP | Open, self-describing log container format used for replays |
| ORACLE | A baseline that receives privileged information; upper bound only |
| RTF | Real-time factor of the simulator |
| SPL | Success weighted by path length |
| Tier F / Tier H | FastSim / high-fidelity PX4+Gazebo |

---

*End of AERIS_TECHNICAL_SPEC.md v1.0.0 (Phase 0).*

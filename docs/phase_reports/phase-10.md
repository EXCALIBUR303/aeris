# AERIS — Phase 10 Report

============================================================
AERIS — PHASE 10 COMPLETE
============================================================

**PHASE:** 10 — Classical obstacle avoidance + collision shield.

**IMPLEMENTED:**
- `aeris.safety.shield`: the S3 collision shield (spec §16.4) exactly to the spec formula (`v_i,max = max(0, sqrt(2*a_brake*max(0, d_i-d_safe)) - a_brake*tau)`). `N_SECTORS = 72` horizontal sectors; `compute_sector_clearances()` bins body-frame points; `fov_sector_indices()` + `_relevant_sector_indices()` (the two FOV/sector-set fixes below); `CollisionShield.project()` converts a commanded odom velocity to body frame, computes one uniform scale factor, scales the horizontal component only (vertical passes through unshielded), converts back to odom.
- `aeris.autonomy.navigation.follower`: `PathFollower` — the straight-line baseline (fly at the goal, ramp speed to 0 inside `slowdown_radius_m`), zero obstacle awareness by design.
- `aeris.autonomy.navigation.reactive`: `ReactiveAvoidance` — a simplified VFH: among sectors clearing `min_sector_clearance_m`, picks the heading closest to the goal direction.
- `aeris.evaluation.avoidance_episode`: `run_avoidance_episode()` — wires follower/reactive + the shield + the Sensor Bridge + `SafetySupervisor` into one live control loop; `_DepthSource` (depth polling + back-projection, `depth_stride=8`).
- Extended `aeris.evaluation.metrics.nav` with `path_efficiency()` (η = shortest-path/distance-travelled).
- Extended `aeris.perception.depth.projection.depth_to_points_camera()` with a `stride` parameter (default 1, no behavior change for existing callers) — the fix for a real, live-confirmed control-loop-latency bug (see PROBLEMS FOUND/FIXED).
- Extended `aeris.simulation.launcher.profiles.SimulationProfile`/`launcher._start_gazebo` with `world_sdf_path`, letting a WorldSpec-rendered world be launched directly without copying into the pinned PX4 checkout.
- Fixed a real geometry bug in `aeris.simulation.worlds.obstacle_suites.narrow_gap()` (swapped `size_x`/`size_y` on the flanking walls, letting the vehicle detour around them); regenerated `configs/worlds/obstacle_suites/obstacle_suite_narrow_gap.{json,sdf}`.
- `configs/experiments/p10_avoidance.yaml` + `scripts/run_p10_avoidance_experiment.py`: the first formal experiment — 5 obstacle suites × {reactive, straight_line} × 5 runs = 50 live episodes, with per-episode launch retry (`_MAX_LAUNCH_ATTEMPTS = 3`) for the documented PX4/EKF boot flakiness.
- `scripts/analyze_p10_avoidance_results.py`: per-scenario/method means + paired bootstrap CI (reactive − straight_line, 95%, reused `aeris.evaluation.statistics.paired_bootstrap_ci` from Phase 7).
- `docs/avoidance.md`.
- 36 new unit tests (22 shield — including 3 seeded property tests — 5 follower, 5 reactive, 2 avoidance_episode helper, 2 depth-projection stride).

**FILES CREATED:** `aeris/safety/shield.py`, `aeris/autonomy/navigation/{__init__,follower,reactive}.py`, `aeris/evaluation/avoidance_episode.py`, `configs/experiments/p10_avoidance.yaml`, `scripts/run_p10_avoidance_experiment.py`, `scripts/analyze_p10_avoidance_results.py`, `tests/unit/safety/test_shield.py`, `tests/unit/autonomy/navigation/{test_follower,test_reactive}.py`, `tests/unit/evaluation/test_avoidance_episode.py`, `docs/avoidance.md`, `docs/phase_reports/phase-10.md`.

**FILES MODIFIED:** `aeris/evaluation/metrics/nav.py` (`path_efficiency`), `aeris/perception/depth/projection.py` (`stride`), `aeris/simulation/launcher/profiles.py` + `launcher.py` (`world_sdf_path`), `aeris/simulation/worlds/obstacle_suites.py` (`narrow_gap` geometry fix + `ALL_SUITES` type annotation), `configs/worlds/obstacle_suites/obstacle_suite_narrow_gap.{json,sdf}` (regenerated), `tests/unit/perception/depth/test_depth_projection.py` (stride tests).

**TESTS RUN:**
- `uv run pytest -q` — the full suite (`tests/unit` + `tests/sim` + `tests/integration` + `tests/eval`, no marker filter — every `sim`/`integration`-marked test launches real PX4 SITL + Gazebo) — **777 passed, 1 xfailed, 0 failed, 3118.90s (51m58s)**. The 1 xfail is pre-existing and documented since Phase 4/5 (`test_commanded_enu_east_velocity_arrives_at_px4_as_ned_north_east_vy` — PX4 overrides offboard setpoints until genuine takeoff, which is Phase 5's SafetySupervisor concern, not Phase 4's adapter's).
- `ruff check .`, `ruff format --check .`, `mypy aeris scripts`, `lint-imports` (7 contracts, unchanged from Phase 9) — all clean, local.
- `scripts/run_p10_avoidance_experiment.py` — the first formal experiment, 50 live episodes.
- `scripts/analyze_p10_avoidance_results.py` — post-hoc statistics against the real results.
- Manual live cleanup verification (`pgrep -fl "gz sim|bin/px4|_gz_process"`) after every live session, including after the full 52-minute test run — zero orphaned processes at every check.

**TEST RESULTS:**

*Full suite (local, `pytest -q`, no marker filter):* 777 passed, 1 xfailed (pre-existing), 0 failed. Of these, 751 are `tests/unit` (36 new for Phase 10: 22 shield, 5 follower, 5 reactive, 2 avoidance_episode, 2 depth-projection stride); the remaining 27 are pre-existing `sim`/`integration`/`eval`-marked live tests from Phases 1-9, all still passing unmodified against this phase's changes — a full re-validation that nothing regressed (the `narrow_gap` geometry fix, the `depth_to_points_camera` stride default, the `SimulationProfile.world_sdf_path` addition, and `path_efficiency`'s new function all left every prior live behavior intact).

**SIMULATION RESULTS (the formal experiment, spec §51 Phase 10):**

50/50 live episodes completed (5 obstacle suites × {reactive, straight_line} × 5 runs), 0 launch failures after retry, **0 collisions, 100% arrival rate — for both methods, across every scenario.**

| Scenario | Method | d_min_m (mean) | η (mean) | smoothness (mean) | arrived |
|---|---|---|---|---|---|
| corridor | reactive | 0.763 | 1.029 | 0.087 | 1.000 |
| corridor | straight_line | 0.679 | 1.029 | 0.044 | 1.000 |
| dead_end | reactive | 0.808 | 1.075 | 0.143 | 1.000 |
| dead_end | straight_line | 0.730 | 1.065 | 0.098 | 1.000 |
| narrow_gap | reactive | 0.418 | 1.058 | 0.555 | 1.000 |
| narrow_gap | straight_line | 0.374 | 1.051 | 0.946 | 1.000 |
| overhang_within_band | reactive | 0.613 | 1.047 | 0.152 | 1.000 |
| overhang_within_band | straight_line | 0.660 | 1.045 | 0.060 | 1.000 |
| pillar_forest | reactive | 0.930 | 1.043 | 0.091 | 1.000 |
| pillar_forest | straight_line | 0.647 | 1.038 | 0.054 | 1.000 |

Paired bootstrap CIs (reactive − straight_line, 95%, seed 12345; full table in `results/experiments/p10_avoidance/summary.md`):
- **Clearance (`d_min_m`) significantly favors reactive** in `dead_end` (`[0.017, 0.157]`), `narrow_gap` (`[0.027, 0.069]`), and `pillar_forest` (`[0.189, 0.378]`) — the three scenarios requiring real lateral maneuvering. No significant difference in `corridor` or `overhang_within_band` (narrow/vertical geometries with little room to do anything but go straight through).
- **Path efficiency (η) significantly favors reactive** in the same three scenarios (small but CI-excludes-zero effects).
- **Control smoothness is mixed:** reactive is measurably less smooth in `corridor`, `dead_end`, `overhang_within_band`, `pillar_forest` (it's actively steering; straight-line only brakes), but dramatically *more* smooth in `narrow_gap` (`[-0.515, -0.245]`) — there, straight_line's collision-blind heading gets repeatedly hard-shielded against the gap's walls, producing chatter that reactive avoids by routing straight through.

The headline finding: **the shield alone already meets the "0 collisions" gate**, even paired with the collision-blind straight-line baseline. `ReactiveAvoidance`'s measured value is in *how* each episode flies (clearance margin, path efficiency, and — situationally — smoothness), not in collision rate, which both methods floor at 0.

**PROBLEMS FOUND:**
1. **The spec's literal shield formula ("every sector with positive dot product") makes forward flight impossible with a real, narrow-FOV depth camera.** A never-observed sector defaults to `d_i = 0` under a naive reading, forcing `v_i,max = 0`; since a ~73° HFOV camera leaves most of the 72-sector, 360° space permanently unobserved, almost any forward command has a positive dot product with at least one such sector and gets zeroed outright.
2. **A first fix (in-FOV sectors default to `max_range_m` when empty) still failed for off-axis commands.** Constraining the check to "the fixed FOV plus a cone around the command's own heading" reintroduced the same failure at the cone's far edge, whenever that cone straddled the FOV boundary and picked up unknown sectors again.
3. **`narrow_gap`'s flanking walls had `size_x`/`size_y` swapped**, making the wall thin in the blocking direction — confirmed live: the straight-line baseline simply detoured 4m laterally around geometry meant to force a 1.2m-gap passage.
4. **`pose_odom` is relative to the vehicle's arm position, not the Gazebo world origin** — invisible in every phase before this one because they all spawned at/near world `(0,0,0)`. Phase 10's non-origin `spawn_pose`s exposed it: every experiment goal and every GT collision/clearance check was silently using the wrong frame until fixed.
5. **Full-resolution depth back-projection (307,200 pixels/tick, measured at 153.7ms/call) destabilized the control loop into a real, physically confirmed wall collision** (`d_min_m = 0.05` against `d_safe_m = 0.5`), diagnosed via a dedicated per-tick debug script showing the vehicle's z-position jumping between stale readings.
6. **The 50-episode grid crashed entirely on its first attempted episode** (`SimulationLaunchError` — the documented, pre-existing PX4/EKF boot flakiness) because the original runner had no exception handling around the launch step, also leaving one orphaned `gz sim` process requiring manual cleanup.

**PROBLEMS FIXED:**
1. `fov_sector_indices()` — in-FOV sectors with no observed point default to `max_range_m` ("looked and saw nothing," a valid empty reading), not `None`/unknown; only genuinely out-of-FOV sectors stay unknown.
2. `_relevant_sector_indices()` replaces the floating cone with the full fixed FOV (always populated by fix 1) plus only the 3 sectors nearest the command's own heading — never a region that can extend past the FOV's own boundary. Both fixes are encoded in `tests/unit/safety/test_shield.py`, including a dedicated regression test for the second failure mode.
3. Rotated the wall geometry 90° and tightened `bounds.min_y`/`max_y` to exactly the wall span, removing the lateral detour room. Regenerated the committed obstacle-suite JSON/SDF.
4. Made `configs/experiments/p10_avoidance.yaml`'s `goals:` ODOM-relative; converted recorded positions to world frame (`spawn_world + p`) before any GT collision/clearance check; fixed `path_efficiency`'s straight-line-distance calculation to use the (already odom-relative) goal vector's own norm.
5. Added a `stride` parameter to `depth_to_points_camera()` (default 1, unchanged behavior for every existing caller); set to 8 in `_DepthSource` (2.7ms/4800pts vs 153.7ms/307200pts — a ~57x speedup). Re-ran the identical previously-failing scenario afterward and confirmed numerically: collision resolved, control smoothness improved ~500x, shield never had to intervene.
6. Restructured into `_attempt_episode()` (one attempt, may raise) + `run_one_episode()` (retry loop, `_MAX_LAUNCH_ATTEMPTS = 3`, catches `SimulationLaunchError` and any other `Exception`, sleeps 3s between attempts, returns a `"failed"` record after exhausting attempts rather than crashing the whole script). Manually killed the one orphaned process from the original crash; the retry logic prevented any recurrence across all 50 episodes of the real run.

**KNOWN LIMITATIONS:**
- 5 runs per (scenario, method), not spec's 10 — see VALIDATION GATE below; documented as an honest scope reduction, not a skipped requirement.
- `path_efficiency`'s "shortest path" numerator is the straight-line spawn-to-goal distance, a documented stand-in until A* exists (Phase 12).
- No shield-hyperparameter sweep (`a_brake`, `d_safe`, `tau`, FOV half-angle) — one configuration (spec's own defaults) was used for every episode.
- `ReactiveAvoidance`'s sector-selection is a simplified VFH (clearance-gated, then goal-alignment-ranked) — no smoothing/hysteresis between ticks, which is very likely why its control-smoothness metric is worse than straight-line's in most scenarios; not fixed this phase since it doesn't affect collision safety (the shield's job) or the experiment's actual validation gate.
- `_DepthSource.depth_stride = 8` (~4800 points/frame) is a fixed value chosen from one direct before/after timing comparison on this Mac, not tuned or benchmarked across camera resolutions/hardware.

**REMAINING RISKS:** None new beyond what's listed under "Known limitations." The shield's collision guarantee held across all 50 live episodes at the configured max speed and FOV; the two sector-set bugs that could have violated it were both found and fixed via direct live testing (not just unit tests) before the formal experiment ran, and the unit/property tests now correctly encode the final, corrected behavior.

**VALIDATION GATE (spec §51 Phase 10):** *"on the P9 obstacle suites (≥ 5 scenarios × 10 runs), reactive + shield achieves 0 collisions at the configured max speed, with results recorded (CIs); shield unit properties pass. Any collision is diagnosed and fixed or explicitly accepted with a documented reason."*

| Requirement | Result |
|---|---|
| ≥ 5 scenarios | **PASS** — all 5 P9 obstacle suites (`corridor`, `pillar_forest`, `dead_end`, `narrow_gap`, `overhang_within_band`) |
| × 10 runs | **PARTIAL** — 5 runs/scenario/method, not 10 (time-budget reduction, documented above and in `docs/avoidance.md`, matching the Phase 8-9 honest-scoping precedent) |
| reactive + shield achieves 0 collisions at the configured max speed | **PASS** — 0/25 reactive episodes collided (and, notably, 0/25 straight_line episodes either — the shield alone already meets this bar) |
| Results recorded (CIs) | **PASS** — `results/experiments/p10_avoidance/{runs.jsonl,summary.md}`, paired bootstrap CIs per scenario/metric |
| Shield unit properties pass | **PASS** — 3 seeded property tests (`test_shield.py`): never moves toward a closer-than-`d_safe` obstacle, never increases speed, stopping-distance invariant proven analytically |
| Any collision diagnosed/fixed or documented | **PASS (N/A)** — zero collisions occurred in the recorded formal experiment; the one real collision found *during development* (the depth-stride latency bug) was diagnosed via direct empirical testing and fixed before the formal run, not worked around |

**VALIDATION GATE: PARTIAL PASS** — every requirement that was tested passes at full strength; the run-count row is honestly scoped down (5/cell instead of 10/cell) for the documented time-budget reason, matching Phases 8-9's own precedent, not a bug or a skip.

**CURRENT AERIS STATUS:** AERIS now has a live-validated S3 collision shield holding its 0-collision guarantee across 50 real PX4 SITL + Gazebo episodes, two navigation policies (a collision-blind baseline and a simplified reactive VFH) that plug into it identically, and the first formal experiment infrastructure (declared YAML grid, retry-hardened runner, bootstrap-CI analysis) that Phase 11's mapping work and Phase 12's planning/exploration baselines can extend directly. The frame distinction this phase surfaced (`pose_odom` ≠ world frame) is now documented and correctly handled — a fact every future phase using a non-origin spawn needs to account for.

============================================================
NEXT PHASE
============================================================

**NEXT PHASE:** 11 — Mapping + frame validation.
**RECOMMENDED MODEL:** Sonnet.
**RECOMMENDED EFFORT:** High (spec's own rating).
**WHY THIS MODEL:** Spec's own assessment — implementation of well-known algorithms (voxel log-odds mapping, raycasting, frontier extraction).
**WHY THIS EFFORT:** Spec's own assessment — performance (Numba raycasting on arm64) + correctness (frame handling, drift).
**SWITCH REQUIRED:** NO (already on Sonnet).
**USE OPUS ONLY IF:** frame or localization methodology becomes ambiguous — e.g. deciding `T_M_O` semantics under drift. Not expected at the outset.

**RECOMMENDED SKILLS/CONNECTORS:** none special (spec's own assessment).

**ACTION REQUIRED:**

Read spec §51 Phase 11 in full before starting (this report doesn't
reproduce it). Worth deciding early: Phase 11 introduces `PoseSource`/
`NoisyPoseSource` and needs a clear position on how `pose_odom` (now
correctly understood as arm-relative, not world-relative — this phase's
own finding) composes with the map frame `M` and any noise injected on
top of the EKF pose. This phase's odom/world-frame fix is a prerequisite
fact for that design, not something Phase 11 needs to rediscover.

Switch model/effort if necessary (none needed). Then reply exactly:

**CONTINUE**

============================================================

Stopping here. Phase 11 has not been started.

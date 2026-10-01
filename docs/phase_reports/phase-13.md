# AERIS — Phase 13 Report

============================================================
AERIS — PHASE 13 COMPLETE
============================================================

**PHASE:** 13 — RL environment: system ID + FastSim + Gymnasium env.

**IMPLEMENTED:**
- **System identification** (task 1): `scripts/sysid/record_tier_h.py` records live Tier H step responses (x/y 0.5–2.0 m/s, yaw 0.3–0.9 rad/s, two repetitions: rep 0 trains, rep 1 is held out), open-loop command sequences and depth frames. `aeris/simulation/fastsim/sysid.py` + `scripts/sysid/fit_dynamics.py` fit first-order lag + delay + accel clip per axis with bootstrap CIs → `configs/fastsim/dynamics.yaml`.
- **FastSim (Tier F)** `aeris/simulation/fastsim/`: `world.py` (WorldSpec → voxel bank, equal to Phase 9's `voxelize`), `sensors.py` (Numba parallel DDA raycaster, real `front_depth` intrinsics), `dynamics.py` (identified batch dynamics), `shield.py` (Numba port of S3, bit-identical), `batch.py` (camera rig, collision, pose-error model, geodesic field), `agent_map.py`, `vehicle.py` (`FastSimVehicle` implementing `VehicleInterface` + `CommandPort` with Tier H command semantics; task 2).
- **Pose-error model** (task 4): `scripts/sysid/analyze_openloop.py` → `configs/fastsim/pose_noise.yaml` (GT vs EKF).
- **Shared spaces** `aeris/learning/spaces/` (`spec.py`, `local_nav.py`, `exploration.py`): observation specs with per-field provenance and a spec hash, observation builders and action decoders used by both FastSim and Tier H call sites (spec §27.6).
- **Envs** `aeris/learning/envs/`: `LocalNavEnv`/`LocalNavVectorEnv` (native batched, same-step autoreset), `ExplorationEnv` (decision-level, 25 masked actions, Tier H's execution loop), `rewards.py` (v1, task 5), `wrappers.py` (runtime observation contract + spec hash, task 6).
- **Randomization ranges** (task 7): `configs/fastsim/randomization.yaml`, each range derived from the sysid CIs; `None` is the A5/H4 "off" arm.
- **Throughput benchmark** (task 8): `scripts/fastsim/benchmark_throughput.py`.
- **Parity experiments** (task 9): `scripts/sysid/depth_parity.py`, `scripts/sysid/analyze_openloop.py`, `scripts/fastsim/parity_exploration.py`, plus `scripts/fastsim/trace_tierh_exploration.py` (instrumented re-run of a Phase 12 Tier H episode, unmodified code).
- New import-linter contract: `aeris.learning.spaces` may not import any simulator or env (deployment-shared); `aeris.learning` added to the no-ground-truth contract.
- A `gym` optional extra (gymnasium only); CI installs it instead of `learn`, so CI doesn't pull multi-GB torch wheels nothing uses yet.
- `docs/fastsim.md`.

**FILES CREATED:** `aeris/simulation/fastsim/{__init__,world,sensors,dynamics,sysid,shield,batch,agent_map,vehicle}.py`, `aeris/learning/__init__.py`, `aeris/learning/spaces/{__init__,spec,local_nav,exploration}.py`, `aeris/learning/envs/{__init__,local_nav,exploration,rewards,wrappers}.py`, `configs/fastsim/{dynamics,pose_noise,randomization}.yaml`, `scripts/sysid/{record_tier_h,fit_dynamics,analyze_openloop,depth_parity}.py`, `scripts/fastsim/{benchmark_throughput,parity_exploration,trace_tierh_exploration}.py`, `tests/unit/simulation/fastsim/{test_fastsim_dynamics,test_fastsim_sysid,test_fastsim_world_sensors,test_fastsim_shield_parity,test_fastsim_vehicle}.py`, `tests/unit/learning/{test_spaces,test_local_nav_env,test_exploration_env,test_leakage_audit,test_rewards,test_wrappers}.py`, `docs/fastsim.md`, `docs/phase_reports/phase-13.md`.

**FILES MODIFIED:** `aeris/simulation/worlds/occupancy.py` (memoized box inverse + exact bounding-sphere early-out in `point_in_box`; ~15× faster coverage scoring, output bit-identical, see PROBLEMS #6); `tests/unit/simulation/worlds/test_occupancy.py` (equivalence test for that); `pyproject.toml` (import contracts, `gym` extra); `uv.lock`; `.github/workflows/ci.yml` (`--extra gym`); `docs/phase_reports/phase-12.md` (its NEXT PHASE block wrongly recommended Sonnet for Phase 13; spec says Opus 5.5, now corrected).

**TESTS RUN:**
- `uv run pytest -q` (full suite: unit + sim + integration + eval, no marker filter) as the mandatory pre-commit gate, on the working tree that will be committed (Phase 12 `6a82f6f` + the altitude follow-up `a1514c1` + Phase 13).
- Standalone retries of every full-suite failure, plus one further retry of the one that failed differently.
- `ruff check .`, `ruff format --check .`, `mypy aeris scripts`, `lint-imports` (8 contracts, 1 new): all clean.
- Live Tier H (real PX4 SITL + Gazebo) data collection: sysid step session, 10 open-loop sequences, 8 depth-parity frames, and 3 instrumented Phase 12 exploration episodes (`trace_tierh_exploration.py`).
- FastSim experiments: throughput benchmark; gate-6 parity, 54 episodes (rerun from scratch after each execution-semantics fix, PROBLEMS #7–#9).

**TEST RESULTS:**

*Unit:* **991 passed**, 0 failed. 79 are new for Phase 13: `tests/unit/simulation/fastsim/` (dynamics vs analytic step responses, sysid recovery of known parameters, raycast vs brute force, voxelization vs Phase 9, shield bit-parity vs `aeris.safety.shield`, `FastSimVehicle` command semantics), `tests/unit/learning/` (spaces round-trip, provenance guard raises on ORACLE, spec hash, cross-call-site builder identity, decoder vs the real `CommandValidator`, reward functions on hand transitions, Gymnasium `check_env`, D0 bitwise determinism, split discipline, termination vs truncation, leakage counterfactuals, runtime observation contract, exploration decision semantics incl. clock-advance regressions), plus the `point_in_box` fast-path equivalence test.

*Full suite:* **1014 passed, 5 failed, 1 xfailed** (32 min). All 5 failures are in live-sim test files neither Phase 13 nor a1514c1 touches. Mission, safety and vehicle code is byte-identical to Phase 12.
- 4 × `SimulationLaunchError: no MAVLink heartbeat received on port 14540 within 30.0s` (telemetry rate, RTF at speed 1.0, triangle missions, twenty box flights), the flaky-boot category documented in Phases 11–12.
- 1 × `test_ten_consecutive_start_stop_cycles_are_clean`: **a false positive I caused.** Its orphan detector pgreps for `gz sim|bin/px4`, and my own background wait loop's command line contained that literal string.

*Standalone retries:* telemetry rate, RTF, and start/stop cycles **passed**. `test_twenty_consecutive_box_flights` hit the heartbeat timeout again, the statistically fragile 20-launch test Phases 11–12 already characterized. `test_five_consecutive_runs_of_each_mission_template[triangle]` failed differently: 1/5 runs aborted "never landed", 52 s wall vs ~27 s nominal, consistent with host stress after many hours of continuous live sim. A further standalone retry **passed 5/5**. No orphaned `gz sim`/`px4` processes after any run.


**SIMULATION RESULTS (all live Tier H data from real PX4 SITL + Gazebo):**

*Gate 1, system ID* (`configs/fastsim/dynamics.yaml`; 16 xy + 6 yaw training steps, the same count held out): pooled-xy τ = 0.44 s [0.41, 0.45], delay 0.28 s [0.26, 0.30], a_max 4.0 m/s² [2.9, 4.6]; yaw τ = 0.23 s [0.20, 0.24], delay 0.12 s [0.10, 0.14], 2.5 rad/s² [1.6, 2.7]. **Held-out RMSE 0.059 m/s** (xy) and 0.024 rad/s (yaw). Per-axis fits (vx 0.062, vy 0.056 m/s held out) are kept in the YAML so the vx/vy accel asymmetry (3.4 vs 4.5) stays visible.

*Gate 2, open-loop trajectory parity* (10 seeded 20 s sequences): position RMSE 0.118–0.206 m (mean 0.160 m), worst final error 0.33 m. Pose-error model from near-stationary GT-vs-EKF pairs: white σ 1.94 cm, random-walk step 0.81 mm per 0.1 s, yaw σ 0.030 rad.

*Gate 3, depth parity* (f2_office_10000, 8 matched yaws at 1 m): pooled median |Δd|/d **1.5%** at the 0.1 m training resolution (worst single frame 5.96%); 0.05 m: 1.5% / 6.5%; 0.025 m: 0.10% / 0.41%.

*Gate 4, throughput* (local-nav vector env, 3 train worlds): 9,900 / 10,814 / **12,124** env-steps/s at 16 / 32 / 64 envs.

*Gate 6, exploration parity* (`results/fastsim/parity_exploration.json`; 3 val worlds × {nearest_frontier, random} × 3 seeds × 3 conditions = 54 FastSim episodes, against Phase 12's 3 Tier H runs per cell):

| World | FastSim NF, nominal (range) | Tier H NF (range) | Gap | face_travel NF |
|---|---|---|---|---|
| f1_rubble | 0.146 (0.145–0.146) | 0.157 (0.146–0.172) | −0.011 | 0.623 |
| f2_office | 0.667 (0.244–0.880) | 0.900 (0.873–0.954) | −0.233 | 0.951 |
| f3_warehouse | 0.213 (0.213–0.213) | 0.203 (0.191–0.213) | +0.009 | 0.542 |

**Office has no valid Tier H reference:** the retroactive ULog altitude audit from the follow-up commit a1514c1 finds all three Phase 12 office NF episodes grounded below the band for 3.3–13.6 s after handover; all rubble and warehouse episodes are valid. Against valid references, mean |gap| = **0.010** (rubble −0.011, warehouse +0.009, both inside Tier H's own range). Including the invalid office reference, 0.084 (`tierh_rate`: 0.085). Frontier − random, pooled paired bootstrap: nominal −0.264 [−0.457, −0.073] (Tier H/Phase 12: −0.072 [−0.164, +0.009]); face_travel −0.034 [−0.152, +0.092]. Random is where parity is weakest (rubble FastSim 0.711 vs Tier H 0.371). Interpretation and mechanism are in `docs/fastsim.md`. In short: in rubble and warehouse **both tiers are shield-deadlocked** (instrumented Tier H runs moved at most 0.23 m and 0.18 m from spawn), and yawing toward the travel direction removes both the deadlock and the H0.1 reversal, in FastSim.

*Instrumented Tier H re-runs* (`scripts/fastsim/trace_tierh_exploration.py`, Phase 12 code unmodified): rubble NF C = 0.488, **invalid under a1514c1's GT-altitude rule** (11.6 s out of band; 87 subgoals, 3 scans, replans every 0.86 s median, max displacement 0.23 m, ~10 s grounded mid-episode); warehouse NF C = 0.207 (max displacement 0.18 m, airborne throughout); office NF **invalid** (never climbed, C = 0.985 scored from the floor).


**PROBLEMS FOUND:**
1. **Sysid delay landed on the grid edge** (0.30 s, the grid's maximum) in the first fit. Widening the grid to 0.5 s moved the optimum to an interior 0.28 s, so the edge was a real constraint, not the answer.
2. **Yaw pose-noise was contaminated by command timing.** A heading error measured while the vehicle was rotating included the yaw-rate × timestamp mismatch, not just estimator error.
3. **Depth parity first showed ~30% error on the ground plane.** Gazebo's `x500_base` model pose sits 0.24 m below `base_link` (`<pose>0 0 .24` in model.sdf), and sensor extrinsics are relative to `base_link`. Composing the GT model pose directly with extrinsics put the camera 0.24 m low.
4. **Remaining near-wall depth error at 0.1 m voxels** (one frame at 5.96%). My first read of the 0.05 m run called this "not quantization"; the 0.025 m run (worst frame 0.41%) showed it is quantization, the wall's offset relative to the voxel grid.
5. **ActionDecoder outputs could trip the S1 validator** (2.0 m/s² between consecutive setpoints): a full-scale action reversal is a 4 m/s jump in one 0.1 s step.
6. **The Phase 12 coverage evaluator took ~30 min to score one 180 s episode.** `point_in_box` rebuilt and inverted every box's quaternion transform per ray sample, the same cost Phase 9 had already found and fixed in `voxelize` but never in the raycaster. It made gate 6's first attempt infeasible (an estimated ~27 h).
7. **The FastSim mirror of Phase 12's loop differed from Tier H in three places**, found by reading Phase 12's code line by line after the first gate-6 episodes stalled: (a) an A* failure returned instantly instead of running Tier H's 2 s rotate scan. A policy choosing unexecutable targets could then loop without advancing the clock; one office episode logged 927 zero-time decisions. (b) The map-change trigger counted changes since decision start; Tier H diffs the inflated grid tick to tick. (c) A* started from a snapped free cell; Tier H uses the raw cell. The action mask also snapped its start, so a "valid" action could fail at execution.
8. **Tier H's first replan always runs before any depth frame is integrated**, returns None, and starts every episode with a rotate scan (confirmed live). FastSim integrated a frame at reset, never turned, and in the shield-deadlocked worlds saw only its spawn heading (C = 0.054 vs Tier H 0.15).
9. **Zero-time "arrived" decisions froze the clock.** A frontier target inside the 0.4 m arrival radius ended its decision with zero ticks, so the strategy re-picked it forever (one gate-6 episode hung for 30+ min of wall time with the clock at t = 9.0 s). Tier H can't do this, since every loop iteration is a real tick.
10. **Tier H exploration episodes have undetected altitude failures.** Of three instrumented Tier H episodes, the office run never climbed (every A* failed, and Phase 12's loop only corrects altitude while path-following), so it sat on the floor for 180 s and still scored C = 0.985. The rubble run dropped to the floor for ~10 s mid-episode while PX4's EKF believed it was at 0.6–0.9 m. The coverage evaluator can't see either.

**PROBLEMS FIXED:**
1. Delay grid widened to 0.5 s; the fit is interior and reported with CIs.
2. Pose-noise fit restricted to near-stationary samples (speed < 0.3 m/s, |yaw rate| < 0.1 rad/s).
3. `GZ_MODEL_TO_BASE_LINK_M = (0, 0, 0.24)` in `fastsim/batch.py`, applied wherever GT model pose meets sensor extrinsics (depth parity script).
4. Not a code fix: both readings are reported in the gate table, and 0.1 m stays the training resolution for throughput, documented as a known near-wall cost.
5. `decode_actions_batch` slew-limits to 1.8 m/s² (below the validator's 2.0); tested against the real `CommandValidator`.
6. Memoized box inverse plus an exact bounding-sphere early-out (rotation preserves distance, with a float margin so the early-out can never reject a point the rotated test accepts). Bit-identical on a 20-pose trace and on 200 random rotated boxes × 44 points incl. faces/corners (new unit test); 19.6 s → 1.3 s.
7. `ExplorationEnv.execute_subgoal` now mirrors Tier H's fallbacks: A* failure and masked actions both become the rotate scan (flagged `failed`, so the clock always advances), a tick-to-tick map diff on the inflated grid, and raw-cell starts in both execution and the action mask. New tests: masked action = failed rotate scan advancing the clock; an always-masked policy still truncates.
8. The gate-6 scripted runner issues Tier H's startup scan explicitly (documented in the script and env docstrings). It isn't baked into the env, since it's an artifact of Tier H's startup timing, not a design choice.
9. Every decision now spends at least one control tick (immediate arrival hovers for one tick, as Tier H's loop iteration does). New tests: immediate arrival costs exactly one tick; every env step advances the clock under a random policy including masked actions. All 54 gate-6 episodes were rerun from scratch after this fix.
10. Fixed outside this phase's commit: raised as a follow-up task and done in a separate session as commit **a1514c1** on branch `fix/tierh-altitude-hold`. It adds an explicit EKF altitude hold on every Tier H loop command plus a GT-altitude validity check (> 2 s out of band → invalid; out-of-band poses never count as coverage), and persists GT trajectories. It diagnosed the rubble drop as EKF2 vertical-velocity divergence after an IMU-sample dropout. Its retroactive ULog audit is what invalidates gate 6's office reference. Evidence for the original finding: `results/fastsim/tierh_trace/*/{INVALID,ANOMALY}.md`.

**KNOWN LIMITATIONS:**
- Vertical dynamics aren't identified; every FastSim task holds altitude (`FastSimVehicle`'s vertical lag only gets through takeoff and landing).
- The exploration agent map is per-column 2D log-odds, an approximation of Phase 11's 3D voxel map + band projection. It doesn't reproduce Tier H's per-tick map flicker from EKF jitter in 3D (Tier H replans every ~0.9 s while stationary).
- 0.1 m voxels limit depth fidelity within ~1 m of walls (gate 3).
- The pose-noise model is fit on near-stationary data; error growth during rotation isn't modeled.
- Depth-noise randomization is one draw per vector env, resampled on each reset.
- Gate 6's valid Tier H reference is Phase 12's 3 runs per world for rubble and warehouse only (plus one valid instrumented warehouse run, C = 0.207); office has none. n = 3 per world is small.

**REMAINING RISKS:**
- In two of the three procedural worlds, the classical exploration loop is shield-deadlocked in **both** tiers (the vehicle barely leaves spawn), so gate 6 there compares "where the camera ends up pointing", not exploration. Parity in that regime says little about parity once a policy actually travels. The `face_travel` condition is the first evidence for the travelling regime, and it has no Tier H counterpart yet.
- Phase 12's Tier H office numbers are unreliable (7 of 8 audited episodes grounded after handover; a1514c1's audit), and so are the office pairs inside H0.1's pooled CI. Rubble, which drives the H0.1 result, is clean. Re-running Phase 12's experiment with the a1514c1 harness would give valid office numbers and a cleaner H0.1.
- The 0.24 m GT-model/base_link offset affects earlier phases that composed GT model pose with extrinsics (e.g. Phase 11's GT-condition map); not re-run here.
- Phase 11 attributed map error to pose error using PX4's self-reported eph (0.13–0.17 m). The measured stationary GT-vs-EKF error is ~1.9 cm, which weakens that attribution, at least for translation.

**VALIDATION GATE (spec §51 Phase 13):**

| # | Requirement | Result |
|---|---|---|
| 1 | Sysid held-out velocity RMSE ≤ 0.15 m/s | **PASS**: 0.059 m/s (xy pooled), 0.024 rad/s yaw |
| 2 | Open-loop position RMSE ≤ 0.5 m over 20 s, 10 sequences | **PASS**: worst 0.206 m, all 10 |
| 3 | Median absolute depth error ≤ 5% on matched frames | **PASS** on the spec's wording (pooled median 1.5% at 0.1 m). A stricter every-frame reading **fails at 0.1 m** (worst 5.96%, near-wall voxel quantization) and passes at 0.025 m (0.41%). Both readings reported. |
| 4 | ≥ 2,000 env-steps/s (local nav) | **PASS**: 12,124 |
| 5 | Leakage audit | **PASS**: no ORACLE fields; import contracts (8 kept); counterfactual tests (privileged reward corrupted → observations bit-identical; unseen geometry → identical obs/mask; in-view geometry → changed); runtime observation contract on every step |
| 6 | Scripted frontier in the FastSim exploration env reproduces Tier H frontier coverage within a reported gap | **MET, gap reported**: against valid Tier H references, rubble −0.011 and warehouse +0.009 (mean \|gap\| 0.010, both inside Tier H's run-to-run range). Office can't be assessed: all three Tier H office references fail the GT-altitude check. Against them anyway the gap is −0.233, with 1 of 3 FastSim seeds deadlocking where Tier H's map flicker would have unstuck it. The spec sets no numeric threshold and none was invented. Caveat: rubble and warehouse agree between two shield-deadlocked vehicles, so this validates heading/rotation behavior, not travel. |

**VALIDATION GATE: PASS** (gates 1, 2, 4, 5 pass outright; gate 3 passes on the spec's stated wording, with the stricter per-frame reading reported as failing at the training resolution; gate 6 is met as specified, gap reported, with the regime caveat above).


**CURRENT AERIS STATUS:** AERIS now has a trainable RL substrate: an identified, vectorized FastSim at ~12k env-steps/s on this laptop's CPU, Gymnasium local-nav and exploration envs whose observation builders and action decoders are shared with Tier H deployment, versioned rewards with declared privileged terms, runtime and static leakage guards, and a parity report against Tier H. Building that parity report also surfaced real Tier H harness defects (altitude loss, shield deadlock) that matter for every later Tier F↔H comparison.

============================================================
NEXT PHASE
============================================================

**NEXT PHASE:** 14 — PPO implementation.
**RECOMMENDED MODEL:** Opus 5.5 (spec §51 Phase 14: "PPO mathematical correctness is an explicit Opus responsibility").
**RECOMMENDED EFFORT:** High.
**SWITCH REQUIRED:** NO (Phase 13 ran on Opus 5.5).
**ACTION REQUIRED:** Read spec §51 Phase 14 and §27.2–27.4 in full before starting. Phase 14 is the first phase that needs `torch` (the `learn` extra); CI currently installs only `gym`, so decide then whether CI runs the PPO unit tests on CPU torch. The envs to train against are `LocalNavVectorEnv` (wrap with `VectorObservationContract` and stamp `spec_hash` into checkpoints) and `ExplorationEnv` (masked discrete). Consider resolving the Tier H altitude follow-up task before any Phase 15+ Tier H evaluation relies on exploration coverage.

============================================================

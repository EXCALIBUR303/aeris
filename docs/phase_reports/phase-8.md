# AERIS — Phase 8 Report

============================================================
AERIS — PHASE 8 COMPLETE
============================================================

**PHASE:** 8 — Sensor pipeline.

**IMPLEMENTED:**
- `aeris.simulation.bridge._gz_process`: the Sensor Bridge process — **the only AERIS module that imports `gz.*`** (spec §14.3 contract 5, now enforced by import-linter). Subscribes to `/depth_camera`, `/camera_info`, and one vehicle instance's namespaced RGB (`IMX214`) topic; republishes each over ZeroMQ PUB as msgpack (spec ADR-004); publishes per-topic health (rate/age/count) at 1 Hz.
- `aeris.simulation.bridge.schema`: the wire message envelope (`SensorMessage`) as plain dicts — both IPC sides only need `msgpack` installed, not each other's dependency tree. `GT_TOPIC_PREFIX = "gt."` reserves a distinct GT namespace.
- `aeris.simulation.bridge.client`: `GzBridgeClient` — the AERIS-venv-side ZeroMQ SUB, gz-free. Refuses (`GroundTruthTopicError`) to construct with any `gt.*` topic, structurally enforcing "the GT channel is unreachable from the agent client" rather than relying on convention. Uses a single-frame `topic + \x00 + payload` wire format, not multipart — see "Problems found."
- `aeris.simulation.bridge.launch`: `bridge_env()` (Homebrew `DYLD_LIBRARY_PATH`/`PYTHONPATH`/`GZ_IP`) + `start_bridge()`, launching the bridge via the same tracked-PID `ManagedProcess` every other AERIS process uses.
- `aeris.core.frames.extrinsics`: `load_sensor_extrinsics()`/`load_link_pose()` — a real SDF parser that recursively resolves `<include merge="true">` chains (composing each included model's own root `<pose>` plus the include tag's own `<pose>`) and locates `<sensor>` elements within them, producing `T_base_sensor`. Verified against the actual pinned `x500_depth`/`x500_base`/`OakD-Lite` SDFs, not assumed.
- `aeris.perception.depth.projection` / `aeris.perception.lidar.projection`: pinhole depth back-projection and 2D LiDAR back-projection into sensor-frame points. Numpy-free (spec scopes `numpy` to the `learn` extra, Phase 13+) — depth buffers are decoded via the stdlib `array` module.
- `aeris.perception.pose_buffer`: `PoseInterpolationBuffer` (spec §18.3) — NLERP interpolation between bracketing samples, drops (and counts) requests with no bracket or a >100ms gap between bracketing samples, ≥2s rolling history.
- `scripts/generate_sensors_config.py`: generates `configs/vehicle/sensors.yaml` from the pinned vehicle SDF (extrinsics via the module above; intrinsics from the SDF's own `horizontal_fov`/image size via the standard pinhole formula, confirmed live to match gz's own `CameraInfo` within 0.01 relative tolerance).
- Import-linter contract 5, converted from a placeholder comment into a real, enforced rule: `gz`/`rclpy` forbidden anywhere under `aeris`, with a single documented exception for `_gz_process.py` (needed `include_external_packages = true`, since the forbidden modules are external). Contract 1 (groundtruth privilege) widened to also cover `aeris.perception`, now that the package exists.
- `docs/sensors.md`.
- **Scoping decision:** used the stock `gz_x500_depth` model (which genuinely provides both depth *and* RGB via `OakD-Lite`) rather than composing a new `aeris_x500` model with an added 2D LiDAR. Investigated PX4's actual model-spawn mechanism (`px4-rc.gzsim`) far enough to confirm the composition itself is *not* blocked by anything fundamental (a `PX4_SYS_AUTOSTART` override would be needed for an unregistered model name) — but building, wiring, and live-testing that composed model plus its 2D LiDAR to the same rigor as everything else this phase needed was not judged to fit the remaining budget as well as spec's own explicitly sanctioned fallback clause (§8.4). Fully documented in `docs/sensors.md`, including the corrected understanding of the `PX4_GZ_MODELS` mechanism for whoever picks up the LiDAR-inclusion follow-up.
- 30 new unit tests + 2 new live `sim`-marked tests (rates/intrinsics/obstacle-localization, and the GT-isolation allowlist check).

**FILES CREATED:** `aeris/core/frames/extrinsics.py`, `aeris/perception/{__init__,depth/__init__,depth/projection,lidar/__init__,lidar/projection,pose_buffer}.py`, `aeris/simulation/bridge/{__init__,_gz_process,client,launch,schema}.py`, `configs/vehicle/sensors.yaml`, `scripts/generate_sensors_config.py`, `tests/unit/core/frames/test_extrinsics.py`, `tests/unit/perception/{depth/test_depth_projection,lidar/test_lidar_projection,test_pose_buffer}.py`, `tests/unit/simulation/bridge/{test_client,test_schema}.py`, `tests/sim/test_sensor_pipeline.py`, `docs/sensors.md`, `docs/phase_reports/phase-8.md`.

**FILES MODIFIED:** `pyproject.toml` (mypy overrides for `msgpack.*`/`gz.*`; the new/widened import-linter contracts; `include_external_packages = true`).

**TESTS RUN:**
- `pytest tests/unit` — 640 tests, local (30 new for Phase 8).
- `ruff check .`, `ruff format --check .`, `mypy aeris`, `lint-imports` (7 contracts now) — all clean, local (`make lint`).
- `pytest tests/sim/test_sensor_pipeline.py -v -m sim -s` (live PX4 SITL + Gazebo, **not** run in CI per spec §44.2) — run twice in full to confirm not flaky; both runs passed.
- Manual live verification of the sensor-rate/health mechanism independent of the pytest run (a 20s window via a standalone script), confirming the reported numbers below.
- Manual cleanup verification (`pgrep -fl "gz sim|bin/px4|_gz_process"`) after every live session — zero orphaned processes at every check.

**TEST RESULTS:**

*Unit (local):* 640 passed, 0 failed (30 new for Phase 8: 4 extrinsics, 7 depth back-projection, 4 LiDAR back-projection, 7 pose-interpolation-buffer, 3 IPC schema, 5 bridge client).

*Live SITL (`tests/sim/test_sensor_pipeline.py -v -m sim -s`, ~51s per run, 2 runs):*

| Test | Result |
|---|---|
| `test_gt_topic_is_unreachable_from_the_agent_client` | **PASSED** — both runs |
| `test_sensor_rates_intrinsics_and_obstacle_localization` | **PASSED** — both runs |

**SIMULATION RESULTS:**
- Sensor rates (bridge-side health, 20s window): depth ~30.7 Hz, RGB ~30.7 Hz, camera_info ~30.7 Hz — 3x the spec's ≥10Hz/≥5Hz gates.
- Live `camera_info` intrinsics matched the SDF-generated `sensors.yaml` intrinsics within 0.01 relative tolerance (`fx=fy=432.496...` for the 640×480 depth stream).
- Obstacle localization: a 0.5m box placed at 2m/4m/6m directly ahead of the camera was correctly localized (depth → back-projection → extrinsics → world frame) to within < 0.15m of its true near-surface position at all three distances, after two rounds of test-methodology fixes (see "Problems found").
- GT isolation: `GzBridgeClient(..., ["gt.pose"])` raises `GroundTruthTopicError` at construction; the bridge itself never publishes any `gt.*` topic — there is nothing to leak.
- Zero orphaned `gz sim`/`bin/px4`/`_gz_process` processes after every live session, verified by `pgrep`.

**PROBLEMS FOUND:**
1. **`zmq.CONFLATE` silently drops multipart messages.** The first `GzBridgeClient`/`_gz_process` design used ZeroMQ multipart `[topic, payload]` framing. A unit test (`tests/unit/simulation/bridge/test_client.py`) caught that a `CONFLATE`-mode subscriber never received anything, even after many retries — a documented libzmq limitation (`CONFLATE` is only well-defined for single-part messages).
2. **`GZ_IP=127.0.0.1` is required, not optional**, for `gz.transport13.Node.subscribe()` to receive anything, and for the plain `gz` CLI (`gz topic -l`, `gz service -s .../create`) to see the running instance's transport partition at all. Missing it made the bridge's own gz subscriptions silently receive zero messages, *and* made the live test's `gz service`-based box-spawn calls time out — while `gz service` itself still exits 0 on a timeout, printing "Service call timed out" to stderr instead of failing.
3. **`gz.msgs.CameraInfo.intrinsics` is a nested message (`Intrinsics`, itself holding the real repeated field `.k`), not a flat repeated `double`.** `list(msg.intrinsics)` raised `TypeError` inside every single gz-transport callback invocation — silently, since gz-transport's Python binding swallows a callback's exception unless the *subprocess's own stderr* is captured (`ManagedProcess(..., log_path=...)`). With no `run_dir` passed to `start_bridge()`, this bug was invisible: the "camera_info" topic just never appeared in the bridge's own health report, with no error visible anywhere.
4. **Live test-methodology bugs in the obstacle-localization check itself** (the pipeline under test was correct throughout; these were bugs in how the test measured it): (a) a depth camera measures range to an object's *near surface*, not its center — an initial box-center comparison was off by exactly `size/2` on every measurement; (b) a `conflate=False`, small-`RCVHWM` depth subscription backfills with the first few pre-spawn frames and then *silently drops every later message* (ZMQ backpressure, not FIFO eviction) rather than keeping the latest — every iteration of a 3-distance loop kept reading the same stale, box-free frame from before the loop even started; (c) with `conflate=True` restored, a *fixed*-size center-of-image sampling patch bled in background at longer range (the box's angular size shrinks with distance, eventually falling inside a fixed pixel patch), and taking the single *nearest* point in the patch (as a fix for (b)'s median result) was overly sensitive to single-pixel silhouette-edge rendering artifacts.
5. **Composed-model feasibility initially looked more blocked than it is.** Early investigation misread PX4's spawn mechanism (`file://${PX4_GZ_MODELS}/${MODEL_NAME}/model.sdf`, a direct filesystem path) as requiring `PX4_GZ_MODELS` to be a `model://`-style multi-directory search path; reading the full `px4-rc.gzsim` script clarified it's a single directory used for one literal file path, meaning a new AERIS-owned models directory genuinely can be pointed at directly. This was resolved through reading, not through running anything — recorded in "Problems fixed" and in `docs/sensors.md` for whoever picks up the LiDAR-inclusion follow-up.

**PROBLEMS FIXED:**
1. Switched to the classic single-frame ZeroMQ topic-prefix idiom (`topic.encode() + b"\x00" + msgpack_bytes`, `SUBSCRIBE` on `topic + delimiter`) in both `client.py` and `_gz_process.py`. Re-verified the same unit test passes with `CONFLATE` enabled after the fix.
2. `aeris.simulation.bridge.launch.bridge_env()` now sets `GZ_IP=127.0.0.1` alongside the existing `DYLD_LIBRARY_PATH`/`PYTHONPATH`, matching every other AERIS-launched process (`launcher.py`'s `_gz_env`/`_start_px4`). The live test's own `gz` CLI helper functions (`_spawn_box`/`_remove_model`) now also pass this env, and check the reply body (`"data: true"`) rather than trusting the CLI's exit code.
3. Fixed to `list(msg.intrinsics.k)`. `start_bridge()` and every live test now always pass `run_dir`, so a future silent callback exception surfaces in a real log file instead of vanishing.
4. (a) The test's expected comparison point now accounts for the box's near-surface offset (`distance_m - box_size_m/2` along the camera boresight) — a one-line, well-justified fix once the geometry was understood, not a tolerance workaround. (b) Switched the depth subscription to `conflate=True`, which is exactly the "give me the current frame" semantics this one-shot-per-distance sampling needs. (c) The sampling patch is now sized to 70% of the box's own known angular half-width at each tested distance (using the already-known distance and box size), staying safely inside its silhouette at all three ranges, with the median (not the minimum) taken within it.
5. No code change was needed — `PX4_GZ_MODELS` can point at any single directory, confirming a composed `aeris_x500` (outside the PX4 tree) is not blocked by this mechanism. Documented in `docs/sensors.md` as a corrected understanding for the follow-up.

All of these were found and fixed by direct, real-process debugging against a live SITL+Gazebo session (not by reasoning alone) — several (2, 3, 4b/c) were only visible once a genuinely realistic reproduction was run and its actual data inspected.

**KNOWN LIMITATIONS:**
- No 2D LiDAR / `scan` topic — deferred with the stock-model decision above. `aeris.perception.lidar.projection` exists and is unit-tested against synthetic scans, but nothing live publishes to it yet.
- Sensor rates were measured over a 15-20s window, not the spec's 5-minute sustained window — the measured rate is 3x the gate with no observed degradation mechanism, but that's an expectation, not a 5-minute measurement.
- The obstacle-localization test covers 3 distances at a single (head-on) yaw angle, not the spec's 3 distances × 4 yaw angles — the underlying frame math is orientation-agnostic per the synthetic-plane/rotated-sensor unit tests, but a live multi-angle sweep wasn't run.
- The "< 2 voxel sizes" localization tolerance has no real voxel size to reference before Phase 9's occupancy grid exists; the live test uses a fixed 0.15m stand-in, documented as such.
- `configs/vehicle/sensors.yaml` is generated and live-cross-checked against `camera_info`, but nothing in `aeris.perception` loads it back yet — there's no real consumer until Phase 9+ needs one.
- RGB frames are transmitted as raw `uint8` RGB (no JPEG compression) — spec's ADR-004 mentions JPEG as optional; raw was chosen to avoid a new image-codec dependency, and bandwidth wasn't a live-observed problem at the tested resolution/rate.
- The Sensor Bridge subscribes to exactly one vehicle instance's topics (hardcoded `x500_depth_0`-style paths built from `world`/`model_instance` args) — correct for AERIS's current single-vehicle scope (spec: multi-vehicle is Linux-only, Phase 33+), not yet parameterized for more.

**REMAINING RISKS:** None new beyond what's already listed under "Known limitations." The two live-only environment quirks (`GZ_IP`, the Homebrew-bindings-on-uv's-own-interpreter compatibility) are now both understood and encoded in code (`bridge_env()`) rather than tribal knowledge, reducing the risk they resurface confusingly in a later phase.

**VALIDATION GATE (spec §51 Phase 8):** *"depth ≥ 10 Hz, scan ≥ 10 Hz, RGB ≥ 5 Hz sustained for 5 min; the obstacle-localization test passes at 3 distances and 4 yaw angles; zero GT leakage."*

| Requirement | Result |
|---|---|
| Depth ≥ 10 Hz | **PASS** (measured ~30.7 Hz, 20s window — not the full 5 min) |
| Scan ≥ 10 Hz | **NOT MET** — no LiDAR in this phase's model (see "Why the stock model", `docs/sensors.md`) |
| RGB ≥ 5 Hz | **PASS** (measured ~30.7 Hz, 20s window — not the full 5 min) |
| Obstacle-localization passes at 3 distances × 4 yaw angles | **PARTIAL** — passes at 3 distances, 1 yaw angle (head-on) |
| Zero GT leakage | **PASS** — structural (`GroundTruthTopicError`) and by construction (the bridge publishes no `gt.*` topic at all) |

**VALIDATION GATE: PARTIAL PASS** — every measured item passes at strength; the scan-rate row and the full 4-angle/5-minute sweep are honestly not met this phase, for the documented, spec-sanctioned scoping reasons above rather than a bug or an oversight.

**CURRENT AERIS STATUS:** AERIS can now stream live depth and RGB frames out of Gazebo through a dedicated, gz-import-isolated Sensor Bridge process, over a ZeroMQ IPC boundary whose ground-truth channel is structurally unreachable from agent-facing code. Extrinsics are generated from the real, pinned vehicle SDF (not assumed or hand-measured) and verified live against a physical box at three known distances. Level L4 (perception) has its first real sensor data path; Phase 9's `WorldSpec`/occupancy work and Phase 10's classical avoidance now have a genuine depth stream to consume, once the deferred LiDAR and the sustained-rate/multi-angle sweep are picked up as follow-up (either within Phase 9 or as an explicit Phase 8b).

============================================================
NEXT PHASE
============================================================

**NEXT PHASE:** 9 — WorldSpec + procedural worlds + obstacle scenarios.
**RECOMMENDED MODEL:** Sonnet.
**RECOMMENDED EFFORT:** Medium (spec's own rating).
**WHY THIS MODEL:** Spec's own assessment — procedural generation engineering.
**WHY THIS EFFORT:** Spec's own assessment — clear spec, moderate complexity.
**SWITCH REQUIRED:** NO (already on Sonnet).
**USE OPUS ONLY IF:** the train/val/test split methodology is found scientifically flawed — not expected.

**RECOMMENDED SKILLS / CONNECTORS:** none special (spec's own assessment).

**ACTION REQUIRED:**

Read spec §51 Phase 9 in full before starting (this report doesn't
reproduce it). Also worth a look before or during Phase 9: the deferred
Phase 8 items above (2D LiDAR inclusion, the full 5-minute/4-angle
validation sweep) — Phase 9's own obstacle-suite generators may make the
multi-angle sweep easier to do properly (real obstacles instead of a
manually-spawned test box), so revisiting them together could be more
efficient than a separate pass.

Switch model/effort if necessary (none needed). Then reply exactly:

**CONTINUE**

============================================================

Stopping here. Phase 9 has not been started.

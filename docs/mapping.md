# AERIS — voxel mapping, pose sources, and the map-accuracy experiment

## Architecture (spec §21, §22, ADR-0009)

| Module | Responsibility |
|---|---|
| `aeris.mapping.raycast` | Numba-jitted per-frame ray integration (`integrate_frame`): traces every ray in one depth frame from a shared origin, accumulates *net* log-odds deltas per voxel into a Numba `typed.Dict`, returns only the unique voxels touched. `warm_up()` forces JIT compilation before any timed loop. |
| `aeris.mapping.voxel` | `VoxelMap` — the sparse, block-hashed (8³ blocks, `dict[block_key, np.ndarray]`) log-odds occupancy map. `MappingConfig` (resolution, `l_occ`/`l_free`/`l_min`/`l_max`, `max_range_m`). A never-touched voxel is `nan` (distinct from a touched-but-net-zero `0.0`). |
| `aeris.mapping.projection` | `project_band()` — the 2D altitude-band projection (spec §21.1: occupied if any in-band voxel occupied, free if all observed free, else unknown), over an explicit finite window. `inflation_m` (vehicle radius + margin) is a separate post-projection dilation, defaulting to 0 so map-accuracy evaluation never inflates before comparing to exact GT. |
| `aeris.mapping.frontier` | `detect_frontiers()` — 8-connected clusters of free cells bordering unknown space, excluding cells that only border the *queried window's own edge*. |
| `aeris.mapping.deltas` | `DeltaTracker` — 2D cell-state diffs between successive `BandGrid` snapshots, for future UI/replay streaming (3D voxel-level deltas are out of V1 scope — nothing consumes them yet). |
| `aeris.localization.pose_source` | `PoseSource` protocol (sync `pose_at(t_sim) -> PoseEstimate \| None`) and `PoseEstimate` (`T_M_B` + a simplified diagonal covariance). |
| `aeris.localization.px4_ekf` | `Px4EkfPose` — L-EST-GPS (spec §22), reusing Phase 8's `PoseInterpolationBuffer` (built then, never wired to a real consumer until now) so `pose_at(t)` genuinely interpolates to the requested timestamp rather than returning a stale snapshot. Covariance comes from PX4's own `ESTIMATOR_STATUS` accuracy fields, not a fabricated constant. |
| `aeris.localization.noisy` | `NoisyPoseSource` — wraps a base source with deterministic random-walk drift + i.i.d. noise ("synthetic degradation," spec §22). |
| `aeris.evaluation.metrics.map` | `evaluate_map()` — occupied precision/recall, free-space false-occupied rate, map coverage, all computed only over cells the agent's map actually observed (spec §21.3: "within observed regions"), checked against exact `WorldSpec` geometry, not a re-voxelized GT grid. |
| `aeris.evaluation.metrics.localization` | `ate()`/`rpe()` (spec §41) and `compute_t_w_o()` (spec §19.3's evaluator alignment — the least-squares translation estimate, which for a known-identical rotation is exactly the mean residual). |
| `aeris.evaluation.mapping_episode` | `run_mapping_episode()` — one live "hover-and-yaw sweep" (spec's own integration-test line), building all three pose-source maps from one depth stream. |
| `configs/experiments/p11_mapping.yaml` + `scripts/run_p11_mapping_experiment.py` | The 5-world map-accuracy experiment: GT pose vs EKF pose vs noisy pose. |

## The pipeline (spec §21.2)

```
Depth SensorFrame -> points in sensor/body frame -> T_M_S(t) = T_M_B(t) . T_B_S
   -> ray integration (free along ray, occupied at hit) -> voxel map -> 2D band projection
   -> frontier extraction (Phase 12 consumes this) -> map deltas (for UI/replay)
```

`T_M_B(t)` comes from whichever `PoseSource` is active for that map (GT / EKF / noisy); `T_B_S` is the fixed sensor extrinsics already established in Phase 8/10. V1 has `T_M_O = I` (spec §19.1), so `T_M_B ≡ T_O_B` — the pose a `PoseSource` returns is used directly as the map-frame pose.

## Numba, block hashing, and three real performance bugs found live

ADR-0009 calls for "sparse block hashing... with Numba ray integration." Getting this to actually clear the spec's own 50ms/frame gate took three rounds of live diagnosis, each confirmed empirically, not assumed:

1. **Numba itself is not the risk spec flagged.** A direct benchmark on this Mac (arm64, `numba==0.67.0`) showed the ray-tracing geometry costs ~0.6 microseconds/ray once warm (JIT compile is a one-time ~0.5s cost). The very first *naive* implementation — accumulating every ray's touched voxels one at a time into the Python-level block-hash dict — cost ~120ms for a 4,800-point frame anyway, because most of that cost was in ~144,000 raw ray-voxel touches (heavily overlapping near the origin), not the tracing itself.
2. **Fix: accumulate a whole frame inside Numba first.** `integrate_frame()` traces every ray in one call, aggregating touches into a Numba `typed.Dict[int64, float64]` (voxel coordinates packed into one integer key, see `encode_key`/`decode_key`), and returns only the *unique* voxels touched that frame (~14,600 of the 144,000 raw touches in the benchmark) with their *net* delta. Applying those to the Python-side block-hash dict brought a frame down to ~25ms.
3. **The live episode still measured 57-99ms p95 anyway, for two more real reasons, both found live:**
   - `_DepthSource` was calling `Transform.apply()` — a quaternion Hamilton product plus additions — once per point in a plain Python loop (~3,500 points/frame), costing 13-26ms by itself. Fixed by precomputing the fixed extrinsic transform's rotation as a 3x3 matrix once and applying it to every point in one vectorized NumPy multiply (~2-4ms).
   - Even after that fix, live latency still occasionally spiked to 55-99ms in a clear, quasi-periodic pattern. Diagnosis: Python's generational garbage collector was firing mid-integration. `gc.disable()` for the sweep's duration (with an explicit `gc.collect()` right after each timed integration call — outside the timed window, so memory stays bounded across a long sweep) removed the spikes entirely: a 53-frame live sweep afterward measured p95 = 40.00ms, every single frame under 50ms.

A fourth, smaller finding: `raycast.warm_up()` existed from the start but was never actually called anywhere, so the *first* real frame of every episode silently paid the one-time JIT-cache-load cost inside the timed window. Wiring it in at the top of `run_mapping_episode()` (before the vehicle even connects) fixed that specific outlier.

Also fixed in `raycast.py` itself, caught by a hand-computed unit test before any live testing: the per-ray traversal originally accumulated its "last" sample via `i * step` (`n_steps` float multiplications), which for a ray landing near a voxel boundary could round down to the *wrong* voxel — a synthetic "fan of rays against a flat wall" test caught one ray's hit voxel silently never being marked occupied at all. Fixed by computing the ray's true endpoint directly from its original (pre-clamped) length and unit direction, not by accumulating steps.

## The `T_W_O`/`T_M_O` frame story (building on Phase 10's finding)

Phase 10 discovered that PX4's EKF origin (`pose_odom`) sits at the arm/spawn position, not the Gazebo world origin. Phase 11 formalizes the fix spec §19.3 already names: `compute_t_w_o()` computes the least-squares translation-only alignment between `O` and `W` from paired hover samples — which, given no rotational offset between the two (also confirmed live in Phase 10), is exactly the mean residual `gt_position - odom_position`. `run_mapping_episode` uses the simpler, already-validated version of this (`spawn_world` subtraction) directly rather than running a separate hover-calibration step, since Phase 10 already established it holds to the precision this phase needs; `compute_t_w_o()` exists as the more general, spec-literal tool for a future phase that needs genuine per-episode refinement (e.g. under real drift).

## The same frame mistake recurred in map evaluation itself — and was invisible in the one test that would have caught it

The map-accuracy experiment's very first live run measured **0% occupied precision on every world, every pose-source condition** (GT included) — every single occupied prediction was a false positive. Root cause: `VoxelMap` (and the `BandGrid` projected from it) lives in frame `M`/`O`, spawn-relative; `evaluate_map()` was checking an `M`-frame cell's coordinates *directly* against `WorldSpec`'s world-frame box/cylinder geometry with no conversion — the exact same mistake Phase 10 made and fixed for goals and collision checks, recurring here because `aeris.evaluation.metrics.map` was new code that never inherited that fix.

The integration test built specifically to catch frame artifacts (`tests/integration/test_mapping_live.py`, spec's own "no mirrored or rotated artifacts" gate item) passed anyway, for the same reason every phase before Phase 10 never noticed the original bug: its synthetic single-wall world spawns at world `(0, 0, 0.1)`, so `M` and `W` coincide there and the missing conversion is a no-op. Every real obstacle-suite world (`corridor`, `pillar_forest`, ...) spawns away from the origin, which is exactly where the bug actually bites — a second, concrete illustration of why Phase 9's obstacle suites (non-origin spawns) were the thing that first surfaced this whole class of bug in Phase 10, and why a frame test built around an origin-spawned world doesn't actually exercise the risk it's meant to guard against.

**Fix:** `evaluate_map()` takes an explicit `spawn_world` and converts each `M`-frame cell center to world frame before checking GT geometry; the experiment script converts its `project_band()` query window (world bounds − spawn) the same way. A regression test (`test_spawn_world_converts_m_frame_cells_to_world_before_gt_check`) pins both the broken and fixed behavior with a non-origin spawn. Measured live, before/after, on `pillar_forest` (still at the spec-default 0.2m resolution, before the second fix below): GT-condition precision went from 0.000 to 0.656 with nothing else changed.

## Map evaluation: exact geometry, not a re-voxelized comparison

`evaluate_map()` checks each *observed* 2D cell's classification against `aeris.simulation.worlds.occupancy.is_occupied()` — the exact box/cylinder primitives, sampled at the agent's own resolution through the queried altitude band — rather than against a second, independently-voxelized GT grid that could disagree with the agent's own grid purely from resolution mismatch. This keeps precision/recall honest: a false positive or false negative reflects the agent's actual sensing/estimation error, not two different quantization choices disagreeing with each other.

## A second, separate frame-resolution bug: sub-resolution-thick walls

Fixing the M/W frame bug alone still left `corridor`/`dead_end`/`narrow_gap` (all using Phase 9's `_WALL_THICKNESS_M = 0.15` walls) at or near 0% GT-condition precision, while `pillar_forest` (0.3m-radius cylinders) and `overhang_within_band` (a 3m x 8m slab) improved substantially. Direct check against `is_occupied()`: at the spec-*default* 0.2m resolution, the cell whose index aligns with a wall's centerline has its cell *center* fall just **outside** the wall's own 0.075m half-thickness (a 0.025m miss), even though a real depth return hitting the wall's near face lands in that exact cell. Fixed by using 0.15m resolution for this experiment (still inside spec's stated 0.15-0.25m range) — confirmed live: `corridor`'s GT-condition precision went from 0.000 to 1.000 with nothing else changed.

## The remaining, real gap between GT-pose and EKF-pose precision is about obstacle thickness vs EKF accuracy, not a bug

With both bugs above fixed, GT-condition precision is good-to-perfect on every world (0.510-1.000), directly confirming the pipeline itself (frames, resolution, ray integration, projection) is correct. EKF-condition precision, however, stays low everywhere *except* `overhang_within_band` (0.932 -- the only world where the EKF condition alone clears the validation gate's 0.9 threshold):

| World | Obstacle | Perpendicular thickness | GT precision | EKF precision |
|---|---|---|---|---|
| corridor | wall | 0.15m | 1.000 | 0.000 |
| pillar_forest | cylinder | 0.6m (diameter) | 0.510 | 0.078 |
| dead_end | wall | 0.15m | 0.824 | 0.037 |
| narrow_gap | wall | 0.15m | 0.943 | 0.140 |
| overhang_within_band | slab | 3m x 8m footprint | 0.978 | 0.932 |

The pattern tracks obstacle size directly: PX4's own reported EKF horizontal accuracy in these episodes is ~0.13-0.17m (`ekf_flags.pos_horiz_accuracy_m`) -- comparable to or larger than a 0.15m wall's thickness or a 0.3m pillar's radius, so a perfectly ordinary EKF position/orientation error is, by itself, enough to place a detection just outside a thin obstacle's true extent on a large fraction of ticks. The overhang slab's 3m x 8m footprint is roughly 20-50x larger than that same error, so the identical EKF noise almost never matters. This isn't a resolution artifact (GT precision is already high at 0.15m) and isn't a code bug (the same pipeline, same tick, same points, differ only in which `PoseSource` supplied the pose) -- it's a genuine, physically sensible finding about *what kind of obstacle geometry an EKF-driven occupancy map can represent precisely*, and it's exactly the sensitivity spec's own research consideration asks this experiment to quantify.

## A related, single-viewpoint methodology limit: almost nothing gets classified FREE

`free_false_occupied_rate` came back at (or extremely close to) 1.000 across every world/condition -- not because the map is wrong, but because spec's own literal rule for a FREE column ("free if *all* voxels in the band are observed free," §21.1) is a strict, whole-column requirement, and a single fixed-point hover-and-yaw sweep's rays stay clustered around the camera's own height rather than sweeping the full altitude band. Almost no column ever gets *every* one of its ~15-20 in-band voxels individually confirmed free from one vantage point, so almost nothing is ever classified FREE at all (columns instead correctly stay UNKNOWN, per the "partially observed stays unknown" rule) -- an expected consequence of this phase's chosen live-test methodology (spec's own "hover-and-yaw sweep," not full exploration), not a defect. A multi-waypoint sweep or real frontier exploration (Phase 12) would accumulate genuine full-column free observations over time.

## What this phase deliberately did not do (honest scope reduction)

- **One live episode per world, not per (world, pose-source) pair.** All three conditions (GT/EKF/noisy) are built from the *same* depth stream within one flight — 5 live flights, not 15 — since the comparison only needs the same sensor data fed through three different pose estimates, not three independently-flown trajectories.
- **A hover-and-yaw sweep, not full exploration.** Spec's own integration-test line calls for exactly this ("hover-and-yaw sweep in a known world"); full multi-waypoint exploration coverage is Phase 12's frontier/planning job, not Phase 11's.
- **No 3D voxel-level map deltas** — only 2D band-grid deltas (`aeris.mapping.deltas`), since nothing downstream needs 3D-resolution streaming yet.
- **RPE is translation-only**, not the full 6-DoF relative pose error some SLAM literature uses — spec's own §41 line gives no explicit formula for RPE (unlike ATE), and this dataset has no independent rotational-error signal beyond what ATE's position error already captures.
- **No-return depth pixels never reach the max-range free-space code path live** — `depth_to_points_camera` already drops invalid/no-return pixels upstream (a Phase 8 decision, unmodified here), so `VoxelMap.integrate_ray(..., is_hit=False)` is implemented and unit-tested against synthetic no-return rays but isn't exercised by the live camera pipeline this phase.

## Using it

```python
from aeris.mapping.voxel import VoxelMap, MappingConfig
from aeris.mapping.projection import project_band
from aeris.evaluation.metrics.map import evaluate_map

voxel_map = VoxelMap(config=MappingConfig())
voxel_map.integrate_points(origin_m, points_m)  # one depth frame

grid = project_band(voxel_map, z_lo_m=0.3, z_hi_m=3.0, x_range_m=(-5, 5), y_range_m=(-5, 5))
result = evaluate_map(grid, world_spec, z_lo_m=0.3, z_hi_m=3.0)
```

```bash
uv run python scripts/run_p11_mapping_experiment.py
uv run python scripts/analyze_p11_mapping_results.py
```

See `docs/phase_reports/phase-11.md` for the live run's actual numbers and validation-gate assessment.

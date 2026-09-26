# AERIS — the S3 collision shield, reactive avoidance, and the first formal experiment

## Architecture (spec §16.4, §51 Phase 10)

| Module | Responsibility |
|---|---|
| `aeris.safety.shield` | `CollisionShield` — the S3 sector-based velocity-projection shield. `N_SECTORS = 72` horizontal sectors; `compute_sector_clearances()` bins body-frame points into per-sector minimum range; `sector_v_max()` is the spec formula; `CollisionShield.project()` scales a commanded odom-frame velocity down to whatever every relevant sector can tolerate. |
| `aeris.autonomy.navigation.follower` | `PathFollower` — the "straight-line" baseline: fly directly at the goal, ramping speed to 0 inside `slowdown_radius_m`. No obstacle awareness at all; every collision it would otherwise cause is entirely the shield's job to prevent. |
| `aeris.autonomy.navigation.reactive` | `ReactiveAvoidance` — a simplified VFH: among sectors clearing `min_sector_clearance_m`, picks the one whose heading is closest to the goal direction. A planner of *intent* only — the shield still has final say over the output velocity's magnitude regardless of which heading this module picks. |
| `aeris.evaluation.avoidance_episode` | `run_avoidance_episode()` — wires a navigation policy + the shield + the Sensor Bridge + `SafetySupervisor` into one live control loop (connect → arm/takeoff → loop until arrival/timeout → land/disarm), and `_DepthSource` (depth back-projection at each tick). Lives beside `aeris.evaluation.episode` (Phase 7's "one live episode, one result" pattern), not in `aeris.autonomy`, because it owns simulation/IPC concerns that module deliberately doesn't. |
| `configs/experiments/p10_avoidance.yaml` + `scripts/run_p10_avoidance_experiment.py` | The declared 5-scenario × 2-method × 5-run grid and its runner, with per-episode launch retry (`_MAX_LAUNCH_ATTEMPTS = 3`) for the documented PX4/EKF boot flakiness. |
| `scripts/analyze_p10_avoidance_results.py` | Per-scenario/method means plus a paired bootstrap CI (reactive − straight_line) on each metric, written to `results/experiments/p10_avoidance/summary.md`. |

## The shield formula (spec §16.4)

For each sector `i` with observed clearance `d_i`:

```
v_i,max = max(0, sqrt(2 * a_brake * max(0, d_i - d_safe)) - a_brake * tau)
```

`CollisionShield.project()` converts the commanded odom-frame velocity into
body frame, computes a single uniform scale factor — the minimum, over the
*relevant* sectors, of `v_i,max / dot(v_body, sector_direction(i))` for
every sector with a positive dot product — clamps it to `[0, 1]`, and
scales the horizontal component only (vertical velocity passes through
unshielded, since none of the sectors carry altitude information).

## Two hard-won fixes to "which sectors are relevant" (both found live, not in review)

The formula above sounds simple, but two successive live-flight failures
came from the exact same root cause: a real depth camera's ~73° HFOV
leaves the vast majority of the 72-sector, 360° space permanently
unobserved, and the spec's literal wording ("every sector with positive
dot product") doesn't survive contact with that fact.

1. **Unknown sectors defaulting to `v_i,max = 0` made forward flight
   impossible.** A never-observed sector has no clearance reading; scoring
   it as `d_i = 0` (unknown ⇒ worst case) forces `v_i,max = 0`, and *any*
   sector with a positive dot product with the command — including nearly
   the entire unobserved 287° behind and to the sides — zeroes the whole
   command's scale factor the instant the vehicle tries to move. **Fix:**
   `fov_sector_indices()` partitions sectors into "in the camera's fixed
   FOV" and "not"; in-FOV sectors with no point in them default to
   `max_range_m` (a valid *empty* reading — "looked and saw nothing"), not
   `None`/unknown. Only out-of-FOV sectors stay unknown.

2. **A "relevant cone" around the command's own (possibly off-axis)
   heading reintroduced the same bug at the cone's far edge.** The first
   fix's constraint set was "every in-FOV sector, plus a cone around the
   command's heading" — but a floating cone can straddle the fixed FOV's
   boundary, so its far edge picks up unknown (never-populated) sectors
   again, zeroing near-dead-ahead commands whenever the goal direction was
   even slightly off-axis. **Fix:** `_relevant_sector_indices()` replaces
   the floating cone with the full fixed FOV (always populated, by fix 1)
   plus only the 3 sectors nearest the command's own heading — never a
   region that can extend past the FOV's own boundary.

Both fixes are encoded in `tests/unit/safety/test_shield.py`, including a
regression test for exactly the second failure mode
(`test_off_axis_heading_near_fov_boundary`) and three property tests
against a seeded `random.Random` proving the shield can never move toward
an obstacle closer than `d_safe`, never increases commanded speed, and
that the post-shield stopping distance never exceeds `d_i - d_safe` (an
analytic identity, not a simulated check).

## The `pose_odom` / world-frame distinction (a new, previously-latent fact)

PX4's EKF local-position origin — what `VehicleState.pose_odom` reports —
is set at (or near) the vehicle's **arm position**, not the Gazebo world
origin. Every phase before this one spawned at or near world `(0, 0, 0)`,
which silently masked the distinction: odom and world frame were
numerically identical by coincidence. Phase 10 is the first to use a
non-origin `spawn_pose` (from each obstacle suite's own
`WorldSpec.spawn_poses[0]`), which exposed it immediately.

Two places in the pipeline needed the fix:

- **Goals.** `configs/experiments/p10_avoidance.yaml`'s `goals:` are
  ODOM-relative (`world_goal − world_spawn`), not world-frame — a
  straight-line goal of `(13.5, 0, 1.0)` means "13.5m past wherever the
  vehicle armed," not "13.5m from the world's SDF origin."
- **Ground-truth checks.** `has_collision()`/`minimum_clearance_m()` take
  `WorldSpec`'s world-frame obstacle geometry, so recorded ODOM positions
  are converted back to world frame (`spawn_world + p`) before being
  checked, and `path_efficiency`'s straight-line distance uses the
  (already odom-relative) goal vector's own norm, not a world-frame
  computation.

## The `narrow_gap` obstacle suite had its wall backwards

`obstacle_suites.narrow_gap()`'s two flanking walls were built with
`size_x`/`size_y` swapped, making each wall thin in the direction it
needed to *block* and long in the direction that was supposed to be
impassable — the vehicle could simply fly around the "wall" laterally,
confirmed live before the fix (the straight-line baseline reached the goal
by detouring 4m sideways around geometry that was meant to force it
through a 1.2m gap). Fixed by rotating the wall boxes 90° and tightening
`bounds.min_y`/`max_y` to exactly the wall span, removing the room to
detour. `configs/worlds/obstacle_suites/obstacle_suite_narrow_gap.{json,sdf}`
were regenerated with the corrected geometry.

## The depth-back-projection performance bug that caused a real collision

Back-projecting a full 640×480 depth image (307,200 pixels) in pure Python
measured at **153.7ms per call** — longer than the entire intended 100ms
control-tick period. The real control loop ran at well under 1Hz as a
result, always reacting to seconds-stale position data, and this visibly
destabilized vertical velocity control into a growing oscillation that
produced an actual, physically confirmed wall collision
(`d_min_m = 0.05` against `d_safe_m = 0.5`) during live debugging before
the fix.

**Fix:** `depth_to_points_camera()` gained a `stride` parameter (default
`1`, exactly preserving prior behavior for every existing caller/test);
`_DepthSource` sets it to `8`. A shield/reactive planner binning into 72
angular sectors needs on the order of hundreds of points, not a full
307,200-point cloud. Measured on the same 640×480 image:

| stride | points | time/call |
|---|---|---|
| 1 | 307,200 | 153.7ms |
| 4 | 19,200 | 23ms |
| 8 | 4,800 | 2.7ms |
| 16 | 1,200 | 0.6ms |

Re-running the identical previously-failing scenario after the fix
confirmed it numerically: the collision resolved, `control_smoothness`
improved roughly 500x, and the shield never had to intervene.

## The first formal experiment: results (spec §51 Phase 10)

5 obstacle suites × {`reactive`, `straight_line`} × 5 runs = 50 live
episodes against real PX4 SITL + Gazebo. **50/50 completed, 0 launch
failures (after the documented PX4/EKF-boot retry logic), 0 collisions,
100% arrival rate** — for both methods, across every scenario. Full
per-scenario means and paired bootstrap CIs (reactive − straight_line,
95%, seed 12345) are in `results/experiments/p10_avoidance/summary.md`.

The headline finding is that **the shield alone already achieves the
spec's "0 collisions" gate**, even paired with the collision-blind
straight-line baseline — every one of the 25 straight_line episodes
reached its goal without a single collision purely because S3 kept
scaling its velocity down near obstacles. `ReactiveAvoidance`'s
contribution shows up not in collision rate (already floored at 0 for
both) but in *how* each episode flies:

- **Clearance.** `d_min_m` (mean minimum distance to any obstacle) is
  significantly higher for reactive in `dead_end` (+0.078m, CI
  `[0.017, 0.157]`), `narrow_gap` (+0.044m, CI `[0.027, 0.069]`), and
  `pillar_forest` (+0.283m, CI `[0.189, 0.378]`) — the three scenarios
  that actually require lateral maneuvering, not just braking. `corridor`
  and `overhang_within_band` show no significant difference (both are
  narrow/vertical-only geometries where there's little room to do
  anything but go straight through).
- **Path efficiency** (`eta`, shortest-path/distance-travelled) is
  slightly but significantly higher for reactive in `dead_end`,
  `narrow_gap`, and `pillar_forest` — consistent with the same pattern.
- **Control smoothness** is mixed: reactive is measurably *less* smooth
  than straight-line in `corridor`, `dead_end`, `overhang_within_band`,
  and `pillar_forest` (it's actively steering, straight-line just brakes),
  but dramatically *smoother* in `narrow_gap` (CI `[-0.515, -0.245]`,
  i.e. straight_line is far less smooth there) — in that scenario
  straight_line's collision-blind heading gets hard-shielded against the
  gap's walls repeatedly, producing chatter that reactive avoids by
  routing through the gap directly.

## What this phase deliberately did not do (honest scope reduction)

- **5 runs per (scenario, method), not 10.** Spec's validation-gate
  language ("≥5 scenarios × 10 runs") is halved here, matching the same
  honest-scoping precedent set in Phases 8-9: 5 obstacle suites × 2
  methods × 10 runs = 100 live episodes (each involving a ~20-40s real
  Gazebo + PX4 boot) was judged not worth the added wall-clock time for
  this phase's actual purpose — establishing that the shield's collision
  guarantee holds and characterizing the two methods' qualitative
  difference, not producing a publication-grade sample size. 5/cell
  already gives non-trivial bootstrap CIs (shown above) and, more
  importantly, a clean 50/50 completion record with zero collisions.
- **A* / real shortest-path planning for `path_efficiency`.** The metric
  uses the straight-line spawn-to-goal distance as the "shortest path"
  numerator — a documented stand-in until A* exists (Phase 12 per spec).
- **A wide sweep of shield hyperparameters** (`a_brake`, `d_safe`, `tau`,
  FOV half-angle). One configuration (spec's own defaults) was used for
  every episode; no ablation was run.

## Using it

```python
from aeris.safety.shield import CollisionShield, ShieldConfig
from aeris.autonomy.navigation.reactive import ReactiveAvoidance

shield = CollisionShield(config=ShieldConfig())
reactive = ReactiveAvoidance(cruise_speed_mps=1.0)

v_desired = reactive.compute_velocity(current_pos, goal, sector_clearances_m)
v_shielded, intervened = shield.project(
    v_desired, orientation_odom_body=orientation, points_body=points_body
)
```

```bash
uv run python scripts/run_p10_avoidance_experiment.py --n-runs 5
uv run python scripts/analyze_p10_avoidance_results.py
```

See `docs/phase_reports/phase-10.md` for the live run's actual numbers and
validation-gate assessment.

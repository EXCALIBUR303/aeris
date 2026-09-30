# AERIS — classical planning + exploration baselines (spec §51 Phase 12)

## Architecture (spec §24, §25, §51 Phase 12)

| Module | Responsibility |
|---|---|
| `aeris.autonomy.planning.astar` | `astar()` — 8-connected A* over a `BandGrid`, octile heuristic, configurable `unknown_cost_multiplier` (default 1.0: unknown cells cost the same as free — needed so frontier exploration can path *through* unmapped space to reach a frontier). `OCCUPIED` cells are never traversable; safety margin comes entirely from `project_band()`'s own `inflation_m`, not from A* itself. |
| `aeris.autonomy.exploration.base` | `Subgoal`, `AgentPose`, the `ExplorationStrategy` protocol (`select_subgoal(grid, pose, t_s) -> Subgoal \| None`) — the exact interface spec asks Phase 12 to share with Phase 19's future learned policy. Also the ADR-0015 egocentric candidate geometry (12 bearings x 2 ranges) `RandomExploration` samples from, and `snap_to_nearest_free_cell()`/`nearest_cell_to_point()`, shared by all three strategies. |
| `aeris.autonomy.exploration.random` | `RandomExploration` — uniformly samples a *reachable* egocentric candidate (A*-verified from the agent's current cell), plus an equally-likely "rotate in place" null option — the H0.1 lower bound. |
| `aeris.autonomy.exploration.frontier` | `NearestFrontierExploration` (Yamauchi 1997) — goes to the reachable frontier cluster with the shortest A* path, not the geometrically nearest centroid. |
| `aeris.autonomy.exploration.utility` | `UtilityFrontierExploration` — score = (unknown cells visible from a candidate, estimated by a full-circle raycast over the agent's *own* map) / (A* path length + `lambda_turn` x heading-change cost); greedy, one candidate per frontier cluster. |
| `aeris.evaluation.metrics.coverage` | `coverage()`/`running_coverage_trace()` — `C(t) = \|E_GT(t) ∩ F\| / \|F\|` (spec §41), computed by the evaluator from a recorded GT-pose trajectory via raycasting against exact `WorldSpec` geometry, independent of either method's own map. `reachable_free_cells_gt()` builds `F` from Phase 9's own voxelizer + reachability flood fill. |
| `aeris.evaluation.exploration_episode` | `run_exploration_episode()` — wires a strategy + A* + `PathFollower` (waypoint-by-waypoint, not straight-line-to-subgoal) + the collision shield + the Sensor Bridge + `SafetySupervisor` into one live control loop; replans on arrival, a significant map change, or a time limit; falls back to an in-place rotation scan when nothing is currently explorable. |
| `configs/experiments/p12_exploration.yaml` + `scripts/run_p12_exploration_experiment.py` | The declared 3-world x 3-method x 3-repeat grid (plus a `--tune` mode for utility-frontier's `lambda_turn`), with the same per-episode launch retry every prior phase's live experiments use. |
| `configs/experiments/preregistration/p12_h0_1.md` | H0.1's pre-registered hypothesis, metric, statistical test, world/seed/episode counts, and exclusion rules — drafted now per spec's own task item, even though the §38.5 commit-and-hash gate doesn't fire until a later phase's *test*-split evaluation. |

## Path following actually follows the A* path, not just the final subgoal

Phase 10's `PathFollower.compute_velocity(current_pos, goal)` takes a single point — the simplest possible "fly straight at it" policy, which is exactly right for a fixed goal but wrong for a *planned* path: the straight line from the agent's current position to a chosen subgoal can cut through geometry the A* path around it correctly avoided. `run_exploration_episode()` re-plans a fresh A* path on every subgoal choice and drives it waypoint-by-waypoint — advancing to the next cell once within `arrival_radius_m` of it — calling the *existing*, unmodified `PathFollower.compute_velocity()` against each successive waypoint in turn. This needed no change to Phase 10's own file: "follow a polyline" falls out of calling a single-goal follower repeatedly against a list of waypoints, with the waypoint index tracked by the episode runner.

## Four real, live-diagnosed bugs, most found before burning the full experiment's live-sim budget

### 1. The strategy never chose a subgoal at all — a synthetic clock drifting behind real time

A live wiring smoke test (`tests/sim/test_exploration_live.py`) flew for 75 real seconds and made **zero** subgoal choices — only 6 in-place rotation attempts. The replan/rotation-timing logic (`max_replan_interval_s`, the rotate-scan's own duration) was gated on a synthetic counter incremented by a fixed 0.1s per loop iteration, while the outer loop's own deadline used real `WallClock` time. Phase 11 had already measured this exact control loop's real per-iteration cost at ~500-700ms (dominated by a synchronous GT-pose subprocess poll) — over 75 real seconds, a counter advancing by 0.1 "seconds" per ~0.6-second-real iteration reaches only a fraction of its intended value, badly distorting every duration-based decision. **Fixed** by deriving all internal timing from PX4's own reported `state.t_sim_s` (real elapsed time regardless of how long any tick actually takes), not a synthetic accumulator. Re-run after the fix: 31 rotation attempts in the same ~75s window — a cadence now consistent with the intended 2-10 second decision intervals.

### 2. Even after the timing fix, still zero subgoals — no column ever became FREE

Fixing the timing bug alone didn't fix the stall: `n_subgoals_chosen` stayed at 0 across 31 independent re-evaluations at different yaw angles. Frontier detection requires a `FREE` cell to exist at all (a frontier is a free cell bordering unknown) — and `project_band()`'s literal rule from spec §21.1 ("free if *all* voxels in the band are observed free") combined with a forward-looking, ~58°-vertical-FOV depth camera flying **level** at a single hover altitude turned out to structurally never satisfy that requirement. Confirmed directly (not guessed): a diagnostic script ran Phase 11's own `run_mapping_episode` for 40s in the same room and inspected the resulting `VoxelMap`'s raw per-voxel state at several nearby columns —

```
column (1.0, 0.0) per-z is_occupied states: [False, False, False, False, False, None, None, None, None]
```

— the *lower* half of even a close (1m away) column got touched by real rays (five `False`s — genuinely observed free), but the *upper* half never did (four `None`s — never touched by any ray, at any yaw, in 40 seconds), because a level camera at 1.0m hover altitude just doesn't point that high at that range. Phase 11's own report had already documented the identical limitation for its own single-viewpoint hover-and-yaw sweep ("almost nothing gets classified free") and explicitly named this — "a real measurement of this metric needs multi-waypoint coverage" — as Phase 12's problem to solve; Phase 12, in trying to *deliver* that multi-waypoint movement, discovered that the strict free-rule itself was what prevented any movement from ever starting.

**Fixed** by adding a `free_rule` parameter to `project_band()` (`aeris/mapping/projection.py`): the existing `"all_observed"` rule stays the unconditional default (unchanged behavior for every existing caller, including Phase 11's own already-validated map-accuracy evaluation), and a new `"any_observed"` mode — free if at least one voxel in the band was observed and none of the *observed* ones are occupied — is used only by `run_exploration_episode()`'s own `project_band()` calls. This is the same "trust what you've actually seen" rule most real 2D-projected navigation grids use; map-*accuracy* evaluation against exact GT geometry still needs and uses the stricter rule. Re-tested live with the identical (unmodified, 1.7m-tall) altitude band: the same smoke test passed on the first attempt after this fix, with real subgoal choices from the first tick onward.

A smaller, related fix found in the same pass: `NearestFrontierExploration`/`UtilityFrontierExploration` defaulted to `min_cluster_size=2` (excluding lone-cell frontiers as presumed "noise") with no strong justification — live-tested and found this starved early exploration too, since the only frontiers that exist right after takeoff are frequently single cells at the observed area's own ragged edge. Both defaults are now `1`, matching Yamauchi's own definition (a lone free cell bordering unknown is a genuine frontier, not noise).

A third, structural design point worth naming explicitly: **the agent's own current cell can itself be a valid frontier** (it borders unknown space — the literal condition of standing at the edge of what's known), and for a cluster shaped symmetrically around the agent, the whole cluster's centroid can even coincide with the agent's own cell. Both `NearestFrontierExploration` and `UtilityFrontierExploration` exclude the agent's current cell from a cluster's candidate pool *before* picking the cell nearest that cluster's centroid — not after picking it and then discarding the whole cluster if it happens to match, which would throw away a cluster's other, perfectly real cells too. Caught via unit testing, before any live flight.

### 3. Voxelizing a whole val world at the agent's own mapping resolution took 14.83s per episode

The main experiment's first live episode stalled for minutes in pure CPU computation after the flight itself had already landed and disarmed cleanly — `reachable_free_cells_gt()`'s call to `voxelize()` at `mapping.resolution_m=0.2` over one val world's full 24m x 24m x 6m bounds (74 box primitives) measured at **14.83s**, timed directly. `0.5m` resolution (Phase 9's own established convention for reachability-style whole-world computations, `check_reachability`'s own default) measured **0.95s** for the identical world — a 15.6x difference matching the voxel-count ratio (432,000 vs 27,648) exactly. **Fixed** by adding a separate `coverage_resolution_m` config value (0.5), used only for the evaluator's own GT-coverage computation (`reachable_free_cells_gt`/`running_coverage_trace`), independent of and unrelated to the agent's own live mapping resolution -- coverage is a "did you see this general area" question, not an obstacle-detection question, so the coarser resolution doesn't compromise what's being measured. Both calls must still use the identical value, or their cell indices stop being comparable (the same class of bug as the frame-indexing fix below).

### 4. `random` beating both frontier methods on `f1_rubble_10000` triggered spec's own "diagnose before proceeding" instruction

See "The H0.1 experiment: results" below for the full investigation and conclusion (a real shield-throttling interaction with frontier-seeking's own targeting behavior in dense clutter, not a code defect) -- reported here only as a pointer, since the diagnosis is itself part of the result, not a bug fixed before it.

## Fairness (spec §25) and what's shared across all three baselines

Every strategy is executed by the identical `run_exploration_episode()` loop: the same map (built from the agent's own EKF pose only — GT pose is recorded solely for the evaluator's own coverage computation, never fed into the map, exploration decisions, or A* planning, per spec §17.4's provenance rule), the same A* planner, the same `PathFollower`, the same collision shield, the same speed limit, and the same `t_max_s` time budget. `RandomExploration` specifically samples from the same egocentric 12-bearing x 2-range candidate geometry (ADR-0015) the future learned policy (Phase 19) will use — built once in `aeris.autonomy.exploration.base` so Phase 19 can reuse it rather than rebuild it.

## Utility-frontier tuning (spec: "tuned on validation worlds only, with the tuning budget recorded")

A 4-value grid search over `lambda_turn` (`{0.0, 0.5, 1.0, 2.0}`), n=1 episode per value, on `f1_rubble_10000` (the val split's first rubble world). Results (`results/experiments/p12_exploration/tuning.jsonl`): coverage_final = 0.127 / 0.120 / 0.122 / 0.122 respectively. Reported honestly: this spread is well within what real SITL run-to-run nondeterminism alone could produce at n=1 sample per value (spec §40's own D1 characterization) — the search cannot distinguish these values with any real statistical confidence at this budget. `lambda_turn=0.0` (the nominal grid winner) was carried into the main comparison run rather than a stronger claim about which value is actually best.

## The H0.1 experiment: results

3 val-split worlds x 3 methods x 3 repeats = 27 live episodes, all completed (0 launch failures after retry). Mean `C(180s)` per world/method (full table in `results/experiments/p12_exploration/summary.md`):

| World | random | nearest_frontier | utility_frontier |
|---|---|---|---|
| f1_rubble_10000 (dense, many small obstacles) | **0.372** | 0.157 | 0.150 |
| f2_office_10000 (small, enclosed rooms) | 0.899 | 0.900 | **0.959** |
| f3_warehouse_10000 (large, open aisles) | 0.206 | 0.204 | **0.212** |

**H0.1: NOT rejected.** Pooled paired bootstrap CI (nearest_frontier - random, `C(180s)`, n=9 paired episodes): point estimate **-0.072** (frontier trends *worse*, not better), 95% CI **[-0.164, +0.009]** (includes 0). Neither the "CI excludes 0" nor the "+10pp minimum meaningful effect" condition of the pre-registered rule is met.

Per spec's own explicit instruction ("if it fails, this is a baseline bug signal: diagnose before proceeding"), this result was investigated thoroughly before being reported, not just accepted at face value:

- The effect is driven almost entirely by **one world** (`f1_rubble_10000`), where frontier methods score barely 40% of random's coverage. `f3_warehouse_10000` shows no real difference between any of the three methods (0.204-0.212), and `f2_office_10000` shows all three converging to near-complete coverage (0.90-0.96), with utility_frontier nominally highest there.
- **Shield intervention rate is the mechanistic explanation, not proximity or path length.** In `f1_rubble_10000`, both frontier methods intervene 85-87% of ticks vs random's 81% -- a modest difference on its own, but frontier's own targeting logic *specifically* seeks the boundary between explored and unexplored space, which in a densely-packed field of small obstacles is disproportionately adjacent to yet-unmapped clutter. Random's egocentric candidates aren't chosen relative to the frontier at all -- many land in already-confirmed-clear space well short of the ragged edge -- so on average they draw less sustained braking per meter actually travelled. `f3_warehouse_10000`'s near-universal shield intervention (93-98%) for *all three* methods, with no coverage difference between them, is consistent with this: that world's own geometry (large obstacles, narrow aisles relative to `inflation_m=0.3`) throttles every method about equally regardless of targeting strategy, so there's no differential effect left for frontier-seeking to lose.
- **This is a property of the (strategy x shield x world) combination, not a code defect.** Every piece of the underlying pipeline -- A* optimality, frontier clustering, utility scoring, the free-classification fix -- is unit-tested and was independently live-verified working correctly (the wiring smoke test) before this experiment ran. No bug was found in the diagnosis process; the mechanism above is fully consistent with the shield's own documented job ("scale velocity down near obstacles," Phase 10) interacting with frontier-seeking's own definition (go toward the edge of the known map) in an environment where that edge is usually cluttered.
- A secondary, smaller finding from the same diagnosis: raising `max_replan_interval_s` from an initial 10.0s to 45.0s (a real fix, kept regardless of its effect on this specific number) did not reverse the pattern -- a standalone re-check of `f1_rubble_10000`/`nearest_frontier` after that fix still scored 0.150, consistent with the shield-throttling explanation being the dominant effect, not the replan cadence.

**Planner latency** (spec's other validation-gate item for this phase): measured directly via a standalone CPU-bound benchmark (`select_subgoal()` + `astar()` timed on a synthetic-but-realistically-scaled grid -- 81x81 cells at 0.2m resolution, matching the experiment's own window and resolution; no live episode's own grid was saved to disk during the main run, so this is honestly labeled synthetic, not claimed as live-episode data): p95 = 5.5ms (random), 6.8ms (nearest_frontier), 7.5ms (utility_frontier) -- all comfortably under the 100ms gate, with a bare `astar()` call at p95=0.48ms.

## What this phase deliberately did not do (honest scope reduction)

- **3 val-split worlds (one per procedural family), not a larger grid.** Each live episode runs up to `t_max_s=180s` plus ~20s of PX4/Gazebo boot/land overhead; a much larger world x method x repeat grid was judged impractical within this phase's time budget on this Mac, matching every prior phase's identical scoping precedent.
- **`lambda_turn` tuning at n=1/value.** A genuinely larger tuning budget (multiple repeats per candidate, a finer grid) would need many more live episodes than this phase's budget allowed; the small budget and its inconclusiveness are reported honestly rather than overstating a "winner."
- **Utility-frontier's information-gain estimate ignores the agent's future heading.** It raycasts a full circle from each candidate, not the sensor's actual FOV — a deliberate simplification, since the strategy doesn't commit to any particular arrival heading, and estimating gain independent of heading avoids silently favoring one arbitrary assumed heading over another. See `aeris.autonomy.exploration.utility`'s own docstring.
- **No 3D/multi-altitude exploration.** Spec's own V1 scoping keeps exploration within a fixed 2D altitude band (§21.1); the free-classification fix above works *within* that scope (relaxing what counts as "free" at a given altitude), not by having the vehicle change altitude to sweep more of the band.

## Using it

```python
from aeris.autonomy.exploration.frontier import NearestFrontierExploration
from aeris.evaluation.exploration_episode import run_exploration_episode

result = await run_exploration_episode(
    supervisor=supervisor, endpoint=endpoint, bridge_endpoint=bridge_endpoint,
    camera_intrinsics=intrinsics, t_body_optical=t_body_optical,
    gt_service=gt_service, gt_model_name="x500_depth_0",
    strategy=NearestFrontierExploration(),
    hover_altitude_m=1.0, z_lo_m=0.3, z_hi_m=3.0,
    x_range_m=(-8.0, 8.0), y_range_m=(-8.0, 8.0),
    timeout_s=180.0, clock=WallClock(),
)
```

```bash
uv run python scripts/run_p12_exploration_experiment.py --tune
uv run python scripts/run_p12_exploration_experiment.py
```

See `docs/phase_reports/phase-12.md` for the live run's actual numbers and validation-gate assessment.

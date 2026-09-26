# AERIS — WorldSpec, procedural generators, and the world pipeline

## Architecture (spec §9.3, §33, ADR-007, ADR-010)

| Module | Responsibility |
|---|---|
| `aeris.simulation.worlds.spec` | `WorldSpec` — the single source of world truth. `Box`/`Cylinder` primitives (walls and slabs are just oriented boxes), `Target`/`SpawnPose` slots, `WorldFamily`/`WorldSplit` enums. `F4 (collapsed)` is hard-paired to `split=test_ood` by a model validator — never generated with any other split. `content_hash()` for determinism. |
| `aeris.simulation.worlds.occupancy` | `point_in_box`/`point_in_cylinder`, `voxelize()` (a sparse GT occupancy grid), `OccupancyGrid.reachable_free_cells()` (6-connected flood fill), `check_reachability()`. |
| `aeris.simulation.worlds.sdf` | `render()` — a `WorldSpec` into a Gazebo `.sdf` world, boilerplate-matched to PX4's own `default.sdf` so a rendered world is a drop-in replacement for `"default"` in a `SimulationProfile`. |
| `aeris.simulation.worlds.splits` | `SEED_RANGES` (spec §33.2's exact ranges), `require_training_seed()`/`require_tuning_seed()` (the split-guard entry points), `write_splits_lock()`/`verify_against_lock()`. |
| `aeris.simulation.worlds.generators.{rubble,office,warehouse,collapsed}` | F1-F4 (spec §33.1). Each is a pure function of `(seed, split)` → `WorldSpec`, using only a seeded `random.Random` (no global random state), so identical inputs always produce byte-identical output. |
| `aeris.simulation.worlds.obstacle_suites` | Five hand-designed scenarios for Phase 10 (`corridor`, `pillar_forest`, `dead_end`, `narrow_gap`, `overhang_within_band`) — committed under `configs/worlds/obstacle_suites/`. |
| `aeris.simulation.worlds.batch` | `generate_batch()` — the `aeris worlds generate` CLI's implementation; writes JSON + SDF under `results/worlds/` (gitignored — generated run data, not source). |
| `aeris.evaluation.metrics.collision` | Extended this phase: `BoxObstacle`/`CylinderObstacle` (analytic clearance, not voxel-quantized) alongside the existing `SphereObstacle`; `from_world_spec()` converts a real `WorldSpec` into the same `GroundTruthGeometry` interface Phase 7's collision/clearance metrics already use. |

## The four families (spec §33.1)

- **F1 Rubble** — an open 24×24m field, scattered debris boxes at a seed-derived density (3-15% footprint coverage), rejection-sampled to avoid overlap.
- **F2 Office** — a graph-based floorplan: rooms form a grid graph, and a **randomized Kruskal spanning tree** over that graph decides which room-adjacency walls get a door — every room is reachable from every other *by construction* (a spanning tree is connected by definition), not by chance. A configurable fraction of non-tree adjacent walls also get doors for realism.
- **F3 Warehouse** — parallel shelf rows (tall thin boxes) with aisle spacing, an entrance aisle, and scattered pallets that reject-sample around the shelves.
- **F4 Collapsed** — built on the same floorplan generator as F2, with narrower doors and a few tilted slabs (boxes with random roll/pitch, spec: "within the altitude band") dropped into the layout; retries (up to 20 attempts) if a given seed produces an unreachable layout. **Always `split=test_ood`** — never used for training, tuning, or model selection (spec: "F4 is held out").

## A real performance bug caught while building this: `voxelize()` was recomputing each box's rotation inverse per voxel

The first `voxelize()` implementation called the general-purpose `point_in_box()` helper for every voxel × every box pair, which recomputes that box's `Transform.inverse()` (a quaternion composition) from scratch on every single call. For a 24×24×6m rubble field at 0.5m resolution (~27,600 cells × ~36 boxes), this took over 11 seconds. Precomputing each box's inverse transform once, plus a cheap bounding-sphere pre-filter before the exact rotation-aware check, brought the same computation down to ~0.45s — a >20x speedup, needed to make a 50-worlds-per-family batch tractable at all.

## Reachability: a real flood fill, not a heuristic

`check_reachability()` voxelizes the world at a coarse resolution and runs a genuine 6-connected BFS from each spawn pose, confined to the world's bounds and altitude band. It's used both as a post-generation guard (F4's generator retries on failure) and as an external test oracle in the unit tests, including a deliberately-sealed-in negative case (four walls fully enclosing the spawn point) confirming the check actually catches an unreachable world, not just always returning `True`.

## What this phase deliberately did not do (honest scope reduction)

- **50 worlds per family loaded live.** The live test loads 8 worlds per family (32 total, plus F4), not 50×4=200 — at ~5-6s of real Gazebo startup per load, 200 loads is roughly 20 minutes of pure load time. 8/family gives real, repeated confidence in the SDF renderer without that cost; see `docs/phase_reports/phase-9.md` for the actual pass/fail count.
- **3 distances × 4 yaw angles for the consistency check**, matching spec §51 Phase 8's own already-reduced-scope obstacle-localization test. Instead, Phase 9's own consistency check is broader in a different way: it checks *every* primitive in one generated world (not just one obstacle at a few positions) against its live Gazebo pose.
- **A `FastSimRenderer`.** Spec §9.3 names it as WorldSpec's second consumer (alongside `SdfRenderer`), but FastSim itself doesn't exist until Phase 13 — nothing to render into yet.
- **Family diversity statistics** (free-area, corridor-width, clutter distributions) as a committed research artifact — the generators' seed-derived randomization ranges are documented inline in each module, but no aggregate statistics report was produced this phase.

## Using it

```bash
uv run aeris worlds generate --family office --split train --n 10
```

```python
from aeris.simulation.worlds.generators import office
from aeris.simulation.worlds.spec import WorldSplit
from aeris.simulation.worlds import sdf
from aeris.evaluation.metrics.collision import from_world_spec

spec = office.generate(seed=42, split=WorldSplit.TRAIN)
world_sdf = sdf.render(spec)               # -> Gazebo world file
geometry = from_world_spec(spec)           # -> GroundTruthGeometry for collision/clearance metrics
```

See `docs/phase_reports/phase-9.md` for the live run's actual numbers.

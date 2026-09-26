# AERIS — Experiments, metrics, ground truth, and replay

## Architecture (spec §38, §39, §41)

| Module | Responsibility |
|---|---|
| `aeris.experiments.manifest` | `RunManifest` (spec §38.2's mandatory fields) — git SHA/dirty, config hash, task/method/tier, software/hardware snapshot, status. `new_manifest()`/`write_manifest()`/`read_manifest()`. |
| `aeris.experiments.registry` | `results/index.sqlite`: `scan_runs()`/`rebuild_registry()` — rebuildable from the filesystem, which is the source of truth. |
| `aeris.experiments.runner` | `run_evaluation()`: one run end to end — fresh `SimulationLauncher`, one episode, retry-once on failure (spec §38.4), writes `manifest.json`/`config.resolved.yaml`/`metrics.jsonl`/`replays/episode.mcap`. |
| `aeris.experiments.preregistration` | `require_preregistration()` — refuses test-split evaluation without a committed `configs/experiments/preregistration/<exp>.md` (spec §38.5). |
| `aeris.evaluation.metrics.nav` | Distance travelled, flight time, control smoothness, mean waypoint error, return-to-base success — computable from a trajectory alone. |
| `aeris.evaluation.metrics.collision` | Minimum clearance / collision against `GroundTruthGeometry` — a flat-ground stub as of Phase 7 (see below). |
| `aeris.evaluation.statistics` | Paired bootstrap CI + Holm correction. |
| `aeris.evaluation.episode` | `run_episode()`: one mission attempt, recorded and metriced, given an already-connected `SafetySupervisor`. |
| `aeris.simulation.groundtruth` | `GroundTruthService` — privileged, gz world-frame poses via the `gz` CLI (not a `gz.*` Python import — see below). |
| `aeris.replay` | `ReplayRecorder`/`ReplayReader` (MCAP, ADR-012) + `channels.py`'s channel names. |

## Why `GroundTruthService` shells out to `gz`, not `gz.transport`

Spec §14.3 contract 5 reserves `gz.*` Python imports for the future
Sensor Bridge process alone (Phase 8). Spec's own Phase 7 task list
anticipated this ("for P7 this can use a tiny gz-Python helper") but a
subprocess call to `gz topic -e` — the same tool
`aeris.simulation.launcher.readiness` already uses for world-readiness
checks — achieves the same thing without needing a `gz.*` import in the
main venv at all, or waiting for the Sensor Bridge to exist. The text
proto it returns is brace-depth-aware parsed (nested `position {}`/
`orientation {}` blocks inside each top-level `pose {}` block use the
same `{`/`}` tokens, so a naive regex would misparse them).

## Why collision/clearance use a flat-ground stub

`GroundTruthGeometry` (in `aeris.evaluation.metrics.collision`) is a
ground plane plus a list of spherical obstacles — spec's own Phase 7 task
list explicitly allows "a `WorldSpec` stub or stock-world geometry until
P9". `default_world_geometry()` represents exactly the world every AERIS
profile has used through Phase 7 (PX4's own `default.sdf`): no obstacles.
Collision/clearance against it are honestly "never collides, unbounded
clearance" for *that* world, not a faked value — a real `WorldSpec`
(Phase 9) plugs into the identical interface.

## Why `run_episode` samples telemetry concurrently rather than instrumenting `MissionExecutive`

A background `asyncio` task polls `SafetySupervisor.get_vehicle_state()`
(and, best-effort, `GroundTruthService.get_pose()`) at a fixed period
while `MissionExecutive.run()` proceeds independently, rather than adding
recording hooks into the executive itself. This is what makes "re-reading
the MCAP reproduces the metrics exactly" (spec's own validation-gate
item) true *by construction*: the metrics are computed from precisely the
samples that got written to the MCAP, not from separate internal
bookkeeping that might drift from what was recorded.

## Using it

```python
from aeris.experiments.runner import run_evaluation
from aeris.autonomy.mission.spec import load_mission_spec
from aeris.safety.envelope import load_envelope
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import resolve_px4_layout
from scripts.fly_box import default_params

manifest, result = await run_evaluation(
    layout=resolve_px4_layout(),
    profile=SimulationProfile(name="square_run", model="gz_x500"),
    envelope=load_envelope("configs/vehicle/safety.yaml"),
    spec=load_mission_spec("configs/missions/square.yaml"),
    params=default_params(),
    name="square_smoke",
)
print(manifest.status, result.metrics)
```

Every run lands under `results/runs/<YYYYMMDD-HHMMSS>_<name>_<cfghash8>/`
(spec §38.1) with `manifest.json`, `config.resolved.yaml`,
`metrics.jsonl`, and `replays/episode.mcap`.

## What Phase 7 deliberately did not build

- **A matrix-expansion layer.** Spec's "declared matrix (methods × seeds
  × world splits × conditions)" has no real axis to expand yet — one
  method (`scripted`), no world splits (Phase 33), no randomization a
  seed would meaningfully vary. `run_evaluation()` (one run in, one
  manifest+MCAP+metrics out) is the tested, live-verified building block;
  a matrix layer on top of it is worth building once Phase 9+ gives it
  real axes.
- **Localization, coverage, SPL, target-recall, and other spec §41
  metrics** that need perception, mapping, or `WorldSpec` infrastructure
  that doesn't exist yet — spec's own Phase 7 task list scopes the metric
  library to exactly what's implemented here (waypoint error, distance,
  flight time, smoothness, collision, clearance).

## Validated results (Phase 7)

See `docs/phase_reports/phase-7.md` for the live run's actual numbers and
`docs/reproducibility.md` for the D1 nondeterminism report — this file
describes the mechanism; those have the data.

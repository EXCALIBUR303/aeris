# AERIS — Architecture summary

This is a short pointer document. The authoritative architecture is
**[`AERIS_TECHNICAL_SPEC.md`](../AERIS_TECHNICAL_SPEC.md)** — read that
first; this file exists so `docs/` has an entry point that doesn't require
opening the full spec, and so the ADRs have somewhere to be indexed from.

## The one-paragraph version

PX4 Autopilot handles flight control and state estimation; Gazebo Harmonic
handles physics and sensors. Everything AERIS adds sits above that: a
deterministic Safety layer is the *only* path to the vehicle; an Autonomy
Engine (perception → mapping → planning → exploration → mission executive)
runs on top of it; a two-tier simulation setup (a fast FastSim for RL
training, the real PX4+Gazebo stack for all reported evaluation) keeps
training feasible without ever reporting a result that wasn't validated in
the high-fidelity simulator; and an Experiment/Replay layer makes every run
reproducible and inspectable. See spec §13 for the full layered diagram.

## Layers (spec §13.2)

```
Control Center (React) → AERIS Server (FastAPI) → Autonomy Engine
  → Safety Layer (the only path to the vehicle) → Vehicle Interface
  → PX4 SITL → Gazebo Harmonic
```

A privileged `GroundTruthService` sits beside this stack, readable only by
the evaluator, world generator, and offline label tooling — never by
autonomy or learning code (spec §17.4). This is enforced by import-linter
contracts (see `pyproject.toml` → `[tool.importlinter]`) and, at the value
level, by the `Provenance` tagging in `aeris.core.types`.

## Architecture Decision Records

Each ADR below is one deliberate, load-bearing decision from the spec, kept
short and linked to its spec section. Read the ADR for the "why"; read the
spec section for the full detail and how it interacts with everything
else.

| ADR | Decision |
|---|---|
| [0001](adr/0001-mac-native-first.md) | Mac-native first; isolate Linux-only components |
| [0002](adr/0002-px4-external-pinned-dependency.md) | PX4 as a pinned external dependency; no firmware modifications |
| [0003](adr/0003-vehicle-transport-mavsdk-primary.md) | MAVSDK-Python primary transport, pymavlink secondary (**at risk**, see Phase 1) |
| [0004](adr/0004-sensor-bridge-process.md) | Separate Sensor Bridge process for Gazebo Python bindings |
| [0005](adr/0005-no-ros2-in-v1.md) | No ROS 2 dependency in V1; adapter seam reserved |
| [0006](adr/0006-two-tier-simulation.md) | Two-tier simulation: FastSim for learning, PX4+Gazebo for evaluation |
| [0007](adr/0007-worldspec-single-source-of-truth.md) | `WorldSpec` as the single source of world truth for both tiers |
| [0008](adr/0008-enu-flu-internal-frames.md) | ENU/FLU internal frames; NED/FRD only inside the PX4 adapter |
| [0009](adr/0009-voxel-log-odds-mapping.md) | 3D voxel log-odds + 2D altitude-band projection for V1 mapping |
| [0010](adr/0010-seed-range-splits.md) | Seed-range splits + held-out world family + split guards |
| [0011](adr/0011-no-rgb-fastsim-v1.md) | No RGB in FastSim V1; RQ3 gated on a feasible render path |
| [0012](adr/0012-mcap-replay-format.md) | MCAP replay format with detail levels; replay = playback |
| [0013](adr/0013-frontend-stack.md) | React/TS/Vite + tokens + Radix + R3F + uPlot + Observable Plot; no Bootstrap |
| [0014](adr/0014-in-house-ppo.md) | In-house PPO (CleanRL-style), with an SB3 dev-only sanity reference |
| [0015](adr/0015-hierarchical-exploration-action-space.md) | Hierarchical learned exploration via masked subgoal actions |

## `aeris.core` (Phase 2)

The only package that exists so far. Everything else depends on it;
nothing in it depends on any other `aeris.*` package. See each module's
docstring for what it does and why:

- `errors.py` — the typed exception hierarchy (spec §14.2: no bare
  `except Exception: pass`, no reused generic exceptions).
- `types.py` — shared enums, principally `Provenance` (spec §17.4).
- `units.py` — the SI-units-internally convention's helper functions.
- `clock.py` — the `Clock` protocol (spec §13.2: autonomy code takes a
  clock, never calls `time.time()`).
- `config.py` — YAML `extends:`-chain composition, CLI overrides, and
  SHA-256 config hashing (spec §38.1).
- `logging.py` — structured (JSON) logging carrying `run_id`/`t_sim_s`
  (spec §14.2).
- `constants.py` — named constants with a cited source (spec §14.2: no
  magic numbers).

## Enforced dependency contracts

Declared in `pyproject.toml` → `[tool.importlinter]`. Only two are active
today (both scoped to `aeris.core`, since that's all that exists); the
comment block in that file lists every contract still to be added, and
which phase adds it, matching spec §14.3 exactly. `tests/unit/core/
test_import_contracts.py` proves both that the real contracts pass and
that a deliberately broken fixture package is actually caught.

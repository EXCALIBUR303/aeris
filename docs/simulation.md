# AERIS — Simulation

## Tiers (spec §17)

AERIS uses two simulation tiers (ADR-0006). Only **Tier H** exists as of
Phase 3:

- **Tier H** — PX4 SITL + Gazebo Harmonic. Full physics, real (simulated)
  sensors, PX4's actual estimator and controllers. Used for classical
  baselines and **all headline evaluation** — a learned-autonomy result is
  only reported as an AERIS result if it was evaluated here.
- **Tier F** — AERIS FastSim, a vectorized, geometry-based simulator for
  RL training. Built in Phase 13; not implemented yet.

## `aeris.simulation.launcher` (Phase 3)

Programmatic, reproducible Tier-H lifecycle management — see
[`docs/px4.md`](px4.md) for exactly how it launches Gazebo and PX4, and
why. Summary of what exists:

| Module | Responsibility |
|---|---|
| `px4_paths.py` | Resolves the external PX4 checkout and its build layout (spec §8.3). |
| `launcher/profiles.py` | Named, validated launch configurations (`configs/simulation/*.yaml`). |
| `launcher/ports.py` | Per-instance MAVLink port math, transcribed from PX4's own init script. |
| `launcher/process.py` | Tracked-PID subprocess lifecycle (start, log capture, SIGTERM→SIGKILL). |
| `launcher/readiness.py` | Blocks until Gazebo's world and PX4's heartbeat/EKF are actually up. |
| `launcher/params.py` | AERIS's simple PX4 parameter-profile format + MAVLink application. |
| `launcher/launcher.py` | `SimulationLauncher`: orchestrates all of the above; `start()`/`stop()`/`reset()`. |
| `launcher/state.py` | On-disk PID record so the CLI's separate `up`/`down`/`status` invocations can find a running instance. |

## Using it

```bash
uv run aeris sim up --profile headless_x500
# ... Ctrl-C to stop, or from another terminal:
uv run aeris sim down
uv run aeris sim status
```

Or programmatically:

```python
from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.profiles import load_profile
from aeris.simulation.px4_paths import resolve_px4_layout

layout = resolve_px4_layout()
profile = load_profile("configs/simulation/headless_x500.yaml")
launcher = SimulationLauncher(layout)
result = launcher.start(profile)
print(result)          # pids, ports, how long each readiness stage took
launcher.stop()
```

## Profiles

`configs/simulation/`: `_base.yaml` (shared defaults; never loaded
directly), `headless_x500.yaml` (the default smoke-test profile),
`headless_x500_depth.yaml`, `headless_x500_lidar_2d.yaml` (both confirmed
producing real headless sensor data in Phase 1), and `gui_x500.yaml`
(interactive debugging only — needs XQuartz, not installed as of Phase 1).

## Determinism (spec §40)

Tier H is **statistically**, not bitwise, reproducible — Gazebo physics
timing, thread scheduling, and process startup all introduce variation.
Phase 3's `tests/sim/test_launcher.py::TestStartupPoseNondeterminism`
measures spawn-pose variance across 5 identical fresh restarts as an
initial, narrowly-scoped data point (SITL boot determinism, not flight
determinism — that needs Phase 5's flight control and is that phase's
responsibility to measure per spec §40's fuller D1 report).

## Validated results (Phase 3)

See `docs/phase_reports/phase-3.md` for the actual 10-cycle reliability
run, reset-cost measurement, and RTF table — this file describes the
mechanism; the phase report has the numbers.

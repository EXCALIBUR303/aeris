# AERIS — Reproducibility (spec §40)

Spec §51 Phase 7 validation-gate item 4: "The Tier H nondeterminism
measurement (P3) is formalized into a D1 report." This is that report —
it also folds in flight-level variance data Phases 5/6 collected after
Phase 3, since that's a second, complementary measurement of the same
determinism class.

## Determinism classes (spec §40)

| Class | Scope | Status |
|---|---|---|
| **D0** (bitwise) | FastSim + PPO on CPU, `torch.use_deterministic_algorithms(True)` | Not applicable yet — FastSim is Phase 13. |
| **D1** (statistical) | Tier H: Gazebo physics + PX4 lockstep. Deterministic in principle; thread scheduling, UDP timing, EKF initialization and process startup introduce real variation. | **Measured below.** |
| **D2** | MPS training | Not applicable yet — learned training is Phase 14+. |

AERIS never claims bitwise reproducibility for Tier H. What follows is
what's actually been measured, at two different stages of the pipeline,
across three phases' worth of live SITL runs — not a single number
asserted once.

## Boot/spawn-pose variance (Phase 3)

5 identical fresh `SimulationLauncher` restarts, same profile
(`gz_x500`, headless), spawn pose read via `LOCAL_POSITION_NED`
immediately after each restart:

| Axis | Spread (max − min across 5 restarts) |
|---|---|
| x | 2.04 cm |
| y | 0.75 cm |
| z | 5.75 cm |

This measures SITL *boot* variance only — before any command is ever
issued to the vehicle. Full data: `docs/phase_reports/phase-3.md`.

## Flight-level variance (Phases 5 and 6)

Once real programmatic flight existed (Phase 5), waypoint arrival error
across many independent, fresh-SITL-per-run cycles gives a second,
end-to-end measurement of the same D1 class — now including EKF
convergence, controller settling, and the full arm→climb→navigate→land
cycle, not just boot:

| Source | Runs | Waypoint-error range | Notes |
|---|---|---|---|
| Phase 5 `fly_box.py` (single-sample arrival check) | 20 | 0.43–0.50 m | `tests/sim/test_safety_flight.py::test_twenty_consecutive_box_flights` |
| Phase 6 `MissionExecutive` (radius+dwell arrival hysteresis) | 15 (5×3 templates) | 0.15–0.19 m | `tests/sim/test_missions.py` |

The tighter, more consistent Phase 6 numbers aren't a determinism
improvement in the underlying simulator — they're the dwell-based arrival
hysteresis (spec's own "hysteresis on arrival" mitigation) filtering out
single-sample noise that Phase 5's simpler check didn't. Both ranges are
real measurements from independent live runs, not a single cherry-picked
number: 35 total independent Tier-H flights across Phases 5–7 all landed
within a consistent few-tens-of-centimeters band, with zero PX4 failsafes
and zero AERIS-side crashes.

**Conclusion:** on this machine, Tier H is statistically reproducible in
the sense spec §40 means — repeated identical episodes produce metrics
within a bounded, measured variance, not exact repetition. This machine's
boot-level variance (≤ ~6 cm) is small relative to the flight-level
arrival-error band (15–50 cm), meaning most of the observed spread comes
from flight dynamics and controller behavior, not from SITL startup
nondeterminism.

## Environment pinning

- `uv.lock` — exact Python dependency versions.
- `configs/versions.lock.yaml` — the pinned PX4 tag/SHA (spec §8.3, ADR-0002) and other frozen external versions.
- `pyproject.toml`'s `[project]` version + `git_sha`/`git_dirty` (captured in every run's `manifest.json`, spec §38.2) together identify exactly what produced a given run.

No `scripts/env_report.py` exists as a separate script as of Phase 7 —
`aeris.experiments.manifest.software_versions()`/`hardware_snapshot()`
capture the equivalent information directly into every manifest instead,
which is where spec's own manifest field list (§38.2) actually needs it.

## Regeneration

No committed figures exist yet (nothing in AERIS produces plots as of
Phase 7 — that's Phase 28+'s job). The policy this repo will hold to once
figures exist: every figure is produced by a script from run data under
`results/`, and its caption cites the run/experiment ID(s) it came from
— matching spec §40's regeneration rule.

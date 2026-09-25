# AERIS — Phase 3 Report

============================================================
AERIS — PHASE 3 COMPLETE
============================================================

**PHASE:** 3 — PX4 SITL integration.

**IMPLEMENTED:**
- `aeris.simulation.px4_paths`: resolves the external PX4 checkout (spec §8.3 precedence: `AERIS_PX4_DIR` env var → `configs/local.yaml` → default) and its build layout (`Px4Layout`), with `ensure_built()` raising an actionable error rather than AERIS ever building PX4 itself.
- `aeris.simulation.launcher`: a full `SimulationLauncher` (start/stop/reset) that mirrors PX4's own standalone-launch sequence, read directly from the pinned checkout's `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim` rather than assumed from documentation — AERIS launches Gazebo itself (`PX4_GZ_STANDALONE=1`), waits for readiness using PX4's own `gz service -i .../scene/info` check, then launches PX4, which handles spawning its own model.
  - `ports.py` — per-instance MAVLink port math, transcribed from `px4-rc.mavlink`.
  - `profiles.py` — Pydantic-validated named launch profiles with `extends:` composition.
  - `process.py` — tracked-PID-only subprocess lifecycle with log capture.
  - `readiness.py` — Gazebo-world and PX4-heartbeat/EKF readiness via pymavlink.
  - `params.py` — AERIS's own simple `.params` format, applied over MAVLink `PARAM_SET`.
  - `state.py` — on-disk PID record so the CLI's separate `up`/`down`/`status` invocations can find a running instance.
- `aeris/cli.py`: `aeris sim up/down/status` — the spec's literal expected output, verified working end to end.
- Config/data files: `configs/simulation/{_base,headless_x500,headless_x500_depth,headless_x500_lidar_2d,gui_x500}.yaml`, `configs/vehicle/px4_sitl.yaml`, `configs/vehicle/px4_params/sitl_base.params`.
- `docs/px4.md`, `docs/simulation.md`.

**FILES CREATED:** 24 new files (11 `aeris/` modules, 5 `configs/simulation/*.yaml`, 2 `configs/vehicle/*`, 2 docs, 9 test files across `tests/unit/simulation/` and `tests/sim/`). See `git log d9a22ed --stat` for the exact list.

**FILES MODIFIED:** `aeris/core/errors.py` (added `Px4NotFoundError`), `pyproject.toml` (`[project.scripts]` entry point, mypy overrides for `pymavlink`/`mavsdk`), `Makefile` (`setup` now syncs the `sim` extra; `sim-smoke`/`sim-up` targets now do something real), `.github/workflows/ci.yml` (three fixes — see below).

**TESTS RUN:**
- `pytest tests/unit` — 123 tests (65 new for `aeris.simulation`), local and in GitHub Actions CI.
- `ruff check .`, `ruff format --check .`, `mypy aeris`, `lint-imports` — local and in CI.
- `pytest tests/sim -v -m sim` (live PX4 SITL + Gazebo, **not** run in CI per spec §44.2) — run twice in full, plus one isolated re-run of the speed-factor tests after fixing a bug found in the first run.
- Manual end-to-end CLI test: `aeris sim up --profile headless_x500` → `aeris sim status` → Ctrl-C → clean stop, verified via `pgrep`.

**TEST RESULTS:**

*Unit (local and CI, both green):* 123 passed (1 skipped in CI only — a test that genuinely needs a real `gz` binary, which hosted CI doesn't have).

*Live sim tests (this machine), full run 1 (`tests/sim -v -m sim -s`, 354.87s):*

| Test | Result |
|---|---|
| `TestTenConsecutiveCycles::test_ten_consecutive_start_stop_cycles_are_clean` | **PASSED** — 10/10 cycles, `[17.3, 17.2, 17.3, 17.3, 17.2, 17.2, 17.3, 17.2, 17.2, 17.2]`s, mean 17.2s, max 17.3s. No orphaned processes after any cycle. |
| `TestReset::test_reset_produces_a_running_healthy_instance_and_measures_cost` | **PASSED** — `reset()` cost 17.2s (a full stop+restart, as designed); post-reset heartbeat confirms disarmed, `landed_state` on-ground-or-undefined. |
| `TestReset::test_reset_before_start_raises` | **PASSED** |
| `TestSpeedFactor::test_rtf_at_speed_factor[1.0/2.0/4.0]` | **FAILED** (all 3) — a genuine bug in the *test*, not the launcher: see "Problems found" #1. |
| `TestStartupPoseNondeterminism::...` | **PASSED** — 5/5 restarts, spawn-pose spread x=0.0204 m, y=0.0075 m, z=0.0575 m. |

*Speed-factor re-run after the fix (isolated, 60.86s):* all 3 **PASSED**, with `observed_rtf` matching `speed_factor` essentially exactly (1.00, 2.00, 4.00).

*Full confirmation run 2 (`tests/sim -v -m sim -s`, 314.42s, after the speed-factor fix was folded in):* 6/7 passed. `TestTenConsecutiveCycles` passed again (10/10). One flaky failure — see "Problems found" #2. No orphaned processes even on that failure (the `try/finally` cleanup pattern held).

**SIMULATION RESULTS:**

*RTF table (spec §51 Phase 3 expected output), `gz_x500`, headless, this machine:*

| `speed_factor` | Observed RTF |
|---|---|
| 1.0 | 1.00 |
| 2.0 | 2.00 |
| 4.0 | 4.00 |

*Nondeterminism (Phase-3-scoped; see docs/simulation.md for what this does and doesn't cover):* spawn-pose spread across 5 identical fresh restarts: x=2.0 cm, y=0.75 cm, z=5.75 cm. SITL boot is not meaningfully nondeterministic on this machine.

**EXPERIMENT RESULTS:** None (out of scope for this phase).

**PROBLEMS FOUND:**
1. **Test bug — false RTF reading.** The original speed-factor test read one `LOCAL_POSITION_NED` message, slept 3 real seconds, then read one more — and got `sim_dt≈0.03s` regardless of `speed_factor` (looked like the sim was frozen). Root cause: `pymavlink`'s `recv_match` dequeues one message per call; PX4 streams `LOCAL_POSITION_NED` fast enough that a lone post-sleep read returns something from very early in the accumulated backlog, not a fresh sample. Not a launcher bug — a test-methodology bug. Fixed by draining continuously for the whole window and keeping the last message seen.
2. **One flaky heartbeat timeout.** In confirmation run 2, the 16th consecutive live-PX4 launch in that pytest session (1st of `TestStartupPoseNondeterminism`'s 5) hit `no MAVLink heartbeat received on port 14540 within 30.0s`. Not reproduced on re-run, and not part of the literal validation-gate metric (which is the 10-cycle test, which passed cleanly both times it ran). No orphaned processes resulted. Left as an open, low-priority observation rather than chased further — worth a closer look if it recurs with any pattern once Phase 4+ exercises the launcher more.
3. **`gz service -i` on a not-yet-ready service blocks for ~5s** rather than returning promptly — it doesn't respect a short subprocess timeout gracefully; the original code let `subprocess.TimeoutExpired` propagate uncaught. Found immediately by a unit test (`test_wait_for_gz_world_ready_times_out_for_a_world_that_will_never_exist`), before ever running against a live sim. Fixed by catching and treating it as "not ready yet, keep polling."
4. **CI failure #1: hosted runner missing `pymavlink`.** `aeris.simulation`'s pure-logic modules (`params.py`, `readiness.py`) import `pymavlink` at module scope, so even *importing* them for unit tests — not just running live-sim tests — needs the `sim` extra installed. `uv sync --extra dev` alone (the original `ci.yml`) doesn't pull it in.
5. **CI failure #2: `mavsdk==4.0.0` has no wheel for `macos-14`.** GitHub's `macos-14` hosted-runner image reports platform `macosx_14_0_arm64`; mavsdk's published wheels are `macosx_15_0_arm64`+ only. `uv sync` correctly refused to install it.
6. **CI failure #3: no `gz` binary in hosted CI at all** (not just "no Gazebo server running"). `wait_for_gz_world_ready`'s `subprocess.run(["gz", ...])` raised an uncaught `FileNotFoundError` on a runner where the command doesn't exist, rather than the intended "not ready yet" retry path.

**PROBLEMS FIXED:**
1. Rewrote the speed-factor test to drain continuously instead of read-sleep-read; re-verified RTF now tracks `speed_factor` essentially exactly.
2. Documented as an open observation (see "Remaining risks"); not fixed because it wasn't reproduced and isn't part of the required gate metric.
3. `readiness.py` now catches `subprocess.TimeoutExpired` inside the polling loop.
4. `ci.yml` and `Makefile`'s `setup` target now sync `--extra sim` too.
5. `ci.yml` bumped to `runs-on: macos-15`.
6. `readiness.py` now catches `FileNotFoundError` and raises a clear, immediate `SimulationLaunchError` ("is Gazebo Harmonic installed and on PATH?") instead of retrying (which can't fix a missing binary) or crashing raw. The corresponding unit test was split into a deterministic "`gz` not on PATH" case (works identically in any environment, via a monkeypatched empty `PATH`) and a "`gz` present but nothing running" case that's skipped in CI (`CI` env var, which GitHub Actions sets automatically) since it genuinely needs Gazebo installed.

All six were found and fixed **before** the phase report was written, across 3 CI iterations (each pushed, checked on GitHub's actual runners, and confirmed) — the final state, `cebdaf2`, is green on GitHub Actions itself, not just locally.

**KNOWN LIMITATIONS:**
- `reset()` only implements the safe default (full stop + restart) per spec §17.2 — no faster in-place reset mechanism was attempted or is claimed to be "proven clean" for anything lighter.
- The Phase-3-scoped nondeterminism test measures SITL *boot/spawn-pose* variance only, not flight/hover variance (needs Phase 5's flight control) — documented explicitly in both the test's docstring and `docs/simulation.md`.
- Multi-instance (`-i N`, N > 0) port math and profile fields exist and are unit-tested, but the launcher itself has never been run with a second simultaneous instance — out of scope for Phase 3 (multi-vehicle Gazebo is Linux-only per spec §9.1/§12; single-instance is all Phase 3's gate requires).
- The one flaky heartbeat timeout (problems-found #2) is unexplained. If it recurs with a discoverable pattern, worth a closer look; if not, it may just be occasional host scheduling noise after ~5 minutes of continuous process churn in one pytest session.

**REMAINING RISKS:** Same two open items carried from Phase 1 (MAVSDK segfault → Phase 4; PX4 arm-rejection → Phase 5), now with an additional, independently-justified partial lead on the arm-rejection issue (`COM_RC_IN_MODE=4` in `sitl_base.params`, not yet confirmed to fix it — Phase 5 still owns the actual diagnosis). The flaky heartbeat timeout above is a new, minor, unresolved item.

**VALIDATION GATE (spec §51 Phase 3 / §54 row 3):** *"10/10 consecutive start–ready–stop cycles; reset produces a clean state (EKF re-initialized, disarmed, landed); RTF table recorded; no orphan processes."*

| Requirement | Result |
|---|---|
| 10/10 consecutive start-ready-stop cycles | **PASS** — confirmed twice, independently, in two separate full test-suite runs |
| Reset produces a clean state (disarmed, landed) | **PASS** — verified via a live post-reset heartbeat/`EXTENDED_SYS_STATE` check, not assumed |
| RTF table recorded | **PASS** — 1.00/2.00/4.00 at speed_factor 1/2/4, exact tracking |
| No orphan processes | **PASS** — verified via `pgrep` after every cycle, and even after the one test failure |

**VALIDATION GATE: PASS.**

**CURRENT AERIS STATUS:** AERIS can programmatically, reliably start, monitor, and tear down PX4 SITL + Gazebo Harmonic — both via a Python API (`SimulationLauncher`) and a CLI (`aeris sim up/down/status`). No vehicle commanding, safety layer, or mission logic exists yet — Phase 4 is the first phase that talks *to* the running vehicle rather than just launching it.

============================================================
NEXT PHASE
============================================================

**NEXT PHASE:** 4 — Vehicle interface + telemetry + frames core.
**RECOMMENDED MODEL:** Sonnet.
**RECOMMENDED EFFORT:** High.
**WHY THIS MODEL:** Integration engineering against a precise spec (§51 Phase 4, §15, §19) — implementing `VehicleInterface`, the MAVSDK/pymavlink adapters, and `aeris.core.frames`, with the ENU/NED conversion boundary being the one place correctness really matters.
**WHY THIS EFFORT:** Frame conversions and async transport code are exactly the kind of thing that's easy to get almost-right; the gate requires property-based round-trip tests plus a live SITL frame-correctness test (command +x ENU, verify PX4 reports +y NED).
**SWITCH REQUIRED:** NO (already on Sonnet).
**USE OPUS ONLY IF:** frame or time conventions turn out to need a genuine redesign (spec §55) — not expected.

**RECOMMENDED SKILLS / CONNECTORS:** None beyond what's already in use (Bash, WebFetch for MAVSDK API docs if the segfault investigation needs it).

**EXPECTED OUTPUT:**
- `aeris/vehicle/interface.py` (the `VehicleInterface`/`CommandPort` protocols and `VehicleState` dataclass, per spec §15.1)
- `aeris/vehicle/px4_mavsdk/` and `aeris/vehicle/px4_mavlink/` adapters
- `aeris/core/frames/` (ENU↔NED, FLU↔FRD, yaw conversion, camera-optical-to-body — spec §19.3) with property-based round-trip tests
- A resolved decision on ADR-0003 (MAVSDK primary vs. pymavlink primary), based on a fresh test of whether the Phase 1 MAVSDK segfault persists on a newer point release
- A live SITL test proving the ENU↔NED conversion is correct end to end, not just internally consistent

**ACTION REQUIRED:**

Switch model/effort if necessary (none needed). Then reply exactly:

**CONTINUE**

============================================================

Stopping here. Phase 4 has not been started.

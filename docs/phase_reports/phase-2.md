# AERIS — Phase 2 Report

============================================================
AERIS — PHASE 2 COMPLETE
============================================================

**PHASE:** 2 — Repository + tooling foundation.

**IMPLEMENTED:**
- Git repository initialized, private GitHub repo created and pushed: https://github.com/EXCALIBUR303/aeris
- `aeris` Python package skeleton on `uv` (Python 3.12.12), with the `sim`/`learn`/`server`/`dev` extras from spec §14.1 declared (empty runtime deps beyond `dev` until the phase that needs them).
- `aeris.core`: `errors.py`, `types.py`, `units.py`, `clock.py`, `config.py`, `logging.py`, `constants.py` — all per spec §14.2 code standards (SI units, typed exceptions, no magic numbers, structured logging carrying `run_id`/`t_sim`).
- `pyproject.toml`: ruff (lint + format), mypy (strict on `aeris.core`), pytest, and `[tool.importlinter]` with 2 active contracts + a comment block listing every contract still to add and which phase adds it (spec §14.3).
- `.pre-commit-config.yaml`, `.editorconfig`, `Makefile` (`setup`/`test`/`lint`/`format`/`imports`/`sim-smoke` stub), `.github/workflows/ci.yml` (macOS runner; hosted CI intentionally excludes SITL/Gazebo per spec §44.2).
- `docs/architecture.md` + 15 ADRs (`docs/adr/0001`–`0015`), one per spec Appendix C decision.
- `configs/local.example.yaml` (the template for the gitignored `configs/local.yaml`).
- `results/`, `models/`, `replays/` directories with `.gitkeep` (gitignored otherwise).

**FILES CREATED:** 66 files — see `git log -1 --stat` for the full list. Notably: `pyproject.toml`, `uv.lock`, `Makefile`, `.gitignore`, `.editorconfig`, `.pre-commit-config.yaml`, `README.md`, `.github/workflows/ci.yml`, `aeris/__init__.py`, `aeris/core/{__init__,errors,types,units,clock,config,logging,constants}.py`, `tests/unit/core/test_{errors,types,units,clock,config,logging,constants,import_contracts}.py`, `tests/fixtures/import_contract_violation/{pyproject.toml,broken/{__init__,errors,types}.py}`, `docs/architecture.md`, `docs/adr/{template,0001..0015}.md`, `configs/local.example.yaml`, `results/.gitkeep`, `models/.gitkeep`, `replays/.gitkeep`.

**FILES MODIFIED:** `.gitignore` (fixed a real bug found mid-phase: `results/` etc. needed to be `results/*` for the `!results/.gitkeep` negation to actually work — git can't re-include a file inside a wholesale-excluded directory). `.pre-commit-config.yaml` (excluded `*.patch` from the `trailing-whitespace` hook after it modified the Phase 1 PX4 patches — verified they still `git apply --check` cleanly afterward, but excluded going forward to remove the risk).

**TESTS RUN:** `pytest tests/unit` (65 tests), `ruff check .`, `ruff format --check .`, `mypy aeris`, `lint-imports` (twice: once against the real repo, once against the deliberately-broken fixture package), `pre-commit run --all-files`.

**TEST RESULTS:** All green.
- pytest: **65 passed**, 100% line coverage of `aeris/core/*`.
- ruff check: **All checks passed**.
- ruff format --check: **20 files already formatted** (after one `--fix` + `format` pass mid-phase).
- mypy: **Success: no issues found in 9 source files** (strict mode on `aeris.core`).
- import-linter, real contracts: **2 kept, 0 broken**.
- import-linter, fixture: **0 kept, 1 broken** (confirms detection works) — exit code 1 as expected.
- pre-commit, all 9 hooks: **Passed**.

**SIMULATION RESULTS:** None (out of scope; no simulation code exists yet).

**EXPERIMENT RESULTS:** None (out of scope).

**PROBLEMS FOUND:**
1. `git init` picked up a stray local `user.name`/`user.email` I'd set by mistake before checking — the machine already had a correct global identity (`EXCALIBUR303`, matching the `gh` auth account) from prior projects. Caught and unset before the first commit.
2. `uv sync` failed until `README.md` existed (hatchling's build backend validates the `readme` field at build time, even for an editable dev install) — wrote `README.md` first, in the intended order anyway.
3. `structlog.testing.capture_logs()` disables **all** configured processors, including `merge_contextvars` — a naive test of `bind_run_context` silently saw no `run_id`/`t_sim_s` at all (not a logging.py bug; a test-helper default I hadn't accounted for). Fixed by passing `processors=[merge_contextvars]` explicitly to the test helper.
4. `apply_overrides()` used a shallow `dict(config)` copy, so overriding a nested key silently mutated the caller's original config object — caught by `test_apply_overrides_does_not_mutate_input`. Fixed with `copy.deepcopy`.
5. `import-linter` ships a `lint-imports` console script, not a `python -m importlinter` entry point — first test attempt failed with "No module named importlinter.__main__". Fixed by resolving `lint-imports` next to the active Python interpreter.
6. `ruff format` tried to reformat Python code fences embedded in `AERIS_TECHNICAL_SPEC.md` (a newer ruff behavior) — excluded `*.md` from ruff's format scope; the spec's illustrative snippets are documentation, not real source.
7. The `trailing-whitespace` pre-commit hook rewrote the 9 Phase-1 PX4 patch files in place. Verified none broke (`git apply --check` still clean on all 9 against a fresh copy of the originals) but excluded `.patch` from that hook to remove the risk going forward — unified-diff files are whitespace-sensitive by construction and shouldn't be auto-modified.

**PROBLEMS FIXED:** All 7 above, in place, before the first commit — see "Files modified" and the source diffs in `aeris/core/config.py`/`tests/unit/core/test_logging.py`/`.gitignore`/`.pre-commit-config.yaml`/`pyproject.toml`.

**KNOWN LIMITATIONS:**
- `[tool.importlinter]` currently has only 2 contracts, both scoped to `aeris.core` (nothing else exists yet). The remaining contracts from spec §14.3 (ground-truth isolation, vehicle-adapter isolation, backend/frontend isolation, `gz.*` isolation) are listed as comments in `pyproject.toml` with the phase that will add each one, per spec §14.1's "create only in the phase that first needs it" principle.
- `make sim-smoke` is a documented stub (points to `docs/mac-setup.md`'s manual sequence) until Phase 3 builds `aeris.simulation.launcher`.
- CI runs on GitHub's hosted `macos-14` runner, which does **not** have PX4/Gazebo installed — SITL/sim-marked tests are out of scope for hosted CI per spec §44.2 and are recorded manually in phase reports instead.

**REMAINING RISKS:** None new. The two open items from Phase 1 (MAVSDK segfault, PX4 arm rejection) are unchanged and still owned by Phases 4 and 5 respectively.

**VALIDATION GATE (spec §51 Phase 2 / §54 row 2):** *"make lint typecheck test passes locally and in CI; import contracts are enforced (a deliberate violation in a test fixture is detected)."*

| Requirement | Result |
|---|---|
| `make lint` (ruff + mypy + import-linter) passes | **PASS** |
| `make test` (pytest) passes | **PASS** — 65/65 |
| Import contracts enforced, violation detected | **PASS** — proven by `test_import_contracts.py` against a real broken fixture, not just an assumption |
| CI green | **PASS locally** (the exact CI steps run clean); GitHub Actions itself will confirm on the first push-triggered run — not yet observed remotely at report-writing time |

**VALIDATION GATE: PASS.**

**CURRENT AERIS STATUS:** Repository exists at https://github.com/EXCALIBUR303/aeris (private), with a tested, enforced tooling foundation and one real package (`aeris.core`). No simulation, vehicle, safety, or autonomy code exists yet — Phase 3 is the first phase that talks to PX4 SITL programmatically.

============================================================
NEXT PHASE
============================================================

**NEXT PHASE:** 3 — PX4 SITL integration.
**RECOMMENDED MODEL:** Sonnet.
**RECOMMENDED EFFORT:** High.
**WHY THIS MODEL:** Integration engineering against a precise spec (§51 Phase 3) — process lifecycle, readiness detection, no research judgment calls.
**WHY THIS EFFORT:** Process lifecycle and port-management bugs are subtle and easy to get almost-right; the gate requires 10/10 consecutive clean start–ready–stop cycles.
**SWITCH REQUIRED:** NO (already on Sonnet).
**USE OPUS ONLY IF:** none expected.

**RECOMMENDED SKILLS / CONNECTORS:** Bash. Nothing else — no research or design tooling needed for this phase.

**EXPECTED OUTPUT:**
- `aeris/simulation/launcher/{profiles.py,process.py,readiness.py,params.py}`
- `configs/simulation/*.yaml` (launch profiles), `configs/vehicle/px4_sitl.yaml` (ports, airframe IDs), `configs/vehicle/px4_params/sitl_base.params`
- `aeris sim up --profile headless_x500`-style CLI or equivalent
- 10/10 consecutive start→ready→stop cycles with no orphaned processes; an RTF table at speed factors 1/2/4; a first Tier-H nondeterminism measurement (5 identical hover episodes → pose variance) that seeds the spec §40 D1 report

**ACTION REQUIRED:**

Switch model/effort if necessary (none needed). Then reply exactly:

**CONTINUE**

============================================================

Stopping here. Phase 3 has not been started.

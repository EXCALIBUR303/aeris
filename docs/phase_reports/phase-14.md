# AERIS — Phase 14 Report

============================================================
AERIS — PHASE 14 COMPLETE
============================================================

**PHASE:** 14 — PPO implementation.

**IMPLEMENTED:**
- **Networks** `aeris/learning/networks/`:
  - `distributions.py`: diagonal Gaussian with a state-independent log-std, and a masked categorical (masked logits −1e9, zero probability, no entropy contribution), both as explicit closed forms;
  - `encoders.py`: per-modality encoders (3-layer stride-2 CNN 32-64-64 → 256 for images, Linear → 64 for vectors, trunk 256);
  - `recurrent.py`: `GRUCore` with the §28 reset mask `h·(1 − start)` at every step input;
  - `heads.py`: `ActorCritic` with orthogonal init (√2 / 0.01 / 1.0), `step()` for rollout and `evaluate()` for the update's sequence replay.
- **PPO** `aeris/learning/ppo/`:
  - `buffer.py`: `[T, N, …]` storage, with feed-forward shuffling and recurrent whole-env-sequence minibatches;
  - `gae.py`: GAE with termination/truncation semantics, bootstrapping truncations from the final-observation value;
  - `losses.py`: clipped surrogate, optional clipped value loss, entropy, approx-KL (k3), clip fraction;
  - `diagnostics.py`: explained variance, running obs normalization, return-based reward scaling, NaN guard;
  - `checkpoint.py`: `weights_only`-loadable checkpoints plus an env snapshot for bitwise resume, with compatibility refusals;
  - `trainer.py`: `PPOTrainer`, with Adam (eps 1e-5), linear LR anneal, grad clip 0.5, K epochs, optional KL early stop, per-update §27.3 diagnostics, deterministic and stochastic evaluation on validation-split envs, best-on-validation checkpoint;
  - `config.py`: Pydantic schema, `extra="forbid"`;
  - `run.py`: spec §38 run directory, manifest, metrics/eval JSONL, summary, NaN diagnostics.
- **`aeris train --config <name>`** CLI (spec's expected output). `--set key=value` overrides, `--resume`, `--max-updates`.
- **Configs** `configs/learning/`: `ppo_base` (§27.2 defaults), `ppo_localnav_smoke`, `ppo_localnav_dev`, `ppo_exploration_smoke`, `ppo_toy_point_goal`, `ppo_toy_memory_gru`.
- **Env plumbing:**
  - `aeris/learning/envs/registry.py`: config → vector env + provenance; train-split worlds for training, val-split for evaluation;
  - `ExplorationVectorEnv`: same-step-autoreset batching of the decision-level env;
  - `aeris/learning/envs/toy.py`: contextual bandit, masked bandit, 1-D point goal, memory cue.
- **Benchmarks / references:**
  - `scripts/learning/benchmark_device.py`: CPU vs MPS, including a numerical-parity check;
  - `scripts/learning/sb3_reference.py`: dev-only SB3 sanity comparison, run with `uv run --with`, not a project dependency.
- New import contract: torch is confined to `aeris.learning` (safety, vehicle, simulation, mapping, localization, autonomy, evaluation, experiments and the deployment-shared `learning.spaces` stay torch-free).
- Strict mypy now covers `aeris.learning.ppo` and `aeris.learning.networks`. Spec §14.2 required it, but the override had only a "doesn't exist yet" placeholder.
- `docs/reinforcement-learning.md`: the math ↔ code mapping, every design choice with its citation, the device decision, and the test map.

**FILES CREATED:** `aeris/learning/networks/{__init__,distributions,encoders,recurrent,heads}.py`, `aeris/learning/ppo/{__init__,buffer,gae,losses,diagnostics,checkpoint,trainer,config,run}.py`, `aeris/learning/envs/{registry,toy}.py`, `configs/learning/{ppo_base,ppo_localnav_smoke,ppo_localnav_dev,ppo_exploration_smoke,ppo_toy_point_goal,ppo_toy_memory_gru}.yaml`, `scripts/learning/{benchmark_device,sb3_reference}.py`, `tests/unit/learning/ppo/test_ppo_{gae,losses,distributions,buffer,recurrent,known_answer,determinism_resume,trainer}.py`, `docs/reinforcement-learning.md`, `docs/phase_reports/phase-14.md`.

**FILES MODIFIED:**
- `aeris/cli.py`: `train` command, lazy torch import.
- `aeris/core/errors.py`: `LearningError`, `NonFiniteTrainingError`, `CheckpointMismatchError`.
- `aeris/experiments/manifest.py`: torch / numpy / gymnasium versions in `software_versions()`, as its docstring anticipated for the first phase that imports them.
- `aeris/learning/envs/exploration.py`: `ExplorationVectorEnv`, plus a D0 fix (PROBLEMS #1).
- `aeris/learning/envs/local_nav.py`: the vector env now exposes `obs_spec`, `reward_version` and `privileged_reward_notes` like the single env.
- `aeris/simulation/worlds/batch.py`: public in-memory `generate_world()`; `generate_batch` uses it, behavior unchanged.
- `pyproject.toml`: torch contract, strict mypy for PPO, SB3 mypy override, `gym` extra removed (PROBLEMS #2).
- `.github/workflows/ci.yml`: `--extra learn`.
- `uv.lock`.

**TESTS RUN:**
- `uv run pytest -q`: the full suite (unit + sim + integration + eval, no marker filter) as the mandatory pre-commit gate, on exactly the tree being committed. The background wait loop watched the pytest PID only, so it couldn't trip the launcher's orphan detector the way it did in Phase 13.
- A standalone retry of the one full-suite failure.
- `ruff check .`, `ruff format --check .`, `mypy aeris scripts` (now strict on `aeris.learning.ppo` / `networks`), `lint-imports` (9 contracts, 1 new): all clean.
- Experiments: `scripts/learning/benchmark_device.py` (twice, with consistent numbers); `scripts/learning/sb3_reference.py` (3 seeds × 2 implementations); the known-answer tasks on MPS; `aeris train --config ppo_localnav_smoke` end to end.

**TEST RESULTS:**

*Unit:* **1042 passed**, 0 failed. 51 are new for Phase 14 in `tests/unit/learning/ppo/`, covering spec §27.4 items 1–10 and §28 (a), (c):

| Spec item | Tests | Result |
|---|---|---|
| 1. GAE vs hand computation, (γ, λ) ∈ {(0.99, 0.95), (1, 1), (0.9, 0)}, with a mid-episode termination and a truncation | `test_ppo_gae.py` (formulas written out term by term) | pass |
| 2. Truncation bootstraps from the final-obs value | same file: changing the reset-obs value leaves Â₃ unchanged, changing the final value shifts it by exactly γ·Δ | pass |
| 3. θ = θ_old ⇒ ρ ≡ 1, clipfrac 0, L_clip = −E[Â] | `test_ppo_losses.py` | pass |
| 4. Zero clip gradient outside the trust region, both signs; non-zero where min() keeps the unclipped term | `test_ppo_losses.py` (5 cases) | pass |
| 5. Gaussian / masked categorical vs `torch.distributions`; masked p = 0, never sampled, entropy ignores masked | `test_ppo_distributions.py` | pass |
| 6. Buffer shapes; recurrent minibatches = whole sequences, no cross-env mixing, correct h₀ | `test_ppo_buffer.py` | pass |
| 7. Known-answer tasks (bandit, masked bandit, 1-D point goal ≥ 95%, memory GRU yes / MLP no), each < 2 min | `test_ppo_known_answer.py` (~20 s total) | pass |
| 8. Same seed → bitwise-identical losses (Gaussian MLP, GRU categorical, masked categorical) | `test_ppo_determinism_resume.py` | pass |
| 9. Checkpoint → resume → bitwise-identical losses and weights, including **FastSim LocalNav** | `test_ppo_determinism_resume.py` | pass |
| 10. NaN guard: injected NaN reward and a corrupted weight both halt with diagnostics; the run writes `nan_diagnostics.json` and fails its manifest | `test_ppo_trainer.py` | pass |
| §28 (a) leak test, (c) shape/device; rollout ≡ replay incl. mid-sequence resets | `test_ppo_recurrent.py` | pass |

Also covered: checkpoint compatibility refusals (config hash, network, action/obs space); `weights_only` loading; frozen eval normalization; masked actions never sampled on the real FastSim exploration env; and `aeris train` end to end through the CLI.

*Full suite:* **1069 passed, 1 failed, 1 xfailed** (55 min). The single failure was `test_five_consecutive_runs_of_each_mission_template[square]` with `SimulationLaunchError: no MAVLink heartbeat received on port 14540 within 30.0s`: the documented flaky-boot category, in mission code this phase didn't touch. **A standalone retry passed (5/5).** No orphaned `gz sim`/`px4` processes after either run.


**SIMULATION RESULTS (FastSim / CPU+MPS; no Tier H runs this phase, since PPO training is Tier F by design):**

*Known-answer tasks* (deterministic policy, 400 evaluation episodes, CPU):

| Task | Result | Bar |
|---|---|---|
| Contextual bandit | 1.00 optimal arm | converges |
| Masked bandit | 1.00 best *available* arm; a masked action is never taken (the env raises if one is) | converges |
| 1-D point goal (~200k steps) | 0.995 / 1.000 / 1.000 on seeds 0–2 (SB3-comparison runs; 150k-step check: 0.998 / 1.000 / 1.000) | ≥ 0.95 |
| Memory cue, GRU | **1.00** | solved |
| Memory cue, MLP | 0.525 (chance = 0.5; the test requires ≤ 0.65) | **not** solved |

Each trains in about 1–10 s, far under the 2-minute SMOKE budget. The point goal and the GRU memory task were also trained entirely on MPS and reach 1.00.

*Device benchmark* (`results/learning/device_benchmark.json`; M5 Air, macOS 27.2, torch 2.14, 4 threads):

| Workload | CPU | MPS |
|---|---|---|
| Local-nav training (depth CNN), env-steps/s end to end | 392 | **1,603** |
| Memory task (small GRU), env-steps/s | **6,400** | 1,096 |
| Exploration update (8×64×64 + GRU), ms | 136 | **36** |
| Depth-CNN policy forward, N = 16, ms | 5.9 | **0.9** |

CPU vs MPS parity on identical weights and batch: max |Δ| ≤ 6.2e-7 for log-prob, value and gradients (float32 rounding). **Device decision:**
- MPS for image-observation tasks at DEV/FULL budget (`ppo_localnav_dev`);
- CPU for low-dimensional tasks;
- CPU for every SMOKE config and test, since bitwise D0 resume is CPU-only and hosted CI has no MPS. The fallback to CPU is automatic.

*SB3 reference* (`results/learning/sb3_reference.json`; 1-D point goal, matched hyperparameters, 3 seeds, 200k steps; deterministic / stochastic success at the end):

| | AERIS | SB3 2.9.0 |
|---|---|---|
| Deterministic success, mean of 3 seeds | **0.998** | 0.508 |
| Stochastic success, mean of 3 seeds | **1.000** | 0.871 |

No gross bug in AERIS: it matches or beats SB3 on every seed and metric. SB3's deterministic shortfall is explained for seeds 0 and 2 (near-goal saturation of the Gaussian mean at the ±1 bound, so the policy hops across the goal window) and **unexplained for seed 1** (see PROBLEMS #6).

*End to end:* `aeris train --config ppo_localnav_smoke` runs rollout → GAE → update → evaluation on val-split FastSim worlds → checkpoint. That gives 4 updates at ~390 env-steps/s on CPU, a completed manifest, metrics/eval JSONL, and a final checkpoint (5.7 MB) plus env snapshot (23 MB). It isn't meant to learn the task at this budget; that's Phase 15.

**PROBLEMS FOUND:**
1. **Phase 13's `ExplorationEnv.reset(seed=None)` reseeded from OS entropy.** It called `np.random.default_rng(seed)` unconditionally, so every unseeded autoreset (which is every reset after the first in a vector env) broke D0 determinism. Found while building `ExplorationVectorEnv`.
2. **My Phase 13 rationale for the separate `gym` extra was wrong.** I wrote that CI would otherwise pull "multi-GB torch wheels". CI runs on **macos-15** (arm64), where torch is a CPU/MPS wheel well under 100 MB; the multi-GB CUDA wheels are a Linux concern.
3. **Strict mypy never covered `aeris.learning.ppo`**, despite spec §14.2. The override listed it only in a "doesn't exist yet" comment.
4. **Spec §7's assumption that CPU is faster for small RL nets is false for any network with an image encoder:** MPS is 4.1× faster end to end for local-nav training. It holds for small low-dimensional networks, where CPU is 5.8× faster.
5. **The 1-D point goal at 100k steps was seed-sensitive for the deterministic policy** (0.84–0.96, while stochastic was ≥ 0.995): the mean action hasn't sharpened near the goal yet.
6. **SB3 underperforms AERIS on the reference task.** Seeds 0 and 2 are explained by near-goal saturation. Seed 1's deterministic 0.57 with zero saturation is not. The gym adapter passes SB3's `check_env` and an oracle-policy check, so a harness bug is unlikely but not excluded. The two also differ in architecture.
7. **A torch tensor still requiring grad was converted with `float()` in logging**, raising a PyTorch warning.

**PROBLEMS FIXED:**
1. `ExplorationEnv.reset` reseeds only when given a seed; unseeded resets continue the env's own stream. Covered by the determinism and bitwise-resume tests, which run the masked-categorical path.
2. The `gym` extra is removed; CI installs `learn` (gymnasium + torch). The CI comment now states the real wheel size.
3. `aeris.learning.ppo.*` and `aeris.learning.networks.*` added to the strict override; both pass with no changes needed.
4. The decision is recorded in `configs/learning/ppo_base.yaml`, `ppo_localnav_dev.yaml` and `docs/reinforcement-learning.md`, backed by the benchmark and the parity check.
5. The known-answer test uses 200k steps (~8 s). The ≥ 95% bar is unchanged; at 150k steps all three seeds already scored ≥ 0.998, so the budget has margin.
6. Not fixed: reported as-is in `docs/reinforcement-learning.md`. The comparison's purpose (catching gross AERIS bugs) is met. SB3's behavior isn't this project's to fix.
7. `.detach()` before conversion.

**KNOWN LIMITATIONS:**
- **The separate-critic variant is not implemented.** Spec §27.2 makes it an ablation "if value interference is suspected", and there's no evidence of that yet. The `critic_privileged` variant (§27.8) doesn't exist yet either.
- **Bitwise resume needs the env snapshot pickle** (`.env.pkl`, a trusted local file; ~23 MB for FastSim LocalNav). Without it, resume is valid but not bitwise. Checkpoints keep every snapshot; no retention policy yet.
- **MPS training is D2**: not bitwise reproducible, by spec §40, and recorded as such.
- **The core PPO code is 918 non-comment lines** (trainer 415), at the top of spec §27.1's "about 600–900". With config/checkpoint/run bookkeeping it's 1,197.
- **`ExplorationVectorEnv` steps its envs sequentially in one process** (no subprocess parallelism yet). Exploration training throughput is Phase 19's concern.
- **Changing `ExplorationSpaceConfig.episode_time_s` doesn't change the observation-spec hash**, even though it rescales the time-remaining feature. The spec hash covers field shapes, bounds and provenance, not the builder's scaling constants. Found while writing the exploration smoke config; left for the phase that varies it.

**REMAINING RISKS:**
- **The training loop, not FastSim, is now the throughput bottleneck:** ~390 env-steps/s on CPU / ~1,600 on MPS for local-nav, against FastSim's ~12k env-only. Spec §27.7's DEV budget (2M steps) is ~21 min on MPS and ~85 min on CPU. FULL budgets (10–20M) are hours, feasible but worth planning for in Phase 15.
- **The known-answer tasks validate the algorithm, not the FastSim tasks themselves.** Whether local-nav PPO learns (≥ 80% success on val worlds) is Phase 15's gate.
- **SB3 seed 1** (PROBLEMS #6) is an open, small unknown in the reference check.

**VALIDATION GATE (spec §51 Phase 14):** *"all tests pass; known-answer tasks are solved within budget; the GRU-only memory task is solved by the GRU and **not** by the MLP (proves the memory path); resume is bitwise identical on CPU; SPS benchmark recorded (CPU vs MPS) with the device decision."*

| Requirement | Result |
|---|---|
| All tests pass | **PASS**: 1042/1042 unit tests (all 10 §27.4 items, §28). Full suite 1069/1070 with the one live-sim heartbeat failure passing on standalone retry. |
| Known-answer tasks solved within budget | **PASS**: bandit 1.00, masked bandit 1.00, 1-D point goal ≥ 0.995 (bar 0.95). Each trains in ≤ ~10 s against the 2-minute budget. |
| GRU solves the memory task, MLP does not | **PASS**: GRU 1.00; MLP 0.525 (chance). |
| Resume bitwise identical on CPU | **PASS**: identical losses and weights after resume on three toy configurations **and FastSim LocalNav**. |
| SPS benchmark CPU vs MPS + device decision | **PASS**: recorded with a numerical-parity check. Decision: MPS for image-observation tasks at DEV/FULL budget (4.1× faster), CPU for low-dimensional tasks and all SMOKE/tests. |

**VALIDATION GATE: PASS.**


**CURRENT AERIS STATUS:** AERIS now has a verified, explainable learning core:
- in-house PPO with MLP and GRU variants, Gaussian and masked-categorical heads, and truncation-correct GAE;
- bitwise-reproducible CPU training with bitwise checkpoint/resume, including on FastSim;
- a NaN guard that halts with diagnostics;
- a measured device decision;
- an `aeris train` entry point with spec §38 run bookkeeping.

Every learned-policy phase from 15 on trains through it.

============================================================
NEXT PHASE
============================================================

**NEXT PHASE:** 15 — PPO experiments / debugging on a navigation curriculum.
**RECOMMENDED MODEL:** Sonnet (spec §51 Phase 15: "experiment execution and routine debugging"). Escalate to Opus only for a conceptual training failure, suspected reward hacking not fixable by tuning, or a suspected PPO math error.
**RECOMMENDED EFFORT:** High.
**SWITCH REQUIRED:** YES (Phase 14 ran on Opus 5.5).
**ACTION REQUIRED:** Read spec §51 Phase 15 in full before starting (not reproduced here). Start from `configs/learning/ppo_localnav_dev.yaml` (`device: mps`, 2M steps, ~21 min). Phase 15 needs the curriculum stages (open → sparse → dense), 5-seed runs, sweeps on **validation** worlds with a recorded budget, trajectory audits for the §27.5 reward-hacking watchlist, a `LearnedLocalNav` deployment wrapper with its FastSim-vs-Tier-H parity test, and a 10-episode Tier H spot check. The `dataviz` skill is listed for its learning-curve figures.

============================================================

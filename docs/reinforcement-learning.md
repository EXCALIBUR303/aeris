# Reinforcement learning: in-house PPO

AERIS trains its learned policies with an in-house PPO implementation
(spec §27, Phase 14). It is a small, readable PyTorch implementation with
no hidden framework, following the CleanRL conventions catalogued in
Huang et al. 2022, *The 37 Implementation Details of Proximal Policy
Optimization* (ICLR blog track). This document maps the math to the code
and records every design choice with its source.

## Layout

| Module | Role |
|---|---|
| `aeris/learning/networks/distributions.py` | `DiagGaussian`, `MaskedCategorical`: closed forms, tested against `torch.distributions` |
| `aeris/learning/networks/encoders.py` | Per-modality encoders + trunk |
| `aeris/learning/networks/recurrent.py` | `GRUCore`: masked GRU stepping and sequence replay (§28) |
| `aeris/learning/networks/heads.py` | `ActorCritic`: encoder → [GRU] → actor + critic; `step()` (rollout) and `evaluate()` (update) |
| `aeris/learning/ppo/buffer.py` | `RolloutBuffer` `[T, N, …]` + feed-forward / whole-sequence minibatching |
| `aeris/learning/ppo/gae.py` | `compute_gae` with termination/truncation semantics |
| `aeris/learning/ppo/losses.py` | `ppo_loss`, `normalize_advantages` |
| `aeris/learning/ppo/diagnostics.py` | explained variance, `RunningMeanStd`, `ReturnScaler`, NaN guard |
| `aeris/learning/ppo/checkpoint.py` | save / load / verify / bitwise resume |
| `aeris/learning/ppo/trainer.py` | `PPOTrainer`: collect → GAE → K epochs → log → eval → checkpoint |
| `aeris/learning/ppo/config.py`, `configs/learning/ppo_*.yaml` | Pydantic config, `extends:` composition, hashing |
| `aeris/learning/ppo/run.py`, `aeris train` | One run with spec §38 bookkeeping |
| `aeris/learning/envs/registry.py` | Config → vector env + provenance (obs-spec hash, reward version) |
| `aeris/learning/envs/toy.py` | Known-answer tasks for §27.4 #7 |

`torch` is confined to `aeris.learning` by an import-linter contract:
safety, vehicle, simulation, mapping, autonomy, evaluation and the
deployment-shared `aeris.learning.spaces` stay torch-free.

## Mathematics (spec §27.3)

Gymnasium semantics: `terminated` is a true terminal (collision, success),
and `truncated` is an artificial cut (time limit).

**TD residual.**

δ_t = r_t + γ·(1 − term_t)·V̂_{t+1} − V(s_t),
with V̂_{t+1} = V(s_final⁽ᵗ⁾) if trunc_t, else V(s_{t+1}).

On truncation the bootstrap uses the value of the *true final observation*,
before auto-reset (Pardo et al. 2018, *Time Limits in Reinforcement
Learning*). After a same-step auto-reset, s_{t+1} is the next episode's first
observation, and bootstrapping from it would be wrong.
`PPOTrainer.collect` evaluates `V(info["final_obs"])` for truncated envs,
using the *pre-reset* GRU state, and stores it as `buffer.final_values`.

**GAE** (Schulman et al. 2016, *High-Dimensional Continuous Control Using
Generalized Advantage Estimation*):

Â_t = δ_t + γλ·(1 − done_t)·Â_{t+1}, with done = term ∨ trunc, so the
recursion never crosses an episode boundary. Value targets: R_t = Â_t + V(s_t).

**Clipped surrogate** (Schulman et al. 2017, *Proximal Policy Optimization
Algorithms*):

ρ_t = exp(log π_θ(a_t|s_t) − log π_old(a_t|s_t)),
L_clip = −E[min(ρ_t Â_t, clip(ρ_t, 1−ε, 1+ε) Â_t)],
L = L_clip + c_v·L_V − c_e·H[π], with L_V = ½E[(V − R)²] (clipped variant optional, default off).

**Diagnostics per update** (`metrics.jsonl`): approx-KL = E[(ρ − 1) − log ρ]
(Schulman's unbiased, non-negative "k3" estimator), clip fraction
E[|ρ − 1| > ε], entropy, policy/value loss, explained variance
1 − Var[R − V]/Var[R], gradient norm (pre-clip), SPS, learning rate, and
episode return / length / success.

## Design choices

| Choice | Value | Source |
|---|---|---|
| Init | Orthogonal; gain √2 hidden & conv, 0.01 policy output, 1.0 value output; zero biases | Huang et al. 2022 #2 |
| Optimizer | Adam, lr 3e-4, eps 1e-5, linear annealing to 0 | Huang et al. 2022 #3, #4 |
| Gradient clipping | global norm 0.5 | Huang et al. 2022 #11 |
| Advantage normalization | per minibatch | Huang et al. 2022 #7 |
| Value clipping | off by default (ablation flag) | Huang et al. 2022 #9; Engstrom et al. 2020 found it doesn't help |
| Continuous policy | diagonal Gaussian, **state-independent** log-std (init −0.5); log-prob on the unclipped sample, env-side clip | Huang et al. 2022 continuous #1, #4 |
| Discrete policy | logits; invalid actions → −1e9, giving exactly zero probability and no entropy contribution | Huang & Ontañón 2022, *A Closer Look at Invalid Action Masking* |
| Obs normalization | running mean/var (parallel Welford, Chan et al. 1979) for **low-dim** keys only, clipped ±10, frozen at eval; images use fixed physical scaling from `aeris.learning.spaces` | spec §27.2; Huang et al. 2022 continuous #5 |
| Reward scaling | optional, ÷ running std of the discounted return | Engstrom et al. 2020, *Implementation Matters in Deep Policy Gradients* |
| KL early stop | optional, when an epoch's mean approx-KL > 1.5·target (0.02); always logged | spec §27.2 |
| Recurrent batching | minibatches of whole env sequences; BPTT over T from stored h₀ with per-step reset masks | spec §27.2, §28; Huang et al. 2022 LSTM details |
| Encoders | depth/map: 3×(3×3, stride 2) conv 32-64-64 ReLU → 256; vectors: Linear → 64 tanh; concat (sorted keys) → trunk 256 tanh | spec §27.2 |
| Critic | shared trunk (spec default). The separate-network ablation is not implemented yet: the spec makes it conditional on suspected value interference, which there is no evidence of so far | spec §27.2 |

**Masking the GRU.** `h_t ← h_t·(1 − start_t)` is applied at the input of every
step, where `start_t` = "step t begins an episode" (= done_{t−1}). The
rollout (`ActorCritic.step`) and the training replay
(`ActorCritic.evaluate` → `GRUCore.sequence`) run the same masked
recurrence, and `test_sequence_replay_matches_stepwise_rollout_including_resets`
proves they produce identical outputs.

## Determinism and checkpoints

- Seeds: python `random`, torch global, plus two trainer-owned
  `torch.Generator`s (action sampling; minibatch permutation). Env RNGs are
  per-env `numpy.random.Generator`s. The global NumPy RNG is never used.
- **D0 on CPU**: same seed gives bitwise-identical losses (`test_same_seed_gives_identical_losses`).
  MPS training is **D2** (not guaranteed deterministic, spec §40).
- Checkpoint `<name>.pt` (loads with `torch.load(weights_only=True)`, so no
  arbitrary unpickling) holds the model, optimizer, normalizers, reward
  scaler, all RNG states, update / global step, config + hash, git SHA and
  dirty flag, observation-spec hash, reward version, action/obs shapes, and
  the trainer's runtime state (current obs, GRU state, start flags, partial
  episode returns).
- `<name>.env.pkl` is a pickle of the vector env (FastSim state, env RNGs),
  needed for **bitwise** resume. It's a trusted local file; without it a
  resume is valid but not bitwise. A FastSim LocalNav snapshot is ~23 MB
  (the voxel world bank dominates).
- Resume refuses (`CheckpointMismatchError`) a different observation spec,
  action space, observation shapes, network shape, or config hash (the last
  one overridable). Spec §27.6: never adapt silently.

## Device decision (Phase 14 benchmark)

`scripts/learning/benchmark_device.py` → `results/learning/device_benchmark.json`
(M5 MacBook Air, macOS 27.2, torch 2.14, 4 threads):

| Workload | CPU | MPS |
|---|---|---|
| Local-nav training, depth CNN (env-steps/s, end to end) | 392 | **1,603** |
| Memory task, small GRU (env-steps/s) | **6,400** | 1,096 |
| Exploration update, 8×64×64 maps + GRU, 8 envs × 16 steps (ms) | 136 | **36** |
| Policy forward, depth CNN, N = 16 (ms) | 5.9 | **0.9** |

CPU vs MPS numerical parity on identical weights and batch: max |Δ| 4.8e-7
(log-prob), 5.5e-7 (value), 6.2e-7 (gradients), i.e. float32 rounding. Both
known-answer tasks also reach 1.00 success when trained entirely on MPS.

**Decision.** Spec §7 assumed CPU would win for small RL nets. That holds
for the small low-dimensional networks and is **wrong for anything with an
image encoder**. So: `device: mps` for image-observation tasks at DEV/FULL
budget (`ppo_localnav_dev`), `cpu` for low-dimensional tasks, and `cpu` for
all SMOKE configs and tests (bitwise D0 resume; hosted CI has no MPS).
`resolve_device` falls back to CPU automatically if MPS is unavailable.
Training-loop throughput, not env throughput, is now the bottleneck:
FastSim alone does ~12k env-steps/s.

## Tests (spec §27.4, §28)

| Spec item | Test |
|---|---|
| 1. GAE vs hand computation, 3 (γ, λ) pairs, mid-episode term + trunc | `test_ppo_gae.py::test_gae_matches_hand_computation` |
| 2. Truncation bootstraps from the final obs | `test_ppo_gae.py::test_truncation_bootstraps_from_final_obs_value_not_reset_obs` |
| 3. θ = θ_old ⇒ ρ ≡ 1, clipfrac 0, L_clip = −E[Â] | `test_ppo_losses.py::test_identity_at_theta_equals_theta_old` |
| 4. Clip gradient zero outside the region (both signs) | `test_ppo_losses.py::test_clip_gradient_wrt_log_prob` |
| 5. Gaussian / masked categorical vs torch; masked ⇒ p = 0, no entropy | `test_ppo_distributions.py` |
| 6. Buffer shapes; recurrent minibatch sequence integrity | `test_ppo_buffer.py` |
| 7. Known-answer tasks < 2 min CPU (bandit, masked bandit, 1-D point goal ≥ 95%, memory: GRU yes / MLP no) | `test_ppo_known_answer.py` |
| 8. Same seed → identical losses | `test_ppo_determinism_resume.py::test_same_seed_gives_identical_losses` |
| 9. Checkpoint → resume → identical losses (incl. FastSim LocalNav) | `test_ppo_determinism_resume.py::test_checkpoint_resume_is_bitwise_identical` |
| 10. NaN guard halts with diagnostics | `test_ppo_trainer.py::test_nan_guard_*`, `test_run_writes_nan_diagnostics_and_fails_the_manifest` |
| §28 (a) leak test, (c) shapes/devices | `test_ppo_recurrent.py` |

## Reference sanity check vs Stable-Baselines3

`scripts/learning/sb3_reference.py` (SB3 is dev-only, run with
`uv run --with "stable-baselines3>=2.3" …`) trains both implementations on
the 1-D point goal with matched hyperparameters (16 envs × 32 steps, 4
epochs, 4 minibatches, lr 3e-4 annealed, γ 0.99, λ 0.95, ε 0.2, log-std init
−0.5, obs normalization), 3 seeds, 200k steps. Both are scored with the same
evaluator. Results are in `results/learning/sb3_reference.json`; final values
are deterministic / stochastic success, then near-goal saturation (the
fraction of states 0.05–0.1 from the goal whose mean action is at the ±1
bound; an ideal controller uses |a| < 1 there):

| Seed | AERIS | SB3 2.9.0 |
|---|---|---|
| 0 | 0.99 / 1.00 / 0.03 | 0.47 / 0.87 / 0.47 |
| 1 | 1.00 / 1.00 / 0.00 | 0.57 / 0.99 / 0.00 |
| 2 | 1.00 / 1.00 / 0.01 | 0.47 / 0.75 / 0.51 |

The check exists to catch gross bugs in AERIS's PPO, and it finds none:
AERIS learns the task at least as well as SB3 on every metric and seed.
SB3's deterministic shortfall on seeds 0 and 2 is explained by near-goal
saturation. Its unbounded Gaussian mean drifts past the action bound, the
deterministic policy becomes bang-bang ±1, and 0.1-steps then hop across
the 0.1-wide goal window about half the time (the 0.47 plateau). Seed 1's
deterministic 0.57 with zero saturation is **not** explained. The adapter
passes SB3's `check_env` and terminates correctly under an oracle policy, so
a harness bug is unlikely but not ruled out. The two implementations also
differ in architecture (shared tanh trunk vs SB3's separate [64, 64] actor
and critic MLPs), which may account for it.

## Running

```bash
uv run aeris train --config ppo_localnav_smoke
```

Overrides use `--set key=value` (e.g. `--set seed=3 --set device=mps`);
`--resume results/runs/<id>/checkpoints/update_000050.pt` continues a run.

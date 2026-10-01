"""``aeris train`` (spec §51 Phase 14 expected output): one PPO run end to
end, with spec §38's run bookkeeping.

``results/runs/<YYYYMMDD-HHMMSS>_<name>_<cfghash8>/`` gets the resolved
config, ``manifest.json`` (pessimistic ``failed`` until the run finishes),
``metrics.jsonl`` (one row of §27.3 diagnostics per update),
``eval.jsonl`` (deterministic **and** stochastic evaluation on validation
-split envs), ``checkpoints/`` and ``summary.json``. If the NaN guard trips,
``nan_diagnostics.json`` records what went non-finite and the run fails
loudly (spec §27.4 #10).
"""

from __future__ import annotations

import json
from collections.abc import Callable, Sequence
from pathlib import Path

import yaml

from aeris.core.errors import NonFiniteTrainingError
from aeris.core.types import RunStatus, Tier
from aeris.experiments.manifest import make_run_id, new_manifest, write_manifest
from aeris.learning.ppo.checkpoint import restore_trainer, save_checkpoint
from aeris.learning.ppo.config import load_train_config
from aeris.learning.ppo.trainer import PPOTrainer

RUNS_DIR = Path(__file__).resolve().parents[3] / "results" / "runs"


def run_training(
    config: str,
    overrides: Sequence[str] = (),
    *,
    run_dir: Path | None = None,
    resume: Path | None = None,
    max_updates: int | None = None,
    log: Callable[[str], None] | None = print,
) -> Path:
    cfg, raw, digest = load_train_config(config, tuple(overrides))
    run_dir = run_dir or RUNS_DIR / make_run_id(cfg.name, digest)
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "config_resolved.yaml").write_text(yaml.safe_dump(raw, sort_keys=True))

    trainer = PPOTrainer(cfg, config_hash=digest, run_dir=run_dir, log=log)
    manifest = new_manifest(
        run_id=run_dir.name,
        task=cfg.env.id,
        method="ppo_gru" if cfg.network.recurrent else "ppo_mlp",
        tier=Tier.FASTSIM,
        config_hash=digest,
        seed=cfg.seed,
    )
    manifest.device = str(trainer.device)
    write_manifest(manifest, run_dir)
    if resume is not None:
        bitwise = restore_trainer(trainer, resume)
        if log:
            log(
                f"resumed from {resume} at update {trainer.update}"
                + ("" if bitwise else " (no env snapshot: envs restarted, not bitwise)")
            )
    try:
        trainer.train(max_updates=max_updates)
        evaluation = trainer._evaluate_and_maybe_keep_best()
        final = run_dir / "checkpoints" / "final.pt"
        manifest.checkpoint_path = str(final.relative_to(run_dir))
        manifest.checkpoint_hash = save_checkpoint(trainer, final)
        summary = {
            "updates": trainer.update,
            "global_step": trainer.global_step,
            "obs_spec_hash": trainer.handle.obs_spec_hash,
            "reward_version": trainer.handle.reward_version,
            "privileged_reward_notes": list(trainer.handle.privileged_reward_notes),
            "final_eval": evaluation,
        }
        (run_dir / "summary.json").write_text(json.dumps(summary, indent=2))
        manifest.status, manifest.status_reason = RunStatus.COMPLETED, ""
    except NonFiniteTrainingError as exc:
        (run_dir / "nan_diagnostics.json").write_text(
            json.dumps({"error": str(exc), **exc.diagnostics}, indent=2, default=str)
        )
        manifest.status, manifest.status_reason = RunStatus.FAILED, f"NaN guard: {exc}"
        raise
    finally:
        write_manifest(manifest, run_dir)
    return run_dir

"""Checkpointing (spec §27.2): model, optimizer, normalizers, reward scaler,
RNG states (python / torch global / both trainer generators), global step,
config hash, git SHA, env-version (observation-spec hash + reward version),
plus the trainer's runtime state (current observation, GRU state,
episode-start flags, partial episode returns).

Two files per checkpoint:

- ``<name>.pt``: everything above, written with ``torch.save`` using only
  tensors and plain Python values, so it loads with
  ``torch.load(weights_only=True)``. Loading a checkpoint never unpickles
  arbitrary objects.
- ``<name>.env.pkl``: a pickle of the vector env itself (FastSim state, env
  RNG streams). Bitwise-identical resume needs the env exactly where it was,
  and that state isn't expressible as tensors. It's a *trusted local file*:
  only load env snapshots this project wrote. It's optional: without it a
  resume restarts the envs from their seed (valid training, not bitwise).

Loading refuses (``CheckpointMismatchError``) on a different observation
spec, action space or network shape, or on a different config (unless
explicitly allowed). Spec §27.6: never adapt silently.
"""

from __future__ import annotations

import hashlib
import os
import pickle
import random
from pathlib import Path
from typing import Any

import numpy as np
import torch

from aeris.core.errors import CheckpointMismatchError
from aeris.experiments.manifest import git_info

FORMAT_VERSION = 1


def _atomic_write(path: Path, write: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    write(tmp)
    os.replace(tmp, path)


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def env_snapshot_path(path: Path) -> Path:
    return path.with_name(path.stem + ".env.pkl")


def checkpoint_state(trainer: Any) -> dict[str, Any]:
    sha, dirty = git_info()
    h = trainer.handle
    return {
        "format_version": FORMAT_VERSION,
        "config": trainer.cfg.model_dump(mode="json"),
        "config_hash": trainer.config_hash,
        "git_sha": sha,
        "git_dirty": dirty,
        "obs_spec_hash": h.obs_spec_hash,
        "reward_version": h.reward_version,
        "action": {"kind": h.action.kind, "n": h.action.n},
        "obs_shapes": {k: list(s) for k, s in h.obs_shapes.items()},
        "model": trainer.model.state_dict(),
        "optimizer": trainer.optimizer.state_dict(),
        "obs_rms": {k: r.state_dict() for k, r in trainer.obs_rms.items()},
        "reward_scaler": (
            trainer.reward_scaler.state_dict() if trainer.reward_scaler is not None else None
        ),
        "update": trainer.update,
        "global_step": trainer.global_step,
        "best_eval": list(trainer.best_eval) if trainer.best_eval is not None else None,
        "rng": {
            "python": random.getstate(),
            "torch": torch.get_rng_state(),
            "sample_gen": trainer.sample_gen.get_state(),
            "perm_gen": trainer.perm_gen.get_state(),
        },
        "runtime": {
            "obs": {k: v.detach().cpu() for k, v in trainer.obs.items()},
            "h": trainer.h.detach().cpu(),
            "starts": trainer.starts.detach().cpu(),
            "mask": trainer.mask.cpu() if trainer.mask is not None else None,
            "ep_return": torch.from_numpy(trainer.ep_return.copy()),
            "ep_length": torch.from_numpy(trainer.ep_length.copy()),
        },
    }


def save_checkpoint(trainer: Any, path: Path, *, env_snapshot: bool = True) -> str:
    """Write ``path`` (+ ``.env.pkl``); returns the checkpoint's SHA-256."""
    state = checkpoint_state(trainer)
    _atomic_write(path, lambda p: torch.save(state, p))
    if env_snapshot:
        _atomic_write(
            env_snapshot_path(path),
            lambda p: p.write_bytes(pickle.dumps(trainer.handle.env, protocol=5)),
        )
    return file_sha256(path)


def load_checkpoint(path: Path) -> dict[str, Any]:
    state: dict[str, Any] = torch.load(path, map_location="cpu", weights_only=True)
    if state.get("format_version") != FORMAT_VERSION:
        raise CheckpointMismatchError(
            f"{path}: checkpoint format {state.get('format_version')} != {FORMAT_VERSION}"
        )
    return state


def verify_compatible(state: dict[str, Any], trainer: Any, *, allow_config_change: bool) -> None:
    h = trainer.handle
    problems = []
    if state["obs_spec_hash"] != h.obs_spec_hash:
        problems.append(f"observation spec {state['obs_spec_hash']} != {h.obs_spec_hash}")
    if state["action"] != {"kind": h.action.kind, "n": h.action.n}:
        problems.append(f"action space {state['action']} != {h.action}")
    if state["obs_shapes"] != {k: list(s) for k, s in h.obs_shapes.items()}:
        problems.append("observation shapes differ")
    if state["config"]["network"] != trainer.cfg.network.model_dump(mode="json"):
        problems.append("network architecture differs")
    if not allow_config_change and state["config_hash"] != trainer.config_hash:
        problems.append(f"config hash {state['config_hash'][:8]} != {trainer.config_hash[:8]}")
    if problems:
        raise CheckpointMismatchError("refusing checkpoint: " + "; ".join(problems))


def restore_trainer(
    trainer: Any, path: Path, *, allow_config_change: bool = False, restore_env: bool = True
) -> bool:
    """Load ``path`` into an already-constructed ``trainer``. Returns whether
    the env snapshot was restored (``True`` = bitwise continuation)."""
    state = load_checkpoint(path)
    verify_compatible(state, trainer, allow_config_change=allow_config_change)
    dev = trainer.device
    trainer.model.load_state_dict(state["model"])
    trainer.optimizer.load_state_dict(state["optimizer"])
    for k, r in trainer.obs_rms.items():
        r.load_state_dict(state["obs_rms"][k])
    if trainer.reward_scaler is not None and state["reward_scaler"] is not None:
        trainer.reward_scaler.load_state_dict(state["reward_scaler"])
    trainer.update = int(state["update"])
    trainer.global_step = int(state["global_step"])
    trainer.best_eval = tuple(state["best_eval"]) if state["best_eval"] is not None else None
    rng = state["rng"]
    random.setstate(rng["python"])
    torch.set_rng_state(rng["torch"])
    trainer.sample_gen.set_state(rng["sample_gen"])
    trainer.perm_gen.set_state(rng["perm_gen"])
    snap = env_snapshot_path(path)
    if not (restore_env and snap.is_file()):
        # Envs restart from their seed (the trainer's freshly reset obs /
        # zero memory / episode-start flags stay as constructed): a valid
        # continuation of training, but not a bitwise one.
        return False
    trainer.handle.env = pickle.loads(snap.read_bytes())
    rt = state["runtime"]
    trainer.obs = {k: v.to(dev) for k, v in rt["obs"].items()}
    trainer.h = rt["h"].to(dev)
    trainer.starts = rt["starts"].to(dev)
    trainer.mask = rt["mask"].to(dev) if rt["mask"] is not None else None
    trainer.ep_return = rt["ep_return"].numpy().astype(np.float64)
    trainer.ep_length = rt["ep_length"].numpy().astype(np.int64)
    return True

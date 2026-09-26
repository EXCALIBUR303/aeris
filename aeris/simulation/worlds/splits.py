"""Seed-range splits and the split lock file (spec §33.2, ADR-010).

Seed ranges are global across every world family — what matters is that a
world tagged ``split=X`` was generated from a seed inside ``X``'s range,
not that the range is family-exclusive. F4 (collapsed) is the sole
exception: it is pinned to ``TEST_OOD`` by :class:`~aeris.simulation.worlds.spec.WorldSpec`'s
own validator, never generated from another split's range at all.
"""

from __future__ import annotations

import json
from pathlib import Path

from aeris.core.errors import SplitViolationError
from aeris.simulation.worlds.spec import WorldSplit

SEED_RANGES: dict[WorldSplit, range] = {
    WorldSplit.TRAIN: range(0, 10_000),
    WorldSplit.VAL: range(10_000, 10_200),
    WorldSplit.TEST_ID: range(20_000, 20_100),
    WorldSplit.TEST_OOD: range(30_000, 30_100),
}


def validate_seed_for_split(seed: int, split: WorldSplit) -> None:
    if seed not in SEED_RANGES[split]:
        raise SplitViolationError(
            f"seed {seed} is not in {split.value}'s range {SEED_RANGES[split]}"
        )


def require_training_seed(seed: int) -> None:
    """The training entry point refuses val/test ranges (spec §33.2)."""
    if seed not in SEED_RANGES[WorldSplit.TRAIN]:
        raise SplitViolationError(
            f"training refuses seed {seed}: outside the train range {SEED_RANGES[WorldSplit.TRAIN]}"
        )


def require_tuning_seed(seed: int) -> None:
    """The tuning entry point refuses test ranges, but allows train/val (spec §33.2)."""
    if seed in SEED_RANGES[WorldSplit.TEST_ID] or seed in SEED_RANGES[WorldSplit.TEST_OOD]:
        raise SplitViolationError(f"tuning refuses seed {seed}: inside a test range")
    if seed not in SEED_RANGES[WorldSplit.TRAIN] and seed not in SEED_RANGES[WorldSplit.VAL]:
        raise SplitViolationError(f"tuning refuses seed {seed}: not in the train or val range")


def write_splits_lock(*, test_world_hashes: dict[str, str], path: Path) -> None:
    """Freeze the seed ranges plus a hash of every generated test WorldSpec (spec §33.2).

    ``test_world_hashes`` maps a world's ``name`` to its
    :meth:`WorldSpec.content_hash`, for every world in ``test_id``/
    ``test_ood`` a caller has generated and wants locked.
    """
    doc = {
        "seed_ranges": {split.value: [r.start, r.stop] for split, r in SEED_RANGES.items()},
        "test_world_hashes": dict(sorted(test_world_hashes.items())),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2, sort_keys=True) + "\n")


def load_splits_lock(path: Path) -> dict[str, object]:
    result: dict[str, object] = json.loads(path.read_text())
    return result


def verify_against_lock(*, world_name: str, world_hash: str, lock: dict[str, object]) -> None:
    """Raise if a regenerated test world's hash no longer matches the committed lock."""
    test_world_hashes = lock["test_world_hashes"]
    assert isinstance(test_world_hashes, dict)
    locked = test_world_hashes.get(world_name)
    if locked is None:
        raise SplitViolationError(f"world {world_name!r} is not in the committed split lock")
    if locked != world_hash:
        raise SplitViolationError(
            f"world {world_name!r} hash changed: locked={locked} actual={world_hash}"
        )

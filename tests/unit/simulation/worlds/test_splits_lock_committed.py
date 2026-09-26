"""Regression test: the committed ``configs/worlds/splits.lock`` still matches
what the generators actually produce (spec §33.2: "the split lock is committed").

If a generator changes in a way that alters its output for an already-locked
seed, this test fails loudly -- exactly the guard the spec's split lock exists
to provide.
"""

from __future__ import annotations

from pathlib import Path

from aeris.simulation.worlds.generators import collapsed, office, rubble, warehouse
from aeris.simulation.worlds.spec import WorldSplit
from aeris.simulation.worlds.splits import load_splits_lock, verify_against_lock

_REPO_ROOT = Path(__file__).resolve().parents[4]
_LOCK_PATH = _REPO_ROOT / "configs" / "worlds" / "splits.lock"
_N_TEST_ID_SEEDS = 5
_N_TEST_OOD_SEEDS = 5


def test_committed_lock_matches_regenerated_test_id_worlds() -> None:
    lock = load_splits_lock(_LOCK_PATH)
    for i in range(_N_TEST_ID_SEEDS):
        seed = 20_000 + i
        for generate in (rubble.generate, office.generate, warehouse.generate):
            spec = generate(seed, WorldSplit.TEST_ID)
            verify_against_lock(world_name=spec.name, world_hash=spec.content_hash(), lock=lock)


def test_committed_lock_matches_regenerated_test_ood_worlds() -> None:
    lock = load_splits_lock(_LOCK_PATH)
    for i in range(_N_TEST_OOD_SEEDS):
        spec = collapsed.generate(30_000 + i)
        verify_against_lock(world_name=spec.name, world_hash=spec.content_hash(), lock=lock)

"""Unit tests for :mod:`aeris.simulation.worlds.splits` (spec §33.2, ADR-010)."""

from __future__ import annotations

import pytest

from aeris.core.errors import SplitViolationError
from aeris.simulation.worlds.spec import WorldSplit
from aeris.simulation.worlds.splits import (
    SEED_RANGES,
    load_splits_lock,
    require_training_seed,
    require_tuning_seed,
    validate_seed_for_split,
    verify_against_lock,
    write_splits_lock,
)


def test_ranges_are_disjoint_and_match_spec() -> None:
    assert SEED_RANGES[WorldSplit.TRAIN] == range(0, 10_000)
    assert SEED_RANGES[WorldSplit.VAL] == range(10_000, 10_200)
    assert SEED_RANGES[WorldSplit.TEST_ID] == range(20_000, 20_100)
    assert SEED_RANGES[WorldSplit.TEST_OOD] == range(30_000, 30_100)


def test_validate_seed_for_split_accepts_in_range() -> None:
    validate_seed_for_split(5000, WorldSplit.TRAIN)  # no raise


def test_validate_seed_for_split_rejects_out_of_range() -> None:
    with pytest.raises(SplitViolationError):
        validate_seed_for_split(20050, WorldSplit.TRAIN)


def test_training_entry_point_refuses_val_and_test_seeds() -> None:
    require_training_seed(500)  # no raise
    with pytest.raises(SplitViolationError):
        require_training_seed(10_050)
    with pytest.raises(SplitViolationError):
        require_training_seed(20_050)


def test_tuning_entry_point_refuses_test_seeds_but_allows_train_and_val() -> None:
    require_tuning_seed(500)
    require_tuning_seed(10_050)
    with pytest.raises(SplitViolationError):
        require_tuning_seed(20_050)
    with pytest.raises(SplitViolationError):
        require_tuning_seed(30_050)


def test_write_and_load_splits_lock_round_trips(tmp_path) -> None:
    lock_path = tmp_path / "splits.lock"
    write_splits_lock(test_world_hashes={"w1": "abc123"}, path=lock_path)
    lock = load_splits_lock(lock_path)
    assert lock["test_world_hashes"]["w1"] == "abc123"


def test_verify_against_lock_passes_for_matching_hash(tmp_path) -> None:
    lock_path = tmp_path / "splits.lock"
    write_splits_lock(test_world_hashes={"w1": "abc123"}, path=lock_path)
    lock = load_splits_lock(lock_path)
    verify_against_lock(world_name="w1", world_hash="abc123", lock=lock)  # no raise


def test_verify_against_lock_rejects_changed_hash(tmp_path) -> None:
    lock_path = tmp_path / "splits.lock"
    write_splits_lock(test_world_hashes={"w1": "abc123"}, path=lock_path)
    lock = load_splits_lock(lock_path)
    with pytest.raises(SplitViolationError):
        verify_against_lock(world_name="w1", world_hash="different", lock=lock)


def test_verify_against_lock_rejects_unknown_world(tmp_path) -> None:
    lock_path = tmp_path / "splits.lock"
    write_splits_lock(test_world_hashes={}, path=lock_path)
    lock = load_splits_lock(lock_path)
    with pytest.raises(SplitViolationError):
        verify_against_lock(world_name="unknown", world_hash="x", lock=lock)

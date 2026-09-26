"""Unit tests for :mod:`aeris.simulation.worlds.batch` (the ``aeris worlds generate`` CLI's core)."""

from __future__ import annotations

import pytest

from aeris.core.errors import ConfigCompositionError
from aeris.simulation.worlds.batch import generate_batch, load_world_spec
from aeris.simulation.worlds.spec import WorldFamily, WorldSplit


def test_generate_batch_writes_n_json_and_sdf_files(tmp_path) -> None:
    written = generate_batch(
        family=WorldFamily.OFFICE, split=WorldSplit.TRAIN, n=3, out_dir=tmp_path
    )
    assert len(written) == 3
    for json_path in written:
        assert json_path.exists()
        assert json_path.with_suffix(".sdf").exists()


def test_generate_batch_seeds_are_sequential_from_range_start(tmp_path) -> None:
    written = generate_batch(
        family=WorldFamily.RUBBLE, split=WorldSplit.TRAIN, n=3, out_dir=tmp_path
    )
    specs = [load_world_spec(p) for p in written]
    assert sorted(s.seed for s in specs) == [0, 1, 2]


def test_generate_batch_rejects_n_larger_than_split_range(tmp_path) -> None:
    with pytest.raises(ConfigCompositionError):
        generate_batch(family=WorldFamily.OFFICE, split=WorldSplit.VAL, n=1000, out_dir=tmp_path)


def test_generate_batch_collapsed_ignores_split_mismatch_by_raising(tmp_path) -> None:
    with pytest.raises(ConfigCompositionError):
        generate_batch(family=WorldFamily.COLLAPSED, split=WorldSplit.TRAIN, n=1, out_dir=tmp_path)


def test_generate_batch_collapsed_with_test_ood_succeeds(tmp_path) -> None:
    written = generate_batch(
        family=WorldFamily.COLLAPSED, split=WorldSplit.TEST_OOD, n=2, out_dir=tmp_path
    )
    assert len(written) == 2
    specs = [load_world_spec(p) for p in written]
    assert all(s.split is WorldSplit.TEST_OOD for s in specs)


def test_load_world_spec_round_trips(tmp_path) -> None:
    written = generate_batch(
        family=WorldFamily.OFFICE, split=WorldSplit.TRAIN, n=1, out_dir=tmp_path
    )
    spec = load_world_spec(written[0])
    assert spec.family is WorldFamily.OFFICE

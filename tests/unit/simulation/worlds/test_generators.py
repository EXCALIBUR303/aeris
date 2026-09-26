"""Unit tests for the F1-F4 world generators (spec §33.1): determinism + reachability.

Determinism ("same seed -> same hash") is the spec's own literal test
requirement. Reachability is checked at a coarser resolution than the
generators' own internal check for speed, but the generators already
verify (or, for F4, retry until) reachability at generation time -- these
tests confirm that guarantee holds from the outside too.
"""

from __future__ import annotations

import pytest

from aeris.simulation.worlds.generators import collapsed, office, rubble, warehouse
from aeris.simulation.worlds.occupancy import check_reachability
from aeris.simulation.worlds.spec import WorldFamily, WorldSplit

_PROCEDURAL_GENERATORS = [
    (rubble.generate, (12345, WorldSplit.TRAIN), WorldFamily.RUBBLE),
    (office.generate, (12345, WorldSplit.TRAIN), WorldFamily.OFFICE),
    (warehouse.generate, (12345, WorldSplit.TRAIN), WorldFamily.WAREHOUSE),
]


@pytest.mark.parametrize(
    "generate,args,family", _PROCEDURAL_GENERATORS, ids=["rubble", "office", "warehouse"]
)
def test_same_seed_produces_identical_hash(generate, args, family) -> None:
    spec_a = generate(*args)
    spec_b = generate(*args)
    assert spec_a.content_hash() == spec_b.content_hash()
    assert spec_a.family is family


@pytest.mark.parametrize(
    "generate,args,_family", _PROCEDURAL_GENERATORS, ids=["rubble", "office", "warehouse"]
)
def test_different_seed_produces_different_hash(generate, args, _family) -> None:
    seed, split = args
    spec_a = generate(seed, split)
    spec_b = generate(seed + 1, split)
    assert spec_a.content_hash() != spec_b.content_hash()


@pytest.mark.parametrize(
    "generate,args,_family", _PROCEDURAL_GENERATORS, ids=["rubble", "office", "warehouse"]
)
def test_generated_world_has_at_least_one_reachable_spawn(generate, args, _family) -> None:
    spec = generate(*args)
    assert len(spec.spawn_poses) >= 1
    assert check_reachability(spec, resolution_m=0.5, min_free_fraction=0.2)


@pytest.mark.parametrize(
    "generate,args,_family", _PROCEDURAL_GENERATORS, ids=["rubble", "office", "warehouse"]
)
def test_generated_world_has_targets_in_spec_range(generate, args, _family) -> None:
    spec = generate(*args)
    assert 3 <= len(spec.targets) <= 8


def test_collapsed_generator_is_deterministic() -> None:
    spec_a = collapsed.generate(30_000)
    spec_b = collapsed.generate(30_000)
    assert spec_a.content_hash() == spec_b.content_hash()


def test_collapsed_generator_is_always_test_ood() -> None:
    spec = collapsed.generate(30_001)
    assert spec.family is WorldFamily.COLLAPSED
    assert spec.split is WorldSplit.TEST_OOD


def test_office_generator_connects_every_room_via_spanning_tree() -> None:
    # A regression guard on the spanning-tree connectivity claim itself,
    # independent of the voxel-based reachability check above: every
    # generated office world's reachable-fraction must be high, since a
    # spanning tree connects every room by construction.
    spec = office.generate(999, WorldSplit.TRAIN)
    assert check_reachability(spec, resolution_m=0.5, min_free_fraction=0.6)

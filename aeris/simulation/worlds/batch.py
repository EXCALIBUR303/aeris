"""Batch world generation -- the ``aeris worlds generate`` CLI's implementation (spec §51 Phase 9).

Generated worlds are run data, not source (spec §57): written under
``results/worlds/<family>/<split>/`` (gitignored), never under ``configs/``.
Seeds are drawn sequentially from the front of the split's own range (spec
§33.2), so ``--n 10`` for ``train`` always reuses the same first 10 seeds
run to run -- reproducible without needing a separate meta-seed.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from aeris.core.errors import ConfigCompositionError
from aeris.simulation.worlds import sdf
from aeris.simulation.worlds.generators import collapsed, office, rubble, warehouse
from aeris.simulation.worlds.spec import WorldFamily, WorldSpec, WorldSplit
from aeris.simulation.worlds.splits import SEED_RANGES, validate_seed_for_split

_GENERATORS: dict[WorldFamily, Callable[..., WorldSpec]] = {
    WorldFamily.RUBBLE: rubble.generate,
    WorldFamily.OFFICE: office.generate,
    WorldFamily.WAREHOUSE: warehouse.generate,
}


def generate_batch(
    *, family: WorldFamily, split: WorldSplit, n: int, out_dir: Path, render_sdf: bool = True
) -> list[Path]:
    """Generate ``n`` worlds for ``family``/``split``, writing JSON (+ optionally SDF).

    Returns the list of written JSON paths. Raises :class:`ConfigCompositionError`
    if ``n`` would run past the end of the split's seed range.
    """
    if family is WorldFamily.COLLAPSED:
        if split is not WorldSplit.TEST_OOD:
            raise ConfigCompositionError(
                "family f4_collapsed only ever generates split=test_ood (spec §33.1)"
            )
        seeds = list(
            range(
                SEED_RANGES[WorldSplit.TEST_OOD].start, SEED_RANGES[WorldSplit.TEST_OOD].start + n
            )
        )
    else:
        seed_range = SEED_RANGES[split]
        if n > len(seed_range):
            raise ConfigCompositionError(
                f"n={n} exceeds {split.value}'s range size ({len(seed_range)})"
            )
        seeds = list(range(seed_range.start, seed_range.start + n))

    dest = out_dir / family.value / split.value
    dest.mkdir(parents=True, exist_ok=True)

    written: list[Path] = []
    for seed in seeds:
        spec = generate_world(family, split, seed)

        json_path = dest / f"{spec.name}.json"
        json_path.write_text(spec.model_dump_json(indent=2))
        written.append(json_path)

        if render_sdf:
            (dest / f"{spec.name}.sdf").write_text(sdf.render(spec))

    return written


def generate_world(family: WorldFamily, split: WorldSplit, seed: int) -> WorldSpec:
    """One world, in memory (no files) -- the seed must belong to ``split``."""
    if family is WorldFamily.COLLAPSED:
        return collapsed.generate(seed)
    validate_seed_for_split(seed, split)
    return _GENERATORS[family](seed, split)


def load_world_spec(path: Path) -> WorldSpec:
    return WorldSpec.model_validate_json(path.read_text())

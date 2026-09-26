"""F4 Collapsed structure: irregular partially-blocked rooms with tilted slabs
and narrow passages (spec §33.1) -- **OOD test only**, never train/val/test_id.

Built on the same room-graph floorplan as F2 (office.py), narrower doors,
plus a few tilted slab obstacles (boxes with nonzero roll/pitch) dropped
into a subset of rooms. Always ``split=test_ood`` -- :class:`WorldSpec`'s
own validator rejects any other pairing for this family, so this module
doesn't accept a ``split`` argument at all.
"""

from __future__ import annotations

import math
import random

from aeris.simulation.worlds.generators._common import overlaps_any, sample_free_point
from aeris.simulation.worlds.generators.office import build_floorplan
from aeris.simulation.worlds.occupancy import check_reachability
from aeris.simulation.worlds.spec import (
    Bounds,
    Box,
    SpawnPose,
    Target,
    WorldFamily,
    WorldSpec,
    WorldSplit,
)

_NARROW_DOOR_WIDTH_M = 0.8
_MAX_GENERATION_ATTEMPTS = 20


def _add_tilted_slabs(
    rng: random.Random, walls: list[Box], bounds: Bounds, n_slabs: int
) -> list[Box]:
    slabs: list[Box] = []
    for _ in range(n_slabs):
        for _ in range(50):
            x = rng.uniform(bounds.min_x + 1.0, bounds.max_x - 1.0)
            y = rng.uniform(bounds.min_y + 1.0, bounds.max_y - 1.0)
            tilt = rng.uniform(math.radians(10), math.radians(35))
            slab = Box(
                x=x,
                y=y,
                z=rng.uniform(0.8, 1.8),
                size_x=rng.uniform(1.2, 2.0),
                size_y=rng.uniform(0.8, 1.4),
                size_z=0.08,
                roll_rad=rng.choice([-1, 1]) * tilt,
                pitch_rad=rng.choice([-1, 1]) * tilt * 0.5,
            )
            if not overlaps_any(slab, boxes=walls + slabs, cylinders=[], margin_m=0.3):
                slabs.append(slab)
                break
    return slabs


def generate(seed: int, *, name: str | None = None) -> WorldSpec:
    rng = random.Random(seed)

    for _attempt in range(_MAX_GENERATION_ATTEMPTS):
        rows, cols = rng.randint(2, 3), rng.randint(2, 3)
        walls, bounds = build_floorplan(
            rng, rows=rows, cols=cols, door_width_m=_NARROW_DOOR_WIDTH_M
        )
        slabs = _add_tilted_slabs(rng, walls, bounds, n_slabs=rng.randint(2, 5))
        all_boxes = walls + slabs

        try:
            spawn = sample_free_point(
                rng,
                min_x=bounds.min_x + 0.5,
                max_x=bounds.max_x - 0.5,
                min_y=bounds.min_y + 0.5,
                max_y=bounds.max_y - 0.5,
                z=0.1,
                boxes=all_boxes,
                cylinders=[],
            )
        except RuntimeError:
            continue

        targets = []
        try:
            for _ in range(rng.randint(3, 8)):
                t = sample_free_point(
                    rng,
                    min_x=bounds.min_x + 0.5,
                    max_x=bounds.max_x - 0.5,
                    min_y=bounds.min_y + 0.5,
                    max_y=bounds.max_y - 0.5,
                    z=1.0,
                    boxes=all_boxes,
                    cylinders=[],
                )
                targets.append(Target(x=t.x, y=t.y, z=t.z))
        except RuntimeError:
            continue

        spec = WorldSpec(
            name=name or f"f4_collapsed_{seed}",
            family=WorldFamily.COLLAPSED,
            split=WorldSplit.TEST_OOD,
            seed=seed,
            bounds=bounds,
            boxes=tuple(all_boxes),
            targets=tuple(targets),
            spawn_poses=(SpawnPose(x=spawn.x, y=spawn.y, z=spawn.z),),
        )
        if check_reachability(spec, min_free_fraction=0.4):
            return spec

    raise RuntimeError(
        f"could not generate a reachable F4 world for seed {seed} in {_MAX_GENERATION_ATTEMPTS} attempts"
    )

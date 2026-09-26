"""F1 Rubble field: open outdoor area, scattered blocks/debris, varying density (spec §33.1)."""

from __future__ import annotations

import random

from aeris.simulation.worlds.generators._common import overlaps_any, sample_free_point
from aeris.simulation.worlds.spec import (
    Bounds,
    Box,
    SpawnPose,
    Target,
    WorldFamily,
    WorldSpec,
    WorldSplit,
)

_HALF_SIZE_M = 12.0  # a 24x24m open field
_MAX_PLACEMENT_ATTEMPTS = 200


def generate(seed: int, split: WorldSplit, *, name: str | None = None) -> WorldSpec:
    rng = random.Random(seed)
    bounds = Bounds(
        min_x=-_HALF_SIZE_M,
        min_y=-_HALF_SIZE_M,
        min_z=0.0,
        max_x=_HALF_SIZE_M,
        max_y=_HALF_SIZE_M,
        max_z=6.0,
    )

    density = rng.uniform(0.03, 0.15)  # fraction of the field's area covered by debris footprints
    field_area = (2 * _HALF_SIZE_M) ** 2
    target_footprint = density * field_area

    boxes: list[Box] = []
    placed_footprint = 0.0
    while placed_footprint < target_footprint:
        size_x = rng.uniform(0.3, 1.4)
        size_y = rng.uniform(0.3, 1.4)
        size_z = rng.uniform(0.2, 1.0)
        placed = False
        for _ in range(_MAX_PLACEMENT_ATTEMPTS):
            x = rng.uniform(-_HALF_SIZE_M + 1.0, _HALF_SIZE_M - 1.0)
            y = rng.uniform(-_HALF_SIZE_M + 1.0, _HALF_SIZE_M - 1.0)
            candidate = Box(
                x=x,
                y=y,
                z=size_z / 2.0,
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                yaw_rad=rng.uniform(0, 6.28),
            )
            if not overlaps_any(candidate, boxes=boxes, cylinders=[]):
                boxes.append(candidate)
                placed_footprint += size_x * size_y
                placed = True
                break
        if not placed:
            break  # field is dense enough that placement is failing; stop rather than loop forever

    spawn = sample_free_point(
        rng,
        min_x=-_HALF_SIZE_M + 1.0,
        max_x=_HALF_SIZE_M - 1.0,
        min_y=-_HALF_SIZE_M + 1.0,
        max_y=_HALF_SIZE_M - 1.0,
        z=0.1,
        boxes=boxes,
        cylinders=[],
    )

    targets = []
    for _ in range(rng.randint(3, 8)):
        t = sample_free_point(
            rng,
            min_x=-_HALF_SIZE_M + 1.0,
            max_x=_HALF_SIZE_M - 1.0,
            min_y=-_HALF_SIZE_M + 1.0,
            max_y=_HALF_SIZE_M - 1.0,
            z=1.0,
            boxes=boxes,
            cylinders=[],
        )
        targets.append(Target(x=t.x, y=t.y, z=t.z))

    return WorldSpec(
        name=name or f"f1_rubble_{seed}",
        family=WorldFamily.RUBBLE,
        split=split,
        seed=seed,
        bounds=bounds,
        boxes=tuple(boxes),
        targets=tuple(targets),
        spawn_poses=(SpawnPose(x=spawn.x, y=spawn.y, z=spawn.z),),
    )

"""F3 Warehouse: shelf aisles, pallets, open bays (spec §33.1)."""

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

_SHELF_DEPTH_M = 0.6
_SHELF_HEIGHT_M = 2.2
_AISLE_WIDTH_M = 2.5
_BAY_LENGTH_M = 12.0
_ENTRANCE_AISLE_M = 3.0


def generate(seed: int, split: WorldSplit, *, name: str | None = None) -> WorldSpec:
    rng = random.Random(seed)
    n_rows = rng.randint(3, 6)
    total_width = _ENTRANCE_AISLE_M + n_rows * (_SHELF_DEPTH_M + _AISLE_WIDTH_M)
    half_len = _BAY_LENGTH_M / 2.0

    bounds = Bounds(
        min_x=-half_len - 1.0,
        min_y=-_ENTRANCE_AISLE_M / 2.0 - 1.0,
        min_z=0.0,
        max_x=half_len + 1.0,
        max_y=total_width,
        max_z=_SHELF_HEIGHT_M + 1.0,
    )

    boxes: list[Box] = []
    row_y_centers: list[float] = []
    y = _ENTRANCE_AISLE_M + _SHELF_DEPTH_M / 2.0
    for _row in range(n_rows):
        row_y_centers.append(y)
        boxes.append(
            Box(
                x=0.0,
                y=y,
                z=_SHELF_HEIGHT_M / 2.0,
                size_x=_BAY_LENGTH_M,
                size_y=_SHELF_DEPTH_M,
                size_z=_SHELF_HEIGHT_M,
            )
        )
        y += _SHELF_DEPTH_M + _AISLE_WIDTH_M

    # Pallets scattered in the aisles between shelf rows and at bay ends.
    n_pallets = rng.randint(4, 10)
    for _ in range(n_pallets):
        row_gap_index = rng.randint(0, n_rows)  # 0 = entrance aisle, else between rows[i-1]/rows[i]
        if row_gap_index == 0:
            y_center = _ENTRANCE_AISLE_M / 2.0
        else:
            y_center = (
                row_y_centers[row_gap_index - 1] + _SHELF_DEPTH_M / 2.0 + _AISLE_WIDTH_M / 2.0
            )
        x = rng.uniform(-half_len + 1.0, half_len - 1.0)
        pallet = Box(
            x=x,
            y=y_center + rng.uniform(-0.5, 0.5),
            z=0.25,
            size_x=0.8,
            size_y=1.2,
            size_z=0.5,
            yaw_rad=rng.uniform(0, 6.28),
        )
        if not overlaps_any(pallet, boxes=boxes, cylinders=[]):
            boxes.append(pallet)

    spawn = sample_free_point(
        rng,
        min_x=-half_len + 0.5,
        max_x=half_len - 0.5,
        min_y=0.2,
        max_y=_ENTRANCE_AISLE_M - 0.2,
        z=0.1,
        boxes=boxes,
        cylinders=[],
    )

    targets = []
    for _ in range(rng.randint(3, 8)):
        t = sample_free_point(
            rng,
            min_x=-half_len + 0.5,
            max_x=half_len - 0.5,
            min_y=0.2,
            max_y=total_width - 0.5,
            z=1.0,
            boxes=boxes,
            cylinders=[],
        )
        targets.append(Target(x=t.x, y=t.y, z=t.z))

    return WorldSpec(
        name=name or f"f3_warehouse_{seed}",
        family=WorldFamily.WAREHOUSE,
        split=split,
        seed=seed,
        bounds=bounds,
        boxes=tuple(boxes),
        targets=tuple(targets),
        spawn_poses=(SpawnPose(x=spawn.x, y=spawn.y, z=spawn.z),),
    )

"""Shared helpers for the F1-F4 generators (spec §33.1) -- not part of the public API."""

from __future__ import annotations

import random

from aeris.core.frames.vector import Vec3
from aeris.simulation.worlds.occupancy import point_in_box, point_in_cylinder
from aeris.simulation.worlds.spec import Box, Cylinder


def _inflated_box(b: Box, margin_m: float) -> Box:
    return b.model_copy(
        update={
            "size_x": b.size_x + 2 * margin_m,
            "size_y": b.size_y + 2 * margin_m,
            "size_z": b.size_z + 2 * margin_m,
        }
    )


def _inflated_cylinder(c: Cylinder, margin_m: float) -> Cylinder:
    return c.model_copy(update={"radius_m": c.radius_m + margin_m})


def sample_free_point(
    rng: random.Random,
    *,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    z: float,
    boxes: list[Box],
    cylinders: list[Cylinder],
    margin_m: float = 0.4,
    max_attempts: int = 500,
) -> Vec3:
    """Rejection-sample a point clear of every obstacle (inflated by ``margin_m``)."""
    inflated_boxes = [_inflated_box(b, margin_m) for b in boxes]
    inflated_cylinders = [_inflated_cylinder(c, margin_m) for c in cylinders]
    for _ in range(max_attempts):
        p = Vec3(rng.uniform(min_x, max_x), rng.uniform(min_y, max_y), z)
        if any(point_in_box(p, b) for b in inflated_boxes):
            continue
        if any(point_in_cylinder(p, c) for c in inflated_cylinders):
            continue
        return p
    raise RuntimeError(
        f"could not sample a free point in {max_attempts} attempts (obstacles too dense)"
    )


def overlaps_any(
    box: Box, *, boxes: list[Box], cylinders: list[Cylinder], margin_m: float = 0.1
) -> bool:
    """Whether a candidate box (any of its 8 corners) intersects existing geometry, inflated by a margin."""
    half = box.half_extent
    corners = [
        Vec3(box.x + sx * half.x, box.y + sy * half.y, box.z + sz * half.z)
        for sx in (-1, 1)
        for sy in (-1, 1)
        for sz in (-1, 1)
    ]
    inflated_boxes = [_inflated_box(b, margin_m) for b in boxes]
    inflated_cylinders = [_inflated_cylinder(c, margin_m) for c in cylinders]
    return any(point_in_box(p, b) for p in corners for b in inflated_boxes) or any(
        point_in_cylinder(p, c) for p in corners for c in inflated_cylinders
    )

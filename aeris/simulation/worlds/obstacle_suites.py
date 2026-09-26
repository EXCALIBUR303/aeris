"""Obstacle suites for Phase 10's classical-avoidance experiment (spec §51 Phase 9 task list).

Five hand-designed scenarios, each a small, deliberately simple
:class:`WorldSpec` exercising one specific avoidance behavior rather than
a randomized family. Tagged ``family=f1_rubble`` / ``split=test_id``
(arbitrary but documented here): these are fixed evaluation scenarios, not
members of the train/val/test-ID *procedural* population the F1-F4
generators produce, so the family/split fields are metadata of
convenience rather than a real generalization claim.
"""

from __future__ import annotations

from aeris.simulation.worlds.spec import (
    Bounds,
    Box,
    Cylinder,
    SpawnPose,
    WorldFamily,
    WorldSpec,
    WorldSplit,
)

_TAG_FAMILY = WorldFamily.RUBBLE
_TAG_SPLIT = WorldSplit.TEST_ID
_WALL_HEIGHT_M = 2.5
_WALL_THICKNESS_M = 0.15


def corridor(*, length_m: float = 15.0, width_m: float = 2.0) -> WorldSpec:
    """A long, straight corridor: two parallel walls, spawn at one end."""
    half_len = length_m / 2.0
    half_w = width_m / 2.0
    walls = (
        Box(
            x=0.0,
            y=-half_w,
            z=_WALL_HEIGHT_M / 2.0,
            size_x=length_m,
            size_y=_WALL_THICKNESS_M,
            size_z=_WALL_HEIGHT_M,
        ),
        Box(
            x=0.0,
            y=half_w,
            z=_WALL_HEIGHT_M / 2.0,
            size_x=length_m,
            size_y=_WALL_THICKNESS_M,
            size_z=_WALL_HEIGHT_M,
        ),
    )
    bounds = Bounds(
        min_x=-half_len - 1.0,
        min_y=-half_w - 1.0,
        max_x=half_len + 1.0,
        max_y=half_w + 1.0,
        max_z=_WALL_HEIGHT_M + 1.0,
    )
    return WorldSpec(
        name="obstacle_suite_corridor",
        family=_TAG_FAMILY,
        split=_TAG_SPLIT,
        seed=0,
        bounds=bounds,
        boxes=walls,
        spawn_poses=(SpawnPose(x=-half_len + 0.5, y=0.0, z=0.1),),
    )


def pillar_forest(
    *, n_pillars_per_axis: int = 4, spacing_m: float = 3.0, radius_m: float = 0.3
) -> WorldSpec:
    """A grid of cylindrical pillars, spaced to require reactive lateral avoidance."""
    half_span = (n_pillars_per_axis - 1) * spacing_m / 2.0
    pillars = tuple(
        Cylinder(
            x=i * spacing_m - half_span,
            y=j * spacing_m - half_span,
            z=1.25,
            radius_m=radius_m,
            height_m=2.5,
        )
        for i in range(n_pillars_per_axis)
        for j in range(n_pillars_per_axis)
    )
    bounds = Bounds(
        min_x=-half_span - 2.0,
        min_y=-half_span - 2.0,
        max_x=half_span + 2.0,
        max_y=half_span + 2.0,
        max_z=3.5,
    )
    return WorldSpec(
        name="obstacle_suite_pillar_forest",
        family=_TAG_FAMILY,
        split=_TAG_SPLIT,
        seed=0,
        bounds=bounds,
        cylinders=pillars,
        spawn_poses=(SpawnPose(x=bounds.min_x + 1.0, y=0.0, z=0.1),),
    )


def dead_end(*, length_m: float = 8.0, width_m: float = 2.0) -> WorldSpec:
    """A corridor that terminates in a wall -- tests recognizing a blocked path."""
    half_len = length_m / 2.0
    half_w = width_m / 2.0
    walls = (
        Box(
            x=0.0,
            y=-half_w,
            z=_WALL_HEIGHT_M / 2.0,
            size_x=length_m,
            size_y=_WALL_THICKNESS_M,
            size_z=_WALL_HEIGHT_M,
        ),
        Box(
            x=0.0,
            y=half_w,
            z=_WALL_HEIGHT_M / 2.0,
            size_x=length_m,
            size_y=_WALL_THICKNESS_M,
            size_z=_WALL_HEIGHT_M,
        ),
        Box(
            x=half_len,
            y=0.0,
            z=_WALL_HEIGHT_M / 2.0,
            size_x=_WALL_THICKNESS_M,
            size_y=width_m,
            size_z=_WALL_HEIGHT_M,
        ),
    )
    bounds = Bounds(
        min_x=-half_len - 1.0,
        min_y=-half_w - 1.0,
        max_x=half_len + 1.0,
        max_y=half_w + 1.0,
        max_z=_WALL_HEIGHT_M + 1.0,
    )
    return WorldSpec(
        name="obstacle_suite_dead_end",
        family=_TAG_FAMILY,
        split=_TAG_SPLIT,
        seed=0,
        bounds=bounds,
        boxes=walls,
        spawn_poses=(SpawnPose(x=-half_len + 0.5, y=0.0, z=0.1),),
    )


def narrow_gap(*, gap_width_m: float = 0.9, wall_span_m: float = 6.0) -> WorldSpec:
    """Two wall segments with a single narrow gap between them, barely wider than the vehicle."""
    half_gap = gap_width_m / 2.0
    seg_len = (wall_span_m - gap_width_m) / 2.0
    walls = (
        Box(
            x=-half_gap - seg_len / 2.0,
            y=0.0,
            z=_WALL_HEIGHT_M / 2.0,
            size_x=seg_len,
            size_y=_WALL_THICKNESS_M,
            size_z=_WALL_HEIGHT_M,
        ),
        Box(
            x=half_gap + seg_len / 2.0,
            y=0.0,
            z=_WALL_HEIGHT_M / 2.0,
            size_x=seg_len,
            size_y=_WALL_THICKNESS_M,
            size_z=_WALL_HEIGHT_M,
        ),
    )
    bounds = Bounds(
        min_x=-wall_span_m / 2.0 - 2.0,
        min_y=-3.0,
        max_x=wall_span_m / 2.0 + 2.0,
        max_y=3.0,
        max_z=_WALL_HEIGHT_M + 1.0,
    )
    return WorldSpec(
        name="obstacle_suite_narrow_gap",
        family=_TAG_FAMILY,
        split=_TAG_SPLIT,
        seed=0,
        bounds=bounds,
        boxes=walls,
        spawn_poses=(SpawnPose(x=bounds.min_x + 1.0, y=0.0, z=0.1),),
    )


def overhang_within_band(
    *, overhang_z: float = 1.8, altitude_band_m: tuple[float, float] = (0.3, 3.0)
) -> WorldSpec:
    """A horizontal slab obstruction inside the altitude band -- tests vertical-constraint avoidance."""
    bounds = Bounds(min_x=-6.0, min_y=-4.0, max_x=6.0, max_y=4.0, max_z=4.0)
    overhang = Box(x=0.0, y=0.0, z=overhang_z, size_x=3.0, size_y=8.0, size_z=0.2)
    return WorldSpec(
        name="obstacle_suite_overhang_within_band",
        family=_TAG_FAMILY,
        split=_TAG_SPLIT,
        seed=0,
        bounds=bounds,
        boxes=(overhang,),
        altitude_band_m=altitude_band_m,
        spawn_poses=(SpawnPose(x=-5.0, y=0.0, z=0.1),),
    )


ALL_SUITES = {
    "corridor": corridor,
    "pillar_forest": pillar_forest,
    "dead_end": dead_end,
    "narrow_gap": narrow_gap,
    "overhang_within_band": overhang_within_band,
}

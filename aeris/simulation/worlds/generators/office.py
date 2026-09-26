"""F2 Office: rooms + corridors + doors, a graph-based floorplan generator (spec §33.1).

Rooms form a grid graph (4-connected). A random spanning tree over that
graph (randomized Kruskal) determines which room-adjacency walls get a
door opening -- this guarantees every room is reachable from every other
by construction, not just by chance, since a spanning tree is connected by
definition. A configurable fraction of the remaining (non-tree) adjacent
walls also get doors, for a more realistic floorplan than a bare tree.
"""

from __future__ import annotations

import random

from aeris.simulation.worlds.generators._common import sample_free_point
from aeris.simulation.worlds.spec import (
    Bounds,
    Box,
    SpawnPose,
    Target,
    WorldFamily,
    WorldSpec,
    WorldSplit,
)

_ROOM_SIZE_M = 5.0
_WALL_THICKNESS_M = 0.15
_WALL_HEIGHT_M = 2.5
_DOOR_WIDTH_M = 1.2
_EXTRA_DOOR_PROBABILITY = 0.3


def build_floorplan(
    rng: random.Random,
    *,
    rows: int,
    cols: int,
    room_size_m: float = _ROOM_SIZE_M,
    wall_thickness_m: float = _WALL_THICKNESS_M,
    wall_height_m: float = _WALL_HEIGHT_M,
    door_width_m: float = _DOOR_WIDTH_M,
    extra_door_probability: float = _EXTRA_DOOR_PROBABILITY,
) -> tuple[list[Box], Bounds]:
    """Build a ``rows`` x ``cols`` grid of rooms with a spanning-tree-connected door graph.

    Returns ``(walls, bounds)``. Room ``(i, j)``'s center is at
    ``(j * room_size_m, i * room_size_m)``; the whole floorplan is centered
    on the origin.
    """
    total_w = cols * room_size_m
    total_h = rows * room_size_m
    origin_x, origin_y = -total_w / 2.0, -total_h / 2.0

    def room_center(i: int, j: int) -> tuple[float, float]:
        return (origin_x + (j + 0.5) * room_size_m, origin_y + (i + 0.5) * room_size_m)

    # Every adjacent room pair, both horizontal and vertical.
    edges: list[tuple[tuple[int, int], tuple[int, int], str]] = []
    for i in range(rows):
        for j in range(cols):
            if j + 1 < cols:
                edges.append(((i, j), (i, j + 1), "vertical_wall"))  # wall runs N-S, shared x
            if i + 1 < rows:
                edges.append(((i, j), (i + 1, j), "horizontal_wall"))  # wall runs E-W, shared y

    rng.shuffle(edges)

    # Randomized Kruskal spanning tree over the room grid graph.
    parent: dict[tuple[int, int], tuple[int, int]] = {
        (i, j): (i, j) for i in range(rows) for j in range(cols)
    }

    def find(cell: tuple[int, int]) -> tuple[int, int]:
        while parent[cell] != cell:
            cell = parent[cell]
        return cell

    doored_edges: set[tuple[tuple[int, int], tuple[int, int]]] = set()
    for a, b, _kind in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
            doored_edges.add((a, b))
    for a, b, _kind in edges:
        if (a, b) not in doored_edges and rng.random() < extra_door_probability:
            doored_edges.add((a, b))

    walls: list[Box] = []
    for (i, j), (i2, j2), kind in edges:
        has_door = ((i, j), (i2, j2)) in doored_edges
        cx1, cy1 = room_center(i, j)
        cx2, cy2 = room_center(i2, j2)
        mid_x, mid_y = (cx1 + cx2) / 2.0, (cy1 + cy2) / 2.0

        if kind == "vertical_wall":  # separates rooms side by side (shared x boundary)
            wall_len = room_size_m
            if not has_door:
                walls.append(
                    Box(
                        x=mid_x,
                        y=mid_y,
                        z=wall_height_m / 2.0,
                        size_x=wall_thickness_m,
                        size_y=wall_len,
                        size_z=wall_height_m,
                    )
                )
            else:
                seg_len = (wall_len - door_width_m) / 2.0
                for sign in (-1, 1):
                    seg_center_y = mid_y + sign * (door_width_m / 2.0 + seg_len / 2.0)
                    walls.append(
                        Box(
                            x=mid_x,
                            y=seg_center_y,
                            z=wall_height_m / 2.0,
                            size_x=wall_thickness_m,
                            size_y=seg_len,
                            size_z=wall_height_m,
                        )
                    )
        else:  # horizontal_wall: separates rooms stacked in y (shared y boundary)
            wall_len = room_size_m
            if not has_door:
                walls.append(
                    Box(
                        x=mid_x,
                        y=mid_y,
                        z=wall_height_m / 2.0,
                        size_x=wall_len,
                        size_y=wall_thickness_m,
                        size_z=wall_height_m,
                    )
                )
            else:
                seg_len = (wall_len - door_width_m) / 2.0
                for sign in (-1, 1):
                    seg_center_x = mid_x + sign * (door_width_m / 2.0 + seg_len / 2.0)
                    walls.append(
                        Box(
                            x=seg_center_x,
                            y=mid_y,
                            z=wall_height_m / 2.0,
                            size_x=seg_len,
                            size_y=wall_thickness_m,
                            size_z=wall_height_m,
                        )
                    )

    # Outer perimeter walls (the grid's outer boundary has no neighbor to share with).
    for i in range(rows):
        for j in range(cols):
            cx, cy = room_center(i, j)
            half = room_size_m / 2.0
            if j == 0:
                walls.append(
                    Box(
                        x=cx - half,
                        y=cy,
                        z=wall_height_m / 2.0,
                        size_x=wall_thickness_m,
                        size_y=room_size_m,
                        size_z=wall_height_m,
                    )
                )
            if j == cols - 1:
                walls.append(
                    Box(
                        x=cx + half,
                        y=cy,
                        z=wall_height_m / 2.0,
                        size_x=wall_thickness_m,
                        size_y=room_size_m,
                        size_z=wall_height_m,
                    )
                )
            if i == 0:
                walls.append(
                    Box(
                        x=cx,
                        y=cy - half,
                        z=wall_height_m / 2.0,
                        size_x=room_size_m,
                        size_y=wall_thickness_m,
                        size_z=wall_height_m,
                    )
                )
            if i == rows - 1:
                walls.append(
                    Box(
                        x=cx,
                        y=cy + half,
                        z=wall_height_m / 2.0,
                        size_x=room_size_m,
                        size_y=wall_thickness_m,
                        size_z=wall_height_m,
                    )
                )

    bounds = Bounds(
        min_x=origin_x - wall_thickness_m,
        min_y=origin_y - wall_thickness_m,
        min_z=0.0,
        max_x=origin_x + total_w + wall_thickness_m,
        max_y=origin_y + total_h + wall_thickness_m,
        max_z=wall_height_m + 1.0,
    )
    return walls, bounds


def generate(seed: int, split: WorldSplit, *, name: str | None = None) -> WorldSpec:
    rng = random.Random(seed)
    rows, cols = rng.randint(2, 3), rng.randint(2, 3)
    walls, bounds = build_floorplan(rng, rows=rows, cols=cols)

    spawn = sample_free_point(
        rng,
        min_x=bounds.min_x + 0.5,
        max_x=bounds.max_x - 0.5,
        min_y=bounds.min_y + 0.5,
        max_y=bounds.max_y - 0.5,
        z=0.1,
        boxes=walls,
        cylinders=[],
    )

    n_targets = rng.randint(3, 8)
    targets = []
    for _ in range(n_targets):
        t = sample_free_point(
            rng,
            min_x=bounds.min_x + 0.5,
            max_x=bounds.max_x - 0.5,
            min_y=bounds.min_y + 0.5,
            max_y=bounds.max_y - 0.5,
            z=1.0,
            boxes=walls,
            cylinders=[],
        )
        targets.append(Target(x=t.x, y=t.y, z=t.z))

    return WorldSpec(
        name=name or f"f2_office_{seed}",
        family=WorldFamily.OFFICE,
        split=split,
        seed=seed,
        bounds=bounds,
        boxes=tuple(walls),
        targets=tuple(targets),
        spawn_poses=(SpawnPose(x=spawn.x, y=spawn.y, z=spawn.z),),
    )

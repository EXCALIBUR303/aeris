"""Exploration coverage (spec §41, §51 Phase 12): ``C(t) = |E_GT(t) inter F| / |F|``.

``F`` is the reachable free-cell set within the search volume/altitude
band (GT) -- built from Phase 9's own voxelizer and 6-connected
reachability flood fill, projected down to 2D columns. ``E_GT(t)`` is
"cells that fell inside the agent's sensor footprint up to t, computed
by the evaluator via raycasting from GT poses" (spec's own wording,
§41): a full sequence of GT poses is swept, each contributing every 2D
cell an evaluator-side raycast against the *exact* ``WorldSpec``
geometry can see from it, and the running union up to each timestamp is
what's reported -- deliberately independent of the agent's own
(possibly wrong) map, since coverage measures what was actually
observable, not what the agent believes.

The raycast's field of view/max range default to the depth camera's own
configured intrinsics (``configs/vehicle/sensors.yaml``: 640x480,
fx=432.496 -> ~73.0 deg horizontal FOV) and Phase 11's mapping
``max_range_m`` default (15.0m) -- not arbitrary values, so this
evaluator-side measurement reflects what the real sensor could actually
have seen.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from aeris.core.frames.vector import Vec3
from aeris.mapping.projection import Cell2D
from aeris.simulation.worlds.occupancy import column_occupied_in_band, voxelize
from aeris.simulation.worlds.spec import WorldSpec

DEPTH_CAMERA_FOV_RAD = 2.0 * math.atan(640.0 / (2.0 * 432.49604203504293))
DEFAULT_MAX_RANGE_M = 15.0


def reachable_free_cells_gt(
    spec: WorldSpec, *, resolution_m: float, spawn_index: int = 0
) -> frozenset[Cell2D]:
    """``F``: the 2D columns, within ``spec``'s own altitude band, that
    are both GT-free-in-band and 6-connected-reachable from
    ``spec.spawn_poses[spawn_index]`` -- built from Phase 9's voxelizer
    and reachability flood fill (not re-implemented here), then
    projected down and re-indexed into the *plain* ``floor(world / res)``
    convention :func:`explored_cells_from_gt_pose` and
    :mod:`aeris.mapping.projection`'s ``BandGrid`` both use -- NOT
    :class:`~aeris.simulation.worlds.occupancy.OccupancyGrid`'s own
    bounds-relative indexing, which differs from it whenever
    ``spec.bounds.min_x``/``min_y`` isn't itself an exact multiple of
    ``resolution_m``. Comparing un-converted cells from the two
    conventions would silently intersect the wrong physical locations."""
    grid = voxelize(spec, resolution_m=resolution_m)
    alt_lo, alt_hi = spec.altitude_band_m
    n_x = math.ceil((spec.bounds.max_x - spec.bounds.min_x) / resolution_m)
    n_y = math.ceil((spec.bounds.max_y - spec.bounds.min_y) / resolution_m)
    min_cz = math.floor((alt_lo - grid.origin.z) / resolution_m)
    max_cz = math.ceil((alt_hi - grid.origin.z) / resolution_m)
    bounds_cells = ((0, 0, min_cz), (n_x - 1, n_y - 1, max_cz))

    spawn = spec.spawn_poses[spawn_index].position
    reachable_3d = grid.reachable_free_cells(spawn, bounds_cells=bounds_cells)
    result: set[Cell2D] = set()
    for cell in reachable_3d:
        center = grid.cell_center(cell)
        result.add((math.floor(center.x / resolution_m), math.floor(center.y / resolution_m)))
    return frozenset(result)


def _cast_ray(
    spec: WorldSpec,
    origin_xy: tuple[float, float],
    angle_rad: float,
    *,
    z_lo_m: float,
    z_hi_m: float,
    resolution_m: float,
    max_range_m: float,
) -> set[Cell2D]:
    x0, y0 = origin_xy
    dx, dy = math.cos(angle_rad), math.sin(angle_rad)
    n_steps = max(1, int(max_range_m / resolution_m))
    seen: set[Cell2D] = set()
    for step in range(n_steps + 1):
        r = step * resolution_m
        x, y = x0 + dx * r, y0 + dy * r
        cell = (math.floor(x / resolution_m), math.floor(y / resolution_m))
        if cell in seen:
            continue
        seen.add(cell)
        if column_occupied_in_band(
            spec, x, y, z_lo_m=z_lo_m, z_hi_m=z_hi_m, resolution_m=resolution_m
        ):
            break  # the near face of an obstacle is seen; nothing past it is
    return seen


def explored_cells_from_gt_pose(
    spec: WorldSpec,
    position_xy: tuple[float, float],
    yaw_rad: float,
    *,
    z_lo_m: float,
    z_hi_m: float,
    resolution_m: float,
    max_range_m: float = DEFAULT_MAX_RANGE_M,
    fov_rad: float = DEPTH_CAMERA_FOV_RAD,
    n_rays: int = 24,
) -> frozenset[Cell2D]:
    """The 2D cells visible from one GT pose, via an evaluator-side
    raycast against the exact ``WorldSpec`` geometry (not the agent's own
    map)."""
    seen: set[Cell2D] = set()
    half_fov = fov_rad / 2.0
    for i in range(n_rays):
        angle = yaw_rad - half_fov + (fov_rad * i / max(n_rays - 1, 1)) if n_rays > 1 else yaw_rad
        seen |= _cast_ray(
            spec,
            position_xy,
            angle,
            z_lo_m=z_lo_m,
            z_hi_m=z_hi_m,
            resolution_m=resolution_m,
            max_range_m=max_range_m,
        )
    return frozenset(seen)


def coverage(explored_cells: frozenset[Cell2D], reachable_free_cells: frozenset[Cell2D]) -> float:
    """``C(t) = |E_GT(t) inter F| / |F|`` (spec §41). Vacuously 1.0 when
    ``F`` is empty (nothing to cover)."""
    if not reachable_free_cells:
        return 1.0
    return len(explored_cells & reachable_free_cells) / len(reachable_free_cells)


def time_to_coverage_threshold(
    coverage_over_time: Sequence[tuple[float, float]], threshold: float
) -> float | None:
    """``T_c(x) = min{t : C(t) >= x}`` (spec §41). ``None`` if the
    threshold is never reached within the recorded trace -- the caller
    reports this as censored at whatever ``T_max`` the trace itself ran to."""
    for t_s, c in coverage_over_time:
        if c >= threshold:
            return t_s
    return None


def running_coverage_trace(
    spec: WorldSpec,
    gt_poses: Sequence[tuple[float, Vec3, float]],
    *,
    reachable_free_cells: frozenset[Cell2D],
    z_lo_m: float,
    z_hi_m: float,
    resolution_m: float,
    max_range_m: float = DEFAULT_MAX_RANGE_M,
    fov_rad: float = DEPTH_CAMERA_FOV_RAD,
    n_rays: int = 24,
) -> list[tuple[float, float]]:
    """``C(t)`` at every timestamp in ``gt_poses`` (each ``(t_sim_s,
    position, yaw_rad)``), as the running union of every pose's own
    visible cells up to and including that timestamp."""
    trace: list[tuple[float, float]] = []
    explored: set[Cell2D] = set()
    for t_s, position, yaw_rad in gt_poses:
        explored |= explored_cells_from_gt_pose(
            spec,
            (position.x, position.y),
            yaw_rad,
            z_lo_m=z_lo_m,
            z_hi_m=z_hi_m,
            resolution_m=resolution_m,
            max_range_m=max_range_m,
            fov_rad=fov_rad,
            n_rays=n_rays,
        )
        trace.append((t_s, coverage(frozenset(explored), reachable_free_cells)))
    return trace

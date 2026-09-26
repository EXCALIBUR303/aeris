"""Map evaluation vs ground truth, within observed regions (spec §21.3, §41).

Compares an agent-built :class:`~aeris.mapping.projection.BandGrid` (built
from live sensor data, never inflated -- see :func:`aeris.mapping.projection.project_band`'s
own docstring on why) against the exact :class:`~aeris.simulation.worlds.spec.WorldSpec`
geometry, sampled at the same cell resolution the agent's own map used, so
neither side is quantized more coarsely than the other.

``spawn_world`` matters here for the same reason it mattered in Phase 10:
``grid`` is built from a :class:`~aeris.mapping.voxel.VoxelMap` in frame
``M``/``O`` (spawn-relative, per Phase 10's live finding that PX4's EKF
origin sits at the arm/spawn position, not the Gazebo world origin), while
``spec``'s box/cylinder geometry is authored in the *world* frame. Checking
an ``M``-frame cell center directly against world-frame geometry silently
compares against the wrong location whenever the spawn isn't at/near world
(0, 0, 0) -- caught live in Phase 11 exactly the way Phase 10's original
version of this mistake was: every obstacle-suite world has a non-origin
spawn, and every occupied cell came back a false positive (0% precision,
every world, every pose-source condition) until this conversion was added.
"""

from __future__ import annotations

from dataclasses import dataclass

from aeris.core.frames.vector import ZERO, Vec3
from aeris.mapping.projection import BandGrid, CellState
from aeris.simulation.worlds.occupancy import is_occupied
from aeris.simulation.worlds.spec import WorldSpec


@dataclass(frozen=True, slots=True)
class MapEvaluationResult:
    occupied_precision: float
    occupied_recall: float
    free_false_occupied_rate: float
    map_coverage: float
    n_observed_cells: int
    n_total_cells: int


def _gt_column_occupied(
    spec: WorldSpec, x: float, y: float, *, z_lo_m: float, z_hi_m: float, resolution_m: float
) -> bool:
    """Whether *any* point in the queried altitude band at ``(x, y)`` is
    occupied per the exact ``WorldSpec`` geometry -- sampled at
    ``resolution_m`` steps, matching how the agent's own band projection
    decides "any voxel in-band occupied" (spec §21.1), so both sides use
    the same effective vertical granularity."""
    z = z_lo_m + resolution_m / 2.0
    while z <= z_hi_m:
        if is_occupied(spec, Vec3(x, y, z)):
            return True
        z += resolution_m
    return False


def evaluate_map(
    grid: BandGrid,
    spec: WorldSpec,
    *,
    z_lo_m: float,
    z_hi_m: float,
    spawn_world: Vec3 = ZERO,
) -> MapEvaluationResult:
    """spec §21.3: occupied-cell precision/recall, free-space false-occupied
    rate, and map coverage -- all computed only over cells the agent's map
    actually observed (``grid``'s ``inflation_m`` must be 0 for a fair
    comparison, per :func:`aeris.mapping.projection.project_band`'s own
    docstring).

    ``spawn_world`` converts ``grid``'s ``M``-frame cell centers to the
    world frame ``spec``'s geometry is authored in (see this module's own
    docstring) -- leave at the default only when ``grid`` is itself
    already world-frame (e.g. a spawn at/near world origin, or a
    synthetic test grid built directly in world coordinates).
    """
    tp = fp = fn = tn = 0
    observed = 0
    total = 0
    for cell in grid.cells_in_bounds():
        total += 1
        state = grid.state_at(cell)
        if state == CellState.UNKNOWN:
            continue
        observed += 1
        cx, cy = grid.cell_center_xy(cell)
        gt_occupied = _gt_column_occupied(
            spec,
            cx + spawn_world.x,
            cy + spawn_world.y,
            z_lo_m=z_lo_m,
            z_hi_m=z_hi_m,
            resolution_m=grid.resolution_m,
        )
        agent_occupied = state == CellState.OCCUPIED
        if agent_occupied and gt_occupied:
            tp += 1
        elif agent_occupied and not gt_occupied:
            fp += 1
        elif not agent_occupied and gt_occupied:
            fn += 1
        else:
            tn += 1

    # Precision/recall are vacuously 1.0 when their denominator is zero
    # (no positive predictions to be wrong about / no real positives to
    # miss) -- documented rather than raising, since an empty-of-that-kind
    # observed region is a legitimate (if uninformative) outcome, not an error.
    precision = tp / (tp + fp) if (tp + fp) else 1.0
    recall = tp / (tp + fn) if (tp + fn) else 1.0
    false_occupied_rate = fp / (fp + tn) if (fp + tn) else 0.0
    coverage = observed / total if total else 0.0
    return MapEvaluationResult(
        occupied_precision=precision,
        occupied_recall=recall,
        free_false_occupied_rate=false_occupied_rate,
        map_coverage=coverage,
        n_observed_cells=observed,
        n_total_cells=total,
    )

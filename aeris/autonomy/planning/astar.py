"""2D A* on a :class:`~aeris.mapping.projection.BandGrid` (spec §24, §51 Phase 12).

8-connected, octile heuristic (admissible for unit-orthogonal /
sqrt(2)-diagonal step costs), unknown-cell cost configurable via
``unknown_cost_multiplier``. ``OCCUPIED`` cells are never traversable --
the planner must never route through a cell the map calls occupied
(spec §43's "Planner collision" failure mode is prevented at the
*inflation* stage, in :func:`aeris.mapping.projection.project_band`'s own
``inflation_m``, not by giving A* a way to cut through obstacles).
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from aeris.mapping.projection import BandGrid, Cell2D, CellState

_NEIGHBORS: tuple[tuple[Cell2D, float], ...] = (
    ((1, 0), 1.0),
    ((-1, 0), 1.0),
    ((0, 1), 1.0),
    ((0, -1), 1.0),
    ((1, 1), math.sqrt(2)),
    ((1, -1), math.sqrt(2)),
    ((-1, 1), math.sqrt(2)),
    ((-1, -1), math.sqrt(2)),
)


@dataclass(frozen=True, slots=True)
class PlanResult:
    path_cells: tuple[Cell2D, ...]  # empty if no path; else start..goal inclusive
    path_length_m: float  # 0.0 if no path
    reason: str  # "ok" | "no_path" | "start_blocked" | "goal_blocked"


def _octile_heuristic(a: Cell2D, b: Cell2D) -> float:
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)


def astar(
    grid: BandGrid,
    start: Cell2D,
    goal: Cell2D,
    *,
    unknown_cost_multiplier: float = 1.0,
) -> PlanResult:
    """Shortest 8-connected path from ``start`` to ``goal`` in meters.

    ``unknown_cost_multiplier`` scales the step cost of entering an
    ``UNKNOWN`` cell relative to a ``FREE`` one (1.0 = no penalty, matching
    frontier exploration's own need to path *through* unknown space to
    reach a frontier; a caller wanting A* to avoid unknown space when a
    free detour exists can pass e.g. 1.5-2.0). ``OCCUPIED`` cells are
    always blocked, independent of this parameter.
    """
    if grid.state_at(start) == CellState.OCCUPIED:
        return PlanResult(path_cells=(), path_length_m=0.0, reason="start_blocked")
    if grid.state_at(goal) == CellState.OCCUPIED:
        return PlanResult(path_cells=(), path_length_m=0.0, reason="goal_blocked")
    if start == goal:
        return PlanResult(path_cells=(start,), path_length_m=0.0, reason="ok")

    res = grid.resolution_m
    open_heap: list[tuple[float, Cell2D]] = [(0.0, start)]
    g_score: dict[Cell2D, float] = {start: 0.0}
    came_from: dict[Cell2D, Cell2D] = {}
    closed: set[Cell2D] = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while path[-1] != start:
                path.append(came_from[path[-1]])
            path.reverse()
            return PlanResult(
                path_cells=tuple(path), path_length_m=g_score[goal] * res, reason="ok"
            )
        closed.add(current)

        for (dx, dy), step_cost in _NEIGHBORS:
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor in closed or not grid.in_bounds(neighbor):
                continue
            state = grid.state_at(neighbor)
            if state == CellState.OCCUPIED:
                continue
            cost = step_cost * (unknown_cost_multiplier if state == CellState.UNKNOWN else 1.0)
            tentative_g = g_score[current] + cost
            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f = tentative_g + _octile_heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f, neighbor))

    return PlanResult(path_cells=(), path_length_m=0.0, reason="no_path")

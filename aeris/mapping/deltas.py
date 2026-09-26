"""Map deltas for UI/replay streaming (spec §21.2's pipeline diagram: "...
-> map deltas (for UI/replay)").

V1 scope, documented: 2D band-grid cell-state deltas only, matching what
the grid already feeds downstream (planner, frontier detection, and any
future UI) -- not full 3D voxel-level deltas, which no consumer needs yet.
Revisit only once a later phase's frontend/replay work actually needs
3D-resolution streaming.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from aeris.mapping.projection import BandGrid, Cell2D, CellState


@dataclass(frozen=True, slots=True)
class MapDelta:
    t_sim_s: float
    changed: tuple[tuple[Cell2D, CellState], ...]


@dataclass(slots=True)
class DeltaTracker:
    """Tracks which cells changed state between successive :class:`BandGrid`
    snapshots. Stateful by design (spec: streaming deltas, not full
    snapshots) -- call :meth:`diff` once per tick with the latest grid."""

    _previous: dict[Cell2D, CellState] = field(default_factory=dict)

    def diff(self, t_sim_s: float, grid: BandGrid) -> MapDelta:
        changed: list[tuple[Cell2D, CellState]] = []
        current: dict[Cell2D, CellState] = {}
        for cell in grid.cells_in_bounds():
            state = grid.state_at(cell)
            current[cell] = state
            if self._previous.get(cell, CellState.UNKNOWN) != state:
                changed.append((cell, state))
        self._previous = current
        return MapDelta(t_sim_s, tuple(changed))

"""Unit tests for the pure-logic helpers in :mod:`aeris.evaluation.avoidance_episode`.

Most of that module is live-integration glue (the Sensor Bridge, a real
control loop) with nothing meaningful to unit-test in isolation --
covered instead by the live obstacle-suite experiment
(``scripts/run_p10_avoidance_experiment.py``, spec §51 Phase 10). This
file covers the one piece of real logic that doesn't need a live sim.
"""

from __future__ import annotations

from aeris.core.frames.vector import Vec3
from aeris.evaluation.avoidance_episode import _sector_clearances_for_reactive
from aeris.safety.shield import N_SECTORS, fov_sector_indices


def test_out_of_fov_sectors_become_zero_not_none() -> None:
    fov = fov_sector_indices(0.637)
    clearances = _sector_clearances_for_reactive([], fov_sectors=fov, max_range_m=15.0)
    assert len(clearances) == N_SECTORS
    for i, d in enumerate(clearances):
        if i in fov:
            assert d == 15.0
        else:
            assert d == 0.0


def test_in_fov_point_reduces_its_sector_clearance() -> None:
    fov = fov_sector_indices(0.637)
    points = [Vec3(2.0, 0.0, 0.0)]  # straight ahead, angle 0 -> sector 0
    clearances = _sector_clearances_for_reactive(points, fov_sectors=fov, max_range_m=15.0)
    assert clearances[0] == 2.0

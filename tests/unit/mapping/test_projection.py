"""Unit tests for :mod:`aeris.mapping.projection` -- band-aggregation rules
(occupied if any voxel in-band occupied, free if all observed free, else
unknown) and inflation."""

from __future__ import annotations

from aeris.core.frames.vector import Vec3
from aeris.mapping.projection import CellState, project_band
from aeris.mapping.voxel import MappingConfig, VoxelMap

_RES = 0.2


def _map() -> VoxelMap:
    return VoxelMap(config=MappingConfig(resolution_m=_RES))


def test_column_with_an_occupied_voxel_anywhere_in_band_is_occupied() -> None:
    m = _map()
    m.integrate_ray(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 1.5), is_hit=True)

    grid = project_band(m, z_lo_m=0.3, z_hi_m=3.0, x_range_m=(0.0, 2.0), y_range_m=(-1.0, 1.0))
    hit_cell = (int(1.0 / _RES), int(0.0 / _RES))
    assert grid.state_at(hit_cell) == CellState.OCCUPIED


def test_column_fully_observed_free_in_band_is_free() -> None:
    m = _map()
    ix, iy = int(1.0 / _RES), int(0.0 / _RES)
    min_iz, max_iz = int(0.3 / _RES), int(3.0 / _RES)
    for iz in range(min_iz, max_iz + 1):
        m._apply_index_delta(ix, iy, iz, m.config.l_free)  # every in-band voxel observed free

    grid = project_band(m, z_lo_m=0.3, z_hi_m=3.0, x_range_m=(0.0, 2.0), y_range_m=(-1.0, 1.0))
    assert grid.state_at((ix, iy)) == CellState.FREE


def test_column_never_observed_is_unknown() -> None:
    m = _map()
    grid = project_band(m, z_lo_m=0.3, z_hi_m=3.0, x_range_m=(0.0, 2.0), y_range_m=(-1.0, 1.0))
    assert grid.state_at((5, 0)) == CellState.UNKNOWN


def test_column_partially_observed_with_no_occupied_voxel_stays_unknown() -> None:
    """Spec: free requires *all* in-band voxels observed free -- a column
    with some observed-free and some never-touched voxels (but nothing
    occupied) is still unknown, not free."""
    m = _map()
    # Touch only the lowest voxel in the band as free; leave the rest of
    # the (0.3, 3.0) band at this column untouched.
    ix, iy, iz = int(1.0 / _RES), int(0.0 / _RES), int(0.3 / _RES)
    m._apply_index_delta(ix, iy, iz, m.config.l_free)

    grid = project_band(m, z_lo_m=0.3, z_hi_m=3.0, x_range_m=(0.0, 2.0), y_range_m=(-1.0, 1.0))
    assert grid.state_at((ix, iy)) == CellState.UNKNOWN


def test_inflation_dilates_occupied_into_adjacent_free_cells() -> None:
    m = _map()
    m.integrate_ray(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 1.5), is_hit=True)
    m.integrate_ray(Vec3(0.0, 0.0, 0.0), Vec3(1.0 + _RES, 0.0, 1.5), is_hit=False)

    grid = project_band(
        m,
        z_lo_m=0.3,
        z_hi_m=3.0,
        x_range_m=(0.0, 2.0),
        y_range_m=(-1.0, 1.0),
        inflation_m=_RES,
    )
    hit_cell = (int(1.0 / _RES), int(0.0 / _RES))
    neighbor = (hit_cell[0] + 1, hit_cell[1])
    assert grid.state_at(hit_cell) == CellState.OCCUPIED
    assert grid.state_at(neighbor) == CellState.OCCUPIED  # inflated, not free anymore


def test_no_inflation_by_default_leaves_neighbor_free() -> None:
    m = _map()
    m.integrate_ray(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 1.5), is_hit=True)
    hit_cell = (int(1.0 / _RES), int(0.0 / _RES))
    neighbor = (hit_cell[0] + 1, hit_cell[1])
    min_iz, max_iz = int(0.3 / _RES), int(3.0 / _RES)
    for iz in range(min_iz, max_iz + 1):
        m._apply_index_delta(neighbor[0], neighbor[1], iz, m.config.l_free)

    grid = project_band(m, z_lo_m=0.3, z_hi_m=3.0, x_range_m=(0.0, 2.0), y_range_m=(-1.0, 1.0))
    assert grid.state_at(hit_cell) == CellState.OCCUPIED
    assert grid.state_at(neighbor) == CellState.FREE

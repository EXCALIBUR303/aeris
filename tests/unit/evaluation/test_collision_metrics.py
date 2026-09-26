from __future__ import annotations

import pytest

from aeris.core.frames.vector import Vec3
from aeris.evaluation.metrics.collision import (
    GroundTruthGeometry,
    SphereObstacle,
    default_world_geometry,
    has_collision,
    minimum_clearance_m,
)


def test_default_world_geometry_has_no_obstacles() -> None:
    geometry = default_world_geometry()
    assert geometry.obstacles == ()
    assert geometry.ground_z == 0.0


def test_clearance_at_over_flat_ground_is_altitude() -> None:
    geometry = default_world_geometry()
    assert geometry.clearance_at(Vec3(0, 0, 3.0)) == pytest.approx(3.0)


def test_clearance_at_on_ground_is_zero() -> None:
    geometry = default_world_geometry()
    assert geometry.clearance_at(Vec3(5, 5, 0.0)) == pytest.approx(0.0)


def test_clearance_with_obstacle_hand_computed() -> None:
    # Obstacle at (10,0,3), radius 1m. Vehicle at (10,0,7): distance to
    # center = 4m, minus radius 1m = 3m clearance to the obstacle surface;
    # ground clearance at z=7 is 7m. min(3, 7) = 3.
    geometry = GroundTruthGeometry(
        ground_z=0.0, obstacles=(SphereObstacle(center=Vec3(10, 0, 3), radius_m=1.0),)
    )
    assert geometry.clearance_at(Vec3(10, 0, 7)) == pytest.approx(3.0)


def test_minimum_clearance_over_a_trajectory() -> None:
    geometry = default_world_geometry()
    positions = [Vec3(0, 0, 3.0), Vec3(0, 0, 1.0), Vec3(0, 0, 5.0)]
    assert minimum_clearance_m(positions, geometry) == pytest.approx(1.0)


def test_minimum_clearance_rejects_empty_positions() -> None:
    with pytest.raises(ValueError):
        minimum_clearance_m([], default_world_geometry())


def test_has_collision_true_when_clearance_below_vehicle_radius() -> None:
    geometry = default_world_geometry()
    positions = [Vec3(0, 0, 3.0), Vec3(0, 0, 0.2)]  # 0.2m clearance to ground
    assert has_collision(positions, geometry, vehicle_radius_m=0.5)


def test_has_collision_false_when_always_clear() -> None:
    geometry = default_world_geometry()
    positions = [Vec3(0, 0, 3.0), Vec3(0, 0, 2.0)]
    assert not has_collision(positions, geometry, vehicle_radius_m=0.5)


def test_has_collision_with_obstacle() -> None:
    geometry = GroundTruthGeometry(
        ground_z=0.0, obstacles=(SphereObstacle(center=Vec3(5, 0, 3), radius_m=1.0),)
    )
    # Vehicle passes right through the obstacle's center at one sample.
    positions = [Vec3(0, 0, 3), Vec3(5, 0, 3), Vec3(10, 0, 3)]
    assert has_collision(positions, geometry, vehicle_radius_m=0.5)

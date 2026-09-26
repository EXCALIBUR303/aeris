from __future__ import annotations

import pytest

from aeris.core.frames.vector import Vec3
from aeris.evaluation.metrics.collision import (
    BoxObstacle,
    CylinderObstacle,
    GroundTruthGeometry,
    SphereObstacle,
    default_world_geometry,
    from_world_spec,
    has_collision,
    minimum_clearance_m,
)
from aeris.simulation.worlds.spec import (
    Bounds,
    Box,
    Cylinder,
    SpawnPose,
    WorldFamily,
    WorldSpec,
    WorldSplit,
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


def test_box_obstacle_clearance_outside_along_one_axis() -> None:
    box = BoxObstacle(center=Vec3(0, 0, 1), half_extent=Vec3(1, 1, 1))
    # 3m along +x from the box's face (face is at x=1, point at x=4).
    assert box.clearance_at(Vec3(4, 0, 1)) == pytest.approx(3.0)


def test_box_obstacle_clearance_outside_at_a_corner() -> None:
    box = BoxObstacle(center=Vec3(0, 0, 0), half_extent=Vec3(1, 1, 1))
    # 3-4-5 triangle-ish: dx=3, dy=4 beyond the faces -> Euclidean 5.
    assert box.clearance_at(Vec3(4, 5, 0)) == pytest.approx(5.0)


def test_box_obstacle_clearance_inside_is_negative() -> None:
    box = BoxObstacle(center=Vec3(0, 0, 0), half_extent=Vec3(2, 2, 2))
    assert box.clearance_at(Vec3(0, 0, 0)) == pytest.approx(-2.0)


def test_cylinder_obstacle_clearance_radially_outside() -> None:
    cyl = CylinderObstacle(center=Vec3(0, 0, 1), radius_m=1.0, height_m=2.0)
    assert cyl.clearance_at(Vec3(3, 0, 1)) == pytest.approx(2.0)


def test_cylinder_obstacle_clearance_inside_is_negative() -> None:
    cyl = CylinderObstacle(center=Vec3(0, 0, 1), radius_m=1.0, height_m=2.0)
    assert cyl.clearance_at(Vec3(0, 0, 1)) == pytest.approx(-1.0)


def test_from_world_spec_converts_boxes_and_cylinders() -> None:
    spec = WorldSpec(
        name="test",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TRAIN,
        seed=0,
        bounds=Bounds(min_x=-10, min_y=-10, min_z=-100, max_x=10, max_y=10),
        boxes=(Box(x=5, y=0, z=1, size_x=2, size_y=2, size_z=2),),
        cylinders=(Cylinder(x=-5, y=0, z=1, radius_m=1.0, height_m=2.0),),
        spawn_poses=(SpawnPose(x=0, y=0, z=0.1),),
    )
    geometry = from_world_spec(spec)
    assert len(geometry.obstacles) == 2
    # Box spans x in [4, 6] (center 5, half-extent 1), z in [0, 2]. Point at
    # (9, 0, 1) is 3m past the box's x-face and within its y/z extent, so
    # box clearance is exactly 3m; ground_z=-100 makes ground never bind.
    assert geometry.clearance_at(Vec3(9, 0, 1)) == pytest.approx(3.0)

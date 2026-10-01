"""Unit tests for FastSim's world voxelization and raycaster: equivalence
with Phase 9's evaluator voxelizer, and the raycaster against brute-force
marching plus analytic cases (spec's own testing line: "raycast against
brute force")."""

from __future__ import annotations

import math

import numpy as np
import pytest

from aeris.perception.depth.projection import CameraIntrinsics
from aeris.simulation.fastsim.sensors import (
    camera_ray_dirs,
    cast_rays_batch,
    lidar_ray_dirs,
    ranges_to_zdepth,
)
from aeris.simulation.fastsim.world import WorldBank, build_world
from aeris.simulation.worlds.occupancy import voxelize
from aeris.simulation.worlds.spec import (
    Bounds,
    Box,
    Cylinder,
    SpawnPose,
    WorldFamily,
    WorldSpec,
    WorldSplit,
)


def _world(**kw: object) -> WorldSpec:
    base: dict[str, object] = {
        "name": "fastsim_test",
        "family": WorldFamily.RUBBLE,
        "split": WorldSplit.TRAIN,
        "seed": 0,
        "bounds": Bounds(min_x=-4.0, min_y=-4.0, max_x=4.0, max_y=4.0, max_z=3.0),
        "altitude_band_m": (0.3, 2.5),
        "boxes": (),
        "spawn_poses": (SpawnPose(x=0.0, y=0.0, z=0.1),),
    }
    base.update(kw)
    return WorldSpec(**base)  # type: ignore[arg-type]


def _bank(spec: WorldSpec, res: float = 0.1) -> WorldBank:
    return WorldBank.from_worlds([build_world(spec, resolution_m=res)])


def _cast(
    bank: WorldBank,
    pos: tuple[float, float, float],
    d: tuple[float, float, float],
    max_range: float = 20.0,
) -> float:
    dv = np.array([d], dtype=np.float64)
    dv /= np.linalg.norm(dv, axis=1, keepdims=True)
    out = cast_rays_batch(
        bank.occ,
        bank.origins,
        bank.dims,
        bank.ground_z,
        bank.resolution_m,
        np.array([0], dtype=np.int64),
        np.array([pos], dtype=np.float64),
        np.eye(3)[None, :, :].copy(),
        dv,
        max_range,
    )
    return float(out[0, 0])


def _brute(bank: WorldBank, pos: np.ndarray, d: np.ndarray, max_range: float = 20.0) -> float:
    w = bank.worlds[0]
    step = 0.002
    t = 0.0
    while t <= max_range:
        p = pos + d * t
        if w.is_occupied(p):
            return t
        t += step
    return math.inf


def test_voxelization_matches_phase9_evaluator_voxelizer_including_rotated_boxes() -> None:
    spec = _world(
        boxes=(
            Box(x=1.0, y=0.5, z=1.0, size_x=1.2, size_y=0.4, size_z=2.0, yaw_rad=0.7),
            Box(
                x=-2.0,
                y=-1.0,
                z=0.8,
                size_x=1.5,
                size_y=1.0,
                size_z=0.3,
                roll_rad=0.3,
                pitch_rad=-0.2,
            ),
        ),
        cylinders=(Cylinder(x=2.5, y=-2.0, z=1.0, radius_m=0.4, height_m=2.0),),
    )
    res = 0.25
    fast = build_world(spec, resolution_m=res)
    slow = voxelize(spec, resolution_m=res)
    fast_cells = {tuple(int(v) for v in c) for c in np.argwhere(fast.occ)}
    assert fast.occ.shape == (32, 32, 12)
    assert fast_cells == set(slow.occupied)
    assert len(fast_cells) > 50


def test_ray_hits_wall_at_its_near_face() -> None:
    bank = _bank(_world(boxes=(Box(x=2.0, y=0.0, z=1.5, size_x=0.4, size_y=4.0, size_z=3.0),)))
    r = _cast(bank, (0.05, 0.05, 1.05), (1.0, 0.0, 0.0))
    # Near face at x = 1.8; voxel-quantized to within one 0.1m voxel.
    assert r == pytest.approx(1.8 - 0.05, abs=0.1)


def test_ray_hits_analytic_ground_plane_exactly() -> None:
    bank = _bank(_world())
    r = _cast(bank, (0.0, 0.0, 1.0), (1.0, 0.0, -1.0))
    assert r == pytest.approx(math.sqrt(2.0), abs=1e-9)


def test_ray_leaving_the_world_without_a_hit_is_inf() -> None:
    bank = _bank(_world())
    assert math.isinf(_cast(bank, (0.0, 0.0, 1.0), (1.0, 0.0, 0.2)))


def test_max_range_is_respected() -> None:
    bank = _bank(_world(boxes=(Box(x=3.0, y=0.0, z=1.5, size_x=0.4, size_y=4.0, size_z=3.0),)))
    assert math.isinf(_cast(bank, (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), max_range=2.0))


def test_raycaster_matches_brute_force_marching_on_random_rays() -> None:
    spec = _world(
        boxes=(
            Box(x=1.5, y=1.0, z=1.0, size_x=0.8, size_y=0.6, size_z=2.0, yaw_rad=0.4),
            Box(x=-1.5, y=-1.5, z=0.5, size_x=1.0, size_y=1.0, size_z=1.0),
        ),
        cylinders=(Cylinder(x=-1.0, y=2.0, z=1.0, radius_m=0.3, height_m=2.0),),
    )
    bank = _bank(spec)
    rng = np.random.default_rng(0)
    for _ in range(200):
        pos = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(0.5, 1.5)])
        d = rng.normal(size=3)
        d /= np.linalg.norm(d)
        fast = _cast(bank, tuple(pos), tuple(d))
        slow = _brute(bank, pos, d)
        if math.isinf(slow):
            assert math.isinf(fast)
        else:
            assert fast == pytest.approx(slow, abs=0.005)


def test_batch_uses_each_envs_own_world_and_pose() -> None:
    wall = _world(boxes=(Box(x=2.0, y=0.0, z=1.5, size_x=0.4, size_y=4.0, size_z=3.0),))
    empty = _world(name="empty")
    bank = WorldBank.from_worlds([build_world(wall), build_world(empty)])
    out = cast_rays_batch(
        bank.occ,
        bank.origins,
        bank.dims,
        bank.ground_z,
        bank.resolution_m,
        np.array([0, 1], dtype=np.int64),
        np.array([[0.05, 0.05, 1.05], [0.05, 0.05, 1.05]]),
        np.stack([np.eye(3), np.eye(3)]),
        np.array([[1.0, 0.0, 0.0]]),
        20.0,
    )
    assert out[0, 0] == pytest.approx(1.75, abs=0.1)
    assert math.isinf(out[1, 0])


def test_camera_rays_match_intrinsics_and_zdepth_conversion() -> None:
    intr = CameraIntrinsics(width=640, height=480, fx=432.5, fy=432.5, cx=320.0, cy=240.0)
    dirs, zf = camera_ray_dirs(intr, stride=8)
    assert dirs.shape == (60 * 80, 3)
    np.testing.assert_allclose(np.linalg.norm(dirs, axis=1), 1.0)
    # The pixel at the principal point looks straight down the optical axis.
    center = 30 * 80 + 40  # v=240 -> row 30, u=320 -> col 40
    np.testing.assert_allclose(dirs[center], [0.0, 0.0, 1.0], atol=1e-12)
    # A range of 2m along an off-axis ray is a z-depth of 2 * cos(angle).
    zd = ranges_to_zdepth(np.full(dirs.shape[0], 2.0), zf)
    assert zd[0] == pytest.approx(2.0 * dirs[0, 2])


def test_lidar_fan_is_horizontal_and_starts_forward() -> None:
    d = lidar_ray_dirs(8)
    np.testing.assert_allclose(d[0], [1.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(d[2], [0.0, 1.0, 0.0], atol=1e-12)
    assert np.all(d[:, 2] == 0.0)

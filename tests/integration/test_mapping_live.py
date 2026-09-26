"""Live "hover-and-yaw sweep in a known world" integration test (spec's own
Phase 11 testing line). Validates, against a real PX4 SITL + Gazebo
session:

  - map update latency p95 < 50ms per depth frame (spec's own validation
    gate number) -- timed around `map_ekf`'s integration, the production
    condition.
  - no mirrored/rotated frame artifact (spec's own gate item): a single
    known wall placed only to the *east* (+x, ENU) of the spawn point must
    end up mapped as occupied specifically east of the origin, not west,
    north, or south -- the single most direct live check that the M-frame
    pipeline (`T_M_B * T_B_S`) isn't silently flipping or swapping an axis.
"""

from __future__ import annotations

import asyncio
import statistics
from pathlib import Path

import pytest

from aeris.core.clock import WallClock
from aeris.core.frames.conventions import camera_optical_to_body_rotation
from aeris.core.frames.extrinsics import load_sensor_extrinsics
from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import Vec3
from aeris.evaluation.mapping_episode import run_mapping_episode
from aeris.mapping.projection import CellState, project_band
from aeris.perception.depth.projection import CameraIntrinsics
from aeris.safety.envelope import load_envelope
from aeris.safety.supervisor import SafetySupervisor
from aeris.simulation.bridge.launch import start_bridge
from aeris.simulation.groundtruth.service import GroundTruthService
from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.ports import instance_ports
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import Px4Layout
from aeris.simulation.worlds import sdf
from aeris.simulation.worlds.spec import Bounds, Box, SpawnPose, WorldFamily, WorldSpec, WorldSplit
from aeris.vehicle.interface import VehicleEndpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

pytestmark = pytest.mark.integration

_REPO_ROOT = Path(__file__).resolve().parents[2]
_INTRINSICS = CameraIntrinsics(
    width=640, height=480, fx=432.496042035043, fy=432.496042035043, cx=320.0, cy=240.0
)


def _single_wall_world() -> WorldSpec:
    """One wall, due east (+x, ENU) of the spawn point, nothing else --
    an unambiguous "known geometry" world for a mirrored/rotated frame
    check: any wall detected anywhere other than east of the origin means
    the pipeline flipped or swapped an axis somewhere."""
    return WorldSpec(
        name="test_mapping_frame_check",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TEST_ID,
        seed=0,
        bounds=Bounds(min_x=-3.0, min_y=-4.0, max_x=6.0, max_y=4.0, max_z=4.0),
        altitude_band_m=(0.3, 3.0),
        boxes=(Box(x=3.0, y=0.0, z=1.5, size_x=0.3, size_y=2.0, size_z=3.0),),
        spawn_poses=(SpawnPose(x=0.0, y=0.0, z=0.1),),
    )


def _t_body_optical(layout: Px4Layout) -> Transform:
    model_sdf_path = layout.models_dir / "x500_depth" / "model.sdf"
    t_base_depth = load_sensor_extrinsics(model_sdf_path, sensor_name="StereoOV7251")
    return t_base_depth.compose(Transform.from_rotation(camera_optical_to_body_rotation()))


def test_hover_and_yaw_sweep_maps_the_known_wall_without_mirroring(
    px4_layout: Px4Layout, tmp_path: Path
) -> None:
    world_spec = _single_wall_world()
    world_sdf_path = tmp_path / f"{world_spec.name}.sdf"
    world_sdf_path.write_text(sdf.render(world_spec))

    spawn = world_spec.spawn_poses[0]
    profile = SimulationProfile(
        name="p11_mapping_frame_check",
        world=world_spec.name,
        world_sdf_path=world_sdf_path,
        model="gz_x500_depth",
        spawn_pose=(spawn.x, spawn.y, spawn.z, 0.0, 0.0, spawn.yaw_rad),
    )

    launcher = SimulationLauncher(px4_layout, run_dir=tmp_path)
    bridge_process = None
    try:
        launcher.start(profile)
        bridge_process, bridge_endpoint = start_bridge(
            world=world_spec.name, model_instance="x500_depth_0", run_dir=tmp_path
        )

        envelope = load_envelope(str(_REPO_ROOT / "configs" / "vehicle" / "safety.yaml"))
        adapter = Px4MavlinkAdapter()
        supervisor = SafetySupervisor(adapter, envelope=envelope, clock=WallClock())
        ports = instance_ports(profile.instance)
        gt_service = GroundTruthService(world=world_spec.name)

        result = asyncio.run(
            run_mapping_episode(
                supervisor=supervisor,
                endpoint=VehicleEndpoint(host="127.0.0.1", port=ports.offboard_remote),
                bridge_endpoint=bridge_endpoint,
                camera_intrinsics=_INTRINSICS,
                t_body_optical=_t_body_optical(px4_layout),
                gt_service=gt_service,
                gt_model_name="x500_depth_0",
                spawn_world=Vec3(spawn.x, spawn.y, spawn.z),
                hover_altitude_m=1.0,
                # Long enough for a statistically meaningful p95 sample --
                # the loop's real period runs well over its nominal 100ms
                # (the GT-pose subprocess poll dominates), so a short sweep
                # yields too few frames (~22) for p95 to mean anything: 2
                # isolated GC/scheduling-jitter outliers out of 22 samples
                # single-handedly swung the computed p95 across the 50ms
                # line on consecutive live runs, even though 20/22 frames
                # were a comfortable 19-33ms -- confirmed live before
                # widening this window, not assumed.
                sweep_duration_s=45.0,
                yaw_rate_radps=0.45,  # >2 full 360-degree sweeps at this duration
                clock=WallClock(),
            )
        )
    finally:
        if bridge_process is not None:
            bridge_process.stop()
        launcher.stop()

    assert result.n_frames > 0, f"no depth frames integrated (reason={result.reason!r})"

    p95 = statistics.quantiles(result.frame_latencies_s, n=20)[18]
    print(f"\nmap_ekf integration latency: p95={p95 * 1000:.2f}ms, n={result.n_frames}")
    print("per-frame (ms):", [round(t * 1000, 1) for t in result.frame_latencies_s])
    assert p95 < 0.05, f"map update latency p95 ={p95 * 1000:.2f}ms exceeds the 50ms gate"

    grid = project_band(
        result.map_ekf,
        z_lo_m=0.3,
        z_hi_m=3.0,
        x_range_m=(-3.0, 6.0),
        y_range_m=(-4.0, 4.0),
    )
    occupied_xs = [grid.cell_center_xy(c)[0] for c in grid.occupied]
    assert occupied_xs, "the known east wall was never mapped as occupied at all"
    assert all(x > 0.5 for x in occupied_xs), (
        f"occupied cells found west of (or too close to) the origin: {sorted(occupied_xs)[:5]} "
        "-- this would indicate a mirrored/flipped frame somewhere in the M-frame pipeline"
    )
    assert max(occupied_xs) > 2.0, "no occupied cell found near the wall's actual x=3.0 position"

    for cell in grid.occupied:
        assert grid.state_at(cell) == CellState.OCCUPIED

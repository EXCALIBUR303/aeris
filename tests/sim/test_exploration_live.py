"""Live "exploration episodes" sim test (spec's own Phase 12 testing
line). A wiring/smoke test against a real PX4 SITL + Gazebo session: does
nearest-frontier exploration -- map building, subgoal selection, A*
planning, path following, the collision shield -- actually fly a
coherent episode end to end, without crashing, making real progress,
recording a GT trajectory, and keeping the vehicle inside the world's
altitude band the whole time? (The H0.1 validation-gate experiment itself,
with all three strategies across multiple worlds and repeats, is
:mod:`scripts.run_p12_exploration_experiment`, run separately and
reported in the phase report -- this test is the cheap, fast confirmation
that the pipeline itself works before spending that much live sim time.)
"""

from __future__ import annotations

import asyncio
from pathlib import Path

import pytest

from aeris.autonomy.exploration.frontier import NearestFrontierExploration
from aeris.core.clock import WallClock
from aeris.core.frames.conventions import camera_optical_to_body_rotation
from aeris.core.frames.extrinsics import load_sensor_extrinsics
from aeris.core.frames.transform import Transform
from aeris.evaluation.exploration_episode import run_exploration_episode
from aeris.mapping.projection import CellState
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

pytestmark = pytest.mark.sim

_REPO_ROOT = Path(__file__).resolve().parents[2]
_INTRINSICS = CameraIntrinsics(
    width=640, height=480, fx=432.496042035043, fy=432.496042035043, cx=320.0, cy=240.0
)


def _small_room_world() -> WorldSpec:
    """A modest room with one pillar -- enough real, varied frontier
    space for several genuine replanning decisions, and one real
    obstacle to exercise A*'s occupied-cell avoidance + the shield
    together, without the scale of a full obstacle-suite world."""
    return WorldSpec(
        name="test_exploration_smoke_room",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TEST_ID,
        seed=1,
        bounds=Bounds(min_x=-4.0, min_y=-4.0, max_x=4.0, max_y=4.0, max_z=4.0),
        altitude_band_m=(0.3, 2.0),
        boxes=(Box(x=1.5, y=0.0, z=1.0, size_x=0.4, size_y=0.4, size_z=2.0),),
        spawn_poses=(SpawnPose(x=0.0, y=0.0, z=0.1),),
    )


def _t_body_optical(layout: Px4Layout) -> Transform:
    model_sdf_path = layout.models_dir / "x500_depth" / "model.sdf"
    t_base_depth = load_sensor_extrinsics(model_sdf_path, sensor_name="StereoOV7251")
    return t_base_depth.compose(Transform.from_rotation(camera_optical_to_body_rotation()))


def test_nearest_frontier_flies_a_coherent_exploration_episode(
    px4_layout: Px4Layout, tmp_path: Path
) -> None:
    world_spec = _small_room_world()
    world_sdf_path = tmp_path / f"{world_spec.name}.sdf"
    world_sdf_path.write_text(sdf.render(world_spec))

    spawn = world_spec.spawn_poses[0]
    profile = SimulationProfile(
        name="p12_exploration_smoke",
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
            run_exploration_episode(
                supervisor=supervisor,
                endpoint=VehicleEndpoint(host="127.0.0.1", port=ports.offboard_remote),
                bridge_endpoint=bridge_endpoint,
                camera_intrinsics=_INTRINSICS,
                t_body_optical=_t_body_optical(px4_layout),
                gt_service=gt_service,
                gt_model_name="x500_depth_0",
                strategy=NearestFrontierExploration(),
                hover_altitude_m=1.0,
                z_lo_m=0.3,
                z_hi_m=2.0,
                x_range_m=(-4.0, 4.0),
                y_range_m=(-4.0, 4.0),
                timeout_s=75.0,
                clock=WallClock(),
            )
        )
    finally:
        if bridge_process is not None:
            bridge_process.stop()
        launcher.stop()

    assert result.reason in ("", "timeout"), f"episode failed early: reason={result.reason!r}"
    assert result.gt_trajectory, "no GT trajectory recorded at all"
    assert result.n_subgoals_chosen > 0, "the strategy never chose a real subgoal"
    # The vehicle actually stayed in the world's altitude band (GT z) --
    # Phase 12's version of this test passed without ever checking, and
    # its harness could score a vehicle sitting on the floor
    # (docs/exploration.md).
    validity = result.altitude_validity
    assert validity is not None
    assert validity.valid, validity.describe()

    n_observed = sum(
        1
        for cell in result.final_grid.cells_in_bounds()
        if result.final_grid.state_at(cell) != CellState.UNKNOWN
    )
    assert n_observed > 0, "the agent's own map never grew past 'nothing observed'"

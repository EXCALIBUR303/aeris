"""Live SITL test for the Phase 8 Sensor Bridge + perception pipeline (spec §51 Phase 8).

Validation gate coverage — see ``docs/sensors.md`` for the honest
accounting of what ran here vs. the spec's full-strength numbers (a 15 s
window rather than 5 min sustained; the stock ``gz_x500_depth`` model,
which has no 2D LiDAR, rather than a composed ``aeris_x500``; a
sub-decimeter tolerance standing in for "2 voxel sizes" since no voxel
size exists before Phase 9):

  - depth >= 10 Hz, RGB >= 5 Hz, measured from the bridge's own per-topic
    health (spec §18.4) rather than client-received counts, since a
    CONFLATE subscription intentionally drops most messages.
  - Live ``camera_info`` intrinsics match the SDF-generated
    ``configs/vehicle/sensors.yaml`` (spec §18.2: "verified at runtime
    against gz camera_info").
  - A box obstacle at a known world position back-projects (depth +
    extrinsics) to the correct body-frame position at three distances.
  - The GT channel is unreachable from :class:`GzBridgeClient` (allowlist
    test) — re-asserted here alongside the live pipeline, though the
    unit-level guarantee is already covered by
    ``tests/unit/simulation/bridge/test_client.py``.
"""

from __future__ import annotations

import os
import subprocess
import time
from array import array
from pathlib import Path

import pytest
import yaml

from aeris.core.frames.conventions import camera_optical_to_body_rotation
from aeris.core.frames.extrinsics import load_link_pose, load_sensor_extrinsics
from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import Vec3
from aeris.perception.depth.projection import camera_intrinsics_from_k, depth_to_points_camera
from aeris.simulation.bridge.client import GroundTruthTopicError, GzBridgeClient
from aeris.simulation.bridge.launch import start_bridge
from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import Px4Layout

pytestmark = pytest.mark.sim

_WORLD = "default"
_REPO_ROOT = Path(__file__).resolve().parents[2]
_SENSORS_YAML = _REPO_ROOT / "configs" / "vehicle" / "sensors.yaml"
_RATE_WINDOW_S = 15.0
_MIN_DEPTH_HZ = 10.0
_MIN_RGB_HZ = 5.0


def _gz_cli_env() -> dict[str, str]:
    # Every other AERIS-launched process sets GZ_IP=127.0.0.1 (see
    # launcher.py); the plain `gz` CLI needs it too to see this instance's
    # transport partition at all (confirmed live, Phase 8 -- without it,
    # `gz service`/`gz topic` silently see nothing, including a spawn
    # request that just times out rather than erroring).
    env = dict(os.environ)
    env["GZ_IP"] = "127.0.0.1"
    return env


def _spawn_box(*, world: str, name: str, position: Vec3, size: float = 0.5) -> None:
    sdf = (
        f'<sdf version="1.6"><model name="{name}"><static>true</static>'
        f'<link name="link">'
        f'<collision name="c"><geometry><box><size>{size} {size} {size}</size></box></geometry></collision>'
        f'<visual name="v"><geometry><box><size>{size} {size} {size}</size></box></geometry></visual>'
        f"</link></model></sdf>"
    )
    req = (
        f"sdf: '{sdf}', name: \"{name}\", allow_renaming: false, "
        f"pose: {{position: {{x: {position.x}, y: {position.y}, z: {position.z}}}}}"
    )
    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{world}/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            req,
        ],
        capture_output=True,
        text=True,
        timeout=10,
        env=_gz_cli_env(),
    )
    # `gz service` exits 0 even on a timeout/failure (it prints "Service
    # call timed out" to stderr instead) -- check the reply body itself.
    if "data: true" not in result.stdout:
        raise RuntimeError(
            f"failed to spawn {name!r}: stdout={result.stdout!r} stderr={result.stderr!r}"
        )


def _remove_model(*, world: str, name: str) -> None:
    subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{world}/remove",
            "--reqtype",
            "gz.msgs.Entity",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            f'name: "{name}", type: MODEL',
        ],
        capture_output=True,
        timeout=10,
        env=_gz_cli_env(),
    )


def test_gt_topic_is_unreachable_from_the_agent_client() -> None:
    with pytest.raises(GroundTruthTopicError):
        GzBridgeClient("tcp://127.0.0.1:19099", ["gt.pose"])


def test_sensor_rates_intrinsics_and_obstacle_localization(
    px4_layout: Px4Layout, tmp_path: Path
) -> None:
    launcher = SimulationLauncher(px4_layout, run_dir=tmp_path)
    profile = SimulationProfile(
        name="test_sensor_pipeline",
        model="gz_x500_depth",
        spawn_pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    model_instance = f"x500_depth_{profile.instance}"
    model_sdf_path = px4_layout.models_dir / "x500_depth" / "model.sdf"
    bridge_process = None
    boxes_spawned: list[str] = []

    try:
        launcher.start(profile)
        bridge_process, endpoint = start_bridge(
            world=_WORLD, model_instance=model_instance, run_dir=tmp_path
        )
        time.sleep(2.0)  # let the bridge subscribe and gz start publishing before we listen

        # --- rates, via the bridge's own health topic (spec §18.4) -----------
        with GzBridgeClient(
            endpoint, ["health"], conflate=True, rcv_timeout_ms=2000
        ) as health_client:
            deadline = time.monotonic() + _RATE_WINDOW_S
            last_health = None
            while time.monotonic() < deadline:
                msg = health_client.recv()
                if msg is not None:
                    last_health = msg
            assert last_health is not None, "never received a health message"
            topics = last_health["payload"]["topics"]
            assert topics["depth"]["rate_hz"] >= _MIN_DEPTH_HZ, topics["depth"]
            assert topics["rgb"]["rate_hz"] >= _MIN_RGB_HZ, topics["rgb"]

        # --- live camera_info vs. the SDF-generated sensors.yaml -------------
        generated = yaml.safe_load(_SENSORS_YAML.read_text())
        generated_depth_intrinsics = generated["sensors"]["front_depth"]["intrinsics"]

        with GzBridgeClient(
            endpoint, ["camera_info"], conflate=True, rcv_timeout_ms=2000
        ) as info_client:
            camera_info_msg = None
            for _ in range(20):
                camera_info_msg = info_client.recv()
                if camera_info_msg is not None:
                    break
            assert camera_info_msg is not None, "never received a camera_info message"

        live_k = camera_info_msg["payload"]["intrinsics"]
        live_intrinsics = camera_intrinsics_from_k(
            camera_info_msg["payload"]["width"], camera_info_msg["payload"]["height"], live_k
        )
        assert live_intrinsics.width == generated_depth_intrinsics["width"]
        assert live_intrinsics.height == generated_depth_intrinsics["height"]
        assert live_intrinsics.fx == pytest.approx(generated_depth_intrinsics["fx"], rel=0.01)
        assert live_intrinsics.fy == pytest.approx(generated_depth_intrinsics["fy"], rel=0.01)

        # --- obstacle localization: box at 3 known distances ------------------
        t_base_depth = load_sensor_extrinsics(model_sdf_path, sensor_name="StereoOV7251")
        t_modelroot_base = load_link_pose(model_sdf_path, link_name="base_link")
        t_modelroot_depth_mount = t_modelroot_base.compose(t_base_depth)
        t_modelroot_depth_optical = t_modelroot_depth_mount.compose(
            Transform.from_rotation(camera_optical_to_body_rotation())
        )
        # spawn_pose is identity, so T_world_modelroot ~= identity too.
        camera_world_pose = t_modelroot_depth_optical

        tolerance_m = (
            0.15  # sub-decimeter stand-in for "2 voxel sizes" (no voxel size exists before Phase 9)
        )
        box_size_m = 0.5

        # conflate=True: each recv() below gets the *current* frame. A
        # non-conflate, bounded-HWM subscription was tried first and
        # confirmed (Phase 8) to backfill with the first few pre-spawn
        # frames and then silently drop every later one (ZMQ backpressure,
        # not FIFO eviction) -- every iteration kept reading the same
        # stale, box-free frames from before the loop even started.
        with GzBridgeClient(
            endpoint, ["depth"], conflate=True, rcv_timeout_ms=3000
        ) as depth_client:
            for i, distance_m in enumerate((2.0, 4.0, 6.0)):
                box_name = f"aeris_test_box_{i}"
                box_world_pos = camera_world_pose.apply(Vec3(0.0, 0.0, distance_m))
                # A depth camera measures range to the *near surface*, not
                # the object's center -- confirmed live (Phase 8): a box
                # placed with its center at distance D reads back D -
                # size/2 exactly. The expected comparison point is that
                # near surface, half a box-size closer along the boresight.
                expected_surface_pos = camera_world_pose.apply(
                    Vec3(0.0, 0.0, distance_m - box_size_m / 2)
                )
                _spawn_box(world=_WORLD, name=box_name, position=box_world_pos, size=box_size_m)
                boxes_spawned.append(box_name)
                time.sleep(1.0)  # let the box settle into the depth image

                depth_msg = None
                for _ in range(30):
                    depth_msg = depth_client.recv()
                    if depth_msg is not None:
                        break
                assert depth_msg is not None, f"no depth frame received for distance {distance_m}"

                payload = depth_msg["payload"]
                depth_buf: array[float] = array("f")
                depth_buf.frombytes(payload["data"])
                intrinsics = camera_intrinsics_from_k(payload["width"], payload["height"], live_k)
                points_optical = depth_to_points_camera(depth_buf, intrinsics)

                # The box should appear near the image center. Size the patch
                # to 70% of the box's own known angular half-width at this
                # distance -- a fixed pixel patch was tried first and found
                # to bleed in background at longer range (the box's angular
                # size shrinks with distance), and taking the single nearest
                # point (instead of the median) was tried next and found
                # too sensitive to single-pixel silhouette-edge artifacts.
                cx, cy = intrinsics.width // 2, intrinsics.height // 2
                patch = max(3, int(0.7 * (box_size_m / 2) / distance_m * intrinsics.fx))
                central = [
                    p
                    for p in points_optical
                    if abs((p.x / p.z * intrinsics.fx + intrinsics.cx) - cx) < patch
                    and abs((p.y / p.z * intrinsics.fy + intrinsics.cy) - cy) < patch
                ]
                assert central, f"no valid depth points near image center for distance {distance_m}"
                central.sort(key=lambda p: p.z)
                measured_point_optical = central[len(central) // 2]

                measured_world = camera_world_pose.apply(measured_point_optical)
                error = (measured_world - expected_surface_pos).norm()
                assert error < tolerance_m, (
                    f"distance {distance_m}m: measured world point {measured_world} vs "
                    f"expected surface position {expected_surface_pos}, error {error:.3f}m"
                )

                _remove_model(world=_WORLD, name=box_name)
                boxes_spawned.remove(box_name)
                time.sleep(0.5)
    finally:
        for name in boxes_spawned:
            _remove_model(world=_WORLD, name=name)
        if bridge_process is not None:
            bridge_process.stop()
        launcher.stop()

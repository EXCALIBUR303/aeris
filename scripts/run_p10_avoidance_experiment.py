#!/usr/bin/env python3
"""AERIS Phase 10's first formal experiment (spec §51 Phase 10).

Runs the declared matrix in ``configs/experiments/p10_avoidance.yaml``
(obstacle suite x method x run), live against PX4 SITL + Gazebo, and
writes one JSON-lines record per episode to
``results/experiments/p10_avoidance/runs.jsonl``.

Usage::

    uv run python scripts/run_p10_avoidance_experiment.py [--n-runs N] [--scenario NAME] [--method NAME]
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import traceback
from pathlib import Path
from typing import Any

import yaml

from aeris.core.clock import WallClock
from aeris.core.errors import SimulationLaunchError
from aeris.core.frames.conventions import camera_optical_to_body_rotation
from aeris.core.frames.extrinsics import load_sensor_extrinsics
from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import Vec3
from aeris.evaluation.avoidance_episode import run_avoidance_episode
from aeris.evaluation.metrics.collision import from_world_spec, has_collision, minimum_clearance_m
from aeris.evaluation.metrics.nav import control_smoothness, distance_travelled_m, path_efficiency
from aeris.perception.depth.projection import CameraIntrinsics
from aeris.safety.envelope import load_envelope
from aeris.safety.shield import ShieldConfig
from aeris.safety.supervisor import SafetySupervisor
from aeris.simulation.bridge.launch import start_bridge
from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.ports import instance_ports
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import Px4Layout, resolve_px4_layout
from aeris.simulation.worlds import sdf
from aeris.simulation.worlds.obstacle_suites import ALL_SUITES
from aeris.vehicle.interface import VehicleEndpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

_MAX_LAUNCH_ATTEMPTS = (
    3  # transient PX4/EKF boot flakiness is documented (Phase 3): ~1-in-7 to 1-in-31
)

_REPO_ROOT = Path(__file__).resolve().parents[1]
_CONFIG_PATH = _REPO_ROOT / "configs" / "experiments" / "p10_avoidance.yaml"
_OUT_DIR = _REPO_ROOT / "results" / "experiments" / "p10_avoidance"
_VEHICLE_RADIUS_M = 0.3

_INTRINSICS = CameraIntrinsics(
    width=640, height=480, fx=432.496042035043, fy=432.496042035043, cx=320.0, cy=240.0
)


def _t_body_optical(layout: Px4Layout) -> Transform:
    model_sdf_path = layout.models_dir / "x500_depth" / "model.sdf"
    t_base_depth = load_sensor_extrinsics(model_sdf_path, sensor_name="StereoOV7251")
    return t_base_depth.compose(Transform.from_rotation(camera_optical_to_body_rotation()))


async def _attempt_episode(
    *, scenario: str, method: str, goal: Vec3, config: dict[str, Any], run_dir: Path
) -> tuple[Any, Any]:
    """One launch + flight attempt. Raises on any launch/connection failure
    -- the caller (:func:`run_one_episode`) is the retry boundary, matching
    Phase 7's "retry-once on failure" pattern (spec §38.4), widened here to
    cover the launch step too, not just the flight itself: transient PX4/
    EKF boot flakiness is documented since Phase 3 (~1-in-7 to 1-in-31 live
    launches) and confirmed live during this phase's own experiment run."""
    layout = resolve_px4_layout()
    world_spec = ALL_SUITES[scenario]()
    world_sdf_path = run_dir / f"{world_spec.name}.sdf"
    world_sdf_path.write_text(sdf.render(world_spec))

    spawn = world_spec.spawn_poses[0]
    profile = SimulationProfile(
        name=f"p10_{scenario}_{method}",
        world=world_spec.name,
        world_sdf_path=world_sdf_path,
        model="gz_x500_depth",
        spawn_pose=(spawn.x, spawn.y, spawn.z, 0.0, 0.0, spawn.yaw_rad),
    )

    launcher = SimulationLauncher(layout, run_dir=run_dir)
    bridge_process = None
    try:
        launcher.start(profile)
        bridge_process, bridge_endpoint = start_bridge(
            world=world_spec.name, model_instance="x500_depth_0", run_dir=run_dir
        )
        await asyncio.sleep(2.0)

        envelope = load_envelope(str(_REPO_ROOT / "configs" / "vehicle" / "safety.yaml"))
        adapter = Px4MavlinkAdapter()
        supervisor = SafetySupervisor(adapter, envelope=envelope, clock=WallClock())
        ports = instance_ports(profile.instance)

        episode = await run_avoidance_episode(
            supervisor=supervisor,
            endpoint=VehicleEndpoint(host="127.0.0.1", port=ports.offboard_remote),
            bridge_endpoint=bridge_endpoint,
            camera_intrinsics=_INTRINSICS,
            t_body_optical=_t_body_optical(layout),
            goal_odom=goal,
            method=method,  # type: ignore[arg-type]
            takeoff_altitude_m=config["takeoff_altitude_m"],
            cruise_speed_mps=config["cruise_speed_mps"],
            timeout_s=config["episode_timeout_s"],
            shield_config=ShieldConfig(**config["shield"]),
            clock=WallClock(),
        )
    finally:
        if bridge_process is not None:
            bridge_process.stop()
        launcher.stop()

    return episode, world_spec


async def run_one_episode(
    *, scenario: str, method: str, goal: Vec3, config: dict[str, Any], run_dir: Path
) -> dict[str, Any]:
    episode = None
    world_spec = None
    last_error: str = ""
    for attempt in range(_MAX_LAUNCH_ATTEMPTS):
        try:
            episode, world_spec = await _attempt_episode(
                scenario=scenario, method=method, goal=goal, config=config, run_dir=run_dir
            )
            if episode.positions:
                break
            last_error = episode.reason or "no positions recorded"
        except SimulationLaunchError as exc:
            last_error = f"launch failed: {exc}"
        except Exception:
            last_error = f"unexpected error: {traceback.format_exc(limit=3)}"
        print(
            f"  [{scenario}/{method}] attempt {attempt} failed ({last_error}); retrying...",
            flush=True,
        )
        await asyncio.sleep(3.0)  # let any lingering socket/port state settle before retrying
    else:
        return {"scenario": scenario, "method": method, "status": "failed", "reason": last_error}

    assert episode is not None and world_spec is not None
    if not episode.positions:
        return {"scenario": scenario, "method": method, "status": "failed", "reason": last_error}
    spawn = world_spec.spawn_poses[0]

    # episode.positions are ODOM-frame (relative to the spawn/arm point --
    # see the goals comment in p10_avoidance.yaml); the WorldSpec's own
    # obstacle geometry is WORLD-frame. Convert before checking GT
    # collision/clearance against it -- confirmed live (Phase 10) that
    # conflating the two frames here silently checked collision against
    # the wrong region of the world entirely.
    spawn_world = Vec3(spawn.x, spawn.y, spawn.z)
    positions_world = [spawn_world + p for p in episode.positions]
    straight_line_m = goal.norm()  # goal is already odom-relative to spawn
    d_m = distance_travelled_m(list(episode.positions))
    geometry = from_world_spec(world_spec)
    d_min = minimum_clearance_m(positions_world, geometry)
    collided = has_collision(positions_world, geometry, vehicle_radius_m=_VEHICLE_RADIUS_M)
    smoothness = (
        control_smoothness(list(episode.velocities), dt_s=0.1)
        if len(episode.velocities) >= 2
        else 0.0
    )
    eta = path_efficiency(shortest_path_m=straight_line_m, distance_travelled_m=d_m)

    return {
        "scenario": scenario,
        "method": method,
        "status": "completed",
        "arrived": episode.arrived,
        "collided": collided,
        "d_min_m": d_min,
        "distance_travelled_m": d_m,
        "path_efficiency": eta,
        "control_smoothness": smoothness,
        "shield_intervention_rate": episode.shield_intervention_rate,
        "shield_tick_count": episode.shield_tick_count,
        "wall_time_s": episode.wall_time_s,
        "n_positions": len(episode.positions),
        "reason": episode.reason,
    }


async def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--n-runs", type=int, default=None)
    parser.add_argument("--scenario", default=None)
    parser.add_argument("--method", default=None)
    parser.add_argument("--out", default=str(_OUT_DIR / "runs.jsonl"))
    args = parser.parse_args()

    config = yaml.safe_load(_CONFIG_PATH.read_text())
    n_runs = args.n_runs or config["n_runs"]
    scenarios = [args.scenario] if args.scenario else config["scenarios"]
    methods = [args.method] if args.method else config["methods"]
    goals = {name: Vec3(*coords) for name, coords in config["goals"].items()}

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    run_dir_base = _REPO_ROOT / "results" / "experiments" / "p10_avoidance" / "run_logs"
    total = len(scenarios) * len(methods) * n_runs
    done = 0

    with out_path.open("a") as f:
        for scenario in scenarios:
            for method in methods:
                for run_idx in range(n_runs):
                    done += 1
                    print(f"[{done}/{total}] {scenario} / {method} / run {run_idx}", flush=True)
                    run_dir = run_dir_base / f"{scenario}_{method}_{run_idx}"
                    run_dir.mkdir(parents=True, exist_ok=True)
                    record = await run_one_episode(
                        scenario=scenario,
                        method=method,
                        goal=goals[scenario],
                        config=config,
                        run_dir=run_dir,
                    )
                    record["run_idx"] = run_idx
                    f.write(json.dumps(record) + "\n")
                    f.flush()
                    print(
                        f"  -> {record.get('status')} arrived={record.get('arrived')} collided={record.get('collided')}",
                        flush=True,
                    )

    print(f"Wrote {total} records to {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))

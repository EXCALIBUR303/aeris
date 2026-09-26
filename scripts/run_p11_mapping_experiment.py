#!/usr/bin/env python3
"""AERIS Phase 11's map-accuracy experiment (spec §51 Phase 11).

Runs one live "hover-and-yaw sweep" episode per world declared in
``configs/experiments/p11_mapping.yaml``, builds all three pose-source
conditions (GT / PX4 EKF / noisy-EKF) from that one flight's depth stream,
evaluates each against the world's own ground-truth geometry, and writes
one JSON-lines record per world to
``results/experiments/p11_mapping/runs.jsonl``.

Usage::

    uv run python scripts/run_p11_mapping_experiment.py [--scenario NAME]
"""

from __future__ import annotations

import argparse
import asyncio
import json
import statistics
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
from aeris.evaluation.mapping_episode import MappingEpisodeResult, run_mapping_episode
from aeris.evaluation.metrics.localization import ate, rpe
from aeris.evaluation.metrics.map import MapEvaluationResult, evaluate_map
from aeris.mapping.projection import project_band
from aeris.mapping.voxel import MappingConfig, VoxelMap
from aeris.perception.depth.projection import CameraIntrinsics
from aeris.safety.envelope import load_envelope
from aeris.safety.supervisor import SafetySupervisor
from aeris.simulation.bridge.launch import start_bridge
from aeris.simulation.groundtruth.service import GroundTruthService
from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.ports import instance_ports
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import Px4Layout, resolve_px4_layout
from aeris.simulation.worlds import sdf
from aeris.simulation.worlds.obstacle_suites import ALL_SUITES
from aeris.simulation.worlds.spec import WorldSpec
from aeris.vehicle.interface import VehicleEndpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

_MAX_LAUNCH_ATTEMPTS = (
    3  # transient PX4/EKF boot flakiness is documented (Phase 3): ~1-in-7 to 1-in-31
)

_REPO_ROOT = Path(__file__).resolve().parents[1]
_CONFIG_PATH = _REPO_ROOT / "configs" / "experiments" / "p11_mapping.yaml"
_OUT_DIR = _REPO_ROOT / "results" / "experiments" / "p11_mapping"

_INTRINSICS = CameraIntrinsics(
    width=640, height=480, fx=432.496042035043, fy=432.496042035043, cx=320.0, cy=240.0
)


def _t_body_optical(layout: Px4Layout) -> Transform:
    model_sdf_path = layout.models_dir / "x500_depth" / "model.sdf"
    t_base_depth = load_sensor_extrinsics(model_sdf_path, sensor_name="StereoOV7251")
    return t_base_depth.compose(Transform.from_rotation(camera_optical_to_body_rotation()))


async def _attempt_episode(
    *, scenario: str, config: dict[str, Any], run_dir: Path
) -> tuple[MappingEpisodeResult, WorldSpec]:
    """One launch + flight attempt. Raises on any launch/connection failure
    -- the caller (:func:`run_one_scenario`) is the retry boundary,
    matching Phase 10's identical pattern for the same documented
    transient PX4/EKF boot flakiness."""
    layout = resolve_px4_layout()
    world_spec = ALL_SUITES[scenario]()
    world_sdf_path = run_dir / f"{world_spec.name}.sdf"
    world_sdf_path.write_text(sdf.render(world_spec))

    spawn = world_spec.spawn_poses[0]
    profile = SimulationProfile(
        name=f"p11_{scenario}",
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
        gt_service = GroundTruthService(world=world_spec.name)

        mapping_cfg = MappingConfig(**config["mapping"])
        episode = await run_mapping_episode(
            supervisor=supervisor,
            endpoint=VehicleEndpoint(host="127.0.0.1", port=ports.offboard_remote),
            bridge_endpoint=bridge_endpoint,
            camera_intrinsics=_INTRINSICS,
            t_body_optical=_t_body_optical(layout),
            gt_service=gt_service,
            gt_model_name="x500_depth_0",
            spawn_world=Vec3(spawn.x, spawn.y, spawn.z),
            hover_altitude_m=config["hover_altitude_m"],
            sweep_duration_s=config["sweep_duration_s"],
            yaw_rate_radps=config["yaw_rate_radps"],
            mapping_config=mapping_cfg,
            noise_seed=config["noise_seed"],
            noisy_walk_step_m=config["noisy_walk_step_m"],
            noisy_noise_std_m=config["noisy_noise_std_m"],
            clock=WallClock(),
        )
    finally:
        if bridge_process is not None:
            bridge_process.stop()
        launcher.stop()

    return episode, world_spec


def _condition_metrics(
    voxel_map: VoxelMap,
    world_spec: WorldSpec,
    *,
    estimated_positions_m: tuple[Vec3, ...],
    gt_positions_world: tuple[Vec3, ...],
    spawn_world: Vec3,
) -> dict[str, Any]:
    z_lo, z_hi = world_spec.altitude_band_m
    # voxel_map is in frame M/O (spawn-relative, per Phase 10's live
    # finding); world_spec.bounds is in the world frame. The queried
    # window must be M-frame to actually cover where the voxel data is,
    # and evaluate_map needs spawn_world to convert its M-frame cell
    # centers back to world frame before checking GT geometry -- both
    # halves of the same Phase-10-style frame bug, caught live here too.
    grid = project_band(
        voxel_map,
        z_lo_m=z_lo,
        z_hi_m=z_hi,
        x_range_m=(
            world_spec.bounds.min_x - spawn_world.x,
            world_spec.bounds.max_x - spawn_world.x,
        ),
        y_range_m=(
            world_spec.bounds.min_y - spawn_world.y,
            world_spec.bounds.max_y - spawn_world.y,
        ),
    )
    result: MapEvaluationResult = evaluate_map(
        grid, world_spec, z_lo_m=z_lo, z_hi_m=z_hi, spawn_world=spawn_world
    )

    gt_positions_m = tuple(p - spawn_world for p in gt_positions_world)
    n = min(len(estimated_positions_m), len(gt_positions_m))
    ate_m = ate(estimated_positions_m[:n], gt_positions_m[:n]) if n > 0 else None
    try:
        rpe_m = rpe(estimated_positions_m[:n], gt_positions_m[:n]) if n > 0 else None
    except ValueError:
        rpe_m = None  # trajectory shorter than one 10m segment -- not a failure

    return {
        "occupied_precision": result.occupied_precision,
        "occupied_recall": result.occupied_recall,
        "free_false_occupied_rate": result.free_false_occupied_rate,
        "map_coverage": result.map_coverage,
        "n_observed_cells": result.n_observed_cells,
        "n_total_cells": result.n_total_cells,
        "ate_m": ate_m,
        "rpe_m": rpe_m,
    }


async def run_one_scenario(
    *, scenario: str, config: dict[str, Any], run_dir: Path
) -> dict[str, Any]:
    episode: MappingEpisodeResult | None = None
    world_spec: WorldSpec | None = None
    last_error = ""
    for attempt in range(_MAX_LAUNCH_ATTEMPTS):
        try:
            episode, world_spec = await _attempt_episode(
                scenario=scenario, config=config, run_dir=run_dir
            )
            if episode.n_frames > 0:
                break
            last_error = episode.reason or "no frames integrated"
        except SimulationLaunchError as exc:
            last_error = f"launch failed: {exc}"
        except Exception:
            last_error = f"unexpected error: {traceback.format_exc(limit=3)}"
        print(f"  [{scenario}] attempt {attempt} failed ({last_error}); retrying...", flush=True)
        await asyncio.sleep(3.0)
    else:
        return {"scenario": scenario, "status": "failed", "reason": last_error}

    assert episode is not None and world_spec is not None
    if episode.n_frames == 0:
        return {"scenario": scenario, "status": "failed", "reason": last_error}

    spawn = world_spec.spawn_poses[0]
    spawn_world = Vec3(spawn.x, spawn.y, spawn.z)

    conditions = {
        "gt": _condition_metrics(
            episode.map_gt,
            world_spec,
            estimated_positions_m=tuple(p - spawn_world for p in episode.positions_gt_world),
            gt_positions_world=episode.positions_gt_world,
            spawn_world=spawn_world,
        ),
        "ekf": _condition_metrics(
            episode.map_ekf,
            world_spec,
            estimated_positions_m=episode.positions_odom,
            gt_positions_world=episode.positions_gt_world,
            spawn_world=spawn_world,
        ),
        "noisy": _condition_metrics(
            episode.map_noisy,
            world_spec,
            estimated_positions_m=episode.positions_noisy,
            gt_positions_world=episode.positions_gt_world,
            spawn_world=spawn_world,
        ),
    }

    latencies = list(episode.frame_latencies_s)
    if len(latencies) >= 20:
        latency_p95_ms = statistics.quantiles(latencies, n=20)[18] * 1000
    elif latencies:
        latency_p95_ms = max(latencies) * 1000  # too few samples for a real p95 -- report the max
    else:
        latency_p95_ms = None

    return {
        "scenario": scenario,
        "status": "completed",
        "n_frames": episode.n_frames,
        "latency_p95_ms": latency_p95_ms,
        "conditions": conditions,
        "reason": episode.reason,
    }


async def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default=None)
    parser.add_argument("--out", default=str(_OUT_DIR / "runs.jsonl"))
    args = parser.parse_args()

    config = yaml.safe_load(_CONFIG_PATH.read_text())
    scenarios = [args.scenario] if args.scenario else config["scenarios"]

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    run_dir_base = _REPO_ROOT / "results" / "experiments" / "p11_mapping" / "run_logs"
    total = len(scenarios)
    done = 0

    with out_path.open("a") as f:
        for scenario in scenarios:
            done += 1
            print(f"[{done}/{total}] {scenario}", flush=True)
            run_dir = run_dir_base / scenario
            run_dir.mkdir(parents=True, exist_ok=True)
            record = await run_one_scenario(scenario=scenario, config=config, run_dir=run_dir)
            f.write(json.dumps(record) + "\n")
            f.flush()
            status = record["status"]
            print(f"  -> {status}", flush=True)

    print(f"Wrote {total} records to {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))

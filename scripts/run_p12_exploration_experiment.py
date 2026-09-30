#!/usr/bin/env python3
"""AERIS Phase 12's exploration experiment (spec §51 Phase 12, H0.1).

Runs one live exploration episode per (world, method, repeat) triple
declared in ``configs/experiments/p12_exploration.yaml``, computes each
episode's coverage trace post-hoc from its recorded GT trajectory
(``aeris.evaluation.metrics.coverage``), and writes one JSON-lines record
per episode to ``results/experiments/p12_exploration/runs.jsonl``.

Also supports ``--tune`` mode: a small grid search over utility-
frontier's ``lambda_turn`` on one val world, one repeat per value,
written to ``results/experiments/p12_exploration/tuning.jsonl`` -- run
this first and copy the winning value into the main config's
``utility_lambda_turn`` before the full comparison run.

Usage::

    uv run python scripts/run_p12_exploration_experiment.py --tune
    uv run python scripts/run_p12_exploration_experiment.py
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

from aeris.autonomy.exploration.base import ExplorationStrategy
from aeris.autonomy.exploration.frontier import NearestFrontierExploration
from aeris.autonomy.exploration.random import RandomExploration
from aeris.autonomy.exploration.utility import UtilityFrontierExploration
from aeris.core.clock import WallClock
from aeris.core.errors import SimulationLaunchError
from aeris.core.frames.conventions import camera_optical_to_body_rotation
from aeris.core.frames.extrinsics import load_sensor_extrinsics
from aeris.core.frames.transform import Transform
from aeris.evaluation.exploration_episode import ExplorationEpisodeResult, run_exploration_episode
from aeris.evaluation.metrics.coverage import reachable_free_cells_gt, running_coverage_trace
from aeris.mapping.voxel import MappingConfig
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
from aeris.simulation.worlds.batch import generate_batch, load_world_spec
from aeris.simulation.worlds.spec import WorldFamily, WorldSpec, WorldSplit
from aeris.simulation.worlds.splits import SEED_RANGES
from aeris.vehicle.interface import VehicleEndpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

_MAX_LAUNCH_ATTEMPTS = 3  # documented PX4/EKF boot flakiness (Phase 3)

_REPO_ROOT = Path(__file__).resolve().parents[1]
_CONFIG_PATH = _REPO_ROOT / "configs" / "experiments" / "p12_exploration.yaml"
_OUT_DIR = _REPO_ROOT / "results" / "experiments" / "p12_exploration"
_WORLDS_DIR = _REPO_ROOT / "results" / "worlds"

_INTRINSICS = CameraIntrinsics(
    width=640, height=480, fx=432.496042035043, fy=432.496042035043, cx=320.0, cy=240.0
)


def _t_body_optical(layout: Px4Layout) -> Transform:
    model_sdf_path = layout.models_dir / "x500_depth" / "model.sdf"
    t_base_depth = load_sensor_extrinsics(model_sdf_path, sensor_name="StereoOV7251")
    return t_base_depth.compose(Transform.from_rotation(camera_optical_to_body_rotation()))


def _make_strategy(method: str, *, lambda_turn: float) -> ExplorationStrategy:
    if method == "random":
        import random as _random

        return RandomExploration(rng=_random.Random(20261001))
    if method == "nearest_frontier":
        return NearestFrontierExploration()
    if method == "utility_frontier":
        return UtilityFrontierExploration(lambda_turn=lambda_turn)
    raise ValueError(f"unknown method {method!r}")


def _ensure_world(family: WorldFamily, split: WorldSplit, seed: int) -> WorldSpec:
    """Loads (family, split, seed) if already generated, else generates
    it. ``generate_batch``'s own ``n`` means "the first n seeds of the
    split's range," not "up through seed n" -- since this experiment
    only ever wants that range's very first seed (``seed`` is expected to
    equal ``SEED_RANGES[split].start``), ``n=1`` is always correct here;
    this is not a general "generate an arbitrary seed" helper."""
    if seed != SEED_RANGES[split].start:
        raise ValueError(
            f"seed {seed} is not {split.value}'s range start ({SEED_RANGES[split].start}) -- "
            "_ensure_world only supports the split's first seed"
        )
    dest_dir = _WORLDS_DIR / family.value / split.value
    existing = sorted(dest_dir.glob(f"{family.value}_{seed}.json")) if dest_dir.exists() else []
    if existing:
        return load_world_spec(existing[0])
    generate_batch(family=family, split=split, n=1, out_dir=_WORLDS_DIR)
    return load_world_spec(dest_dir / f"{family.value}_{seed}.json")


async def _attempt_episode(
    *, world_spec: WorldSpec, method: str, lambda_turn: float, config: dict[str, Any], run_dir: Path
) -> ExplorationEpisodeResult:
    layout = resolve_px4_layout()
    world_sdf_path = run_dir / f"{world_spec.name}.sdf"
    world_sdf_path.write_text(sdf.render(world_spec))

    spawn = world_spec.spawn_poses[0]
    profile = SimulationProfile(
        name=f"p12_{world_spec.name}_{method}",
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

        half_w = config["x_range_half_width_m"]
        z_lo, z_hi = world_spec.altitude_band_m
        strategy = _make_strategy(method, lambda_turn=lambda_turn)

        episode = await run_exploration_episode(
            supervisor=supervisor,
            endpoint=VehicleEndpoint(host="127.0.0.1", port=ports.offboard_remote),
            bridge_endpoint=bridge_endpoint,
            camera_intrinsics=_INTRINSICS,
            t_body_optical=_t_body_optical(layout),
            gt_service=gt_service,
            gt_model_name="x500_depth_0",
            strategy=strategy,
            hover_altitude_m=config["hover_altitude_m"],
            z_lo_m=z_lo,
            z_hi_m=z_hi,
            x_range_m=(-half_w, half_w),
            y_range_m=(-half_w, half_w),
            timeout_s=config["t_max_s"],
            mapping_config=MappingConfig(**config["mapping"]),
            inflation_m=config["inflation_m"],
            clock=WallClock(),
        )
    finally:
        if bridge_process is not None:
            bridge_process.stop()
        launcher.stop()
    return episode


async def run_one_episode(
    *,
    world_spec: WorldSpec,
    method: str,
    lambda_turn: float,
    config: dict[str, Any],
    run_dir: Path,
    label: str,
) -> dict[str, Any]:
    episode: ExplorationEpisodeResult | None = None
    last_error = ""
    for attempt in range(_MAX_LAUNCH_ATTEMPTS):
        try:
            episode = await _attempt_episode(
                world_spec=world_spec,
                method=method,
                lambda_turn=lambda_turn,
                config=config,
                run_dir=run_dir,
            )
            if episode.gt_trajectory:
                break
            last_error = episode.reason or "no GT trajectory recorded"
        except SimulationLaunchError as exc:
            last_error = f"launch failed: {exc}"
        except Exception:
            last_error = f"unexpected error: {traceback.format_exc(limit=3)}"
        print(f"  [{label}] attempt {attempt} failed ({last_error}); retrying...", flush=True)
        await asyncio.sleep(3.0)
    else:
        return {
            "label": label,
            "world": world_spec.name,
            "method": method,
            "status": "failed",
            "reason": last_error,
        }

    assert episode is not None
    if not episode.gt_trajectory:
        return {
            "label": label,
            "world": world_spec.name,
            "method": method,
            "status": "failed",
            "reason": last_error,
        }

    z_lo, z_hi = world_spec.altitude_band_m
    # Deliberately config["coverage_resolution_m"], not the agent's own
    # (finer) mapping.resolution_m -- see the config file's own comment
    # for the live-timed 15.6x cost difference this avoids. Both calls
    # below must use the same value for their cell indices to be
    # comparable at all.
    resolution_m = config["coverage_resolution_m"]
    reachable = reachable_free_cells_gt(world_spec, resolution_m=resolution_m)
    trace = running_coverage_trace(
        world_spec,
        episode.gt_trajectory,
        reachable_free_cells=reachable,
        z_lo_m=z_lo,
        z_hi_m=z_hi,
        resolution_m=resolution_m,
    )
    c_final = trace[-1][1] if trace else 0.0
    t_final_s = trace[-1][0] if trace else 0.0

    return {
        "label": label,
        "world": world_spec.name,
        "method": method,
        "status": "completed",
        "n_gt_poses": len(episode.gt_trajectory),
        "n_reachable_free_cells": len(reachable),
        "coverage_final": c_final,
        "t_final_s": t_final_s,
        "n_subgoals_chosen": episode.n_subgoals_chosen,
        "n_rotate_scans": episode.n_rotate_scans,
        "shield_intervention_rate": episode.shield_intervention_rate,
        "wall_time_s": episode.wall_time_s,
        "reason": episode.reason,
    }


async def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tune", action="store_true")
    parser.add_argument(
        "--tune-values", default="0.0,0.5,1.0,2.0", help="comma-separated lambda_turn candidates"
    )
    args = parser.parse_args()

    config = yaml.safe_load(_CONFIG_PATH.read_text())
    families = [WorldFamily(f) for f in config["world_families"]]
    split = WorldSplit(config["world_split"])
    seed = config["seed_per_family"]

    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    run_dir_base = _OUT_DIR / "run_logs"

    if args.tune:
        world_spec = _ensure_world(families[0], split, seed)
        values = [float(v) for v in args.tune_values.split(",")]
        out_path = _OUT_DIR / "tuning.jsonl"
        with out_path.open("a") as f:
            for i, lam in enumerate(values):
                label = f"tune_lambda_{lam}"
                print(f"[{i + 1}/{len(values)}] lambda_turn={lam}", flush=True)
                run_dir = run_dir_base / label
                run_dir.mkdir(parents=True, exist_ok=True)
                record = await run_one_episode(
                    world_spec=world_spec,
                    method="utility_frontier",
                    lambda_turn=lam,
                    config=config,
                    run_dir=run_dir,
                    label=label,
                )
                record["lambda_turn"] = lam
                f.write(json.dumps(record) + "\n")
                f.flush()
                print(
                    f"  -> {record['status']} coverage_final={record.get('coverage_final')}",
                    flush=True,
                )
        print(f"Wrote {len(values)} tuning records to {out_path}")
        return 0

    world_specs = [_ensure_world(family, split, seed) for family in families]
    lambda_turn = config["utility_lambda_turn"]
    if lambda_turn is None:
        print("ERROR: utility_lambda_turn is null in config -- run --tune first.", file=sys.stderr)
        return 1

    methods = config["methods"]
    n_repeats = config["n_repeats"]
    total = len(world_specs) * len(methods) * n_repeats
    done = 0

    out_path = _OUT_DIR / "runs.jsonl"
    with out_path.open("a") as f:
        for world_spec in world_specs:
            for method in methods:
                for repeat in range(n_repeats):
                    done += 1
                    label = f"{world_spec.name}_{method}_r{repeat}"
                    print(f"[{done}/{total}] {label}", flush=True)
                    run_dir = run_dir_base / label
                    run_dir.mkdir(parents=True, exist_ok=True)
                    record = await run_one_episode(
                        world_spec=world_spec,
                        method=method,
                        lambda_turn=lambda_turn,
                        config=config,
                        run_dir=run_dir,
                        label=label,
                    )
                    record["repeat"] = repeat
                    f.write(json.dumps(record) + "\n")
                    f.flush()
                    print(f"  -> {record['status']}", flush=True)

    print(f"Wrote {total} records to {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))

"""Live PX4 SITL test for the Phase 7 experiment/evaluation core (spec
§51 Phase 7).

Validation gate:
  1. "A P6 mission run produces a complete manifest + MCAP + per-episode
     metrics."
  2. "Re-reading the MCAP reproduces the metrics exactly."
  3. "The import contract blocks `groundtruth` from autonomy/learning
     (tested)" -- already covered, every unit-test run, by
     ``tests/unit/core/test_import_contracts.py::test_real_aeris_contracts_pass``,
     which re-checks every real contract in ``pyproject.toml`` (including
     the new "groundtruth is privileged" one) against the actual repo.
  4. "The Tier H nondeterminism measurement (P3) is formalized into a D1
     report" -- ``docs/reproducibility.md``, not a test.
"""

from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest
from scripts.fly_box import default_params, load_default_envelope

from aeris.autonomy.mission.spec import load_mission_spec
from aeris.core.frames.vector import Vec3
from aeris.core.types import RunStatus
from aeris.evaluation.metrics.nav import distance_travelled_m
from aeris.experiments.manifest import read_manifest
from aeris.experiments.runner import run_evaluation
from aeris.replay import channels
from aeris.replay.reader import ReplayReader
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import Px4Layout

pytestmark = pytest.mark.sim

_REPO_ROOT = Path(__file__).resolve().parents[2]
_MISSIONS_DIR = _REPO_ROOT / "configs" / "missions"


def test_mission_run_produces_manifest_mcap_and_reproducible_metrics(
    px4_layout: Px4Layout, tmp_path: Path
) -> None:
    envelope = load_default_envelope()
    spec = load_mission_spec(_MISSIONS_DIR / "square.yaml")
    profile = SimulationProfile(name="test_experiment_runner", model="gz_x500")

    manifest, result = asyncio.run(
        run_evaluation(
            layout=px4_layout,
            profile=profile,
            envelope=envelope,
            spec=spec,
            params=default_params(),
            name="square_smoke",
            run_root=tmp_path,
        )
    )

    print(
        f"\nmanifest status={manifest.status.value} reason={manifest.status_reason!r} "
        f"max_waypoint_error={result.metrics.max_waypoint_error_m if result.metrics else None}"
    )

    # --- gate item 1: complete manifest + MCAP + per-episode metrics ---
    run_dir = tmp_path / manifest.run_id
    assert manifest.status == RunStatus.COMPLETED, manifest.status_reason
    assert manifest.git_sha is not None  # this checkout is a real git repo

    reloaded_manifest = read_manifest(run_dir)
    assert reloaded_manifest == manifest

    replay_path = run_dir / "replays" / "episode.mcap"
    assert replay_path.is_file()
    assert replay_path.stat().st_size > 0

    metrics_path = run_dir / "metrics.jsonl"
    assert metrics_path.is_file()
    recorded_metrics = json.loads(metrics_path.read_text().splitlines()[0])
    assert recorded_metrics["max_waypoint_error_m"] <= 1.0  # spec: "within the configured radius"

    # --- gate item 2: re-reading the MCAP reproduces the metrics exactly ---
    with ReplayReader(replay_path) as reader:
        vehicle_state_samples = reader.read_channel(channels.VEHICLE_STATE)
        mission_events = reader.read_channel(channels.MISSION_EVENTS)

    assert len(vehicle_state_samples) > 0
    recomputed_positions = [Vec3(*sample["pos"]) for sample in vehicle_state_samples]
    recomputed_distance = distance_travelled_m(recomputed_positions)
    assert recomputed_distance == pytest.approx(recorded_metrics["distance_travelled_m"])

    event_names = [e["event"] for e in mission_events]
    assert event_names == ["mission_started", "mission_finished"]

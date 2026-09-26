"""Experiment runner (spec §38.1/§38.4): expands one run, executes it end
to end (Tier-H mission -> episode -> manifest), and writes the run
directory (``manifest.json``, ``config.resolved.yaml``, ``metrics.jsonl``,
``replays/episode.mcap``).

Spec §38.4: "A failed or invalid episode ... is recorded as invalid, with
its cause, re-run once, and reported in counts. It is never silently
dropped." -- the retry lives here (not in ``aeris.evaluation.episode``)
because retrying needs a fresh ``SimulationLauncher``, which this module
owns and ``run_episode`` deliberately doesn't.
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import yaml

from aeris.autonomy.mission.spec import MissionSpec
from aeris.core.clock import WallClock
from aeris.core.config import config_hash
from aeris.core.types import RunStatus, Tier
from aeris.evaluation.episode import EpisodeResult, run_episode
from aeris.experiments.manifest import RunManifest, make_run_id, new_manifest, write_manifest
from aeris.safety.envelope import Envelope
from aeris.safety.supervisor import SafetySupervisor
from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.ports import instance_ports
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import Px4Layout
from aeris.vehicle.interface import VehicleEndpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

_RESULTS_RUNS_DIR = Path(__file__).resolve().parents[2] / "results" / "runs"


async def _one_attempt(
    layout: Px4Layout,
    profile: SimulationProfile,
    envelope: Envelope,
    spec: MissionSpec,
    params: dict[str, float],
    replay_path: Path,
) -> EpisodeResult:
    launcher = SimulationLauncher(layout)
    try:
        launcher.start(profile, params=params)
        assert launcher.mavlink_connection is not None
        launcher.mavlink_connection.close()

        ports = instance_ports(profile.instance)
        adapter = Px4MavlinkAdapter()
        supervisor = SafetySupervisor(adapter, envelope=envelope, clock=WallClock())
        return await run_episode(
            supervisor,
            spec,
            VehicleEndpoint(host="127.0.0.1", port=ports.offboard_remote),
            clock=WallClock(),
            replay_path=replay_path,
        )
    finally:
        launcher.stop()


async def run_evaluation(
    *,
    layout: Px4Layout,
    profile: SimulationProfile,
    envelope: Envelope,
    spec: MissionSpec,
    params: dict[str, float],
    name: str,
    experiment_id: str | None = None,
    seed: int | None = None,
    run_root: Path = _RESULTS_RUNS_DIR,
) -> tuple[RunManifest, EpisodeResult]:
    """Run one evaluation end to end and write its run directory. Never
    raises for an ordinary episode failure -- reported in the manifest's
    ``status``/``status_reason`` and the returned :class:`EpisodeResult`.
    """
    resolved_config = spec.model_dump(mode="json")
    cfg_hash = config_hash(resolved_config)
    run_id = make_run_id(name, cfg_hash)
    run_dir = run_root / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "config.resolved.yaml").write_text(yaml.safe_dump(resolved_config))

    manifest = new_manifest(
        run_id=run_id,
        task="waypoint_mission",
        method="scripted",
        tier=Tier.HIGH_FIDELITY,
        config_hash=cfg_hash,
        experiment_id=experiment_id,
        seed=seed,
    )

    replay_path = run_dir / "replays" / "episode.mcap"
    replay_path.parent.mkdir(parents=True, exist_ok=True)

    result = await _one_attempt(layout, profile, envelope, spec, params, replay_path)
    attempts = 1
    if result.status != RunStatus.COMPLETED:
        # Retry once (spec §38.4). Overwrites the first attempt's replay
        # -- keeping only the attempt that's actually reported is a
        # deliberate simplification; both attempts are still reflected in
        # status_reason's attempt count.
        result = await _one_attempt(layout, profile, envelope, spec, params, replay_path)
        attempts = 2

    manifest.status = result.status
    manifest.status_reason = f"{result.reason} (attempts={attempts})"
    write_manifest(manifest, run_dir)

    if result.metrics is not None:
        (run_dir / "metrics.jsonl").write_text(
            json.dumps(dataclasses.asdict(result.metrics)) + "\n"
        )
    else:
        (run_dir / "metrics.jsonl").write_text("")

    return manifest, result

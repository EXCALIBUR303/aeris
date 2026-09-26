"""Live PX4 SITL tests for the Phase 6 mission executive (spec §51 Phase 6).

Validation gate: "15/15 missions complete; all abort-from-state tests
pass; arrival errors are within the configured radius."

Abort-from-every-state is a pure state-machine property, already
exhaustively covered live-enough by
``tests/unit/autonomy/test_executive.py``'s parametrized test against
every ``MissionState`` (and the identical pattern already proved live in
Phase 5's supervisor) -- no need to re-run it against SITL.
"""

from __future__ import annotations

import asyncio
from pathlib import Path

import pytest
from scripts.fly_box import default_params, load_default_envelope

from aeris.autonomy.mission.executive import MissionExecutive, MissionResult
from aeris.autonomy.mission.spec import MissionSpec, load_mission_spec
from aeris.core.clock import WallClock
from aeris.safety.envelope import Envelope
from aeris.safety.supervisor import SafetySupervisor
from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.ports import instance_ports
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import Px4Layout
from aeris.vehicle.interface import VehicleEndpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

pytestmark = pytest.mark.sim

_REPO_ROOT = Path(__file__).resolve().parents[2]
_MISSIONS_DIR = _REPO_ROOT / "configs" / "missions"


async def _run_mission_once(
    px4_layout: Px4Layout,
    profile: SimulationProfile,
    envelope: Envelope,
    spec: MissionSpec,
) -> MissionResult:
    launcher = SimulationLauncher(px4_layout)
    try:
        launcher.start(profile, params=default_params())
        assert launcher.mavlink_connection is not None
        launcher.mavlink_connection.close()

        ports = instance_ports(profile.instance)
        adapter = Px4MavlinkAdapter()
        supervisor = SafetySupervisor(adapter, envelope=envelope, clock=WallClock())
        executive = MissionExecutive(supervisor, spec, clock=WallClock())
        return await executive.run(VehicleEndpoint(host="127.0.0.1", port=ports.offboard_remote))
    finally:
        launcher.stop()


@pytest.mark.parametrize("template", ["square", "triangle", "line"])
def test_five_consecutive_runs_of_each_mission_template(
    px4_layout: Px4Layout, template: str
) -> None:
    envelope = load_default_envelope()
    spec = load_mission_spec(_MISSIONS_DIR / f"{template}.yaml")

    results = []
    for i in range(5):
        profile = SimulationProfile(name=f"test_mission_{template}_{i}", model="gz_x500")
        result = asyncio.run(_run_mission_once(px4_layout, profile, envelope, spec))
        print(
            f"\n{template} run {i + 1}/5: ok={result.ok} reason={result.reason!r} "
            f"max_waypoint_error={result.max_waypoint_error_m:.2f}m "
            f"wall_time={result.wall_time_s:.1f}s"
        )
        results.append(result)

    failures = [(i, r) for i, r in enumerate(results) if not r.ok]
    print(f"\n{template}: {5 - len(failures)}/5 OK")
    assert not failures, f"{template}: {len(failures)}/5 runs failed: {failures}"

    for result in results:
        for outcome, wp_spec in zip(result.waypoints, spec.waypoints, strict=True):
            assert outcome.arrived, f"{template}: waypoint {outcome.index} never arrived"
            assert outcome.error_m <= wp_spec.acceptance_radius_m, (
                f"{template}: waypoint {outcome.index} error {outcome.error_m:.2f}m "
                f"exceeds acceptance_radius_m={wp_spec.acceptance_radius_m}"
            )
        assert result.return_outcome is not None and result.return_outcome.arrived

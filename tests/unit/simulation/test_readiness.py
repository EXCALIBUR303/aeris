import os

import pytest

from aeris.core.errors import SimulationLaunchError
from aeris.simulation.launcher.readiness import wait_for_gz_world_ready, wait_for_px4_heartbeat


def test_wait_for_gz_world_ready_raises_immediately_if_gz_is_not_on_path(
    monkeypatch: pytest.MonkeyPatch,
):
    # Deterministic in any environment (dev machine or hosted CI, which
    # has no `gz` binary at all): an empty PATH guarantees `gz` can't be
    # found, so this proves the fail-fast path without depending on
    # whether Gazebo happens to be installed, or timing out for real.
    monkeypatch.setenv("PATH", "")
    with pytest.raises(SimulationLaunchError, match="'gz' command was not found"):
        wait_for_gz_world_ready(
            world="a-world-that-does-not-exist",
            env={},
            timeout_s=2.0,
        )


@pytest.mark.skipif(
    os.environ.get("CI") is not None, reason="needs a real `gz` binary, not installed in CI"
)
def test_wait_for_gz_world_ready_times_out_for_a_world_that_will_never_exist():
    # Real `gz`, no Gazebo server running — proves the retry/timeout path
    # (distinct from the "gz missing entirely" path above) on a machine
    # that actually has Gazebo installed (spec §51 Phase 3 dev machine).
    with pytest.raises(SimulationLaunchError, match="did not become ready"):
        wait_for_gz_world_ready(
            world="a-world-that-does-not-exist",
            env=dict(os.environ),
            timeout_s=2.0,
        )


def test_wait_for_px4_heartbeat_times_out_on_a_silent_port():
    # Nothing is sending heartbeats to this port — proves the timeout path.
    with pytest.raises(SimulationLaunchError, match="no MAVLink heartbeat"):
        wait_for_px4_heartbeat(port=39123, timeout_s=1.0)

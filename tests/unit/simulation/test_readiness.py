import os

import pytest

from aeris.core.errors import SimulationLaunchError
from aeris.simulation.launcher.readiness import wait_for_gz_world_ready, wait_for_px4_heartbeat


def test_wait_for_gz_world_ready_times_out_for_a_world_that_will_never_exist():
    # No Gazebo server is running at all, so `gz service -i` (or `gz`
    # itself missing) will never report the service — proves the timeout
    # path raises with diagnostic content, without needing a live sim.
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

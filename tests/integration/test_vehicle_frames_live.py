"""Live end-to-end frame-correctness tests (spec §19.4's live SITL item).

    "A live SITL test (P5/P8) commands +x ENU velocity and checks that
    PX4 reports the matching NED velocity ... and that gz GT displacement
    is along world East after alignment."

Phase 4 implements the vehicle side of this before Phase 5's safety layer
or Phase 8's ground-truth channel exist. The original plan was to verify
*just* the wire encoding via PX4's own ``POSITION_TARGET_LOCAL_NED``
setpoint echo, without needing the vehicle to actually fly -- arming was
believed to be the only prerequisite (a known open issue as of Phase
1/3, since fixed in this phase by making the adapter send its own
heartbeat; see ``test_vehicle_frames_live.py``'s git history / the Phase
4 report for that fix).

That plan doesn't survive contact with PX4's source, though. Reading
``MulticopterPositionControl.cpp`` (the pinned checkout, around
"not_taken_off") shows PX4 substitutes an empty all-NaN setpoint plus a
fixed ``(0, 0, 100)`` "high downwards acceleration to make sure there's
no thrust" *in place of* whatever the offboard link sent, for as long as
``_takeoff.getTakeoffState() < TakeoffState::flight`` -- i.e. the
setpoint echo only ever reflects our commanded values once the vehicle
has genuinely left the ground. So the "just check the echo, don't
bother with real flight" shortcut this test was designed around
requires real flight after all -- confirmed by arming, taking off, and
watching the vehicle climb a few centimeters and settle back down
without ever reaching commanded altitude (a SITL hover-thrust-estimator
convergence characteristic on a cold-started x500, not an AERIS bug).
Sequencing a *reliable* arm/takeoff/hover before commanding velocity is
exactly what spec's own "P5/P8" tag on this test was already flagging:
Phase 5's ``SafetySupervisor`` (proper arm/takeoff choreography) and
Phase 8's ``GroundTruthService`` (independent position/velocity
ground truth) are what make this test meaningful and reliable, not
Phase 4's raw adapter. Left in (xfail, not deleted/skipped) so Phase 5
can un-xfail it once real flight sequencing exists.

The GT-displacement half of the spec's full test is deferred to
Phase 7/8, once ``GroundTruthService`` exists, same as before.
"""

from __future__ import annotations

import asyncio
import math
import os

import pytest

from aeris.core.frames.vector import Vec3
from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.ports import instance_ports
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import Px4Layout
from aeris.vehicle.interface import VehicleEndpoint, VelocitySetpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

pytestmark = pytest.mark.integration


@pytest.fixture
def base_profile() -> SimulationProfile:
    return SimulationProfile(name="test_frames_x500", model="gz_x500")


@pytest.mark.xfail(
    reason=(
        "PX4 (MulticopterPositionControl.cpp, 'not_taken_off' branch) "
        "overrides every offboard setpoint with an empty/NaN one until the "
        "vehicle has genuinely taken off; a reliable arm/takeoff/hover "
        "sequence is Phase 5's SafetySupervisor, not Phase 4's adapter. "
        "See this module's docstring."
    ),
    strict=False,
)
def test_commanded_enu_east_velocity_arrives_at_px4_as_ned_north_east_vy(
    px4_layout: Px4Layout, base_profile: SimulationProfile
):
    """Command 1 m/s East (ENU +x) and read back PX4's own setpoint echo.

    spec §19.3: East (ENU +x) -> NED +y. If AERIS's conversion boundary is
    correct, PX4's ``POSITION_TARGET_LOCAL_NED`` echo should show
    ``vy ~= 1.0``, ``vx ~= 0.0``, ``vz ~= 0.0`` -- *once the vehicle is
    actually flying* (see module docstring; currently xfail without real
    takeoff sequencing).
    """

    async def run() -> tuple[float, float, float]:
        launcher = SimulationLauncher(px4_layout)
        try:
            launcher.start(base_profile)
            assert launcher.mavlink_connection is not None
            launcher.mavlink_connection.close()

            ports = instance_ports(0)
            adapter = Px4MavlinkAdapter()
            await adapter.connect(VehicleEndpoint(host="127.0.0.1", port=ports.offboard_remote))
            try:
                result = await adapter.start_offboard(
                    VelocitySetpoint(velocity=Vec3(1.0, 0.0, 0.0), frame="odom")
                )
                assert result.ok, f"OFFBOARD mode switch was rejected: {result}"

                # Give PX4 a few setpoint cycles to receive and echo it back.
                assert adapter._conn is not None  # test-internal access is fine here
                echo = None
                deadline = asyncio.get_event_loop().time() + 5.0
                while asyncio.get_event_loop().time() < deadline:
                    msg = adapter._conn.recv_match(type="POSITION_TARGET_LOCAL_NED", blocking=False)
                    if msg is not None:
                        echo = msg
                        break
                    await asyncio.sleep(0.05)

                assert echo is not None, "PX4 never echoed a POSITION_TARGET_LOCAL_NED setpoint"
                return echo.vx, echo.vy, echo.vz
            finally:
                await adapter.disconnect()
        finally:
            launcher.stop()

    vx_ned, vy_ned, vz_ned = asyncio.run(run())
    print(f"\nPX4 echoed setpoint (NED): vx={vx_ned:.3f} vy={vy_ned:.3f} vz={vz_ned:.3f}")
    assert vx_ned == pytest.approx(0.0, abs=0.05)
    assert vy_ned == pytest.approx(1.0, abs=0.05)
    assert vz_ned == pytest.approx(0.0, abs=0.05)


def test_t_sim_s_tracks_gazebo_clock(px4_layout: Px4Layout, base_profile: SimulationProfile):
    """spec §15.4: PX4's ``time_boot_ms`` "in lockstep SITL tracks
    simulation time" -- verify against Gazebo's own ``/clock`` topic
    (via the ``gz`` CLI, the same tool the launcher itself already uses
    for readiness checks; the full Sensor Bridge doesn't exist until
    Phase 8)."""
    import subprocess

    async def run() -> tuple[float, subprocess.CompletedProcess[str]]:
        launcher = SimulationLauncher(px4_layout)
        try:
            launcher.start(base_profile)
            assert launcher.mavlink_connection is not None
            launcher.mavlink_connection.close()

            ports = instance_ports(0)
            adapter = Px4MavlinkAdapter()
            await adapter.connect(VehicleEndpoint(host="127.0.0.1", port=ports.offboard_remote))
            try:
                await asyncio.sleep(1.0)  # let a few LOCAL_POSITION_NED samples arrive
                state = await adapter.get_vehicle_state()
                # Read gz's /clock *while the server is still alive* --
                # reading it after launcher.stop() hangs (no publisher
                # left) rather than failing fast. GZ_IP must be set for gz
                # transport discovery to find the topic at all (spec §9.2 /
                # Phase 1 finding) -- inherit the rest of the environment.
                gz_env = {**os.environ, "GZ_IP": "127.0.0.1"}
                gz_result = await asyncio.to_thread(
                    subprocess.run,
                    ["gz", "topic", "-e", "-t", "/clock", "-n", "1"],
                    env=gz_env,
                    capture_output=True,
                    text=True,
                    timeout=10,
                )
                return state.t_sim_s, gz_result
            finally:
                await adapter.disconnect()
        finally:
            launcher.stop()

    t_sim_s, result = asyncio.run(run())
    # `gz topic -e` on /clock prints a gz.msgs.Clock text proto with a
    # "sim { sec: N nsec: M }" block.
    sec = nsec = None
    in_sim_block = False
    for line in result.stdout.splitlines():
        line = line.strip()
        if line.startswith("sim"):
            in_sim_block = True
            continue
        if in_sim_block and line.startswith("sec:"):
            sec = int(line.split(":", 1)[1].strip())
        elif in_sim_block and line.startswith("nsec:"):
            nsec = int(line.split(":", 1)[1].strip())
            break
    assert sec is not None and nsec is not None, (
        f"could not parse gz clock output:\n{result.stdout}"
    )
    gz_sim_time_s = sec + nsec / 1e9

    print(f"\nPX4 t_sim_s={t_sim_s:.2f}s, gz /clock={gz_sim_time_s:.2f}s")
    # Not measured at the exact same instant (two separate process calls,
    # some wall time apart) -- a loose bound just confirms they're the
    # same clock, not two unrelated timebases (e.g. PX4 startup offset,
    # or wall-clock time leaking in somewhere).
    assert math.isclose(t_sim_s, gz_sim_time_s, abs_tol=5.0)

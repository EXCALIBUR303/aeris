"""Live PX4 SITL tests for :mod:`aeris.vehicle.px4_mavlink.adapter`.

Spec §51 Phase 4 validation gate: "telemetry rate and latency meet targets
(state >= 20 Hz; recorded p95 latency); refuses non-loopback endpoints."
The endpoint-refusal half is covered by pure unit tests
(``tests/unit/vehicle/test_hardware_guard.py``) — this file covers what
genuinely needs a live PX4 instance.
"""

from __future__ import annotations

import asyncio
import time

import pytest

from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.ports import instance_ports
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import Px4Layout
from aeris.vehicle.interface import VehicleEndpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

pytestmark = pytest.mark.integration


@pytest.fixture
def base_profile() -> SimulationProfile:
    return SimulationProfile(name="test_vehicle_x500", model="gz_x500")


async def _connect_adapter(launcher: SimulationLauncher) -> Px4MavlinkAdapter:
    assert launcher.mavlink_connection is not None
    # The launcher's own connection was only used for its readiness checks
    # -- close it so the adapter's reader thread is the only thing reading
    # this UDP port (two readers racing on one socket would drop messages
    # unpredictably between them).
    launcher.mavlink_connection.close()

    ports = instance_ports(0)
    adapter = Px4MavlinkAdapter()
    await adapter.connect(VehicleEndpoint(host="127.0.0.1", port=ports.offboard_remote))
    return adapter


def test_telemetry_rate_meets_20hz_target(px4_layout: Px4Layout, base_profile: SimulationProfile):
    async def run() -> tuple[int, float]:
        launcher = SimulationLauncher(px4_layout)
        try:
            launcher.start(base_profile)
            adapter = await _connect_adapter(launcher)
            try:
                # Sample the *underlying* data freshness, not just our own
                # poll loop's timing: poll faster than the target and count
                # how many *distinct* t_sim_s values arrive in the window.
                seen_t_sim: set[float] = set()
                window_s = 3.0
                t_end = time.monotonic() + window_s
                while time.monotonic() < t_end:
                    state = await adapter.get_vehicle_state()
                    seen_t_sim.add(state.t_sim_s)
                    await asyncio.sleep(1.0 / 100.0)
                achieved_hz = len(seen_t_sim) / window_s
                return len(seen_t_sim), achieved_hz
            finally:
                await adapter.disconnect()
        finally:
            launcher.stop()

    count, achieved_hz = asyncio.run(run())
    print(f"\ndistinct LOCAL_POSITION_NED samples in 3s: {count} ({achieved_hz:.1f} Hz)")
    assert achieved_hz >= 20.0, (
        f"telemetry updated at only {achieved_hz:.1f} Hz, need >= 20 Hz (spec §51 Phase 4)"
    )


def test_get_vehicle_state_p95_latency(px4_layout: Px4Layout, base_profile: SimulationProfile):
    async def run() -> list[float]:
        launcher = SimulationLauncher(px4_layout)
        try:
            launcher.start(base_profile)
            adapter = await _connect_adapter(launcher)
            try:
                latencies_ms = []
                for _ in range(200):
                    t0 = time.monotonic()
                    await adapter.get_vehicle_state()
                    latencies_ms.append((time.monotonic() - t0) * 1000.0)
                    await asyncio.sleep(0.01)
                return latencies_ms
            finally:
                await adapter.disconnect()
        finally:
            launcher.stop()

    latencies_ms = asyncio.run(run())
    latencies_ms.sort()
    p50 = latencies_ms[len(latencies_ms) // 2]
    p95 = latencies_ms[int(len(latencies_ms) * 0.95)]
    print(f"\nget_vehicle_state() latency: p50={p50:.2f}ms p95={p95:.2f}ms")
    # get_vehicle_state() only reads an in-memory cache (spec's threading
    # model) -- this should be sub-millisecond-to-low-single-digit-ms, not
    # bounded by network RTT. A generous bound just catches a real
    # regression (e.g. accidentally blocking on I/O).
    assert p95 < 50.0, f"get_vehicle_state() p95 latency too high: {p95:.2f}ms"


def test_heartbeat_age_grows_after_px4_stops(
    px4_layout: Px4Layout, base_profile: SimulationProfile
):
    async def run() -> tuple[float, float]:
        launcher = SimulationLauncher(px4_layout)
        launcher.start(base_profile)
        adapter = await _connect_adapter(launcher)
        state_before = await adapter.get_vehicle_state()
        launcher.stop()  # kill PX4 + Gazebo out from under the adapter
        await asyncio.sleep(3.0)
        state_after = await adapter.get_vehicle_state()
        await adapter.disconnect()
        return state_before.link.last_heartbeat_age_s, state_after.link.last_heartbeat_age_s

    age_before, age_after = asyncio.run(run())
    print(f"\nheartbeat age before stop: {age_before:.2f}s, after 3s of silence: {age_after:.2f}s")
    assert age_before < 1.0  # PX4 was alive and heartbeating normally
    assert age_after >= 2.5  # no heartbeat arrived during the 3s wait

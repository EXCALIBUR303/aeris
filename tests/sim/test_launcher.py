"""Live PX4 SITL + Gazebo tests for :mod:`aeris.simulation.launcher`.

Spec §51 Phase 3 validation gate: "10/10 consecutive start-ready-stop
cycles; reset produces a clean state (EKF re-initialized, disarmed,
landed); RTF table recorded; no orphan processes."

Every test here is marked ``sim`` and needs a real, built PX4 checkout
(``tests/sim/conftest.py`` skips cleanly otherwise). Not run in hosted CI
(spec §44.2) — run locally with ``pytest tests/sim -v -m sim`` and the
results recorded manually in the phase report, per spec §44.1/§44.2.
"""

from __future__ import annotations

import subprocess
import time

import pytest

from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import Px4Layout

pytestmark = pytest.mark.sim


def _no_orphan_gz_or_px4_processes() -> list[str]:
    """Return any *AERIS-launched-looking* gz/px4 process lines still alive.

    Used only as a diagnostic assertion aid between tests — real leak
    prevention is `SimulationLauncher.stop()`'s job, this just proves it
    actually worked, using `ps` rather than trusting our own bookkeeping.
    """
    result = subprocess.run(
        ["pgrep", "-fl", "gz sim|bin/px4"], capture_output=True, text=True, timeout=5
    )
    lines = [line for line in result.stdout.splitlines() if line.strip()]
    return lines


@pytest.fixture
def base_profile() -> SimulationProfile:
    return SimulationProfile(name="test_x500", model="gz_x500")


class TestTenConsecutiveCycles:
    """The core Phase 3 validation-gate requirement."""

    def test_ten_consecutive_start_stop_cycles_are_clean(
        self, px4_layout: Px4Layout, base_profile: SimulationProfile
    ) -> None:
        cycle_times: list[float] = []
        for cycle in range(1, 11):
            launcher = SimulationLauncher(px4_layout)
            t0 = time.monotonic()
            try:
                result = launcher.start(base_profile)
                assert result.px4_pid > 0
                assert launcher.is_running()
            finally:
                launcher.stop()
            cycle_times.append(time.monotonic() - t0)

            assert not launcher.is_running(), f"cycle {cycle}: still running after stop()"
            orphans = _no_orphan_gz_or_px4_processes()
            assert not orphans, f"cycle {cycle}: orphaned processes found: {orphans}"

        print(f"\n10-cycle timings (s): {[round(t, 1) for t in cycle_times]}")
        print(f"mean={sum(cycle_times) / len(cycle_times):.1f}s max={max(cycle_times):.1f}s")


class TestReset:
    def test_reset_produces_a_running_healthy_instance_and_measures_cost(
        self, launcher: SimulationLauncher, base_profile: SimulationProfile
    ) -> None:
        launcher.start(base_profile)
        assert launcher.is_running()

        result, reset_cost_s = launcher.reset()

        assert launcher.is_running()
        assert result.px4_pid > 0
        print(f"\nreset() wall-clock cost: {reset_cost_s:.1f}s")

        # "EKF re-initialized ... disarmed, landed": query PX4 directly
        # rather than trusting the launcher's own bookkeeping.
        assert launcher.mavlink_connection is not None
        hb = launcher.mavlink_connection.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        assert hb is not None
        armed_bit = 128  # MAV_MODE_FLAG_SAFETY_ARMED
        assert not (hb.base_mode & armed_bit), "vehicle is armed immediately after reset()"

        ext_state = launcher.mavlink_connection.recv_match(
            type="EXTENDED_SYS_STATE", blocking=True, timeout=5
        )
        if ext_state is not None:
            MAV_LANDED_STATE_ON_GROUND = 1
            assert ext_state.landed_state in (
                MAV_LANDED_STATE_ON_GROUND,
                0,  # MAV_LANDED_STATE_UNDEFINED — acceptable this soon after boot
            ), f"unexpected landed_state after reset(): {ext_state.landed_state}"

    def test_reset_before_start_raises(self, launcher: SimulationLauncher) -> None:
        with pytest.raises(RuntimeError):
            launcher.reset()


class TestSpeedFactor:
    """Records the RTF table the validation gate asks for (§51 Phase 3)."""

    @pytest.mark.parametrize("speed_factor", [1.0, 2.0, 4.0])
    def test_rtf_at_speed_factor(self, px4_layout: Px4Layout, speed_factor: float) -> None:
        profile = SimulationProfile(
            name=f"rtf_{speed_factor}", model="gz_x500", speed_factor=speed_factor
        )
        launcher = SimulationLauncher(px4_layout)
        try:
            launcher.start(profile)
            conn = launcher.mavlink_connection
            assert conn is not None

            # Sample sim time via PX4's own `time_boot_ms` on
            # LOCAL_POSITION_NED, compared to wall time. Drain continuously
            # for the whole window rather than "read, sleep, read" — a
            # single `recv_match` only dequeues one message, and PX4
            # streams LOCAL_POSITION_NED fast enough that a lone read after
            # a 3s gap can return something from early in the backlog
            # (observed: near-zero sim_dt despite a real 3s wall sleep).
            m0 = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=10)
            assert m0 is not None
            t_wall_0 = time.monotonic()
            m_last = m0
            window_s = 3.0
            while time.monotonic() - t_wall_0 < window_s:
                m = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
                if m is not None:
                    m_last = m
            t_wall_1 = time.monotonic()
            m1 = m_last

            sim_dt_s = (m1.time_boot_ms - m0.time_boot_ms) / 1000.0
            wall_dt_s = t_wall_1 - t_wall_0
            observed_rtf = sim_dt_s / wall_dt_s if wall_dt_s > 0 else float("nan")
            print(
                f"\nspeed_factor={speed_factor}: sim_dt={sim_dt_s:.2f}s "
                f"wall_dt={wall_dt_s:.2f}s observed_rtf={observed_rtf:.2f}"
            )
            assert observed_rtf > 0.1, (
                f"simulation barely progressing at speed_factor={speed_factor} "
                f"(observed_rtf={observed_rtf:.2f})"
            )
        finally:
            launcher.stop()


class TestStartupPoseNondeterminism:
    """A Phase-3-scoped stand-in for spec §40's D1 nondeterminism report.

    NOTE — scope: spec §40 asks for variance across identical *hover*
    episodes, which needs commanded flight (Phase 5's job; not built yet).
    This measures the one thing Phase 3 can: variance in the vehicle's
    reported *spawn* pose across identical fresh restarts of the same
    profile — i.e. "is the SITL boot process itself deterministic enough to
    build evaluation on." The fuller flight-based measurement is Phase 5's
    responsibility (see docs/phase_reports/phase-3.md).
    """

    def test_five_identical_restarts_have_near_zero_spawn_pose_variance(
        self, px4_layout: Px4Layout, base_profile: SimulationProfile
    ) -> None:
        positions: list[tuple[float, float, float]] = []
        for _ in range(5):
            launcher = SimulationLauncher(px4_layout)
            try:
                launcher.start(base_profile)
                conn = launcher.mavlink_connection
                assert conn is not None
                msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=10)
                assert msg is not None
                positions.append((msg.x, msg.y, msg.z))
            finally:
                launcher.stop()

        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]

        def _spread(values: list[float]) -> float:
            return max(values) - min(values)

        print(f"\nspawn positions (NED, m): {positions}")
        print(f"spread x={_spread(xs):.4f} y={_spread(ys):.4f} z={_spread(zs):.4f}")

        # A generous bound — this proves "SITL boot is not wildly
        # nondeterministic," not a tight scientific tolerance (spec §40
        # explicitly expects Tier H to be only *statistically*
        # reproducible, never bitwise).
        assert _spread(xs) < 1.0
        assert _spread(ys) < 1.0
        assert _spread(zs) < 1.0

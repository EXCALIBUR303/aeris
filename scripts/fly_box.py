#!/usr/bin/env python3
"""``aeris`` Phase 5's "fly a box" script (spec §51 Phase 5, expected output).

A thin choreography layer over ``aeris.safety.SafetySupervisor`` (the real
logic) and ``aeris.simulation.launcher`` (spec §14.1: "scripts/ ... thin
wrappers over package code"): arm, climb, visit 4 waypoints forming a
square, return to the start corner, land, disarm. Also runs a brief
velocity step-response phase while hovering (spec §51 Phase 5 research
consideration: "collect velocity step responses ... dataset for Phase 13
system ID").

Importable, not just runnable — ``tests/sim/test_safety_flight.py`` calls
:func:`fly_one_box_run` directly for the 20-repeat validation-gate test
and the fault-injection tests, rather than re-implementing the sequence.

Usage::

    uv run python scripts/fly_box.py --runs 20 --altitude-m 3.0 --side-length-m 5.0
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from aeris.core.clock import WallClock
from aeris.core.frames.vector import Vec3
from aeris.safety.envelope import Envelope, assert_strictly_inside_px4_geofence, load_envelope
from aeris.safety.supervisor import SafetySupervisor
from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.params import parse_params_file
from aeris.simulation.launcher.ports import instance_ports
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import Px4Layout, resolve_px4_layout
from aeris.vehicle.interface import (
    PositionSetpoint,
    VehicleEndpoint,
    VehicleState,
    VelocitySetpoint,
)
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

_REPO_ROOT = Path(__file__).resolve().parents[1]
_SAFETY_YAML = _REPO_ROOT / "configs" / "vehicle" / "safety.yaml"
_SITL_BASE_PARAMS = _REPO_ROOT / "configs" / "vehicle" / "px4_params" / "sitl_base.params"
_SAFETY_PARAMS = _REPO_ROOT / "configs" / "vehicle" / "px4_params" / "safety_v1.params"
_PX4_GF_MAX_HOR_DIST_M = 60.0  # must match safety_v1.params
_PX4_GF_MAX_VER_DIST_M = 40.0  # must match safety_v1.params

_ARRIVAL_RADIUS_M = 0.5  # matches the spec §51 Phase 5 gate's own bound
_WAYPOINT_TIMEOUT_S = 30.0
_ALTITUDE_ARRIVAL_TOLERANCE_M = 0.5
_TAKEOFF_TIMEOUT_S = 30.0
_LAND_TIMEOUT_S = 30.0
_TICK_PERIOD_S = 0.1  # supervisor.tick() + autonomy_heartbeat() cadence


@dataclass(frozen=True, slots=True)
class WaypointResult:
    target: Vec3
    achieved: Vec3
    error_m: float


@dataclass(frozen=True, slots=True)
class StepResponseSample:
    t_sim_s: float
    axis: str
    commanded_mps: float
    velocity_odom_mps: Vec3


@dataclass(frozen=True, slots=True)
class FlightRunResult:
    ok: bool
    reason: str
    waypoints: tuple[WaypointResult, ...] = field(default_factory=tuple)
    step_responses: tuple[StepResponseSample, ...] = field(default_factory=tuple)
    failsafe_triggered: bool = False
    wall_time_s: float = 0.0

    @property
    def max_waypoint_error_m(self) -> float:
        if not self.waypoints:
            return 0.0
        return max(w.error_m for w in self.waypoints)


def default_params() -> dict[str, float]:
    """``sitl_base.params`` merged with ``safety_v1.params`` (spec §16.2's
    "AERIS geofence strictly inside PX4's" — this is the PX4-side S0 half)."""
    return {**parse_params_file(_SITL_BASE_PARAMS), **parse_params_file(_SAFETY_PARAMS)}


def load_default_envelope() -> Envelope:
    envelope = load_envelope(_SAFETY_YAML)
    assert_strictly_inside_px4_geofence(
        envelope,
        px4_gf_max_hor_dist_m=_PX4_GF_MAX_HOR_DIST_M,
        px4_gf_max_ver_dist_m=_PX4_GF_MAX_VER_DIST_M,
    )
    return envelope


async def _wait_for(
    supervisor: SafetySupervisor,
    predicate: Callable[[VehicleState], bool],
    *,
    timeout_s: float,
    poll_s: float = 0.5,
) -> bool:
    """Poll ``predicate(state)`` against live telemetry until true or timeout."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = await supervisor.get_vehicle_state()
        if predicate(state):
            return True
        await asyncio.sleep(poll_s)
    return False


async def _fly_to_waypoint(
    supervisor: SafetySupervisor, target: Vec3, *, timeout_s: float = _WAYPOINT_TIMEOUT_S
) -> WaypointResult:
    sp = PositionSetpoint(position_odom=target)
    deadline = time.monotonic() + timeout_s
    last_state = await supervisor.get_vehicle_state()
    while time.monotonic() < deadline:
        result = await supervisor.submit_setpoint(sp)
        if not result.ok:
            break
        supervisor.autonomy_heartbeat()
        await supervisor.tick()
        last_state = await supervisor.get_vehicle_state()
        error = (last_state.pose_odom - target).norm()
        if error <= _ARRIVAL_RADIUS_M:
            return WaypointResult(target=target, achieved=last_state.pose_odom, error_m=error)
        await asyncio.sleep(_TICK_PERIOD_S)
    error = (last_state.pose_odom - target).norm()
    return WaypointResult(target=target, achieved=last_state.pose_odom, error_m=error)


async def _record_step_responses(supervisor: SafetySupervisor) -> tuple[StepResponseSample, ...]:
    """A brief step-response phase (spec §51 Phase 5 research consideration):
    +-0.5/1/2 m/s per horizontal axis, held 1.5s each with a 1s zero settle
    between steps, sampled once per step."""
    samples: list[StepResponseSample] = []
    steps: list[tuple[str, Vec3]] = [
        ("vx", Vec3(0.5, 0.0, 0.0)),
        ("vx", Vec3(1.0, 0.0, 0.0)),
        ("vx", Vec3(2.0, 0.0, 0.0)),
        ("vy", Vec3(0.0, 0.5, 0.0)),
        ("vy", Vec3(0.0, 1.0, 0.0)),
        ("vy", Vec3(0.0, 2.0, 0.0)),
    ]
    zero = VelocitySetpoint(velocity=Vec3(0.0, 0.0, 0.0))
    for axis, v in steps:
        commanded = math.hypot(v.x, v.y)
        sp = VelocitySetpoint(velocity=v)
        result = await supervisor.submit_setpoint(sp)
        if not result.ok:
            continue  # envelope rejected this step (e.g. exceeds v_xy_max) -- skip it
        await asyncio.sleep(1.5)
        state = await supervisor.get_vehicle_state()
        samples.append(
            StepResponseSample(
                t_sim_s=state.t_sim_s,
                axis=axis,
                commanded_mps=commanded,
                velocity_odom_mps=state.velocity_odom_mps,
            )
        )
        await supervisor.submit_setpoint(zero)
        await asyncio.sleep(1.0)
        supervisor.autonomy_heartbeat()
        await supervisor.tick()
    return tuple(samples)


async def fly_one_box_run(
    *,
    layout: Px4Layout,
    profile: SimulationProfile,
    envelope: Envelope,
    params: dict[str, float],
    altitude_m: float,
    side_length_m: float,
    run_step_responses: bool = True,
    run_dir: Path | None = None,
) -> FlightRunResult:
    """Arm, climb, fly a ``side_length_m`` square at ``altitude_m``, land, disarm.

    Returns a :class:`FlightRunResult` with every waypoint's arrival error
    and (if requested) the step-response dataset -- never raises for an
    ordinary flight failure (rejected command, timeout, failsafe): that's
    reported in the result so a caller running many repeats doesn't have
    to wrap each one in a try/except.
    """
    t0 = time.monotonic()
    launcher = SimulationLauncher(layout, run_dir=run_dir)
    try:
        launcher.start(profile, params=params)
        assert launcher.mavlink_connection is not None
        launcher.mavlink_connection.close()

        ports = instance_ports(profile.instance)
        adapter = Px4MavlinkAdapter()
        supervisor = SafetySupervisor(adapter, envelope=envelope, clock=WallClock())
        try:
            await supervisor.connect(VehicleEndpoint(host="127.0.0.1", port=ports.offboard_remote))
            if not await supervisor.preflight_check():
                return FlightRunResult(
                    ok=False, reason="preflight_check failed", wall_time_s=time.monotonic() - t0
                )

            takeoff_result = await supervisor.arm_and_takeoff(altitude_m)
            if not takeoff_result.ok:
                return FlightRunResult(
                    ok=False,
                    reason=f"arm_and_takeoff rejected: {takeoff_result.message}",
                    wall_time_s=time.monotonic() - t0,
                )

            airborne = await _wait_for(
                supervisor,
                lambda s: s.pose_odom.z >= altitude_m - _ALTITUDE_ARRIVAL_TOLERANCE_M,
                timeout_s=_TAKEOFF_TIMEOUT_S,
            )
            if not airborne:
                await supervisor.emergency_stop_simulation()
                return FlightRunResult(
                    ok=False,
                    reason=f"never reached {altitude_m}m within {_TAKEOFF_TIMEOUT_S}s",
                    wall_time_s=time.monotonic() - t0,
                )
            await supervisor.confirm_airborne()
            await supervisor.start_autonomy()

            home = (await supervisor.get_vehicle_state()).pose_odom
            corners = [
                home,
                home + Vec3(side_length_m, 0.0, 0.0),
                home + Vec3(side_length_m, side_length_m, 0.0),
                home + Vec3(0.0, side_length_m, 0.0),
                home,
            ]
            waypoints: list[WaypointResult] = []
            for corner in corners[1:]:
                waypoints.append(await _fly_to_waypoint(supervisor, corner))

            step_responses: tuple[StepResponseSample, ...] = ()
            if run_step_responses:
                step_responses = await _record_step_responses(supervisor)

            await supervisor.stop_autonomy()
            land_result = await supervisor.land()
            if not land_result.ok:
                return FlightRunResult(
                    ok=False,
                    reason=f"land rejected: {land_result.message}",
                    waypoints=tuple(waypoints),
                    step_responses=step_responses,
                    wall_time_s=time.monotonic() - t0,
                )
            landed = await _wait_for(
                supervisor, lambda s: s.landed_state.value == "on_ground", timeout_s=_LAND_TIMEOUT_S
            )
            if not landed:
                return FlightRunResult(
                    ok=False,
                    reason=f"never landed within {_LAND_TIMEOUT_S}s",
                    waypoints=tuple(waypoints),
                    step_responses=step_responses,
                    wall_time_s=time.monotonic() - t0,
                )
            supervisor.confirm_landed()
            await supervisor.disarm()

            max_error = max((w.error_m for w in waypoints), default=0.0)
            return FlightRunResult(
                ok=max_error <= _ARRIVAL_RADIUS_M,
                reason="ok" if max_error <= _ARRIVAL_RADIUS_M else "waypoint error exceeded bound",
                waypoints=tuple(waypoints),
                step_responses=step_responses,
                wall_time_s=time.monotonic() - t0,
            )
        finally:
            await supervisor.disconnect()
    finally:
        launcher.stop()


def _cmd(args: argparse.Namespace) -> int:
    layout = resolve_px4_layout()
    envelope = load_default_envelope()
    params = default_params()
    all_step_responses: list[dict[str, object]] = []
    failures = 0

    for i in range(args.runs):
        profile = SimulationProfile(name=f"fly_box_{i}", model="gz_x500")
        run_dir = Path(args.run_dir) / f"run_{i}" if args.run_dir else None
        print(f"--- run {i + 1}/{args.runs} ---")
        result = asyncio.run(
            fly_one_box_run(
                layout=layout,
                profile=profile,
                envelope=envelope,
                params=params,
                altitude_m=args.altitude_m,
                side_length_m=args.side_length_m,
                run_step_responses=(i == 0),  # once is enough for the dataset
                run_dir=run_dir,
            )
        )
        status = "OK" if result.ok else "FAIL"
        print(
            f"  {status}: {result.reason} max_waypoint_error={result.max_waypoint_error_m:.2f}m "
            f"wall_time={result.wall_time_s:.1f}s"
        )
        if not result.ok:
            failures += 1
        for s in result.step_responses:
            all_step_responses.append(
                {
                    "t_sim_s": s.t_sim_s,
                    "axis": s.axis,
                    "commanded_mps": s.commanded_mps,
                    "velocity_odom_mps": [
                        s.velocity_odom_mps.x,
                        s.velocity_odom_mps.y,
                        s.velocity_odom_mps.z,
                    ],
                }
            )

    if args.step_response_out and all_step_responses:
        Path(args.step_response_out).write_text(json.dumps(all_step_responses, indent=2))
        print(f"step-response dataset written to {args.step_response_out}")

    print(f"\n{args.runs - failures}/{args.runs} runs OK")
    return 0 if failures == 0 else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--altitude-m", type=float, default=3.0)
    parser.add_argument("--side-length-m", type=float, default=5.0)
    parser.add_argument("--run-dir", help="Directory to write per-run logs into")
    parser.add_argument("--step-response-out", help="Path to write the step-response dataset JSON")
    args = parser.parse_args(argv)
    return _cmd(args)


if __name__ == "__main__":
    raise SystemExit(main())

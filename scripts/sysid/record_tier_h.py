#!/usr/bin/env python3
"""Tier H data collection for FastSim system identification and parity
(spec §51 Phase 13 tasks 1, 4, 9). Three sessions, each one fresh
PX4 + Gazebo launch:

- ``steps``: dense (every telemetry sample, ~50 Hz) velocity and yaw-rate
  step responses, +/- several amplitudes per axis, repeated twice --
  repetition 0 is the system-ID *training* set, repetition 1 the
  *held-out* set the validation gate is scored on. (Phase 5's own
  step-response recorder kept one sample per step, 1.5s after the step --
  enough to see steady state, not enough to fit a time constant or a
  delay, which is why this exists.)
- ``openloop``: 10 open-loop 20s command sequences (piecewise-constant
  world-frame velocity + yaw rate, second half the first half's negated
  mirror so the vehicle returns near its start), plus GT-vs-EKF pose
  pairs sampled throughout for the pose-error model.
- ``depth``: a hover in an obstacle world, stopped at 8 yaws; at each stop
  one full-resolution depth frame + GT pose, for depth-statistics parity.

Every command respects the S1 validator's accel check (segment changes
are separated by >=2s, so implied accel stays well under 2 m/s^2).

Usage::

    uv run python scripts/sysid/record_tier_h.py --session steps
    uv run python scripts/sysid/record_tier_h.py --session openloop
    uv run python scripts/sysid/record_tier_h.py --session depth
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time
from array import array
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from aeris.core.clock import WallClock
from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import Vec3
from aeris.safety.envelope import load_envelope
from aeris.safety.supervisor import SafetySupervisor
from aeris.simulation.bridge.client import GzBridgeClient
from aeris.simulation.bridge.launch import start_bridge
from aeris.simulation.groundtruth.service import GroundTruthService
from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.ports import instance_ports
from aeris.simulation.launcher.profiles import SimulationProfile
from aeris.simulation.px4_paths import resolve_px4_layout
from aeris.simulation.worlds import sdf
from aeris.simulation.worlds.batch import generate_batch, load_world_spec
from aeris.simulation.worlds.spec import WorldFamily, WorldSpec, WorldSplit
from aeris.vehicle.interface import VehicleEndpoint, VelocitySetpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

_REPO_ROOT = Path(__file__).resolve().parents[2]
_OUT_ROOT = _REPO_ROOT / "results" / "sysid"
_LOG_PERIOD_S = 0.02
_TICK_S = 0.1

STEP_AMPS_XY = (0.5, 1.0, 1.5, 2.0)
STEP_AMPS_YAW = (0.3, 0.6, 0.9)
STEP_HOLD_S = 2.5
STEP_REST_S = 2.0


def _yaw(q: Quaternion) -> float:
    h = q.rotate(Vec3(1.0, 0.0, 0.0))
    return math.atan2(h.y, h.x)


@dataclass
class _Recorder:
    """Background 50 Hz telemetry log, tagged with the command in force."""

    supervisor: SafetySupervisor
    cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tag: str = ""
    rows: list[list[Any]] = field(default_factory=list)
    running: bool = True

    async def run(self) -> None:
        while self.running:
            s = await self.supervisor.get_vehicle_state()
            self.rows.append(
                [
                    s.t_sim_s,
                    s.pose_odom.x,
                    s.pose_odom.y,
                    s.pose_odom.z,
                    s.velocity_odom_mps.x,
                    s.velocity_odom_mps.y,
                    _yaw(s.orientation_odom),
                    s.angular_velocity_body_radps.z,
                    *self.cmd,
                    self.tag,
                ]
            )
            await asyncio.sleep(_LOG_PERIOD_S)


async def _hold(supervisor: SafetySupervisor, seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        supervisor.autonomy_heartbeat()
        await supervisor.tick()
        await asyncio.sleep(_TICK_S)


async def _command(
    supervisor: SafetySupervisor, rec: _Recorder, vx: float, vy: float, r: float, tag: str
) -> None:
    result = await supervisor.submit_setpoint(
        VelocitySetpoint(velocity=Vec3(vx, vy, 0.0), yaw_rate_radps=r, frame="odom")
    )
    if not result.ok:
        raise RuntimeError(f"setpoint rejected ({tag}): {result.message}")
    rec.cmd = (vx, vy, r)
    rec.tag = tag


async def _fly(
    *,
    world_spec: WorldSpec | None,
    run_dir: Path,
    altitude_m: float,
    body: Any,
    with_bridge: bool = False,
) -> Any:
    """Launch, take off to ``altitude_m``, run ``body(supervisor, ctx)``, land."""
    layout = resolve_px4_layout()
    if world_spec is not None:
        sdf_path = run_dir / f"{world_spec.name}.sdf"
        sdf_path.write_text(sdf.render(world_spec))
        spawn = world_spec.spawn_poses[0]
        profile = SimulationProfile(
            name="p13_sysid",
            world=world_spec.name,
            world_sdf_path=sdf_path,
            model="gz_x500_depth",
            spawn_pose=(spawn.x, spawn.y, spawn.z, 0.0, 0.0, spawn.yaw_rad),
        )
        world_name = world_spec.name
    else:
        profile = SimulationProfile(name="p13_sysid", model="gz_x500_depth")
        world_name = "default"

    launcher = SimulationLauncher(layout, run_dir=run_dir)
    bridge = None
    try:
        launcher.start(profile)
        ctx: dict[str, Any] = {"world": world_name}
        if with_bridge:
            bridge, endpoint = start_bridge(
                world=world_name, model_instance="x500_depth_0", run_dir=run_dir
            )
            ctx["bridge_endpoint"] = endpoint
            await asyncio.sleep(2.0)
        envelope = load_envelope(str(_REPO_ROOT / "configs" / "vehicle" / "safety.yaml"))
        supervisor = SafetySupervisor(Px4MavlinkAdapter(), envelope=envelope, clock=WallClock())
        ports = instance_ports(profile.instance)
        await supervisor.connect(VehicleEndpoint(host="127.0.0.1", port=ports.offboard_remote))
        try:
            if not await supervisor.preflight_check():
                raise RuntimeError("preflight_check failed")
            if not (await supervisor.arm_and_takeoff(altitude_m)).ok:
                raise RuntimeError("arm_and_takeoff rejected")
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                if (await supervisor.get_vehicle_state()).pose_odom.z >= altitude_m - 0.3:
                    break
                await asyncio.sleep(0.5)
            else:
                raise RuntimeError("never reached altitude")
            await supervisor.confirm_airborne()
            await supervisor.start_autonomy()
            await _hold(supervisor, 3.0)
            result = await body(supervisor, ctx)
            await supervisor.stop_autonomy()
            await supervisor.land()
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                if (await supervisor.get_vehicle_state()).landed_state.value == "on_ground":
                    break
                await asyncio.sleep(0.5)
            supervisor.confirm_landed()
            await supervisor.disarm()
            return result
        finally:
            await supervisor.disconnect()
    finally:
        if bridge is not None:
            bridge.stop()
        launcher.stop()


async def _session_steps(supervisor: SafetySupervisor, ctx: dict[str, Any]) -> dict[str, Any]:
    rec = _Recorder(supervisor)
    task = asyncio.create_task(rec.run())
    try:
        for rep in (0, 1):
            for axis, amps in (("vx", STEP_AMPS_XY), ("vy", STEP_AMPS_XY), ("yaw", STEP_AMPS_YAW)):
                for amp in amps:
                    for sign in (1.0, -1.0):
                        a = sign * amp
                        vx, vy, r = (
                            (a, 0.0, 0.0)
                            if axis == "vx"
                            else ((0.0, a, 0.0) if axis == "vy" else (0.0, 0.0, a))
                        )
                        tag = f"rep{rep}:{axis}:{a:+.2f}"
                        await _command(supervisor, rec, vx, vy, r, tag)
                        await _hold(supervisor, STEP_HOLD_S)
                        await _command(supervisor, rec, 0.0, 0.0, 0.0, tag + ":rest")
                        await _hold(supervisor, STEP_REST_S)
    finally:
        rec.running = False
        await task
    return {"rows": rec.rows}


def _openloop_sequences(seed: int, n: int = 10) -> list[list[tuple[float, float, float, float]]]:
    """``n`` sequences of (duration_s, vx, vy, yaw_rate) segments, 20s each:
    five random 2s segments, then the same five negated in reverse order."""
    rng = np.random.default_rng(seed)
    seqs = []
    for _ in range(n):
        first = []
        for _ in range(5):
            speed = rng.uniform(0.0, 1.5)
            heading = rng.uniform(-math.pi, math.pi)
            first.append(
                (2.0, speed * math.cos(heading), speed * math.sin(heading), rng.uniform(-0.6, 0.6))
            )
        second = [(d, -vx, -vy, -r) for d, vx, vy, r in reversed(first)]
        seqs.append(first + second)
    return seqs


async def _session_openloop(supervisor: SafetySupervisor, ctx: dict[str, Any]) -> dict[str, Any]:
    gt = GroundTruthService(world=ctx["world"])
    rec = _Recorder(supervisor)
    pose_pairs: list[list[float]] = []
    stop = asyncio.Event()

    async def gt_sampler() -> None:
        loop = asyncio.get_running_loop()
        while not stop.is_set():
            t0 = rec.rows[-1][0] if rec.rows else math.nan
            pos, q = await loop.run_in_executor(None, gt.get_pose, "x500_depth_0")
            t1 = rec.rows[-1][0] if rec.rows else math.nan
            pose_pairs.append([t0, t1, pos.x, pos.y, pos.z, _yaw(q)])
            await asyncio.sleep(0.2)

    task = asyncio.create_task(rec.run())
    gt_task = asyncio.create_task(gt_sampler())
    seqs = _openloop_sequences(seed=20261013)
    try:
        for i, seq in enumerate(seqs):
            await _command(supervisor, rec, 0.0, 0.0, 0.0, f"seq{i}:pre")
            await _hold(supervisor, 2.0)
            for j, (dur, vx, vy, r) in enumerate(seq):
                await _command(supervisor, rec, vx, vy, r, f"seq{i}:seg{j}")
                await _hold(supervisor, dur)
        await _command(supervisor, rec, 0.0, 0.0, 0.0, "post")
        await _hold(supervisor, 2.0)
    finally:
        stop.set()
        rec.running = False
        await task
        await gt_task
    return {"rows": rec.rows, "pose_pairs": pose_pairs, "sequences": seqs}


async def _session_depth(supervisor: SafetySupervisor, ctx: dict[str, Any]) -> dict[str, Any]:
    gt = GroundTruthService(world=ctx["world"])
    frames: list[np.ndarray] = []
    meta: list[dict[str, Any]] = []
    rec = _Recorder(supervisor)
    with GzBridgeClient(
        ctx["bridge_endpoint"], ["depth"], conflate=True, rcv_timeout_ms=200
    ) as client:
        for k in range(8):
            if k > 0:
                await _command(supervisor, rec, 0.0, 0.0, 0.5, f"rot{k}")
                await _hold(supervisor, (math.pi / 4.0) / 0.5)
            await _command(supervisor, rec, 0.0, 0.0, 0.0, f"stop{k}")
            await _hold(supervisor, 4.0)  # settle fully: stationary pose => exact matching
            state_before = await supervisor.get_vehicle_state()
            pos, q = gt.get_pose("x500_depth_0")
            msg = None
            for _ in range(5):  # drain: conflate keeps the latest, but take a fresh one
                m = client.recv()
                if m is not None:
                    msg = m
            state_after = await supervisor.get_vehicle_state()
            if msg is None:
                continue
            p = msg["payload"]
            buf: array[float] = array("f")
            buf.frombytes(p["data"])
            frames.append(np.array(buf, dtype=np.float32).reshape(p["height"], p["width"]))
            meta.append(
                {
                    "frame_t_sim_s": msg["t_sim_s"],
                    "state_t_sim_s": [state_before.t_sim_s, state_after.t_sim_s],
                    "gt_pos_world": [pos.x, pos.y, pos.z],
                    "gt_quat_wxyz": [q.w, q.x, q.y, q.z],
                    "ekf_speed_mps": state_after.velocity_odom_mps.norm(),
                    "ekf_yaw_rate_radps": state_after.angular_velocity_body_radps.z,
                }
            )
    return {"frames": frames, "meta": meta}


def _ensure_depth_world() -> WorldSpec:
    path = _REPO_ROOT / "results" / "worlds" / "f2_office" / "val" / "f2_office_10000.json"
    if not path.exists():
        generate_batch(
            family=WorldFamily.OFFICE,
            split=WorldSplit.VAL,
            n=1,
            out_dir=_REPO_ROOT / "results" / "worlds",
        )
    return load_world_spec(path)


async def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", choices=["steps", "openloop", "depth"], required=True)
    args = parser.parse_args()

    out = _OUT_ROOT / args.session
    out.mkdir(parents=True, exist_ok=True)
    run_dir = out / "run_logs"
    run_dir.mkdir(exist_ok=True)

    if args.session == "steps":
        res = await _fly(world_spec=None, run_dir=run_dir, altitude_m=2.0, body=_session_steps)
        np.save(out / "rows.npy", np.array([r[:-1] for r in res["rows"]], dtype=np.float64))
        (out / "tags.json").write_text(json.dumps([r[-1] for r in res["rows"]]))
    elif args.session == "openloop":
        res = await _fly(world_spec=None, run_dir=run_dir, altitude_m=2.0, body=_session_openloop)
        np.save(out / "rows.npy", np.array([r[:-1] for r in res["rows"]], dtype=np.float64))
        (out / "tags.json").write_text(json.dumps([r[-1] for r in res["rows"]]))
        (out / "pose_pairs.json").write_text(json.dumps(res["pose_pairs"]))
        (out / "sequences.json").write_text(json.dumps(res["sequences"]))
    else:
        spec = _ensure_depth_world()
        res = await _fly(
            world_spec=spec, run_dir=run_dir, altitude_m=1.0, body=_session_depth, with_bridge=True
        )
        np.savez_compressed(out / "frames.npz", *res["frames"])
        (out / "meta.json").write_text(
            json.dumps({"world": spec.name, "frames": res["meta"]}, indent=2)
        )
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))

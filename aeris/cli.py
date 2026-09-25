"""``aeris`` command-line entry point.

Spec §51 Phase 3 expected output: "``aeris sim up --profile headless_x500``
works." Spec §51 Phase 4 expected output: "``aeris vehicle monitor`` prints
live state in ENU." Kept intentionally small (stdlib ``argparse``, no new
dependency) and grown by whichever phase adds the next subcommand
(``aeris mission``, ...).
"""

from __future__ import annotations

import argparse
import asyncio
import signal
import sys
import time
from pathlib import Path

from aeris.core.logging import configure_logging, get_logger
from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.launcher.params import parse_params_file
from aeris.simulation.launcher.profiles import load_profile
from aeris.simulation.launcher.state import SimState, read_state, stop_by_state, write_state
from aeris.simulation.px4_paths import resolve_px4_layout
from aeris.vehicle.interface import VehicleEndpoint
from aeris.vehicle.px4_mavlink.adapter import Px4MavlinkAdapter

_PROFILES_DIR = Path(__file__).resolve().parents[1] / "configs" / "simulation"
_PARAMS_DIR = Path(__file__).resolve().parents[1] / "configs" / "vehicle" / "px4_params"

_logger = get_logger(component="cli")


def _cmd_sim_up(args: argparse.Namespace) -> int:
    profile_path = _PROFILES_DIR / f"{args.profile}.yaml"
    profile = load_profile(profile_path)

    params = None
    if args.params:
        params = parse_params_file(_PARAMS_DIR / f"{args.params}.params")

    layout = resolve_px4_layout()
    launcher = SimulationLauncher(layout, run_dir=Path(args.run_dir) if args.run_dir else None)

    print(f"Starting profile {profile.name!r} (model={profile.model}, world={profile.world})...")
    result = launcher.start(profile, params=params)
    write_state(
        SimState(
            profile_name=profile.name,
            gz_pid=result.gz_pid,
            px4_pid=result.px4_pid,
            started_at_wall_s=time.time(),
        )
    )
    print(
        f"Ready: gz_pid={result.gz_pid} px4_pid={result.px4_pid} "
        f"world_ready={result.world_ready_s:.1f}s px4_ready={result.px4_ready_s:.1f}s "
        f"offboard_port={result.ports.offboard_remote}"
    )
    print("Press Ctrl-C to stop.")

    stop_requested = False

    def _on_signal(signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    while not stop_requested and launcher.is_running():
        time.sleep(0.5)

    print("Stopping...")
    launcher.stop()
    from aeris.simulation.launcher.state import clear_state

    clear_state()
    print("Stopped.")
    return 0


def _cmd_sim_down(_args: argparse.Namespace) -> int:
    state = stop_by_state()
    if state is None:
        print("No running simulation found (no state file).")
        return 1
    print(
        f"Stopped profile {state.profile_name!r} (gz_pid={state.gz_pid}, px4_pid={state.px4_pid})"
    )
    return 0


def _cmd_sim_status(_args: argparse.Namespace) -> int:
    state = read_state()
    if state is None:
        print("Not running.")
        return 1
    print(f"Running: profile={state.profile_name!r} gz_pid={state.gz_pid} px4_pid={state.px4_pid}")
    return 0


def _cmd_vehicle_monitor(args: argparse.Namespace) -> int:
    async def run() -> int:
        adapter = Px4MavlinkAdapter()
        print(f"Connecting to {args.host}:{args.port}...")
        await adapter.connect(VehicleEndpoint(host=args.host, port=args.port))
        print("Connected. Press Ctrl-C to stop.")

        stop_requested = False

        def _on_signal(signum: int, _frame: object) -> None:
            nonlocal stop_requested
            stop_requested = True

        signal.signal(signal.SIGINT, _on_signal)
        signal.signal(signal.SIGTERM, _on_signal)

        try:
            async for state in adapter.subscribe_telemetry(args.rate):
                if stop_requested:
                    break
                p, v = state.pose_odom, state.velocity_odom_mps
                print(
                    f"t_sim={state.t_sim_s:7.2f}s  armed={state.armed!s:5}  "
                    f"mode={state.flight_mode.value:9}  landed={state.landed_state.value:9}  "
                    f"pos_enu=({p.x:+6.2f},{p.y:+6.2f},{p.z:+6.2f})m  "
                    f"vel_enu=({v.x:+5.2f},{v.y:+5.2f},{v.z:+5.2f})m/s  "
                    f"heartbeat_age={state.link.last_heartbeat_age_s:4.1f}s"
                )
        finally:
            await adapter.disconnect()
        return 0

    return asyncio.run(run())


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="aeris")
    subparsers = parser.add_subparsers(dest="command", required=True)

    sim = subparsers.add_parser("sim", help="Simulation lifecycle (spec §17.2)")
    sim_sub = sim.add_subparsers(dest="sim_command", required=True)

    up = sim_sub.add_parser("up", help="Start a simulation profile and block until Ctrl-C")
    up.add_argument("--profile", required=True, help="Profile name under configs/simulation/")
    up.add_argument(
        "--params", help="Params file name (without .params) under configs/vehicle/px4_params/"
    )
    up.add_argument("--run-dir", help="Directory to write logs into (default: no log capture)")
    up.set_defaults(func=_cmd_sim_up)

    down = sim_sub.add_parser("down", help="Stop the currently-running simulation")
    down.set_defaults(func=_cmd_sim_down)

    status = sim_sub.add_parser("status", help="Show whether a simulation is running")
    status.set_defaults(func=_cmd_sim_status)

    vehicle = subparsers.add_parser("vehicle", help="Vehicle telemetry/commands (spec §15)")
    vehicle_sub = vehicle.add_subparsers(dest="vehicle_command", required=True)

    monitor = vehicle_sub.add_parser(
        "monitor", help="Connect to a running PX4 instance and print live state in ENU"
    )
    monitor.add_argument("--host", default="127.0.0.1", help="Vehicle endpoint host")
    monitor.add_argument(
        "--port", type=int, default=14540, help="Vehicle endpoint port (instance 0's offboard link)"
    )
    monitor.add_argument("--rate", type=float, default=5.0, help="Print rate in Hz")
    monitor.set_defaults(func=_cmd_vehicle_monitor)

    return parser


def main(argv: list[str] | None = None) -> int:
    configure_logging()
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())

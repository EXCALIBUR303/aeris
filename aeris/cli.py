"""``aeris`` command-line entry point.

Spec §51 Phase 3 expected output: "``aeris sim up --profile headless_x500``
works." Kept intentionally small (stdlib ``argparse``, no new dependency)
and grown by whichever phase adds the next subcommand (``aeris vehicle``,
``aeris mission``, ...).
"""

from __future__ import annotations

import argparse
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

    return parser


def main(argv: list[str] | None = None) -> int:
    configure_logging()
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())

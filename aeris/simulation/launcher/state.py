"""A small on-disk record of the currently-running simulation, for the CLI.

The :mod:`aeris.simulation.launcher.launcher` API is in-process (one
``SimulationLauncher`` object, one Python process) — but the CLI (``aeris
sim up`` / ``down`` / ``status``) is invoked as separate, short-lived
processes, so ``down``/``status`` need some way to find what ``up`` started.
This file is that: a minimal JSON record of the tracked PIDs, written on
start and removed on clean stop.

Not used by anything except the CLI — code that already holds a
``SimulationLauncher`` (tests, evaluation runners) should just call
``launcher.stop()`` directly rather than going through this file.
"""

from __future__ import annotations

import contextlib
import json
import os
import signal
import time
from dataclasses import asdict, dataclass
from pathlib import Path

_DEFAULT_STATE_PATH = Path.home() / ".aeris" / "sim_state.json"


@dataclass(frozen=True, slots=True)
class SimState:
    profile_name: str
    gz_pid: int | None
    px4_pid: int
    started_at_wall_s: float


def write_state(state: SimState, *, path: Path = _DEFAULT_STATE_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(asdict(state)))


def read_state(*, path: Path = _DEFAULT_STATE_PATH) -> SimState | None:
    if not path.is_file():
        return None
    data = json.loads(path.read_text())
    return SimState(**data)


def clear_state(*, path: Path = _DEFAULT_STATE_PATH) -> None:
    path.unlink(missing_ok=True)


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True  # exists, just not ours to signal-probe further
    return True


def stop_by_state(
    *, path: Path = _DEFAULT_STATE_PATH, terminate_timeout_s: float = 10.0
) -> SimState | None:
    """Read the state file and SIGTERM (then SIGKILL) its tracked PIDs.

    Returns the state that was stopped, or ``None`` if no state file was
    found (nothing to stop). Clears the state file on completion either way.
    """
    state = read_state(path=path)
    if state is None:
        return None

    for pid in (state.px4_pid, state.gz_pid):
        if pid is None or not _pid_alive(pid):
            continue
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            continue

    deadline = time.monotonic() + terminate_timeout_s
    while time.monotonic() < deadline:
        if not any(_pid_alive(p) for p in (state.px4_pid, state.gz_pid) if p is not None):
            break
        time.sleep(0.2)
    else:
        for pid in (state.px4_pid, state.gz_pid):
            if pid is not None and _pid_alive(pid):
                with contextlib.suppress(ProcessLookupError):
                    os.kill(pid, signal.SIGKILL)

    clear_state(path=path)
    return state

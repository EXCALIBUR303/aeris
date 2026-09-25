import subprocess
import sys
import time
from pathlib import Path

from aeris.simulation.launcher.state import (
    SimState,
    clear_state,
    read_state,
    stop_by_state,
    write_state,
)


def test_read_state_returns_none_when_no_file(tmp_path: Path):
    assert read_state(path=tmp_path / "sim_state.json") is None


def test_write_then_read_round_trips(tmp_path: Path):
    path = tmp_path / "sim_state.json"
    state = SimState(profile_name="headless_x500", gz_pid=111, px4_pid=222, started_at_wall_s=1.5)
    write_state(state, path=path)
    assert read_state(path=path) == state


def test_write_state_creates_parent_directories(tmp_path: Path):
    path = tmp_path / "nested" / "dir" / "sim_state.json"
    write_state(SimState("p", 1, 2, 0.0), path=path)
    assert path.is_file()


def test_clear_state_removes_file_and_is_idempotent(tmp_path: Path):
    path = tmp_path / "sim_state.json"
    write_state(SimState("p", 1, 2, 0.0), path=path)
    clear_state(path=path)
    assert not path.exists()
    clear_state(path=path)  # must not raise on a missing file


def test_stop_by_state_returns_none_when_no_state(tmp_path: Path):
    assert stop_by_state(path=tmp_path / "sim_state.json") is None


def test_stop_by_state_terminates_real_processes_and_clears_file(tmp_path: Path):
    path = tmp_path / "sim_state.json"
    p1 = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    p2 = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    try:
        write_state(
            SimState(profile_name="test", gz_pid=p1.pid, px4_pid=p2.pid, started_at_wall_s=0.0),
            path=path,
        )
        result = stop_by_state(path=path, terminate_timeout_s=5.0)
        assert result is not None
        assert result.gz_pid == p1.pid
        assert result.px4_pid == p2.pid
        assert not path.exists()

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and (p1.poll() is None or p2.poll() is None):
            time.sleep(0.05)
        assert p1.poll() is not None
        assert p2.poll() is not None
    finally:
        for p in (p1, p2):
            if p.poll() is None:
                p.kill()
                p.wait(timeout=5)


def test_stop_by_state_handles_already_dead_pid_gracefully(tmp_path: Path):
    path = tmp_path / "sim_state.json"
    dead = subprocess.Popen([sys.executable, "-c", "pass"])
    dead.wait(timeout=5)
    write_state(
        SimState(profile_name="test", gz_pid=None, px4_pid=dead.pid, started_at_wall_s=0.0),
        path=path,
    )
    result = stop_by_state(path=path)  # must not raise even though the pid is already gone
    assert result is not None
    assert not path.exists()

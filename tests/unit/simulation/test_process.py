import sys
import time
from pathlib import Path

from aeris.simulation.launcher.process import ManagedProcess


def _sleeper(seconds: float = 5.0) -> list[str]:
    return [sys.executable, "-c", f"import time; time.sleep({seconds})"]


def test_start_reports_running_and_a_pid():
    proc = ManagedProcess(name="sleeper", argv=_sleeper())
    proc.start()
    try:
        assert proc.is_running() is True
        assert isinstance(proc.pid, int)
        assert proc.pid > 0
    finally:
        proc.stop()


def test_stop_terminates_and_reports_not_running():
    proc = ManagedProcess(name="sleeper", argv=_sleeper())
    proc.start()
    proc.stop()
    assert proc.is_running() is False


def test_stop_is_idempotent():
    proc = ManagedProcess(name="sleeper", argv=_sleeper())
    proc.start()
    proc.stop()
    proc.stop()  # must not raise


def test_stop_before_start_is_a_noop():
    proc = ManagedProcess(name="never-started", argv=_sleeper())
    assert proc.stop() is None
    assert proc.is_running() is False


def test_double_start_raises():
    proc = ManagedProcess(name="sleeper", argv=_sleeper())
    proc.start()
    try:
        import pytest

        with pytest.raises(RuntimeError):
            proc.start()
    finally:
        proc.stop()


def test_exit_code_after_natural_exit():
    proc = ManagedProcess(name="quick", argv=[sys.executable, "-c", "pass"])
    proc.start()
    deadline = time.monotonic() + 5.0
    while proc.is_running() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert proc.exit_code() == 0
    proc.stop()  # no-op, already exited; must not raise


def test_log_capture_writes_stdout_and_stderr_to_file(tmp_path: Path):
    log_path = tmp_path / "logs" / "out.log"
    proc = ManagedProcess(
        name="talker",
        argv=[
            sys.executable,
            "-c",
            "import sys; print('hello-out'); print('hello-err', file=sys.stderr)",
        ],
        log_path=log_path,
    )
    proc.start()
    deadline = time.monotonic() + 5.0
    while proc.is_running() and time.monotonic() < deadline:
        time.sleep(0.05)
    proc.stop()
    content = log_path.read_text()
    assert "hello-out" in content
    assert "hello-err" in content


def test_no_log_path_does_not_create_a_file(tmp_path: Path):
    proc = ManagedProcess(name="quiet", argv=[sys.executable, "-c", "print('noise')"])
    proc.start()
    proc.stop()
    assert list(tmp_path.iterdir()) == []

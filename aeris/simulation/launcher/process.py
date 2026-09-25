"""Tracked-PID subprocess lifecycle: launch, log-capture, graceful shutdown.

Spec §51 Phase 3: "Tracked-PID shutdown only" — every process AERIS starts
is tracked by its own `subprocess.Popen` handle and torn down explicitly
(SIGTERM, then SIGKILL if it doesn't exit in time). AERIS never kills by
name/pattern (``pkill -f gz``) — that risks taking down something the user
started themselves.
"""

from __future__ import annotations

import signal
import subprocess
import time
from collections.abc import Mapping, Sequence
from pathlib import Path

from aeris.core.clock import Clock, WallClock
from aeris.core.logging import get_logger

_logger = get_logger(component="simulation.process")


class ManagedProcess:
    """One externally-launched process, tracked by PID, with log capture.

    Not a context manager on purpose — callers (the launcher) need to start
    several of these in sequence with readiness checks in between, which
    doesn't fit a single ``with`` block cleanly. Call :meth:`stop` exactly
    once when done.
    """

    def __init__(
        self,
        *,
        name: str,
        argv: Sequence[str],
        cwd: Path | None = None,
        env: Mapping[str, str] | None = None,
        log_path: Path | None = None,
    ) -> None:
        self.name = name
        self._argv = list(argv)
        self._cwd = cwd
        self._env = dict(env) if env is not None else None
        self._log_path = log_path
        self._proc: subprocess.Popen[bytes] | None = None
        self._log_file: object | None = None  # a real file handle once started

    @property
    def pid(self) -> int | None:
        return self._proc.pid if self._proc is not None else None

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def exit_code(self) -> int | None:
        """The process's exit code, or ``None`` if it's still running or never started."""
        return self._proc.poll() if self._proc is not None else None

    def start(self) -> None:
        if self._proc is not None:
            raise RuntimeError(f"ManagedProcess {self.name!r} already started (pid={self.pid})")

        log_handle = None
        if self._log_path is not None:
            self._log_path.parent.mkdir(parents=True, exist_ok=True)
            log_handle = self._log_path.open("wb")
            self._log_file = log_handle

        _logger.info(
            "process.starting", name=self.name, argv=self._argv, log_path=str(self._log_path)
        )
        self._proc = subprocess.Popen(
            self._argv,
            cwd=str(self._cwd) if self._cwd else None,
            env=self._env,
            stdin=subprocess.DEVNULL,
            stdout=log_handle if log_handle is not None else subprocess.DEVNULL,
            stderr=subprocess.STDOUT if log_handle is not None else subprocess.DEVNULL,
            start_new_session=True,  # own process group, so a signal to the
            # pid doesn't also hit whatever launched AERIS itself
        )
        _logger.info("process.started", name=self.name, pid=self._proc.pid)

    def stop(self, *, terminate_timeout_s: float = 10.0, clock: Clock | None = None) -> int | None:
        """Terminate this process: SIGTERM, wait, SIGKILL if it's still alive.

        Idempotent — calling this on an already-stopped or never-started
        process is a no-op. Returns the exit code once known (``None`` if
        the process was never started).
        """
        if self._proc is None:
            return None
        if self._proc.poll() is not None:
            self._close_log()
            return self._proc.returncode

        pid = self._proc.pid
        _logger.info("process.stopping", name=self.name, pid=pid)
        self._proc.send_signal(signal.SIGTERM)

        clock = clock or WallClock()
        deadline = clock.now() + terminate_timeout_s
        while clock.now() < deadline:
            if self._proc.poll() is not None:
                break
            time.sleep(0.1)
        else:
            _logger.warning("process.sigterm_timeout_sigkill", name=self.name, pid=pid)
            self._proc.kill()
            self._proc.wait(timeout=5.0)

        code = self._proc.returncode
        _logger.info("process.stopped", name=self.name, pid=pid, exit_code=code)
        self._close_log()
        return code

    def _close_log(self) -> None:
        if self._log_file is not None:
            self._log_file.close()  # type: ignore[attr-defined]
            self._log_file = None

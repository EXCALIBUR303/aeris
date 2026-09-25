"""Clock abstraction.

Spec §13.2: "All autonomy runs on *simulation time* ... never wall time.
This keeps speed-factor changes and pauses correct." Every component that
needs "now" takes a :class:`Clock`, never calls ``time.time()`` directly.

May depend on ``aeris.core.types``/``units``/``errors`` (import-linter
contract: "core.clock may depend on types/units/errors, not vice versa").
"""

from __future__ import annotations

import time
from typing import Protocol, runtime_checkable


@runtime_checkable
class Clock(Protocol):
    """A source of the current simulation time, in seconds."""

    def now(self) -> float:
        """Current time in seconds. Monotonic within one run; never negative."""
        ...


class WallClock:
    """A :class:`Clock` backed by the host's monotonic clock.

    Used only where sim time genuinely doesn't apply — e.g. measuring this
    process's own wall-clock overhead for a manifest. Autonomy code must
    not use this for anything that affects behaviour (spec §13.2).
    """

    def __init__(self) -> None:
        self._t0 = time.monotonic()

    def now(self) -> float:
        return time.monotonic() - self._t0


class ManualClock:
    """A :class:`Clock` whose time is set explicitly.

    For unit tests (watchdog timing, GAE truncation logic, ...) that need
    deterministic control over "now" without a real simulator. Also usable
    as a minimal stand-in before :class:`aeris.simulation.launcher` exists
    to provide a real ``GzClock``.
    """

    def __init__(self, t0: float = 0.0) -> None:
        if t0 < 0.0:
            raise ValueError(f"ManualClock cannot start negative: t0={t0}")
        self._t = t0

    def now(self) -> float:
        return self._t

    def set(self, t: float) -> None:
        if t < 0.0:
            raise ValueError(f"ManualClock time cannot go negative: t={t}")
        self._t = t

    def advance(self, dt: float) -> float:
        if dt < 0.0:
            raise ValueError(f"ManualClock cannot advance by a negative dt={dt}")
        self._t += dt
        return self._t

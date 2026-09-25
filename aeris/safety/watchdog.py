"""S4 — watchdogs (spec §16.2): "Autonomy tick overdue > T_a -> hover
setpoint; sensor data stale > T_s -> brake + hover; link heartbeat lost ->
PX4 S0 handles; mission time budget exceeded -> RTL/Land."

Phase 5 implements the generic staleness primitive plus the two watchdogs
it has real inputs for: the autonomy tick (``fly_box.py`` acts as the
autonomy loop stand-in until Phase 6's mission executive exists) and
telemetry staleness (``VehicleState.link.last_heartbeat_age_s`` -- a
stand-in for "sensor data" until Phase 8's Sensor Bridge exists; link
heartbeat loss itself is explicitly PX4's own S0 job, not this one).
Mission time budget is Phase 6 scope (no mission executive exists yet).
"""

from __future__ import annotations

from aeris.core.clock import Clock


class StalenessWatchdog:
    """Trips when :meth:`mark` hasn't been called recently enough.

    ``age_s`` is infinite before the first :meth:`mark` -- an un-fed
    watchdog is stale by construction, never silently "not tripped yet."
    """

    def __init__(self, name: str, timeout_s: float, clock: Clock) -> None:
        if timeout_s <= 0.0:
            raise ValueError(f"timeout_s must be > 0, got {timeout_s}")
        self._name = name
        self._timeout_s = timeout_s
        self._clock = clock
        self._last_mark_s: float | None = None

    @property
    def name(self) -> str:
        return self._name

    @property
    def timeout_s(self) -> float:
        return self._timeout_s

    def mark(self) -> None:
        """Record that the thing this watchdog tracks just happened."""
        self._last_mark_s = self._clock.now()

    @property
    def age_s(self) -> float:
        if self._last_mark_s is None:
            return float("inf")
        return self._clock.now() - self._last_mark_s

    @property
    def is_stale(self) -> bool:
        return self.age_s > self._timeout_s

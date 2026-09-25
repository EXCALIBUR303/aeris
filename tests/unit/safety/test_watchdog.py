from __future__ import annotations

import pytest

from aeris.core.clock import ManualClock
from aeris.safety.watchdog import StalenessWatchdog


def test_unmarked_watchdog_is_stale() -> None:
    clock = ManualClock()
    wd = StalenessWatchdog("test", timeout_s=1.0, clock=clock)
    assert wd.age_s == float("inf")
    assert wd.is_stale


def test_marked_watchdog_is_not_stale_immediately() -> None:
    clock = ManualClock()
    wd = StalenessWatchdog("test", timeout_s=1.0, clock=clock)
    wd.mark()
    assert wd.age_s == 0.0
    assert not wd.is_stale


def test_watchdog_becomes_stale_after_timeout() -> None:
    clock = ManualClock()
    wd = StalenessWatchdog("test", timeout_s=1.0, clock=clock)
    wd.mark()
    clock.advance(0.99)
    assert not wd.is_stale
    clock.advance(0.02)
    assert wd.is_stale


def test_mark_resets_staleness() -> None:
    clock = ManualClock()
    wd = StalenessWatchdog("test", timeout_s=1.0, clock=clock)
    wd.mark()
    clock.advance(0.9)
    wd.mark()
    clock.advance(0.9)
    assert not wd.is_stale


def test_age_s_tracks_elapsed_time_exactly() -> None:
    clock = ManualClock()
    wd = StalenessWatchdog("test", timeout_s=5.0, clock=clock)
    wd.mark()
    clock.advance(2.5)
    assert wd.age_s == pytest.approx(2.5)


def test_rejects_non_positive_timeout() -> None:
    clock = ManualClock()
    with pytest.raises(ValueError):
        StalenessWatchdog("test", timeout_s=0.0, clock=clock)
    with pytest.raises(ValueError):
        StalenessWatchdog("test", timeout_s=-1.0, clock=clock)


def test_name_and_timeout_are_exposed() -> None:
    clock = ManualClock()
    wd = StalenessWatchdog("autonomy_tick", timeout_s=2.0, clock=clock)
    assert wd.name == "autonomy_tick"
    assert wd.timeout_s == 2.0

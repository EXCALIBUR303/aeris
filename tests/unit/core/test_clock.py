import pytest

from aeris.core.clock import Clock, ManualClock, WallClock


def test_manual_clock_starts_at_zero_by_default():
    c = ManualClock()
    assert c.now() == 0.0


def test_manual_clock_set_and_now():
    c = ManualClock()
    c.set(5.0)
    assert c.now() == 5.0


def test_manual_clock_advance_returns_new_time():
    c = ManualClock(t0=1.0)
    assert c.advance(2.5) == pytest.approx(3.5)
    assert c.now() == pytest.approx(3.5)


def test_manual_clock_rejects_negative_start():
    with pytest.raises(ValueError):
        ManualClock(t0=-1.0)


def test_manual_clock_rejects_negative_set():
    c = ManualClock()
    with pytest.raises(ValueError):
        c.set(-0.1)


def test_manual_clock_rejects_negative_advance():
    c = ManualClock()
    with pytest.raises(ValueError):
        c.advance(-0.1)


def test_wall_clock_is_monotonic_nonnegative_and_nondecreasing():
    c = WallClock()
    t1 = c.now()
    t2 = c.now()
    assert t1 >= 0.0
    assert t2 >= t1


def test_manual_clock_and_wall_clock_satisfy_clock_protocol():
    assert isinstance(ManualClock(), Clock)
    assert isinstance(WallClock(), Clock)

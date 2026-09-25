import math

import pytest

from aeris.core.units import (
    deg_to_rad,
    ms_to_s,
    rad_to_deg,
    us_to_s,
    wrap_pi,
)


def test_deg_rad_round_trip():
    for deg in [-720.0, -180.0, -1.0, 0.0, 1.0, 90.0, 180.0, 359.0, 720.0]:
        assert rad_to_deg(deg_to_rad(deg)) == pytest.approx(deg)


def test_deg_to_rad_known_values():
    assert deg_to_rad(180.0) == pytest.approx(math.pi)
    assert deg_to_rad(90.0) == pytest.approx(math.pi / 2)
    assert deg_to_rad(0.0) == pytest.approx(0.0)


@pytest.mark.parametrize(
    "angle_rad,expected",
    [
        (0.0, 0.0),
        (math.pi, math.pi),
        (-math.pi, math.pi),  # wraps to the closed +pi end, per docstring
        (3 * math.pi, math.pi),
        (-3 * math.pi, math.pi),
        (math.pi / 2, math.pi / 2),
        (2.5 * math.pi, math.pi / 2),
    ],
)
def test_wrap_pi(angle_rad: float, expected: float):
    assert wrap_pi(angle_rad) == pytest.approx(expected, abs=1e-9)


def test_wrap_pi_always_in_range():
    for k in range(-10, 11):
        w = wrap_pi(k * 0.7)
        assert -math.pi < w <= math.pi + 1e-9


def test_us_to_s_and_ms_to_s():
    assert us_to_s(1_000_000) == pytest.approx(1.0)
    assert ms_to_s(1_000) == pytest.approx(1.0)
    assert us_to_s(0) == 0.0

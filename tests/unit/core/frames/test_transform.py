import math
import random

import pytest

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import Vec3

_RNG = random.Random(20260925)


def _random_transform() -> Transform:
    axis = Vec3(_RNG.uniform(-1, 1), _RNG.uniform(-1, 1), _RNG.uniform(-1, 1)).normalized()
    angle = _RNG.uniform(-math.pi, math.pi)
    q = Quaternion.from_axis_angle(axis, angle)
    t = Vec3(_RNG.uniform(-10, 10), _RNG.uniform(-10, 10), _RNG.uniform(-10, 10))
    return Transform(q, t)


def test_identity_apply_is_noop():
    p = Vec3(1, 2, 3)
    assert Transform.identity().apply(p) == pytest.approx(p)


def test_from_translation_only_translates():
    t = Transform.from_translation(Vec3(1, 2, 3))
    result = t.apply(Vec3(0, 0, 0))
    assert result == pytest.approx(Vec3(1, 2, 3))


def test_apply_vector_ignores_translation():
    t = Transform(Quaternion.identity(), Vec3(100, 100, 100))
    v = Vec3(1, 2, 3)
    result = t.apply_vector(v)
    assert result.x == pytest.approx(v.x)
    assert result.y == pytest.approx(v.y)
    assert result.z == pytest.approx(v.z)


def test_compose_matches_sequential_apply():
    for _ in range(50):
        t_ab = _random_transform()
        t_bc = _random_transform()
        p_c = Vec3(_RNG.uniform(-5, 5), _RNG.uniform(-5, 5), _RNG.uniform(-5, 5))

        via_compose = t_ab.compose(t_bc).apply(p_c)
        via_sequential = t_ab.apply(t_bc.apply(p_c))

        assert via_compose.x == pytest.approx(via_sequential.x, abs=1e-8)
        assert via_compose.y == pytest.approx(via_sequential.y, abs=1e-8)
        assert via_compose.z == pytest.approx(via_sequential.z, abs=1e-8)


def test_inverse_round_trips():
    for _ in range(50):
        t = _random_transform()
        p_a = Vec3(_RNG.uniform(-5, 5), _RNG.uniform(-5, 5), _RNG.uniform(-5, 5))
        p_b = t.inverse().apply(p_a)
        round_tripped = t.apply(p_b)
        assert round_tripped.x == pytest.approx(p_a.x, abs=1e-8)
        assert round_tripped.y == pytest.approx(p_a.y, abs=1e-8)
        assert round_tripped.z == pytest.approx(p_a.z, abs=1e-8)


def test_compose_with_inverse_is_identity():
    for _ in range(20):
        t = _random_transform()
        combined = t.compose(t.inverse())
        p = Vec3(_RNG.uniform(-5, 5), _RNG.uniform(-5, 5), _RNG.uniform(-5, 5))
        result = combined.apply(p)
        assert result.x == pytest.approx(p.x, abs=1e-7)
        assert result.y == pytest.approx(p.y, abs=1e-7)
        assert result.z == pytest.approx(p.z, abs=1e-7)

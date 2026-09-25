import pytest

from aeris.core.frames.vector import ZERO, Vec3


def test_add_sub_neg():
    a = Vec3(1, 2, 3)
    b = Vec3(4, 5, 6)
    assert a + b == Vec3(5, 7, 9)
    assert b - a == Vec3(3, 3, 3)
    assert -a == Vec3(-1, -2, -3)


def test_scale():
    assert Vec3(1, 2, 3).scale(2.0) == Vec3(2, 4, 6)


def test_dot():
    assert Vec3(1, 0, 0).dot(Vec3(0, 1, 0)) == 0.0
    assert Vec3(1, 2, 3).dot(Vec3(1, 2, 3)) == 14.0


def test_cross_right_handed():
    assert Vec3(1, 0, 0).cross(Vec3(0, 1, 0)) == Vec3(0, 0, 1)
    assert Vec3(0, 1, 0).cross(Vec3(0, 0, 1)) == Vec3(1, 0, 0)


def test_norm():
    assert Vec3(3, 4, 0).norm() == pytest.approx(5.0)
    assert ZERO.norm() == 0.0


def test_normalized():
    n = Vec3(3, 4, 0).normalized()
    assert n.norm() == pytest.approx(1.0)
    assert n.x == pytest.approx(0.6)
    assert n.y == pytest.approx(0.8)


def test_normalize_zero_raises():
    with pytest.raises(ValueError):
        ZERO.normalized()


def test_is_a_namedtuple_so_immutable_and_iterable():
    v = Vec3(1, 2, 3)
    x, y, z = v
    assert (x, y, z) == (1, 2, 3)
    with pytest.raises(AttributeError):
        v.x = 5  # type: ignore[misc]

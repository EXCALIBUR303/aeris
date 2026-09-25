import math
import random

import pytest

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import Vec3

_RNG = random.Random(20260925)  # fixed seed: reproducible property-style tests


def _random_unit_vector() -> Vec3:
    while True:
        v = Vec3(_RNG.uniform(-1, 1), _RNG.uniform(-1, 1), _RNG.uniform(-1, 1))
        if v.norm() > 1e-6:
            return v.normalized()


def _random_quaternion() -> Quaternion:
    axis = _random_unit_vector()
    angle = _RNG.uniform(-2 * math.pi, 2 * math.pi)
    return Quaternion.from_axis_angle(axis, angle)


def test_identity_rotates_nothing():
    v = Vec3(1, 2, 3)
    assert Quaternion.identity().rotate(v) == pytest.approx(v)


def test_from_axis_angle_known_case_90deg_about_z():
    q = Quaternion.from_axis_angle(Vec3(0, 0, 1), math.pi / 2)
    result = q.rotate(Vec3(1, 0, 0))
    assert result.x == pytest.approx(0, abs=1e-9)
    assert result.y == pytest.approx(1, abs=1e-9)
    assert result.z == pytest.approx(0, abs=1e-9)


def test_rotation_preserves_vector_length():
    for _ in range(50):
        q = _random_quaternion()
        v = Vec3(_RNG.uniform(-10, 10), _RNG.uniform(-10, 10), _RNG.uniform(-10, 10))
        assert q.rotate(v).norm() == pytest.approx(v.norm(), rel=1e-9)


def test_conjugate_is_inverse_rotation():
    for _ in range(50):
        q = _random_quaternion()
        v = Vec3(_RNG.uniform(-5, 5), _RNG.uniform(-5, 5), _RNG.uniform(-5, 5))
        round_tripped = q.conjugate().rotate(q.rotate(v))
        assert round_tripped.x == pytest.approx(v.x, abs=1e-9)
        assert round_tripped.y == pytest.approx(v.y, abs=1e-9)
        assert round_tripped.z == pytest.approx(v.z, abs=1e-9)


def test_compose_matches_sequential_rotation():
    for _ in range(50):
        q_ab = _random_quaternion()
        q_bc = _random_quaternion()
        v_c = Vec3(_RNG.uniform(-5, 5), _RNG.uniform(-5, 5), _RNG.uniform(-5, 5))
        via_compose = q_ab.compose(q_bc).rotate(v_c)
        via_sequential = q_ab.rotate(q_bc.rotate(v_c))
        assert via_compose.x == pytest.approx(via_sequential.x, abs=1e-9)
        assert via_compose.y == pytest.approx(via_sequential.y, abs=1e-9)
        assert via_compose.z == pytest.approx(via_sequential.z, abs=1e-9)


def test_compose_renormalizes():
    q = _random_quaternion()
    composed = q.compose(q)
    assert composed.norm() == pytest.approx(1.0, abs=1e-12)


def test_normalized_has_unit_norm():
    q = Quaternion(2.0, 0.0, 0.0, 0.0)
    assert q.normalized().norm() == pytest.approx(1.0)


def test_normalize_near_zero_raises():
    with pytest.raises(ValueError):
        Quaternion(0.0, 0.0, 0.0, 0.0).normalized()


def test_is_close_handles_double_cover():
    q = _random_quaternion()
    negated = Quaternion(-q.w, -q.x, -q.y, -q.z)
    assert q.is_close(negated)


def test_is_close_false_for_different_rotations():
    q1 = Quaternion.from_axis_angle(Vec3(0, 0, 1), 0.1)
    q2 = Quaternion.from_axis_angle(Vec3(0, 0, 1), 1.0)
    assert not q1.is_close(q2)


def test_from_matrix_columns_identity():
    q = Quaternion.from_matrix_columns(Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1))
    assert q.is_close(Quaternion.identity())


def test_from_matrix_columns_90deg_about_z_matches_axis_angle():
    # Rz(90deg): e_x -> (0,1,0), e_y -> (-1,0,0), e_z -> (0,0,1)
    q_from_matrix = Quaternion.from_matrix_columns(Vec3(0, 1, 0), Vec3(-1, 0, 0), Vec3(0, 0, 1))
    q_from_axis_angle = Quaternion.from_axis_angle(Vec3(0, 0, 1), math.pi / 2)
    assert q_from_matrix.is_close(q_from_axis_angle)


def test_from_yaw_zero_is_identity():
    assert Quaternion.from_yaw(0.0).is_close(Quaternion.identity())


def test_from_yaw_matches_z_axis_angle():
    for yaw in [0.3, -1.2, math.pi / 2, -math.pi]:
        assert Quaternion.from_yaw(yaw).is_close(Quaternion.from_axis_angle(Vec3(0, 0, 1), yaw))

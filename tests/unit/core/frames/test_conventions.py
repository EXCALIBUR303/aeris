"""Spec §19.4's required frame-correctness tests.

These are the tests the spec calls out explicitly as mandatory for Phase
4: "Round trips: ENU->NED->ENU and FLU->FRD->FLU are identity ... Yaw
conversion at the cardinal points ... Known camera ray ... A frame-graph
test" (the last of those doesn't apply yet — nothing publishes frame_ids
until Phase 8+).
"""

import math
import random

import pytest

from aeris.core.frames.conventions import (
    attitude_enu_body_to_ned_frd,
    attitude_ned_frd_to_enu_body,
    camera_optical_to_body_rotation,
    enu_to_ned_vec,
    enu_yaw_to_ned_yaw,
    flu_to_frd_vec,
    frd_to_flu_vec,
    map_enu_to_threejs,
    ned_to_enu_vec,
    ned_yaw_to_enu_yaw,
)
from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import Vec3
from aeris.core.units import wrap_pi

_RNG = random.Random(20260925)


def _random_vec(scale: float = 10.0) -> Vec3:
    return Vec3(
        _RNG.uniform(-scale, scale), _RNG.uniform(-scale, scale), _RNG.uniform(-scale, scale)
    )


# --- Round trips (spec §19.4, item 1) ---------------------------------------------


def test_enu_ned_round_trip_is_identity():
    for _ in range(100):
        v = _random_vec()
        round_tripped = ned_to_enu_vec(enu_to_ned_vec(v))
        assert round_tripped.x == pytest.approx(v.x, abs=1e-12)
        assert round_tripped.y == pytest.approx(v.y, abs=1e-12)
        assert round_tripped.z == pytest.approx(v.z, abs=1e-12)


def test_flu_frd_round_trip_is_identity():
    for _ in range(100):
        v = _random_vec()
        round_tripped = frd_to_flu_vec(flu_to_frd_vec(v))
        assert round_tripped.x == pytest.approx(v.x, abs=1e-12)
        assert round_tripped.y == pytest.approx(v.y, abs=1e-12)
        assert round_tripped.z == pytest.approx(v.z, abs=1e-12)


def test_enu_ned_matrix_is_an_involution():
    # Applying the conversion twice is identity for ANY vector, which is a
    # stronger statement than "round trip works" (it means the forward and
    # backward conversions literally use the same formula, per spec §19.3).
    for _ in range(50):
        v = _random_vec()
        twice = enu_to_ned_vec(enu_to_ned_vec(v))
        assert twice.x == pytest.approx(v.x, abs=1e-12)
        assert twice.y == pytest.approx(v.y, abs=1e-12)
        assert twice.z == pytest.approx(v.z, abs=1e-12)


def test_flu_frd_matrix_is_an_involution():
    for _ in range(50):
        v = _random_vec()
        twice = flu_to_frd_vec(flu_to_frd_vec(v))
        assert twice.x == pytest.approx(v.x, abs=1e-12)
        assert twice.y == pytest.approx(v.y, abs=1e-12)
        assert twice.z == pytest.approx(v.z, abs=1e-12)


def test_enu_ned_vec_known_values():
    # spec §19.3: (x_n, y_n, z_n) = (y_e, x_e, -z_e)
    assert enu_to_ned_vec(Vec3(1, 2, 3)) == Vec3(2, 1, -3)
    # East unit vector -> NED +Y (East)
    assert enu_to_ned_vec(Vec3(1, 0, 0)) == Vec3(0, 1, 0)
    # North unit vector -> NED +X (North)
    assert enu_to_ned_vec(Vec3(0, 1, 0)) == Vec3(1, 0, 0)
    # Up unit vector -> NED -Z (Down is +Z, so Up is -Z)
    assert enu_to_ned_vec(Vec3(0, 0, 1)) == Vec3(0, 0, -1)


def test_flu_frd_vec_known_values():
    assert flu_to_frd_vec(Vec3(1, 0, 0)) == Vec3(1, 0, 0)  # Forward unchanged
    assert flu_to_frd_vec(Vec3(0, 1, 0)) == Vec3(0, -1, 0)  # Left -> -Right
    assert flu_to_frd_vec(Vec3(0, 0, 1)) == Vec3(0, 0, -1)  # Up -> -Down


def test_attitude_round_trip_is_identity():
    for _ in range(50):
        axis = _random_vec(1.0)
        if axis.norm() < 1e-6:
            continue
        q_enu = Quaternion.from_axis_angle(axis.normalized(), _RNG.uniform(-math.pi, math.pi))
        round_tripped = attitude_ned_frd_to_enu_body(attitude_enu_body_to_ned_frd(q_enu))
        assert q_enu.is_close(round_tripped, atol=1e-9)


# --- Yaw at the cardinal points (spec §19.4, item 2) -------------------------------


def test_enu_yaw_zero_east_maps_to_ned_yaw_north_minus_quarter_turn():
    # spec: "ENU yaw 0 (East) <-> NED yaw pi/2"
    assert enu_yaw_to_ned_yaw(0.0) == pytest.approx(math.pi / 2)


def test_enu_yaw_north_maps_to_ned_yaw_zero():
    # spec: "ENU pi/2 (North) <-> NED 0"
    assert enu_yaw_to_ned_yaw(math.pi / 2) == pytest.approx(0.0, abs=1e-12)


def test_ned_yaw_inverse_matches_at_cardinal_points():
    assert ned_yaw_to_enu_yaw(math.pi / 2) == pytest.approx(0.0, abs=1e-12)
    assert ned_yaw_to_enu_yaw(0.0) == pytest.approx(math.pi / 2)


def test_yaw_round_trip():
    for _ in range(100):
        yaw = _RNG.uniform(-4 * math.pi, 4 * math.pi)
        wrapped = wrap_pi(yaw)
        round_tripped = ned_yaw_to_enu_yaw(enu_yaw_to_ned_yaw(wrapped))
        # both endpoints already wrapped to (-pi, pi]; compare directly
        assert round_tripped == pytest.approx(wrapped, abs=1e-9)


# --- Known camera ray (spec §19.4, item 3) -----------------------------------------


def test_known_camera_ray_at_north_heading_points_north():
    """spec §19.4: "a point 1 m in front of the camera (optical (0,0,1))
    with the vehicle at psi_ENU = pi/2 maps to +y (North) in O."

    Camera mounted facing forward with no extra rotation (T_B_C = identity
    beyond the fixed optical<->body convention), vehicle body aligned with
    its yaw (no roll/pitch), positioned at the map/odom origin.
    """
    point_in_camera_optical = Vec3(0.0, 0.0, 1.0)  # 1 m along the boresight

    r_b_copt = camera_optical_to_body_rotation()
    point_in_body = r_b_copt.rotate(point_in_camera_optical)
    # Sanity: with no mount rotation, optical +z (forward) is body +x
    # (Forward) exactly, per spec §19.3's stated point mapping.
    assert point_in_body.x == pytest.approx(1.0, abs=1e-9)
    assert point_in_body.y == pytest.approx(0.0, abs=1e-9)
    assert point_in_body.z == pytest.approx(0.0, abs=1e-9)

    yaw_enu = math.pi / 2  # vehicle facing North
    q_o_b = Quaternion.from_yaw(yaw_enu)
    point_in_odom = q_o_b.rotate(point_in_body)

    assert point_in_odom.x == pytest.approx(0.0, abs=1e-9)  # no East component
    assert point_in_odom.y == pytest.approx(1.0, abs=1e-9)  # +y = North
    assert point_in_odom.z == pytest.approx(0.0, abs=1e-9)


def test_camera_optical_to_body_point_mappings_exactly_as_specified():
    r = camera_optical_to_body_rotation()
    # spec §19.3: "optical +z -> body +x, optical +x -> body -y, optical +y -> body -z"
    ex = r.rotate(Vec3(1, 0, 0))
    ey = r.rotate(Vec3(0, 1, 0))
    ez = r.rotate(Vec3(0, 0, 1))
    assert ex == pytest.approx(Vec3(0, -1, 0), abs=1e-9)
    assert ey == pytest.approx(Vec3(0, 0, -1), abs=1e-9)
    assert ez == pytest.approx(Vec3(1, 0, 0), abs=1e-9)


# --- three.js mapping (spec §19.3, used by the frontend from Phase 27) -------------


def test_map_enu_to_threejs_known_values():
    # spec: (x_V, y_V, z_V) = (x_M, z_M, -y_M)
    assert map_enu_to_threejs(Vec3(1, 2, 3)) == Vec3(1, 3, -2)
    # Up (ENU +z) -> three.js +Y (up)
    assert map_enu_to_threejs(Vec3(0, 0, 1)) == Vec3(0, 1, 0)
    # North (ENU +y) -> three.js -Z
    assert map_enu_to_threejs(Vec3(0, 1, 0)) == Vec3(0, 0, -1)

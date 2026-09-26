"""Unit + property tests for the S3 collision shield (spec §16.4, Phase 10).

Spec's own required tests: "property-based tests confirm that no shielded
command moves toward an obstacle closer than d_safe" and "stopping-distance
checks" -- both covered here. "Property-based" in this codebase means a
fixed-seed ``random.Random`` driving many samples in a loop (the pattern
established in ``tests/unit/core/frames/test_conventions.py``), not the
``hypothesis`` library (not a project dependency).
"""

from __future__ import annotations

import math
import random

import pytest

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import Vec3
from aeris.safety.shield import (
    N_SECTORS,
    SECTOR_WIDTH_RAD,
    CollisionShield,
    ShieldConfig,
    compute_sector_clearances,
    fov_sector_indices,
    sector_direction,
    sector_index,
    sector_v_max,
)

_RNG = random.Random(20260926)


# --- sector geometry ---------------------------------------------------------------


def test_sector_index_and_direction_are_inverse_at_sector_centers() -> None:
    for i in range(N_SECTORS):
        direction = sector_direction(i)
        assert sector_index(direction) == i


def test_sector_index_wraps_around_zero() -> None:
    assert sector_index(Vec3(1.0, -0.001, 0.0)) == N_SECTORS - 1
    assert sector_index(Vec3(1.0, 0.001, 0.0)) == 0


def test_fov_sector_indices_is_symmetric_about_forward() -> None:
    fov = fov_sector_indices(math.radians(36.5))
    for i in fov:
        angle = (i + 0.5) * SECTOR_WIDTH_RAD
        # mirrored angle should also be in FOV
        mirrored_angle = -angle if angle <= math.pi else 2 * math.pi - angle
        mirrored_index = sector_index(Vec3(math.cos(mirrored_angle), math.sin(mirrored_angle), 0.0))
        assert mirrored_index in fov


def test_compute_sector_clearances_defaults_fov_to_max_range_with_no_points() -> None:
    fov = fov_sector_indices(math.radians(36.5))
    clearances = compute_sector_clearances([], max_range_m=15.0, fov_sectors=fov)
    for i in fov:
        assert clearances[i] == 15.0
    for i in range(N_SECTORS):
        if i not in fov:
            assert clearances[i] is None


def test_compute_sector_clearances_uses_nearest_point_in_sector() -> None:
    fov = fov_sector_indices(math.radians(36.5))
    points = [Vec3(5.0, 0.0, 0.0), Vec3(2.0, 0.0, 0.0)]  # both land in sector 0
    clearances = compute_sector_clearances(points, max_range_m=15.0, fov_sectors=fov)
    assert clearances[0] == pytest.approx(2.0)


def test_compute_sector_clearances_ignores_points_outside_fov() -> None:
    fov = fov_sector_indices(math.radians(36.5))
    # A point straight behind (sector ~36, angle=180deg) is outside this FOV.
    points = [Vec3(-1.0, 0.0, 0.0)]
    clearances = compute_sector_clearances(points, max_range_m=15.0, fov_sectors=fov)
    behind_index = sector_index(Vec3(-1.0, 0.0, 0.0))
    assert behind_index not in fov
    assert clearances[behind_index] is None


# --- sector_v_max formula (spec §16.4) ----------------------------------------------


def test_sector_v_max_zero_at_or_below_d_safe() -> None:
    assert sector_v_max(0.5, a_brake_mps2=2.0, d_safe_m=0.5, tau_s=0.3) == 0.0
    assert sector_v_max(0.3, a_brake_mps2=2.0, d_safe_m=0.5, tau_s=0.3) == 0.0


def test_sector_v_max_matches_hand_computed_value() -> None:
    # d_i=2.5, d_safe=0.5 -> sqrt(2*2*2.0) - 2*0.3 = sqrt(8) - 0.6
    expected = math.sqrt(8.0) - 0.6
    assert sector_v_max(2.5, a_brake_mps2=2.0, d_safe_m=0.5, tau_s=0.3) == pytest.approx(expected)


def test_sector_v_max_increases_with_clearance() -> None:
    v1 = sector_v_max(1.0, a_brake_mps2=2.0, d_safe_m=0.5, tau_s=0.3)
    v2 = sector_v_max(5.0, a_brake_mps2=2.0, d_safe_m=0.5, tau_s=0.3)
    assert v2 > v1


# --- CollisionShield behavior --------------------------------------------------------


def test_clear_forward_path_passes_through_unshielded() -> None:
    shield = CollisionShield()
    v_cmd = Vec3(1.0, 0.0, 0.0)
    v_out, intervened = shield.project(
        v_cmd, orientation_odom_body=Quaternion.identity(), points_body=[]
    )
    assert v_out == v_cmd
    assert not intervened


def test_close_obstacle_ahead_is_blocked() -> None:
    shield = CollisionShield(config=ShieldConfig(d_safe_m=0.5))
    v_out, intervened = shield.project(
        Vec3(1.0, 0.0, 0.0),
        orientation_odom_body=Quaternion.identity(),
        points_body=[Vec3(0.3, 0.0, 0.0)],
    )
    assert v_out == Vec3(0.0, 0.0, 0.0)
    assert intervened


def test_lateral_motion_with_no_sideways_data_is_blocked() -> None:
    shield = CollisionShield()
    v_out, intervened = shield.project(
        Vec3(0.0, 1.0, 0.0), orientation_odom_body=Quaternion.identity(), points_body=[]
    )
    assert v_out == Vec3(0.0, 0.0, 0.0)
    assert intervened


def test_backward_motion_with_no_rear_data_is_blocked() -> None:
    shield = CollisionShield()
    v_out, intervened = shield.project(
        Vec3(-1.0, 0.0, 0.0), orientation_odom_body=Quaternion.identity(), points_body=[]
    )
    assert v_out == Vec3(0.0, 0.0, 0.0)
    assert intervened


def test_off_axis_heading_near_fov_boundary_still_passes_when_clear() -> None:
    """Regression: a heading a few degrees off dead-ahead must not spuriously
    zero out just because its "relevant cone" would have grazed an
    unobserved sector just past the FOV edge (the second shield bug found
    live during Phase 10)."""
    shield = CollisionShield()
    v_cmd = Vec3(1.4985723323727866, -0.0654290810480043, 0.0)  # ~-2.5 deg off-axis
    v_out, intervened = shield.project(
        v_cmd, orientation_odom_body=Quaternion.identity(), points_body=[Vec3(10.0, 0.0, 0.0)]
    )
    assert v_out == v_cmd
    assert not intervened


def test_vertical_component_passes_through_unaffected() -> None:
    shield = CollisionShield()
    v_out, _intervened = shield.project(
        Vec3(1.0, 0.0, 0.5),
        orientation_odom_body=Quaternion.identity(),
        points_body=[Vec3(10.0, 0.0, 0.0)],
    )
    assert v_out.z == pytest.approx(0.5)


def test_zero_command_is_never_intervened() -> None:
    shield = CollisionShield()
    v_out, intervened = shield.project(
        Vec3(0.0, 0.0, 0.0), orientation_odom_body=Quaternion.identity(), points_body=[]
    )
    assert v_out == Vec3(0.0, 0.0, 0.0)
    assert not intervened


def test_intervention_rate_tracks_ticks() -> None:
    shield = CollisionShield()
    shield.project(
        Vec3(1.0, 0, 0), orientation_odom_body=Quaternion.identity(), points_body=[Vec3(10, 0, 0)]
    )
    shield.project(
        Vec3(1.0, 0, 0), orientation_odom_body=Quaternion.identity(), points_body=[Vec3(0.2, 0, 0)]
    )
    assert shield.tick_count == 2
    assert shield.intervention_count == 1
    assert shield.intervention_rate == pytest.approx(0.5)


def test_in_fov_sector_updates_to_clear_on_a_later_empty_reading() -> None:
    """An in-FOV sector is re-derived from *this tick's* points, not stuck on
    stale data: a close obstacle that's moved away must not keep blocking
    once the sensor genuinely reports that sector clear again."""
    shield = CollisionShield()
    shield.project(
        Vec3(1.0, 0, 0), orientation_odom_body=Quaternion.identity(), points_body=[Vec3(0.3, 0, 0)]
    )
    v_out, intervened = shield.project(
        Vec3(1.0, 0, 0), orientation_odom_body=Quaternion.identity(), points_body=[]
    )
    assert v_out == Vec3(1.0, 0.0, 0.0)
    assert not intervened


def test_out_of_fov_sector_last_seen_clearance_is_never_overwritten_by_in_fov_points() -> None:
    """spec §16.4's "last-seen clearance" memory is scoped to sectors a
    sensor can actually cover -- points landing in the (fixed, forward)
    FOV must never touch an out-of-FOV sector's own stored value, however
    it got there (e.g. a future wider sensor)."""
    shield = CollisionShield()
    behind_index = sector_index(Vec3(-1.0, 0.0, 0.0))
    assert behind_index not in shield._fov_sectors
    shield._last_seen_clearance_m[behind_index] = 3.5  # simulate an earlier observation

    shield.project(
        Vec3(1.0, 0, 0), orientation_odom_body=Quaternion.identity(), points_body=[Vec3(10.0, 0, 0)]
    )

    assert shield._last_seen_clearance_m[behind_index] == 3.5


# --- property tests (spec §16.4's explicit requirement) -----------------------------


def test_property_shielded_command_never_moves_toward_a_closer_than_d_safe_obstacle() -> None:
    """For many random (command, obstacle) pairs, the shielded command's
    component toward the obstacle's own sector never exceeds that
    sector's v_i,max -- and if the obstacle is within d_safe, that
    component is exactly zero.
    """
    config = ShieldConfig(a_brake_mps2=2.0, d_safe_m=0.5, tau_s=0.3)
    for _ in range(300):
        speed = _RNG.uniform(0.0, 5.0)
        heading = _RNG.uniform(-math.pi, math.pi)
        v_cmd = Vec3(speed * math.cos(heading), speed * math.sin(heading), 0.0)

        obstacle_range = _RNG.uniform(0.05, 10.0)
        # Place the obstacle exactly along v_cmd's own heading, guaranteeing
        # it's one of the checked "near-heading" sectors.
        obstacle = Vec3(obstacle_range * math.cos(heading), obstacle_range * math.sin(heading), 0.0)

        shield = CollisionShield(config=config)
        v_out, _ = shield.project(
            v_cmd, orientation_odom_body=Quaternion.identity(), points_body=[obstacle]
        )

        obstacle_sector = sector_index(obstacle)
        u_i = sector_direction(obstacle_sector)
        component_toward_obstacle = v_out.x * u_i.x + v_out.y * u_i.y
        v_i_max = sector_v_max(
            obstacle_range,
            a_brake_mps2=config.a_brake_mps2,
            d_safe_m=config.d_safe_m,
            tau_s=config.tau_s,
        )

        assert component_toward_obstacle <= v_i_max + 1e-9

        if obstacle_range <= config.d_safe_m:
            assert component_toward_obstacle <= 1e-9


def test_property_shield_never_increases_speed() -> None:
    """The shield only ever shrinks a command -- its output norm never
    exceeds the input norm, for any random command/observation pair."""
    for _ in range(300):
        speed = _RNG.uniform(0.0, 5.0)
        heading = _RNG.uniform(-math.pi, math.pi)
        v_cmd = Vec3(speed * math.cos(heading), speed * math.sin(heading), _RNG.uniform(-2.0, 2.0))

        n_points = _RNG.randint(0, 5)
        points = [Vec3(_RNG.uniform(-10, 10), _RNG.uniform(-10, 10), 0.0) for _ in range(n_points)]

        shield = CollisionShield()
        v_out, _ = shield.project(
            v_cmd, orientation_odom_body=Quaternion.identity(), points_body=points
        )

        assert v_out.norm() <= v_cmd.norm() + 1e-9


def test_property_stopping_distance_invariant() -> None:
    """For a shielded speed toward an obstacle at range d_i, decelerating at
    a_brake starting after latency tau covers no more than (d_i - d_safe):
    a direct analytic integration of the deceleration profile, standing in
    for spec's own "checks against the FastSim dynamics" (FastSim doesn't
    exist until Phase 13)."""
    config = ShieldConfig(a_brake_mps2=2.0, d_safe_m=0.5, tau_s=0.3)
    for _ in range(200):
        d_i = _RNG.uniform(config.d_safe_m, 20.0)
        v_i_max = sector_v_max(
            d_i, a_brake_mps2=config.a_brake_mps2, d_safe_m=config.d_safe_m, tau_s=config.tau_s
        )

        # Distance covered: latency phase at constant v_i_max for tau_s,
        # then constant deceleration a_brake to a stop.
        latency_distance = v_i_max * config.tau_s
        braking_distance = v_i_max**2 / (2.0 * config.a_brake_mps2)
        total_stopping_distance = latency_distance + braking_distance

        assert total_stopping_distance <= (d_i - config.d_safe_m) + 1e-6

"""aeris.learning.spaces: provenance guard, spec hashing, cross-call-site
identity (spec §27.6), decoder envelope/validator compliance, and the
exploration map/crop/mask semantics."""

from __future__ import annotations

import math

import numpy as np
import pytest

from aeris.autonomy.exploration.base import AgentPose
from aeris.core.errors import OracleObservationError
from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import Vec3
from aeris.core.types import Provenance
from aeris.learning.spaces.exploration import (
    FREE,
    OCCUPIED,
    ROTATE_ACTION,
    DenseAgentMap,
    ExplorationSpaceConfig,
    action_mask,
    build_observation,
    candidate_targets,
    exploration_spec,
)
from aeris.learning.spaces.local_nav import (
    LocalNavSpaceConfig,
    body_frame_inputs,
    build_observation_batch,
    build_observation_from_vehicle_state,
    decode_actions_batch,
    local_nav_spec,
)
from aeris.learning.spaces.spec import FieldSpec, ObservationSpec, verify_checkpoint_spec
from aeris.mapping.projection import BandGrid
from aeris.safety.envelope import load_envelope
from aeris.safety.validator import CommandValidator
from aeris.vehicle.interface import (
    BatterySimState,
    EkfFlags,
    FlightMode,
    GpsFixType,
    GpsStatus,
    LandedState,
    LinkStatus,
    VehicleState,
    VelocitySetpoint,
)

_CFG = LocalNavSpaceConfig()


def test_provenance_guard_raises_on_oracle_fields_unless_declared() -> None:
    spec = ObservationSpec("x", "v", (FieldSpec("gt_pose", (3,), -1, 1, Provenance.ORACLE),))
    with pytest.raises(OracleObservationError):
        spec.check_provenance(oracle_baseline=False)
    spec.check_provenance(oracle_baseline=True)


def test_shipped_specs_contain_no_oracle_fields() -> None:
    for spec in (local_nav_spec(_CFG), exploration_spec(ExplorationSpaceConfig())):
        spec.check_provenance(oracle_baseline=False)
        assert all(f.provenance is not Provenance.ORACLE for f in spec.fields)


def test_spec_hash_is_stable_and_sensitive_to_changes() -> None:
    a, b = local_nav_spec(_CFG), local_nav_spec(_CFG)
    assert a.hash() == b.hash()
    c = local_nav_spec(LocalNavSpaceConfig(image_height=24, image_width=32))
    assert c.hash() != a.hash()
    verify_checkpoint_spec(a.hash(), b)
    with pytest.raises(ValueError):
        verify_checkpoint_spec(c.hash(), a)


def _vehicle_state(x: float, y: float, yaw: float, vx: float, vy: float, r: float) -> VehicleState:
    return VehicleState(
        t_sim_s=0.0,
        armed=True,
        flight_mode=FlightMode.OFFBOARD,
        landed_state=LandedState.IN_AIR,
        pose_odom=Vec3(x, y, 1.0),
        orientation_odom=Quaternion.from_yaw(yaw),
        velocity_odom_mps=Vec3(vx, vy, 0.0),
        angular_velocity_body_radps=Vec3(0.0, 0.0, r),
        home_odom=None,
        gps=GpsStatus(fix_type=GpsFixType.FIX_3D, satellites_visible=10, eph_m=0.5, epv_m=0.8),
        battery_sim=BatterySimState(remaining_fraction=0.9, voltage_v=16.0),
        ekf_flags=EkfFlags(raw_flags=0, pos_horiz_accuracy_m=0.1, pos_vert_accuracy_m=0.1),
        link=LinkStatus(connected=True, last_heartbeat_age_s=0.1),
    )


def test_tier_h_and_fastsim_call_sites_produce_identical_observations() -> None:
    rng = np.random.default_rng(0)
    full = rng.uniform(0.3, 12.0, (480, 640)).astype(np.float32)
    full[rng.random((480, 640)) < 0.1] = np.inf
    x, y, yaw, vx, vy, r = 1.2, -0.7, 0.9, 0.4, -0.3, 0.2
    goal = (6.0, 3.0)
    prev = np.array([0.1, -0.2, 0.3])

    tier_h = build_observation_from_vehicle_state(
        _CFG,
        depth_image_full=full,
        state=_vehicle_state(x, y, yaw, vx, vy, r),
        goal_odom_xy=goal,
        prev_action=prev,
        time_remaining_frac=0.5,
    )
    goal_b, vel_b = body_frame_inputs(
        np.array([[x, y]]), np.array([yaw]), np.array([[vx, vy]]), np.array([goal])
    )
    fastsim = build_observation_batch(
        _CFG,
        depth=full[::16, ::16][None],
        goal_body_xy=goal_b,
        vel_body_xy=vel_b,
        yaw_rate=np.array([r]),
        prev_action=prev[None],
        time_remaining_frac=np.array([0.5]),
    )
    for k in tier_h:
        np.testing.assert_allclose(tier_h[k], fastsim[k][0], rtol=0, atol=1e-6)
    space = local_nav_spec(_CFG).gym_space()
    assert space.contains({k: v.astype(np.float32) for k, v in tier_h.items()})


def test_goal_straight_ahead_has_zero_bearing() -> None:
    goal_b, _ = body_frame_inputs(
        np.array([[0.0, 0.0]]), np.array([math.pi / 2]), np.zeros((1, 2)), np.array([[0.0, 5.0]])
    )
    np.testing.assert_allclose(goal_b[0], [5.0, 0.0], atol=1e-12)


def test_decoder_clamps_to_envelope() -> None:
    cmd = decode_actions_batch(
        _CFG, np.array([[5.0, 5.0, -3.0]]), np.zeros(1), np.array([[1.4, 1.4, 0.0]])
    )
    assert math.hypot(cmd[0, 0], cmd[0, 1]) <= _CFG.v_max_mps + 1e-9
    assert cmd[0, 2] == pytest.approx(-_CFG.yaw_rate_max_radps)


def test_decoded_command_stream_is_always_accepted_by_the_s1_validator() -> None:
    """Random policy actions at 10 Hz, decoded, must never trip the real S1
    validator's implied-acceleration check (or any other rule)."""
    envelope = load_envelope("configs/vehicle/safety.yaml")
    validator = CommandValidator(envelope)
    state = _vehicle_state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    rng = np.random.default_rng(1)
    prev = np.zeros((1, 3))
    yaw = 0.0
    for t in range(300):
        a = rng.uniform(-1, 1, (1, 3))
        cmd = decode_actions_batch(_CFG, a, np.array([yaw]), prev)
        sp = VelocitySetpoint(
            velocity=Vec3(float(cmd[0, 0]), float(cmd[0, 1]), 0.0),
            yaw_rate_radps=float(cmd[0, 2]),
            frame="odom",
        )
        res = validator.validate(sp, state=state, now_s=t * _CFG.control_dt_s)
        assert res.accepted, res.reason
        prev = cmd
        yaw += cmd[0, 2] * _CFG.control_dt_s


def _grid() -> BandGrid:
    free = {(x, y) for x in range(-20, 21) for y in range(-20, 21)}
    occ = {(10, y) for y in range(-5, 6)}
    return BandGrid(0.2, (-25, -25), (25, 25), frozenset(occ), frozenset(free - occ))


def test_dense_map_round_trips_through_bandgrid() -> None:
    g = _grid()
    back = DenseAgentMap.from_bandgrid(g).to_bandgrid()
    assert back.occupied == g.occupied
    assert back.free == g.free
    assert (back.min_cell, back.max_cell) == (g.min_cell, g.max_cell)


def test_heading_aligned_crop_puts_an_obstacle_ahead_in_the_top_rows() -> None:
    cfg = ExplorationSpaceConfig()
    m = DenseAgentMap.from_bandgrid(_grid())
    # Wall at x ~ 2.0 m; facing +x -> the wall is ahead (rows above center).
    ahead = build_observation(
        cfg, agent_map=m, pose_xy=(0.1, 0.1), yaw=0.0, speed_mps=0.0, time_remaining_s=90.0
    )
    occ_rows = np.nonzero(ahead["map"][1].any(axis=1))[0]
    assert occ_rows.size and occ_rows.max() < cfg.crop_size // 2
    # Facing -x -> the same wall is behind (rows below center).
    behind = build_observation(
        cfg, agent_map=m, pose_xy=(0.1, 0.1), yaw=math.pi, speed_mps=0.0, time_remaining_s=90.0
    )
    occ_rows = np.nonzero(behind["map"][1].any(axis=1))[0]
    assert occ_rows.size and occ_rows.min() >= cfg.crop_size // 2
    # Every pixel is exactly one of free/occupied/unknown, at both scales.
    for base in (0, 4):
        np.testing.assert_array_equal(ahead["map"][base : base + 3].sum(axis=0), 1.0)
    assert exploration_spec(cfg).gym_space().contains(ahead)


def test_coarse_pooling_never_drops_a_lone_occupied_cell() -> None:
    free = {(x, y) for x in range(-10, 11) for y in range(-10, 11)}
    g = BandGrid(0.2, (-12, -12), (12, 12), frozenset({(5, 1)}), frozenset(free - {(5, 1)}))
    obs = build_observation(
        ExplorationSpaceConfig(),
        agent_map=DenseAgentMap.from_bandgrid(g),
        pose_xy=(0.0, 0.0),
        yaw=0.0,
        speed_mps=0.0,
        time_remaining_s=1.0,
    )
    assert obs["map"][5].sum() >= 1.0  # coarse occupied channel


def test_action_mask_is_bearing_major_and_rotate_is_always_valid() -> None:
    free = {(x, y) for x in range(-40, 41) for y in range(-40, 41)}
    wall = {(x, y) for x in range(4, 41) for y in range(-40, 41)}  # blocks everything ahead (+x)
    g = BandGrid(0.2, (-45, -45), (45, 45), frozenset(wall), frozenset(free - wall))
    targets = candidate_targets(g, AgentPose(Vec3(0.1, 0.1, 1.0), 0.0))
    mask = action_mask(targets)
    assert mask[ROTATE_ACTION]
    # bearing 0 (straight ahead), long range (index 1): lands in the wall -> masked
    assert not mask[1]
    # bearing 6 (straight behind), both ranges -> open space -> valid
    assert mask[12] and mask[13]
    m = DenseAgentMap.from_bandgrid(g)
    assert m.state[45, 45] == FREE and m.state[45 + 10, 45] == OCCUPIED

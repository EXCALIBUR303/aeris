"""Local-navigation observation/action spaces (spec §26.2, §29), shared
verbatim by FastSim training and Tier H deployment.

Both tiers reduce their raw data to the same primitive inputs first --
a strided z-depth image in the camera's own pixel grid, the estimated
horizontal pose (x, y, yaw) and odom-frame velocity, the goal in the odom
frame -- and then call the *same* functions below. Nothing tier-specific
happens after that point, which is what makes "identical outputs for
identical inputs across both call sites" (spec §27.6) true by
construction; a test drives both call sites to prove it.

The action decoder does two things the policy can't be trusted to do:

- clamps the command to the S1 envelope (speed, yaw rate), and
- **slew-limits** the odom-frame velocity change to ``accel_slew_mps2``
  per control step. Tier H's S1 validator *rejects* any setpoint implying
  more than ``accel_max_mps2`` (2.0 m/s^2) relative to the previous one
  (``aeris.safety.validator``) -- an unconstrained 10 Hz policy jumping
  more than 0.2 m/s in one tick would have its commands refused on the real
  vehicle. Limiting in the decoder, on both tiers, means every command the
  policy emits is one the validator accepts, and FastSim trains on exactly
  the command stream the vehicle will receive.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from aeris.core.frames.vector import Vec3
from aeris.core.types import Provenance
from aeris.learning.spaces.spec import FieldSpec, ObservationSpec
from aeris.vehicle.interface import VehicleState

SPEC_VERSION = "local_nav/v1"


@dataclass(frozen=True, slots=True)
class LocalNavSpaceConfig:
    image_height: int = 30  # 480 / depth_stride
    image_width: int = 40  # 640 / depth_stride
    depth_stride: int = 16
    depth_max_range_m: float = 10.0
    v_max_mps: float = 2.0  # within the 3.0 m/s S1 envelope
    yaw_rate_max_radps: float = 1.0  # == S1 envelope
    accel_slew_mps2: float = 1.8  # 90% of the S1 validator's 2.0 m/s^2
    control_dt_s: float = 0.1  # 10 Hz (spec §26.2)
    goal_range_scale_m: float = 20.0


def local_nav_spec(cfg: LocalNavSpaceConfig) -> ObservationSpec:
    return ObservationSpec(
        name="local_nav",
        version=SPEC_VERSION,
        fields=(
            FieldSpec(
                "depth",
                (1, cfg.image_height, cfg.image_width),
                0.0,
                1.0,
                Provenance.SENSOR,
                "z-depth / depth_max_range_m, no-return = 1.0",
            ),
            FieldSpec(
                "state",
                (10,),
                -1.0,
                1.0,
                Provenance.ESTIMATE,
                "goal range, goal bearing sin/cos (body), body vx, vy, yaw rate, "
                "previous action (3, the agent's own last command), time remaining. "
                "Goal comes from the mission (MISSION_INPUT) but is expressed "
                "relative to the *estimated* pose, so the field as a whole is ESTIMATE.",
            ),
        ),
    )


def body_frame_inputs(
    pose_xy: np.ndarray, yaw: np.ndarray, vel_odom_xy: np.ndarray, goal_odom_xy: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Goal and velocity rotated into the body frame. Batched: ``[N, 2]``, ``[N]``."""
    c, s = np.cos(yaw), np.sin(yaw)
    g = goal_odom_xy - pose_xy
    goal_b = np.stack([c * g[:, 0] + s * g[:, 1], -s * g[:, 0] + c * g[:, 1]], axis=1)
    v = vel_odom_xy
    vel_b = np.stack([c * v[:, 0] + s * v[:, 1], -s * v[:, 0] + c * v[:, 1]], axis=1)
    return goal_b, vel_b


def build_observation_batch(
    cfg: LocalNavSpaceConfig,
    *,
    depth: np.ndarray,  # [N, H, W] z-depth (m), inf/nan = no return
    goal_body_xy: np.ndarray,  # [N, 2]
    vel_body_xy: np.ndarray,  # [N, 2]
    yaw_rate: np.ndarray,  # [N]
    prev_action: np.ndarray,  # [N, 3] in [-1, 1]
    time_remaining_frac: np.ndarray,  # [N] in [0, 1]
) -> dict[str, np.ndarray]:
    n = depth.shape[0]
    d = np.where(np.isfinite(depth), depth, cfg.depth_max_range_m)
    d = np.clip(d / cfg.depth_max_range_m, 0.0, 1.0).astype(np.float32)
    rng = np.hypot(goal_body_xy[:, 0], goal_body_xy[:, 1])
    bearing = np.arctan2(goal_body_xy[:, 1], goal_body_xy[:, 0])
    state = np.empty((n, 10), dtype=np.float32)
    state[:, 0] = np.clip(rng / cfg.goal_range_scale_m, 0.0, 1.0)
    state[:, 1] = np.sin(bearing)
    state[:, 2] = np.cos(bearing)
    state[:, 3:5] = np.clip(vel_body_xy / cfg.v_max_mps, -1.0, 1.0)
    state[:, 5] = np.clip(yaw_rate / cfg.yaw_rate_max_radps, -1.0, 1.0)
    state[:, 6:9] = np.clip(prev_action, -1.0, 1.0)
    state[:, 9] = np.clip(time_remaining_frac, 0.0, 1.0)
    return {"depth": d[:, None, :, :], "state": state}


def build_observation_from_vehicle_state(
    cfg: LocalNavSpaceConfig,
    *,
    depth_image_full: np.ndarray,  # [480, 640] as published by the Sensor Bridge
    state: VehicleState,
    goal_odom_xy: tuple[float, float],
    prev_action: np.ndarray,  # [3]
    time_remaining_frac: float,
) -> dict[str, np.ndarray]:
    """The Tier H call site: a live ``VehicleState`` + a full-resolution
    depth frame, reduced to the same primitives FastSim produces."""
    head = state.orientation_odom.rotate(Vec3(1.0, 0.0, 0.0))
    yaw = math.atan2(head.y, head.x)
    pose = np.array([[state.pose_odom.x, state.pose_odom.y]])
    vel = np.array([[state.velocity_odom_mps.x, state.velocity_odom_mps.y]])
    goal_b, vel_b = body_frame_inputs(pose, np.array([yaw]), vel, np.array([goal_odom_xy]))
    s = cfg.depth_stride
    depth = depth_image_full[::s, ::s][None, : cfg.image_height, : cfg.image_width]
    obs = build_observation_batch(
        cfg,
        depth=depth,
        goal_body_xy=goal_b,
        vel_body_xy=vel_b,
        yaw_rate=np.array([state.angular_velocity_body_radps.z]),
        prev_action=prev_action[None, :],
        time_remaining_frac=np.array([time_remaining_frac]),
    )
    return {k: v[0] for k, v in obs.items()}


def decode_actions_batch(
    cfg: LocalNavSpaceConfig,
    action: np.ndarray,  # [N, 3] in [-1, 1]: body vx, body vy, yaw rate
    yaw: np.ndarray,  # [N] estimated yaw
    prev_cmd_odom: np.ndarray,  # [N, 3]: previous odom-frame (vx, vy, yaw rate)
) -> np.ndarray:
    """Policy action -> odom-frame (vx, vy, yaw rate) command, envelope-clamped
    and slew-limited so the S1 validator never rejects it."""
    a = np.clip(action, -1.0, 1.0)
    vb = a[:, :2] * cfg.v_max_mps
    speed = np.hypot(vb[:, 0], vb[:, 1])
    vb = (
        vb * np.where(speed > cfg.v_max_mps, cfg.v_max_mps / np.maximum(speed, 1e-12), 1.0)[:, None]
    )
    c, s = np.cos(yaw), np.sin(yaw)
    vo = np.stack([c * vb[:, 0] - s * vb[:, 1], s * vb[:, 0] + c * vb[:, 1]], axis=1)
    dv = vo - prev_cmd_odom[:, :2]
    dn = np.hypot(dv[:, 0], dv[:, 1])
    lim = cfg.accel_slew_mps2 * cfg.control_dt_s
    dv = dv * np.where(dn > lim, lim / np.maximum(dn, 1e-12), 1.0)[:, None]
    out = np.empty((action.shape[0], 3))
    out[:, :2] = prev_cmd_odom[:, :2] + dv
    out[:, 2] = a[:, 2] * cfg.yaw_rate_max_radps
    return out

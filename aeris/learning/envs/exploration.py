"""Decision-level learned exploration in FastSim (spec §30, §26.2, ADR-0015).

One env step = one *decision*: a 25-way masked egocentric subgoal (or a
rotate-in-place scan), executed by the same machinery Phase 12's Tier H
classical baselines use -- A* on the agent's own inflated map, waypoint-by-
waypoint :class:`~aeris.autonomy.navigation.follower.PathFollower`, the S3
shield -- until arrival, a significant map change, a timeout, or a
collision. :meth:`ExplorationEnv.execute_subgoal` is that execution loop,
exposed so scripted classical policies (validation gate item 6) can drive
the *identical* loop the learned policy will.

Execution mirrors :func:`aeris.evaluation.exploration_episode.run_exploration_episode`
parameter for parameter (arrival radius, cruise speed, 20-cell map-change
replan, 45 s stuck timeout, 2 s rotate scan at 0.6 rad/s, 0.3 m inflation,
+/-8 m map window around the spawn), with two measured-not-assumed knobs:

- ``control_dt_s``: nominally 0.1 s; Tier H's own loop actually ran at
  ~1.5-2 Hz because of its synchronous GT-pose poll.
- ``face_travel_direction``: Phase 12's Tier H loop commands yaw rate 0 while
  path-following, so the vehicle flies legs in any direction with a
  fixed-heading camera and a shield whose out-of-FOV sectors start blocked.

Fallbacks are Tier H's too: an A* failure or a masked action becomes the
2 s rotate scan (flagged ``failed``), and A* starts from the raw cell under
the pose. One Tier H behavior is *not* built in: its first replan runs
before any depth frame is integrated and so always opens with a rotate
scan; a scripted runner that wants that must issue it (the gate-6 parity
script does).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from aeris.autonomy.exploration.base import AgentPose, world_to_cell
from aeris.autonomy.navigation.follower import PathFollower
from aeris.autonomy.planning.astar import astar
from aeris.core.frames.vector import Vec3
from aeris.learning.envs.local_nav import SplitMode, _check_split
from aeris.learning.envs.rewards import (
    EXPLORATION_REWARD_VERSION,
    ExplorationRewardWeights,
    exploration_reward,
)
from aeris.learning.spaces.exploration import (
    N_ACTIONS,
    ROTATE_ACTION,
    DenseAgentMap,
    ExplorationSpaceConfig,
    action_mask,
    build_observation,
    candidate_targets,
    exploration_spec,
)
from aeris.mapping.projection import BandGrid, Cell2D, cell_center_world
from aeris.simulation.fastsim.agent_map import (
    dilate_occupied,
    integrate_rays_2d,
    map_state,
)
from aeris.simulation.fastsim.batch import CameraRig, FastSimBatch, PoseNoiseParams
from aeris.simulation.fastsim.dynamics import DynamicsParams
from aeris.simulation.fastsim.world import WorldBank, build_world
from aeris.simulation.worlds.spec import WorldSpec

PRIVILEGED_REWARD_NOTES = ("collision judged from ground-truth geometry (reward/termination only)",)


@dataclass(frozen=True, slots=True)
class ExplorationEnvConfig:
    space: ExplorationSpaceConfig = field(default_factory=ExplorationSpaceConfig)
    reward: ExplorationRewardWeights = field(default_factory=ExplorationRewardWeights)
    dynamics: DynamicsParams = field(default_factory=DynamicsParams)
    pose_noise: PoseNoiseParams = field(default_factory=PoseNoiseParams)
    world_resolution_m: float = 0.1
    # base_link world z: Tier H hovers at 1.0 m *odom*, and odom's origin is
    # base_link at the arm point, 0.24 m above the gz model origin (+0.1 m
    # spawn) -- see aeris.simulation.fastsim.batch.GZ_MODEL_TO_BASE_LINK_M.
    altitude_m: float = 1.34
    control_dt_s: float = 0.1
    depth_stride: int = 16
    depth_max_range_m: float = 15.0  # Phase 11/12 mapping max_range_m
    map_half_width_m: float = 8.0  # Phase 12's x_range_half_width_m
    l_occ: float = 0.85
    l_free: float = -0.4
    inflation_m: float = 0.3
    cruise_speed_mps: float = 1.0
    arrival_radius_m: float = 0.4
    max_decision_s: float = 45.0
    replan_map_change_cells: int = 20
    rotate_yaw_rate_radps: float = 0.6
    rotate_duration_s: float = 2.0
    face_travel_direction: bool = False
    yaw_gain: float = 1.5
    yaw_rate_max_radps: float = 1.0
    gt_log_period_s: float = 0.5

    @staticmethod
    def identified(**overrides: Any) -> ExplorationEnvConfig:
        root = Path(__file__).resolve().parents[3] / "configs" / "fastsim"
        base = ExplorationEnvConfig(
            dynamics=DynamicsParams.from_yaml(root / "dynamics.yaml"),
            pose_noise=PoseNoiseParams.from_yaml(root / "pose_noise.yaml"),
        )
        return replace(base, **overrides)


@dataclass(frozen=True, slots=True)
class DecisionResult:
    duration_s: float
    new_area_m2: float
    collided: bool
    shield_interventions: int
    arrived: bool
    failed: bool
    ticks: int
    rotated: bool = False  # executed a rotate scan (chosen, masked, or an A* fallback)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ExplorationEnv(gym.Env[dict[str, np.ndarray], int]):
    metadata = {"render_modes": []}  # noqa: RUF012

    def __init__(
        self,
        worlds: Sequence[WorldSpec],
        *,
        config: ExplorationEnvConfig | None = None,
        split_mode: SplitMode = "train",
    ) -> None:
        _check_split(worlds, split_mode)
        config = config or ExplorationEnvConfig()
        if abs(config.space.fine_res_m - 0.2) > 1e-9:
            raise ValueError("the agent map is built at the spec's 0.2 m fine resolution")
        self.cfg = config
        self.worlds = list(worlds)
        self.obs_spec = exploration_spec(config.space)
        self.obs_spec.check_provenance(oracle_baseline=False)
        self.bank = WorldBank.from_worlds(
            [build_world(w, resolution_m=config.world_resolution_m) for w in worlds]
        )
        self.rig = CameraRig.load(stride=config.depth_stride)
        self.observation_space = self.obs_spec.gym_space()
        self.action_space = spaces.Discrete(N_ACTIONS)
        self.reward_version = EXPLORATION_REWARD_VERSION
        self.privileged_reward_notes = PRIVILEGED_REWARD_NOTES
        self.follower = PathFollower(
            cruise_speed_mps=config.cruise_speed_mps, arrival_radius_m=config.arrival_radius_m
        )
        self._rng = np.random.default_rng(0)
        self.sim: FastSimBatch | None = None

    # -- setup ------------------------------------------------------------------
    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        super().reset(seed=seed)
        self._rng = np.random.default_rng(seed)
        opts = options or {}
        w = int(opts.get("world_index", self._rng.integers(len(self.worlds))))
        world = self.bank.worlds[w]
        cfg = self.cfg
        self.sim = FastSimBatch(
            bank=self.bank,
            rig=self.rig,
            dyn_params=[cfg.dynamics],
            pose_noise=cfg.pose_noise,
            control_dt_s=cfg.control_dt_s,
            depth_max_range_m=cfg.depth_max_range_m,
            seed=int(self._rng.integers(2**31)),
        )
        spawn = world.spawn_world
        self.sim.reset_envs(
            np.array([0]),
            world=np.array([w]),
            pos=np.array([[spawn[0], spawn[1], cfg.altitude_m]]),
            yaw=np.array([world.spawn_yaw_rad]),
        )
        res = cfg.space.fine_res_m
        half = cfg.map_half_width_m
        self._min_cell = (math.floor((spawn[0] - half) / res), math.floor((spawn[1] - half) / res))
        n = math.floor(2 * half / res) + 1
        self._logodds = np.zeros((n, n))
        self._touched = np.zeros((n, n), dtype=np.bool_)
        self._visited = np.zeros((n, n), dtype=np.bool_)
        self._z_band = world.altitude_band_m
        self.t_s = 0.0
        self._next_gt_log = 0.0
        self.gt_trajectory: list[tuple[float, Vec3, float]] = []
        self.shield_ticks = 0
        self.shield_interventions = 0
        self.decisions = 0
        self._est_xy, self._est_yaw = self._tick_sense()
        return self._observe(), self._info()

    # -- per-tick sensing & mapping ------------------------------------------------
    def _tick_sense(self) -> tuple[np.ndarray, float]:
        assert self.sim is not None
        sim, cfg = self.sim, self.cfg
        sim.render()
        xy, yaw = sim.estimated_pose()
        ex, ey, eyaw = float(xy[0, 0]), float(xy[0, 1]), float(yaw[0])
        # The agent registers its rays with its *estimated* pose.
        c, s = math.cos(eyaw), math.sin(eyaw)
        r_wb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        cam = np.array([ex, ey, sim.dyn.state.pos[0, 2]]) + r_wb @ self.rig.t_body_optical
        dirs = self.rig.dirs_optical @ (r_wb @ self.rig.r_body_optical).T
        ranges = sim.depth[0].reshape(-1) / self.rig.z_factor
        integrate_rays_2d(
            self._logodds,
            self._touched,
            self._min_cell[0],
            self._min_cell[1],
            cfg.space.fine_res_m,
            cam,
            dirs,
            ranges,
            cfg.depth_max_range_m,
            self._z_band[0],
            self._z_band[1],
            cfg.l_occ,
            cfg.l_free,
            -4.0,
            4.0,
        )
        i = math.floor(ex / cfg.space.fine_res_m) - self._min_cell[0]
        j = math.floor(ey / cfg.space.fine_res_m) - self._min_cell[1]
        if 0 <= i < self._visited.shape[0] and 0 <= j < self._visited.shape[1]:
            self._visited[i, j] = True
        if self.t_s + 1e-9 >= self._next_gt_log:
            p = sim.dyn.state.pos[0]
            self.gt_trajectory.append((self.t_s, Vec3(*map(float, p)), float(sim.dyn.state.yaw[0])))
            self._next_gt_log += cfg.gt_log_period_s
        return np.array([ex, ey]), eyaw

    def _state(self) -> np.ndarray:
        return map_state(self._logodds, self._touched)

    def agent_map(self) -> DenseAgentMap:
        return DenseAgentMap(
            self._state(), self._visited.copy(), self._min_cell, self.cfg.space.fine_res_m
        )

    def planning_grid(self) -> BandGrid:
        """The agent's own map, inflated like ``project_band(inflation_m=...)``."""
        radius = math.ceil(self.cfg.inflation_m / self.cfg.space.fine_res_m)
        st = dilate_occupied(self._state(), radius)
        return DenseAgentMap(
            st, self._visited, self._min_cell, self.cfg.space.fine_res_m
        ).to_bandgrid()

    def agent_pose(self) -> AgentPose:
        assert self.sim is not None
        return AgentPose(
            Vec3(
                float(self._est_xy[0]), float(self._est_xy[1]), float(self.sim.dyn.state.pos[0, 2])
            ),
            self._est_yaw,
        )

    def _observe(self) -> dict[str, np.ndarray]:
        assert self.sim is not None
        speed = float(np.hypot(*self.sim.dyn.state.vel[0]))
        return build_observation(
            self.cfg.space,
            agent_map=self.agent_map(),
            pose_xy=(float(self._est_xy[0]), float(self._est_xy[1])),
            yaw=self._est_yaw,
            speed_mps=speed,
            time_remaining_s=max(0.0, self.cfg.space.episode_time_s - self.t_s),
        )

    def _info(self, targets: list[Cell2D | None] | None = None) -> dict[str, Any]:
        if targets is None:
            targets = candidate_targets(self.planning_grid(), self.agent_pose())
        self._targets = targets
        return {"action_mask": action_mask(targets), "t_s": self.t_s}

    @property
    def time_up(self) -> bool:
        return self.t_s >= self.cfg.space.episode_time_s - 1e-9

    # -- subgoal execution ----------------------------------------------------------
    def execute_subgoal(self, target: Cell2D | None, *, failed: bool = False) -> DecisionResult:
        """Execute one decision: fly the A* path to ``target`` (or, for
        ``None``, a rotate-in-place scan), exactly as Phase 12's Tier H
        episode loop does between two replans -- including its fallbacks:
        an A* failure becomes a rotate scan (never a zero-time no-op, which
        would let a policy stall the episode clock), and the map-change
        replan trigger diffs the *inflated* planning grid tick-to-tick, as
        ``DeltaTracker.diff`` does on Tier H's ``project_band`` output.
        ``failed=True`` marks a decision that was already known to be
        unexecutable (a masked action) so it is penalized as failed."""
        assert self.sim is not None
        cfg, sim = self.cfg, self.sim
        self.decisions += 1
        known0 = int(self._touched.sum())
        radius = math.ceil(cfg.inflation_m / cfg.space.fine_res_m)
        prev = dilate_occupied(self._state(), radius)
        path: list[Vec3] = []
        if target is not None:
            grid = self.planning_grid()
            # Tier H plans from the raw floor cell of the EKF position (no
            # snapping); an occupied/unknown start fails A* like it does there.
            plan = astar(grid, world_to_cell(grid, self.agent_pose().position_m), target)
            if plan.reason != "ok" or not plan.path_cells:
                failed, target = True, None  # Tier H: fall back to a rotate scan
            else:
                path = [cell_center_world(grid, c, z_m=cfg.altitude_m) for c in plan.path_cells]
        wp = 1 if len(path) > 1 else 0
        t0, ticks, interventions = self.t_s, 0, 0
        arrived = collided = False
        limit = cfg.rotate_duration_s if target is None else cfg.max_decision_s
        while not self.time_up:
            pos = Vec3(float(self._est_xy[0]), float(self._est_xy[1]), cfg.altitude_m)
            if target is None:
                v, r = Vec3(0.0, 0.0, 0.0), cfg.rotate_yaw_rate_radps
            else:
                while wp < len(path) and (pos - path[wp]).norm() <= cfg.arrival_radius_m:
                    wp += 1
                if wp >= len(path):
                    arrived = True
                    if ticks:
                        break
                # Arrived before moving (target inside the arrival radius):
                # Tier H still spends this loop iteration as a real tick, so
                # hover for one tick -- a zero-time decision would let the
                # strategy re-pick the same target forever with the clock frozen.
                v = (
                    Vec3(0.0, 0.0, 0.0)
                    if arrived
                    else self.follower.compute_velocity(pos, path[wp])
                )
                r = 0.0
                if cfg.face_travel_direction and not arrived and math.hypot(v.x, v.y) > 0.05:
                    err = _wrap(math.atan2(v.y, v.x) - self._est_yaw)
                    r = max(
                        -cfg.yaw_rate_max_radps, min(cfg.yaw_rate_max_radps, cfg.yaw_gain * err)
                    )
            hit, shielded = sim.step_control(np.array([[v.x, v.y, r]]))
            self.t_s += cfg.control_dt_s
            ticks += 1
            self.shield_ticks += 1
            if shielded[0]:
                interventions += 1
                self.shield_interventions += 1
            self._est_xy, self._est_yaw = self._tick_sense()
            if hit[0]:
                collided = True
                break
            if arrived:
                break
            if self.t_s - t0 >= limit - 1e-9:
                failed = failed or target is not None  # a stuck timeout, not a scan finishing
                break
            if target is not None:
                now = dilate_occupied(self._state(), radius)
                changed = int(np.count_nonzero(now != prev))
                prev = now
                if changed >= cfg.replan_map_change_cells:
                    break
        return self._finish(
            known0,
            self.t_s - t0,
            collided,
            interventions,
            arrived=arrived,
            failed=failed,
            ticks=ticks,
            rotated=target is None,
        )

    def _finish(
        self,
        known0: int,
        duration: float,
        collided: bool,
        interventions: int,
        *,
        arrived: bool,
        failed: bool,
        ticks: int,
        rotated: bool,
    ) -> DecisionResult:
        new_area = (int(self._touched.sum()) - known0) * self.cfg.space.fine_res_m**2
        return DecisionResult(
            duration, new_area, collided, interventions, arrived, failed, ticks, rotated
        )

    # -- gym API ------------------------------------------------------------------------
    def step(self, action: int) -> tuple[dict[str, np.ndarray], float, bool, bool, dict[str, Any]]:
        a = int(action)
        if a == ROTATE_ACTION:
            res = self.execute_subgoal(None)
        elif self._targets[a] is None:  # masked action: Tier H's fallback scan, penalized as failed
            res = self.execute_subgoal(None, failed=True)
        else:
            res = self.execute_subgoal(self._targets[a])
        reward = exploration_reward(
            self.cfg.reward,
            new_explored_m2=res.new_area_m2,
            decision_duration_s=res.duration_s,
            collided=res.collided,
            shield_interventions=res.shield_interventions,
            subgoal_failed=res.failed,
        )
        terminated = res.collided
        truncated = not terminated and self.time_up
        info = self._info()
        info.update(decision=res)
        return self._observe(), float(reward), terminated, truncated, info

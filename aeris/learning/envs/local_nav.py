"""PointGoal local navigation in FastSim (spec §29, §26.2): a body-frame
``(v_x, v_y, yaw_rate)`` policy at 10 Hz, shielded, reaching a goal in a
cluttered world at a fixed altitude.

One batched core (:class:`LocalNavCore`) serves both a single-instance
``gymnasium.Env`` (``check_env``, generic tooling) and a native batched
``gymnasium.vector.VectorEnv`` (training throughput) -- the two cannot
drift apart because there is only one implementation.

**Termination vs truncation (spec §51 Phase 13 research consideration).**
``terminated`` = a true terminal state of the MDP: collision (the episode
genuinely ends -- the vehicle would be damaged) or success (goal reached).
``truncated`` = the time limit or leaving the world's bounds: the state is
*not* terminal, the episode is just cut off, so a learner must bootstrap
from the value of the final observation rather than treat it as zero.

**Split discipline (spec §33.2).** ``split_mode="train"`` refuses any world
whose split isn't ``train`` or whose seed isn't in the train range
(:func:`aeris.simulation.worlds.splits.require_training_seed`);
``split_mode="val"`` refuses test worlds. Test worlds never enter FastSim.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Literal

import gymnasium as gym
import numpy as np
import yaml
from gymnasium import spaces
from gymnasium.vector import AutoresetMode, VectorEnv

from aeris.core.errors import SplitViolationError
from aeris.learning.envs.rewards import (
    LOCAL_NAV_REWARD_VERSION,
    LocalNavRewardWeights,
    local_nav_reward,
)
from aeris.learning.spaces.local_nav import (
    LocalNavSpaceConfig,
    body_frame_inputs,
    build_observation_batch,
    decode_actions_batch,
    local_nav_spec,
)
from aeris.simulation.fastsim.batch import (
    VEHICLE_RADIUS_M,
    CameraRig,
    FastSimBatch,
    PoseNoiseParams,
    geodesic_field,
    slab_blocked,
)
from aeris.simulation.fastsim.dynamics import DynamicsParams
from aeris.simulation.fastsim.world import WorldBank, build_world
from aeris.simulation.worlds.spec import WorldSpec, WorldSplit
from aeris.simulation.worlds.splits import require_training_seed, require_tuning_seed

SplitMode = Literal["train", "val"]
PRIVILEGED_REWARD_NOTES = (
    "progress term uses geodesic distance on the FastSim ground-truth map (reward only)",
    "collision judged from ground-truth geometry (reward/termination only)",
)


@dataclass(frozen=True, slots=True)
class RandomizationRanges:
    """Per-episode multiplicative ranges around the identified dynamics
    (spec §51 Phase 13 task 7); ``(1.0, 1.0)`` disables an axis."""

    tau_scale: tuple[float, float] = (0.8, 1.25)
    delay_extra_s: tuple[float, float] = (-0.04, 0.06)
    accel_scale: tuple[float, float] = (0.6, 1.2)
    depth_noise_std_frac: tuple[float, float] = (0.0, 0.01)

    @staticmethod
    def from_yaml(path: Path) -> RandomizationRanges:
        """Load ``ranges`` from ``configs/fastsim/randomization.yaml`` (where
        each range's derivation from the sysid CIs is documented)."""
        r = yaml.safe_load(path.read_text())["ranges"]
        return RandomizationRanges(
            **{k: (float(r[k][0]), float(r[k][1])) for k in RandomizationRanges.__slots__}
        )


@dataclass(frozen=True, slots=True)
class LocalNavEnvConfig:
    space: LocalNavSpaceConfig = field(default_factory=LocalNavSpaceConfig)
    reward: LocalNavRewardWeights = field(default_factory=LocalNavRewardWeights)
    dynamics: DynamicsParams = field(default_factory=DynamicsParams)
    pose_noise: PoseNoiseParams = field(default_factory=PoseNoiseParams)
    randomization: RandomizationRanges | None = field(default_factory=RandomizationRanges)
    world_resolution_m: float = 0.1
    altitude_m: float = 1.0
    goal_radius_m: float = 0.5
    goal_distance_m: tuple[float, float] = (3.0, 12.0)
    episode_time_s: float = 60.0
    spawn_margin_m: float = 0.2
    vel_noise_std_mps: float = 0.02

    @staticmethod
    def identified(**overrides: Any) -> LocalNavEnvConfig:
        """The config backed by the committed Tier H fits
        (``configs/fastsim/{dynamics,pose_noise,randomization}.yaml``) rather
        than the dataclass placeholders -- what any real training run should use."""
        root = Path(__file__).resolve().parents[3] / "configs" / "fastsim"
        base = LocalNavEnvConfig(
            dynamics=DynamicsParams.from_yaml(root / "dynamics.yaml"),
            pose_noise=PoseNoiseParams.from_yaml(root / "pose_noise.yaml"),
            randomization=RandomizationRanges.from_yaml(root / "randomization.yaml"),
        )
        return replace(base, **overrides)


def _check_split(specs: Sequence[WorldSpec], mode: SplitMode) -> None:
    for s in specs:
        if mode == "train":
            if s.split is not WorldSplit.TRAIN:
                raise SplitViolationError(
                    f"train-mode env refuses {s.name} (split={s.split.value})"
                )
            require_training_seed(s.seed)
        else:
            if s.split not in (WorldSplit.TRAIN, WorldSplit.VAL):
                raise SplitViolationError(f"val-mode env refuses {s.name} (split={s.split.value})")
            require_tuning_seed(s.seed)


class LocalNavCore:
    """N FastSim local-nav episodes stepped in lockstep, with per-env autoreset."""

    def __init__(
        self,
        worlds: Sequence[WorldSpec],
        *,
        num_envs: int,
        config: LocalNavEnvConfig | None = None,
        split_mode: SplitMode = "train",
        seed: int = 0,
    ) -> None:
        _check_split(worlds, split_mode)
        config = config or LocalNavEnvConfig()
        self.cfg = config
        self.n = num_envs
        self.spec = local_nav_spec(config.space)
        self.spec.check_provenance(oracle_baseline=False)
        self.rng = np.random.default_rng(seed)
        self.bank = WorldBank.from_worlds(
            [build_world(w, resolution_m=config.world_resolution_m) for w in worlds]
        )
        for w in self.bank.worlds:
            lo, hi = w.altitude_band_m
            if not lo <= config.altitude_m <= hi:
                raise ValueError(
                    f"altitude {config.altitude_m} m outside {w.name}'s band {w.altitude_band_m}"
                )
        self._blocked = [
            slab_blocked(self.bank, i, config.altitude_m, VEHICLE_RADIUS_M + config.spawn_margin_m)
            for i in range(len(self.bank.worlds))
        ]
        self._free_cells = [np.argwhere(~b) for b in self._blocked]
        if any(len(fc) == 0 for fc in self._free_cells):
            raise ValueError("a world has no free cell at the flight altitude")
        rig = CameraRig.load(stride=config.space.depth_stride)
        if (rig.height, rig.width) != (config.space.image_height, config.space.image_width):
            raise ValueError("space image size does not match the camera rig at this stride")
        self.sim = FastSimBatch(
            bank=self.bank,
            rig=rig,
            dyn_params=[config.dynamics] * num_envs,
            pose_noise=config.pose_noise,
            control_dt_s=config.space.control_dt_s,
            depth_max_range_m=config.space.depth_max_range_m,
            seed=seed,
        )
        self.max_steps = round(config.episode_time_s / config.space.control_dt_s)
        self.goal = np.zeros((num_envs, 2))
        self.geo = [np.empty((0, 0))] * num_envs
        self.prev_geo = np.zeros(num_envs)
        self.prev_action = np.zeros((num_envs, 3))
        self.prev_cmd = np.zeros((num_envs, 3))
        self.t = np.zeros(num_envs, dtype=np.int64)
        self.episode_return = np.zeros(num_envs)
        self._est_yaw = np.zeros(num_envs)  # the estimate the last observation was built from

    # -- episode setup ----------------------------------------------------------
    def _sample_episode(self, e: int) -> None:
        cfg = self.cfg
        w = int(self.rng.integers(len(self.bank.worlds)))
        world = self.bank.worlds[w]
        res = world.resolution_m
        free = self._free_cells[w]
        lo, hi = cfg.goal_distance_m
        for _ in range(50):
            gi, gj = free[self.rng.integers(len(free))]
            field_ = geodesic_field(self._blocked[w], int(gi), int(gj), res)
            ok = np.argwhere(np.isfinite(field_) & (field_ >= lo) & (field_ <= hi))
            if len(ok):
                si, sj = ok[self.rng.integers(len(ok))]
                break
        else:
            raise RuntimeError(f"could not sample a start/goal pair in {world.name}")
        start = world.origin[:2] + (np.array([si, sj]) + 0.5) * res
        self.goal[e] = world.origin[:2] + (np.array([gi, gj]) + 0.5) * res
        self.geo[e] = field_
        yaw = self.rng.uniform(-math.pi, math.pi)
        pos = np.array([[start[0], start[1], cfg.altitude_m]])
        self.sim.reset_envs(np.array([e]), world=np.array([w]), pos=pos, yaw=np.array([yaw]))
        if cfg.randomization is not None:
            r, base = cfg.randomization, cfg.dynamics

            def u(ab: tuple[float, float]) -> float:
                return float(self.rng.uniform(*ab))

            p = replace(
                base,
                tau_xy_s=base.tau_xy_s * u(r.tau_scale),
                tau_yaw_s=base.tau_yaw_s * u(r.tau_scale),
                delay_xy_s=min(max(0.0, base.delay_xy_s + u(r.delay_extra_s)), 0.5),
                delay_yaw_s=min(max(0.0, base.delay_yaw_s + u(r.delay_extra_s)), 0.5),
                accel_max_xy_mps2=base.accel_max_xy_mps2 * u(r.accel_scale),
                yaw_accel_max_radps2=base.yaw_accel_max_radps2 * u(r.accel_scale),
            )
            self.sim.dyn.set_params(e, p)
            self.sim.depth_noise_std_frac = u(
                r.depth_noise_std_frac
            )  # batch-wide; resampled per reset
        self.prev_geo[e] = self._geodesic_at(e)
        self.prev_action[e] = 0.0
        self.prev_cmd[e] = 0.0
        self.t[e] = 0
        self.episode_return[e] = 0.0

    def _geodesic_at(self, e: int) -> float:
        w = self.sim.world_idx[e]
        world = self.bank.worlds[w]
        p = self.sim.dyn.state.pos[e]
        i = math.floor((p[0] - world.origin[0]) / world.resolution_m)
        j = math.floor((p[1] - world.origin[1]) / world.resolution_m)
        g = self.geo[e]
        if 0 <= i < g.shape[0] and 0 <= j < g.shape[1]:
            return float(g[i, j])
        return math.inf

    def _out_of_bounds(self, e: int) -> bool:
        w = self.bank.worlds[self.sim.world_idx[e]]
        p = self.sim.dyn.state.pos[e, :2]
        hi = w.origin[:2] + np.array(w.occ.shape[:2]) * w.resolution_m
        return bool(np.any(p < w.origin[:2]) or np.any(p > hi))

    # -- observation ---------------------------------------------------------------
    def observe(self) -> dict[str, np.ndarray]:
        self.sim.render()
        xy, yaw = self.sim.estimated_pose()
        self._est_yaw = yaw
        s = self.sim.dyn.state
        vel = s.vel + self.rng.normal(0.0, self.cfg.vel_noise_std_mps, s.vel.shape)
        goal_b, vel_b = body_frame_inputs(xy, yaw, vel, self.goal)
        remaining = 1.0 - self.t / self.max_steps
        return build_observation_batch(
            self.cfg.space,
            depth=self.sim.depth,
            goal_body_xy=goal_b,
            vel_body_xy=vel_b,
            yaw_rate=s.yaw_rate,
            prev_action=self.prev_action,
            time_remaining_frac=remaining,
        )

    def reset(self) -> dict[str, np.ndarray]:
        for e in range(self.n):
            self._sample_episode(e)
        return self.observe()

    def step(
        self, actions: np.ndarray
    ) -> tuple[dict[str, np.ndarray], np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
        """Advance every env one control step. Envs that end are reset in the
        same call; their final observation is in ``info["final_obs"]``."""
        actions = np.asarray(actions, dtype=np.float64).reshape(self.n, 3)
        # Decode with the *same* pose estimate the policy's observation was
        # built from -- as on the real vehicle, where one VehicleState feeds both.
        cmd = decode_actions_batch(self.cfg.space, actions, self._est_yaw, self.prev_cmd)
        collided, shielded = self.sim.step_control(cmd)
        self.t += 1
        geo = np.array([self._geodesic_at(e) for e in range(self.n)])
        dist = np.hypot(*(self.sim.dyn.state.pos[:, :2] - self.goal).T)
        success = (dist <= self.cfg.goal_radius_m) & ~collided
        reward = local_nav_reward(
            self.cfg.reward,
            prev_geodesic_m=self.prev_geo,
            geodesic_m=geo,
            collided=collided,
            shield_intervened=shielded,
            success=success,
        )
        oob = np.array([self._out_of_bounds(e) for e in range(self.n)])
        terminated = collided | success
        truncated = ~terminated & ((self.t >= self.max_steps) | oob)
        self.prev_geo = geo
        self.prev_action = np.clip(actions, -1.0, 1.0)
        self.prev_cmd = cmd
        self.episode_return += reward

        done = terminated | truncated
        info: dict[str, Any] = {
            "success": success.copy(),
            "collision": collided.copy(),
            "shield_intervened": shielded.copy(),
            "episode_return": np.where(done, self.episode_return, 0.0),
            "episode_length": np.where(done, self.t, 0),
        }
        obs = self.observe()
        if done.any():
            info["final_obs"] = {k: v.copy() for k, v in obs.items()}
            for e in np.nonzero(done)[0]:
                self._sample_episode(int(e))
            obs = self.observe()
        return obs, reward, terminated, truncated, info


class LocalNavEnv(gym.Env[dict[str, np.ndarray], np.ndarray]):
    """Single-instance Gymnasium wrapper over :class:`LocalNavCore` (N=1)."""

    metadata = {"render_modes": []}  # noqa: RUF012

    def __init__(
        self,
        worlds: Sequence[WorldSpec],
        *,
        config: LocalNavEnvConfig | None = None,
        split_mode: SplitMode = "train",
    ) -> None:
        config = config or LocalNavEnvConfig()
        self._worlds, self._config, self._split = list(worlds), config, split_mode
        self.core = LocalNavCore(self._worlds, num_envs=1, config=config, split_mode=split_mode)
        self.observation_space = self.core.spec.gym_space()
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self.reward_version = LOCAL_NAV_REWARD_VERSION
        self.privileged_reward_notes = PRIVILEGED_REWARD_NOTES

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        super().reset(seed=seed)
        if seed is not None:
            self.core = LocalNavCore(
                self._worlds, num_envs=1, config=self._config, split_mode=self._split, seed=seed
            )
        obs = self.core.reset()
        return {k: v[0] for k, v in obs.items()}, {}

    def step(
        self, action: np.ndarray
    ) -> tuple[dict[str, np.ndarray], float, bool, bool, dict[str, Any]]:
        obs, r, term, trunc, info = self.core.step(np.asarray(action)[None, :])
        if term[0] or trunc[0]:
            obs = info["final_obs"]  # single-env API: the caller resets explicitly
        return (
            {k: v[0] for k, v in obs.items()},
            float(r[0]),
            bool(term[0]),
            bool(trunc[0]),
            {"success": bool(info["success"][0]), "collision": bool(info["collision"][0])},
        )


class LocalNavVectorEnv(VectorEnv[dict[str, np.ndarray], np.ndarray, np.ndarray]):
    """Native batched vector env (same-step autoreset) over :class:`LocalNavCore`."""

    metadata = {"autoreset_mode": AutoresetMode.SAME_STEP}  # noqa: RUF012

    def __init__(
        self,
        worlds: Sequence[WorldSpec],
        *,
        num_envs: int,
        config: LocalNavEnvConfig | None = None,
        split_mode: SplitMode = "train",
        seed: int = 0,
    ) -> None:
        config = config or LocalNavEnvConfig()
        self._worlds, self._config, self._split = list(worlds), config, split_mode
        self.num_envs = num_envs
        self.core = LocalNavCore(
            self._worlds, num_envs=num_envs, config=config, split_mode=split_mode, seed=seed
        )
        self.obs_spec = self.core.spec
        self.reward_version = LOCAL_NAV_REWARD_VERSION
        self.privileged_reward_notes = PRIVILEGED_REWARD_NOTES
        self.single_observation_space = self.core.spec.gym_space()
        self.single_action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self.observation_space = gym.vector.utils.batch_space(
            self.single_observation_space, num_envs
        )
        self.action_space = gym.vector.utils.batch_space(self.single_action_space, num_envs)

    def reset(
        self, *, seed: int | list[int] | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        if isinstance(seed, int):
            self.core = LocalNavCore(
                self._worlds,
                num_envs=self.num_envs,
                config=self._config,
                split_mode=self._split,
                seed=seed,
            )
        return self.core.reset(), {}

    def step(
        self, actions: np.ndarray
    ) -> tuple[dict[str, np.ndarray], np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
        return self.core.step(actions)

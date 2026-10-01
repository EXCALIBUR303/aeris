"""FastSim exploration env: Gymnasium contract, masked-action handling,
decision semantics, and D0 determinism."""

from __future__ import annotations

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from aeris.autonomy.exploration.base import world_to_cell
from aeris.learning.envs.exploration import ExplorationEnv, ExplorationEnvConfig
from aeris.learning.envs.rewards import exploration_reward
from aeris.learning.spaces.exploration import ROTATE_ACTION
from aeris.simulation.worlds.spec import Bounds, Box, SpawnPose, WorldFamily, WorldSpec, WorldSplit


def _world() -> WorldSpec:
    return WorldSpec(
        name="expl_test",
        family=WorldFamily.OFFICE,
        split=WorldSplit.TRAIN,
        seed=0,
        bounds=Bounds(min_x=-8.0, min_y=-8.0, max_x=8.0, max_y=8.0, max_z=3.0),
        altitude_band_m=(0.3, 2.5),
        boxes=(
            Box(x=3.0, y=0.0, z=1.5, size_x=0.3, size_y=6.0, size_z=3.0),
            Box(x=-3.0, y=2.0, z=1.5, size_x=4.0, size_y=0.3, size_z=3.0),
        ),
        spawn_poses=(SpawnPose(x=0.0, y=0.0, z=0.1),),
    )


_CFG = ExplorationEnvConfig(space=ExplorationEnvConfig().space.__class__(episode_time_s=30.0))


def test_exploration_env_passes_gymnasium_check_env() -> None:
    check_env(ExplorationEnv([_world()], config=_CFG), skip_render_check=True)


def test_rotate_scan_turns_in_place_for_its_fixed_duration() -> None:
    env = ExplorationEnv([_world()], config=_CFG)
    env.reset(seed=0, options={"world_index": 0})
    assert env.sim is not None
    p0, y0 = env.sim.dyn.state.pos[0].copy(), float(env.sim.dyn.state.yaw[0])
    _, _, _, _, info = env.step(ROTATE_ACTION)
    d = info["decision"]
    assert abs(d.duration_s - _CFG.rotate_duration_s) < 1e-6 and not d.failed
    assert np.hypot(*(env.sim.dyn.state.pos[0, :2] - p0[:2])) < 0.05
    assert abs(float(env.sim.dyn.state.yaw[0]) - y0) > 0.5


def test_masked_action_is_a_failed_rotate_scan_that_advances_the_clock() -> None:
    """A masked action gets Tier H's fallback (a rotate scan in place),
    penalized as failed -- never a zero-time no-op, which would let a
    policy that keeps picking masked actions stall the episode forever."""
    env = ExplorationEnv([_world()], config=_CFG)
    _, info = env.reset(seed=0, options={"world_index": 0})
    masked = np.nonzero(~info["action_mask"])[0]
    assert masked.size, "expected some egocentric subgoals to be unreachable at the spawn"
    assert env.sim is not None
    p0, yaw0, t0 = env.sim.dyn.state.pos[0].copy(), float(env.sim.dyn.state.yaw[0]), env.t_s
    _, r, term, trunc, info = env.step(int(masked[0]))
    d = info["decision"]
    assert d.failed and not d.arrived and d.ticks > 0
    assert env.t_s - t0 == pytest.approx(_CFG.rotate_duration_s)
    assert np.hypot(*(env.sim.dyn.state.pos[0, :2] - p0[:2])) < 0.05
    assert float(env.sim.dyn.state.yaw[0]) != yaw0
    assert not term and not trunc
    assert r == pytest.approx(
        exploration_reward(
            _CFG.reward,
            new_explored_m2=d.new_area_m2,
            decision_duration_s=d.duration_s,
            collided=False,
            shield_interventions=d.shield_interventions,
            subgoal_failed=True,
        )
    )


def test_always_masked_policy_still_truncates() -> None:
    env = ExplorationEnv([_world()], config=_CFG)
    _, info = env.reset(seed=0, options={"world_index": 0})
    for _ in range(10_000):
        masked = np.nonzero(~info["action_mask"])[0]
        a = int(masked[0]) if masked.size else ROTATE_ACTION
        _, _, term, trunc, info = env.step(a)
        if term or trunc:
            break
    assert trunc and env.time_up


def test_valid_subgoals_explore_and_episode_truncates_at_the_time_limit() -> None:
    env = ExplorationEnv([_world()], config=_CFG)
    _, info = env.reset(seed=1, options={"world_index": 0})
    total_new, trunc = 0.0, False
    rng = np.random.default_rng(0)
    for _ in range(200):
        valid = np.nonzero(info["action_mask"])[0]
        _, _, term, trunc, info = env.step(int(rng.choice(valid)))
        total_new += info["decision"].new_area_m2
        assert not term  # no collisions: the shield + inflation keep the path clear
        if trunc:
            break
    assert trunc and env.time_up
    assert total_new > 5.0


def test_exploration_rollout_is_deterministic_given_seed() -> None:
    def run() -> list[float]:
        env = ExplorationEnv([_world()], config=_CFG)
        _, info = env.reset(seed=3, options={"world_index": 0})
        out = []
        for _ in range(6):
            a = int(np.nonzero(info["action_mask"])[0][-1])
            obs, r, _, _, info = env.step(a)
            out.append(r)
            out.append(float(obs["map"].sum()))
        return out

    assert run() == run()


def test_immediate_arrival_still_spends_a_tick() -> None:
    """A target inside the arrival radius (here: the vehicle's own cell)
    must still cost one control tick -- a zero-time 'arrived' decision let
    a scripted frontier strategy re-pick the same target forever with the
    clock frozen (found in Phase 13's gate-6 run)."""
    env = ExplorationEnv([_world()], config=_CFG)
    env.reset(seed=0, options={"world_index": 0})
    t0 = env.t_s
    here = world_to_cell(env.planning_grid(), env.agent_pose().position_m)
    d = env.execute_subgoal(here)
    assert d.arrived and not d.failed and d.ticks == 1
    assert env.t_s - t0 == pytest.approx(_CFG.control_dt_s)


def test_every_decision_advances_the_clock() -> None:
    env = ExplorationEnv([_world()], config=_CFG)
    env.reset(seed=3, options={"world_index": 0})
    rng = np.random.default_rng(3)
    while not env.time_up:
        t0 = env.t_s
        _, _, term, trunc, _ = env.step(int(rng.integers(25)))  # masked actions included
        assert env.t_s > t0
        if term or trunc:
            break

"""Observation leakage audit (spec §51 Phase 13 validation gate item 5).

Static guarantees -- no ORACLE field in any shipped spec, and
``aeris.learning`` never imports ground truth / ``aeris.learning.spaces``
never imports a simulator -- are covered by ``test_spaces.py`` and the
import-linter contracts. This file adds *behavioral* counterfactual tests:
change something the agent must not know, keep everything it may know
fixed, and require bit-identical observations.
"""

from __future__ import annotations

import numpy as np

from aeris.learning.envs.exploration import ExplorationEnv, ExplorationEnvConfig
from aeris.learning.envs.local_nav import LocalNavEnvConfig, LocalNavVectorEnv
from aeris.simulation.worlds.spec import Bounds, Box, SpawnPose, WorldFamily, WorldSpec, WorldSplit


def _world(extra: tuple[Box, ...] = (), name: str = "leak") -> WorldSpec:
    return WorldSpec(
        name=name,
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TRAIN,
        seed=0,
        bounds=Bounds(min_x=-10.0, min_y=-10.0, max_x=10.0, max_y=10.0, max_z=3.0),
        altitude_band_m=(0.3, 2.5),
        boxes=(Box(x=3.0, y=0.0, z=1.5, size_x=0.6, size_y=2.0, size_z=3.0), *extra),
        spawn_poses=(SpawnPose(x=0.0, y=0.0, z=0.1, yaw_rad=0.0),),
    )


def test_reward_only_geodesic_field_never_reaches_the_observation() -> None:
    cfg = LocalNavEnvConfig(episode_time_s=100.0)
    a = LocalNavVectorEnv([_world()], num_envs=3, config=cfg, seed=11)
    b = LocalNavVectorEnv([_world()], num_envs=3, config=cfg, seed=11)
    a.reset(seed=11)
    b.reset(seed=11)
    rng = np.random.default_rng(0)
    for e in range(3):  # corrupt B's privileged reward signal completely
        b.core.geo[e] = rng.uniform(0, 100, b.core.geo[e].shape)
        b.core.prev_geo[e] = 50.0
    actions = np.random.default_rng(1).uniform(-1, 1, (30, 3, 3))
    reward_differs = False
    for act in actions:
        oa, ra, *_ = a.step(act)
        ob, rb, *_ = b.step(act)
        for k in oa:
            np.testing.assert_array_equal(oa[k], ob[k])
        reward_differs |= not np.array_equal(ra, rb)
    assert reward_differs  # the corruption was real, it just never leaked into obs


def test_exploration_observation_ignores_geometry_the_sensors_never_saw() -> None:
    """Two worlds identical except a box *behind* the vehicle (outside the
    forward camera's FOV, never scanned): the first observation and action
    mask must be bit-identical -- the agent can't know about it yet."""
    hidden = Box(x=-6.0, y=0.0, z=1.5, size_x=1.0, size_y=4.0, size_z=3.0)
    ea = ExplorationEnv([_world(name="a")], config=ExplorationEnvConfig(), split_mode="train")
    eb = ExplorationEnv(
        [_world((hidden,), name="b")], config=ExplorationEnvConfig(), split_mode="train"
    )
    oa, ia = ea.reset(seed=4, options={"world_index": 0})
    ob, ib = eb.reset(seed=4, options={"world_index": 0})
    for k in oa:
        np.testing.assert_array_equal(oa[k], ob[k])
    np.testing.assert_array_equal(ia["action_mask"], ib["action_mask"])


def test_exploration_observation_does_see_geometry_in_view() -> None:
    """Control for the test above: a box *in front* of the vehicle must change
    the observation -- otherwise "identical" would prove nothing."""
    ahead = Box(x=1.8, y=-1.5, z=1.5, size_x=0.4, size_y=0.4, size_z=3.0)
    ea = ExplorationEnv([_world(name="a")], config=ExplorationEnvConfig(), split_mode="train")
    eb = ExplorationEnv(
        [_world((ahead,), name="b")], config=ExplorationEnvConfig(), split_mode="train"
    )
    oa, _ = ea.reset(seed=4, options={"world_index": 0})
    ob, _ = eb.reset(seed=4, options={"world_index": 0})
    assert not np.array_equal(oa["map"], ob["map"])

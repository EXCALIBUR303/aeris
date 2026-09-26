"""Unit test for the one pure-logic helper in :mod:`aeris.evaluation.mapping_episode`.

The rest of that module is live-integration glue (Sensor Bridge, GT
polling, a real control loop) with nothing meaningful to unit-test in
isolation -- covered instead by the live mapping experiment
(``scripts/run_p11_mapping_experiment.py``, spec §51 Phase 11), matching
Phase 10's identical split for ``avoidance_episode.py``.
"""

from __future__ import annotations

from dataclasses import dataclass

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import Vec3
from aeris.evaluation.mapping_episode import _gt_pose


@dataclass(slots=True)
class _FakeGtService:
    pos: Vec3
    quat: Quaternion

    def get_pose(self, model_name: str) -> tuple[Vec3, Quaternion]:
        return self.pos, self.quat


def test_gt_pose_converts_world_to_odom_by_subtracting_spawn() -> None:
    quat = Quaternion.from_axis_angle(Vec3(0.0, 0.0, 1.0), 0.3)
    service = _FakeGtService(pos=Vec3(12.0, 5.0, 1.5), quat=quat)
    spawn_world = Vec3(10.0, 5.0, 0.1)

    pos_world, pos_odom, out_quat = _gt_pose(service, "x500_depth_0", spawn_world)

    assert pos_world == Vec3(12.0, 5.0, 1.5)
    assert pos_odom == Vec3(2.0, 0.0, 1.4)
    assert out_quat == quat


def test_gt_pose_at_spawn_position_gives_zero_odom_position() -> None:
    service = _FakeGtService(pos=Vec3(3.0, -2.0, 0.1), quat=Quaternion.identity())
    _pos_world, pos_odom, _quat = _gt_pose(service, "x500_depth_0", Vec3(3.0, -2.0, 0.1))
    assert pos_odom == Vec3(0.0, 0.0, 0.0)

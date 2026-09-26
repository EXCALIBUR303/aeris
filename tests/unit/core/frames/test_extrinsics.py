"""Unit tests for :mod:`aeris.core.frames.extrinsics` (spec §18.2, Phase 8).

Uses small synthetic SDF fixtures (not the real PX4 models — those are
covered by a live/integration check) so the include-chain + composition
logic is exercised deterministically and fast.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from aeris.core.frames.extrinsics import ExtrinsicsError, load_sensor_extrinsics
from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import Vec3

_SUB_MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="sub_sensor">
    <pose>0 0 1 0 0 0</pose>
    <link name="sensor_link">
      <sensor name="cam" type="camera">
        <pose>0.5 0 0 0 0 0</pose>
      </sensor>
    </link>
  </model>
</sdf>
"""

_TOP_MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="top">
    <link name="base_link">
      <pose>0 0 2 0 0 0</pose>
    </link>
    <include merge="true">
      <uri>model://sub_sensor</uri>
      <pose>1 0 0 0 0 0</pose>
    </include>
  </model>
</sdf>
"""


@pytest.fixture
def model_dir(tmp_path: Path) -> Path:
    models = tmp_path / "models"
    (models / "top").mkdir(parents=True)
    (models / "sub_sensor").mkdir(parents=True)
    (models / "top" / "model.sdf").write_text(_TOP_MODEL_SDF)
    (models / "sub_sensor" / "model.sdf").write_text(_SUB_MODEL_SDF)
    return models / "top" / "model.sdf"


def test_composes_include_chain_and_model_root_pose(model_dir: Path) -> None:
    # sensor_link in model-root frame: include_pose(1,0,0) . sub_model's own
    # <pose>(0,0,1) = (1,0,1). base_link in model-root frame: (0,0,2).
    # T_base_sensorlink = base_link^-1 . sensor_link = (1,0,-1).
    # Then . sensor's own local pose (0.5,0,0) = (1.5, 0, -1).
    t = load_sensor_extrinsics(model_dir, sensor_name="cam")
    assert t.translation.x == pytest.approx(1.5)
    assert t.translation.y == pytest.approx(0.0)
    assert t.translation.z == pytest.approx(-1.0)
    assert t.rotation.is_close(Transform.identity().rotation)


def test_missing_sensor_raises(model_dir: Path) -> None:
    with pytest.raises(ExtrinsicsError, match="not found"):
        load_sensor_extrinsics(model_dir, sensor_name="nonexistent")


def test_missing_base_link_raises(model_dir: Path) -> None:
    with pytest.raises(ExtrinsicsError, match="base_link"):
        load_sensor_extrinsics(model_dir, sensor_name="cam", base_link="nope")


def test_rpy_rotation_is_fixed_axis_zyx(tmp_path: Path) -> None:
    """A 90-degree yaw on a link rotates its sensor's local +x to world +y."""
    import math

    models = tmp_path / "models"
    (models / "rot").mkdir(parents=True)
    (models / "rot" / "model.sdf").write_text(
        f"""<?xml version="1.0"?>
        <sdf version="1.9">
          <model name="rot">
            <link name="base_link"/>
            <link name="sensor_link">
              <pose>0 0 0 0 0 {math.pi / 2}</pose>
              <sensor name="s" type="camera">
                <pose>1 0 0 0 0 0</pose>
              </sensor>
            </link>
          </model>
        </sdf>
        """
    )
    t = load_sensor_extrinsics(models / "rot" / "model.sdf", sensor_name="s")
    p = t.apply(Vec3(0.0, 0.0, 0.0))
    assert p.x == pytest.approx(0.0, abs=1e-9)
    assert p.y == pytest.approx(1.0, abs=1e-9)

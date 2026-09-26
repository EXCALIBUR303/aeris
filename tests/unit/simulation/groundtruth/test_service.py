from __future__ import annotations

import subprocess
from unittest.mock import patch

import pytest

from aeris.core.errors import SimulationError
from aeris.simulation.groundtruth.service import (
    GroundTruthService,
    _extract_blocks,
    _extract_quaternion,
    _extract_scalar,
    _extract_vec3,
)

# A trimmed real capture of `gz topic -e -t /world/default/pose/info -n 1`
# against a gz_x500 SITL instance (Phase 7 live verification) -- exercises
# the exact quirks that matter: a fully-zero position/orientation block
# prints empty (proto3 default-value omission), a near-zero-but-nonzero
# value prints in full, and multiple `pose { ... }` blocks with nested
# `position {}`/`orientation {}` braces must not confuse a naive
# brace-unaware parser.
_SAMPLE_POSE_INFO = """
header {
  stamp {
    sec: 20
    nsec: 908000000
  }
}
pose {
  name: "ground_plane"
  id: 4
  position {
  }
  orientation {
    w: 1
  }
}
pose {
  name: "x500_0"
  id: 10
  position {
    x: -1.2634255292394455e-18
    y: -2.4360491139049715e-18
    z: -0.013000184415735444
  }
  orientation {
    x: -1.5906378554517978e-18
    y: 1.6247845615037148e-18
    z: -4.3803918357292932e-18
    w: 1
  }
}
pose {
  name: "link"
  id: 5
  position {
  }
  orientation {
    w: 1
  }
}
"""


def test_extract_blocks_finds_every_top_level_pose() -> None:
    blocks = _extract_blocks(_SAMPLE_POSE_INFO, "pose")
    assert len(blocks) == 3


def test_extract_blocks_is_brace_depth_aware() -> None:
    # Each pose block's own content must include its nested
    # position/orientation blocks intact, not truncated at the first `}`.
    blocks = _extract_blocks(_SAMPLE_POSE_INFO, "pose")
    x500_block = next(b for b in blocks if 'name: "x500_0"' in b)
    assert "position {" in x500_block
    assert "orientation {" in x500_block
    assert x500_block.count("{") == x500_block.count("}")


def test_extract_scalar_finds_quoted_name() -> None:
    blocks = _extract_blocks(_SAMPLE_POSE_INFO, "pose")
    x500_block = next(b for b in blocks if 'name: "x500_0"' in b)
    assert _extract_scalar(x500_block, "name") == "x500_0"


def test_extract_scalar_missing_field_returns_none() -> None:
    assert _extract_scalar('name: "foo"', "id") is None


def test_extract_vec3_parses_nonzero_position() -> None:
    blocks = _extract_blocks(_SAMPLE_POSE_INFO, "pose")
    x500_block = next(b for b in blocks if 'name: "x500_0"' in b)
    pos = _extract_vec3(x500_block, "position")
    assert pos.x == pytest.approx(-1.2634255292394455e-18)
    assert pos.z == pytest.approx(-0.013000184415735444)


def test_extract_vec3_defaults_missing_fields_to_zero() -> None:
    blocks = _extract_blocks(_SAMPLE_POSE_INFO, "pose")
    ground_block = next(b for b in blocks if 'name: "ground_plane"' in b)
    pos = _extract_vec3(ground_block, "position")
    assert pos.x == 0.0
    assert pos.y == 0.0
    assert pos.z == 0.0


def test_extract_quaternion_defaults_to_identity_when_all_absent() -> None:
    blocks = _extract_blocks(_SAMPLE_POSE_INFO, "pose")
    ground_block = next(b for b in blocks if 'name: "ground_plane"' in b)
    q = _extract_quaternion(ground_block, "orientation")
    assert q.w == pytest.approx(1.0)
    assert q.x == 0.0
    assert q.y == 0.0
    assert q.z == 0.0


def test_extract_quaternion_parses_nonzero_orientation() -> None:
    blocks = _extract_blocks(_SAMPLE_POSE_INFO, "pose")
    x500_block = next(b for b in blocks if 'name: "x500_0"' in b)
    q = _extract_quaternion(x500_block, "orientation")
    assert q.w == pytest.approx(1.0)
    assert q.x == pytest.approx(-1.5906378554517978e-18)


def test_get_pose_finds_the_requested_model() -> None:
    fake_result = subprocess.CompletedProcess(
        args=[], returncode=0, stdout=_SAMPLE_POSE_INFO, stderr=""
    )
    with patch("subprocess.run", return_value=fake_result) as mock_run:
        service = GroundTruthService()
        pos, orient = service.get_pose("x500_0")
    assert pos.z == pytest.approx(-0.013000184415735444)
    assert orient.w == pytest.approx(1.0)
    mock_run.assert_called_once()
    assert "/world/default/pose/info" in mock_run.call_args.args[0]


def test_get_pose_raises_for_unknown_model() -> None:
    fake_result = subprocess.CompletedProcess(
        args=[], returncode=0, stdout=_SAMPLE_POSE_INFO, stderr=""
    )
    with patch("subprocess.run", return_value=fake_result):
        service = GroundTruthService()
        with pytest.raises(SimulationError, match="not found"):
            service.get_pose("nonexistent_model")


def test_get_pose_raises_on_nonzero_returncode() -> None:
    fake_result = subprocess.CompletedProcess(args=[], returncode=1, stdout="", stderr="boom")
    with patch("subprocess.run", return_value=fake_result):
        service = GroundTruthService()
        with pytest.raises(SimulationError, match="boom"):
            service.get_pose("x500_0")


def test_get_contacts_is_always_empty_as_of_phase_7() -> None:
    assert GroundTruthService().get_contacts("x500_0") == []

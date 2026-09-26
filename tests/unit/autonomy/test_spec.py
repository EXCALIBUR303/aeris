from __future__ import annotations

from pathlib import Path

import pydantic
import pytest

from aeris.autonomy.mission.spec import MissionSpec, WaypointSpec, load_mission_spec

_REPO_ROOT = Path(__file__).resolve().parents[3]
_MISSIONS_DIR = _REPO_ROOT / "configs" / "missions"


def _waypoint(**overrides: object) -> dict[str, object]:
    defaults: dict[str, object] = {"x": 1.0, "y": 2.0, "z": 3.0}
    defaults.update(overrides)
    return defaults


def _spec(**overrides: object) -> dict[str, object]:
    defaults: dict[str, object] = {
        "name": "test",
        "takeoff_altitude_m": 3.0,
        "time_budget_s": 60.0,
        "waypoints": [_waypoint()],
    }
    defaults.update(overrides)
    return defaults


def test_waypoint_position_matches_fields() -> None:
    wp = WaypointSpec.model_validate(_waypoint(x=1.0, y=2.0, z=3.0))
    assert wp.position.x == 1.0
    assert wp.position.y == 2.0
    assert wp.position.z == 3.0


def test_waypoint_defaults() -> None:
    wp = WaypointSpec.model_validate(_waypoint())
    assert wp.yaw_rad is None
    assert wp.acceptance_radius_m == 1.0
    assert wp.dwell_s == 1.0


def test_waypoint_rejects_non_positive_z() -> None:
    with pytest.raises(pydantic.ValidationError):
        WaypointSpec.model_validate(_waypoint(z=0.0))
    with pytest.raises(pydantic.ValidationError):
        WaypointSpec.model_validate(_waypoint(z=-1.0))


def test_waypoint_rejects_non_positive_acceptance_radius() -> None:
    with pytest.raises(pydantic.ValidationError):
        WaypointSpec.model_validate(_waypoint(acceptance_radius_m=0.0))


def test_waypoint_rejects_negative_dwell() -> None:
    with pytest.raises(pydantic.ValidationError):
        WaypointSpec.model_validate(_waypoint(dwell_s=-0.1))


def test_waypoint_zero_dwell_is_allowed() -> None:
    wp = WaypointSpec.model_validate(_waypoint(dwell_s=0.0))
    assert wp.dwell_s == 0.0


def test_mission_spec_requires_at_least_one_waypoint() -> None:
    with pytest.raises(pydantic.ValidationError):
        MissionSpec.model_validate(_spec(waypoints=[]))


def test_mission_spec_rejects_non_positive_takeoff_altitude() -> None:
    with pytest.raises(pydantic.ValidationError):
        MissionSpec.model_validate(_spec(takeoff_altitude_m=0.0))


def test_mission_spec_rejects_non_positive_time_budget() -> None:
    with pytest.raises(pydantic.ValidationError):
        MissionSpec.model_validate(_spec(time_budget_s=0.0))


def test_mission_spec_is_frozen() -> None:
    spec = MissionSpec.model_validate(_spec())
    with pytest.raises(pydantic.ValidationError):
        spec.name = "changed"  # type: ignore[misc]


@pytest.mark.parametrize("filename", ["square.yaml", "triangle.yaml", "line.yaml"])
def test_loads_the_real_committed_mission_templates(filename: str) -> None:
    spec = load_mission_spec(_MISSIONS_DIR / filename)
    assert spec.name == filename.removesuffix(".yaml")
    assert len(spec.waypoints) >= 2
    assert spec.takeoff_altitude_m > 0
    assert spec.time_budget_s > 0

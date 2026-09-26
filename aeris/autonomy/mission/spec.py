"""``MissionSpec`` v1: a plain waypoint mission (spec §51 Phase 6).

Pydantic, per spec §14.2 ("Pydantic v2 for configs"), loaded from
``configs/missions/*.yaml`` via :func:`aeris.core.config.compose_config`
(the same ``extends:`` composition every other AERIS config uses).
"""

from __future__ import annotations

from pathlib import Path

from pydantic import BaseModel, Field

from aeris.core.config import compose_config
from aeris.core.errors import ConfigCompositionError
from aeris.core.frames.vector import Vec3


class WaypointSpec(BaseModel):
    """One waypoint, in the ENU/odom frame (spec §19) -- absolute
    coordinates relative to the vehicle's home/EKF origin, same convention
    as ``aeris.vehicle.interface.PositionSetpoint``."""

    model_config = {"frozen": True}

    x: float
    y: float
    z: float = Field(gt=0.0)
    yaw_rad: float | None = None
    acceptance_radius_m: float = Field(default=1.0, gt=0.0)
    # spec §51 Phase 6: "waypoint arrival criteria (radius + dwell)" --
    # must stay within acceptance_radius_m continuously for this long
    # before being considered arrived (spec's own "Known risks:
    # arrival-detection jitter" / "Failure/rollback: hysteresis on
    # arrival" -- this *is* that hysteresis).
    dwell_s: float = Field(default=1.0, ge=0.0)

    @property
    def position(self) -> Vec3:
        return Vec3(self.x, self.y, self.z)


class MissionSpec(BaseModel):
    """A named sequence of waypoints plus the executive's operating budget."""

    model_config = {"frozen": True}

    name: str
    takeoff_altitude_m: float = Field(gt=0.0)
    waypoints: list[WaypointSpec] = Field(min_length=1)
    time_budget_s: float = Field(gt=0.0)
    waypoint_timeout_s: float = Field(default=60.0, gt=0.0)


def load_mission_spec(path: Path | str) -> MissionSpec:
    """Load and validate a :class:`MissionSpec` from a YAML file (``extends:`` composed)."""
    resolved = compose_config(path)
    if "name" not in resolved:
        resolved = {**resolved, "name": Path(path).stem}
    try:
        return MissionSpec.model_validate(resolved)
    except Exception as exc:  # pydantic.ValidationError, but keep this typed
        raise ConfigCompositionError(f"invalid mission spec at {path}: {exc}") from exc

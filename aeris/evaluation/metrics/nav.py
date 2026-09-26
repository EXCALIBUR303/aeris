"""Navigation/flight metrics (spec §41) computable from a trajectory
alone -- no ground-truth geometry needed (see ``collision.py`` for the
metrics that do)."""

from __future__ import annotations

import itertools
from collections.abc import Sequence

from aeris.core.frames.vector import Vec3


def distance_travelled_m(positions: Sequence[Vec3]) -> float:
    """D = sum ||p_{t+1} - p_t|| (spec §41)."""
    return sum((b - a).norm() for a, b in itertools.pairwise(positions))


def flight_time_s(t_takeoff_s: float, t_land_s: float) -> float:
    """t_land - t_takeoff (spec §41)."""
    if t_land_s < t_takeoff_s:
        raise ValueError(f"t_land_s ({t_land_s}) is before t_takeoff_s ({t_takeoff_s})")
    return t_land_s - t_takeoff_s


def control_smoothness(velocities: Sequence[Vec3], dt_s: float) -> float:
    """J = mean squared acceleration implied by consecutive velocity
    samples (spec §41's "mean squared commanded acceleration" -- computed
    here from the vehicle's *achieved* velocity trace, since
    ``MissionExecutive`` issues position setpoints rather than a raw
    velocity-command stream; label results accordingly)."""
    if len(velocities) < 2:
        return 0.0
    if dt_s <= 0:
        raise ValueError(f"dt_s must be > 0, got {dt_s}")
    squared_accels = [
        (b - a).scale(1.0 / dt_s).norm() ** 2 for a, b in itertools.pairwise(velocities)
    ]
    return sum(squared_accels) / len(squared_accels)


def mean_waypoint_error_m(errors_m: Sequence[float]) -> float:
    if not errors_m:
        return 0.0
    return sum(errors_m) / len(errors_m)


def return_to_base_success(
    *,
    landed: bool,
    disarmed: bool,
    land_position: Vec3,
    home_position: Vec3,
    r_home_m: float = 1.5,
) -> bool:
    """spec §41: "landed and disarmed and ||p_land - p0|| <= r_home"."""
    return landed and disarmed and (land_position - home_position).norm() <= r_home_m

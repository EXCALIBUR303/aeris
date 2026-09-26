"""``ReactiveAvoidance`` — a VFH-style reactive navigation method (spec §51 Phase 10).

Chooses, among the horizontal sectors with sufficient clearance, the one
whose heading is closest to the goal direction (a simplified
Vector-Field-Histogram: candidate headings gated by clearance, ranked by
goal alignment). It is a *planner of intent*, not a safety mechanism --
:mod:`aeris.safety.shield` still has final say over the output velocity's
magnitude regardless of which direction this module picks; the two are
deliberately independent so the shield's guarantee never depends on this
module choosing well.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from aeris.core.frames.vector import Vec3
from aeris.core.units import wrap_pi
from aeris.safety.shield import N_SECTORS, SECTOR_WIDTH_RAD, sector_direction


@dataclass(frozen=True, slots=True)
class ReactiveAvoidance:
    cruise_speed_mps: float = 1.5
    arrival_radius_m: float = 0.3
    slowdown_radius_m: float = 1.5
    min_sector_clearance_m: float = (
        0.6  # a candidate heading must clear at least this much to be usable
    )

    def compute_velocity(
        self, current_pos: Vec3, goal: Vec3, sector_clearances_m: list[float]
    ) -> Vec3:
        to_goal = goal - current_pos
        to_goal_horizontal = Vec3(to_goal.x, to_goal.y, 0.0)
        distance_horizontal = to_goal_horizontal.norm()

        vertical_speed = 0.0
        if abs(to_goal.z) > 1e-6:
            vertical_speed = max(-self.cruise_speed_mps, min(self.cruise_speed_mps, to_goal.z))

        if distance_horizontal <= self.arrival_radius_m:
            return Vec3(0.0, 0.0, vertical_speed)

        goal_angle = math.atan2(to_goal_horizontal.y, to_goal_horizontal.x)

        best_index: int | None = None
        best_heading_error = math.inf
        for i in range(N_SECTORS):
            if sector_clearances_m[i] < self.min_sector_clearance_m:
                continue
            sector_angle = (i + 0.5) * SECTOR_WIDTH_RAD
            heading_error = abs(wrap_pi(sector_angle - goal_angle))
            if heading_error < best_heading_error:
                best_heading_error = heading_error
                best_index = i

        if best_index is None:
            return Vec3(0.0, 0.0, vertical_speed)  # no usable heading; hold horizontally

        speed = self.cruise_speed_mps
        if distance_horizontal < self.slowdown_radius_m:
            speed = self.cruise_speed_mps * (distance_horizontal / self.slowdown_radius_m)

        direction = sector_direction(best_index)
        return Vec3(direction.x * speed, direction.y * speed, vertical_speed)

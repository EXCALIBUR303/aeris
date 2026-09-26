"""``PathFollower`` — the "straight-line" navigation method (spec §51 Phase 10).

The simplest possible goal-seeking policy: fly directly at the goal,
slowing smoothly on approach. Used as the comparison baseline against
:mod:`reactive`'s VFH-style avoidance (spec: "compare 'reactive' vs
'straight-line + shield only'") — on its own this method has no obstacle
awareness at all; every collision it would otherwise cause is entirely
:mod:`aeris.safety.shield`'s job to prevent.
"""

from __future__ import annotations

from dataclasses import dataclass

from aeris.core.frames.vector import Vec3


@dataclass(frozen=True, slots=True)
class PathFollower:
    cruise_speed_mps: float = 1.5
    arrival_radius_m: float = 0.3
    # Speed ramps down linearly to 0 as the vehicle enters this radius of
    # the goal, so the shield never has to fight a small, constant-speed
    # overshoot right at arrival.
    slowdown_radius_m: float = 1.5

    def compute_velocity(self, current_pos: Vec3, goal: Vec3) -> Vec3:
        to_goal = goal - current_pos
        distance = to_goal.norm()
        if distance <= self.arrival_radius_m:
            return Vec3(0.0, 0.0, 0.0)

        direction = to_goal.scale(1.0 / distance)
        speed = self.cruise_speed_mps
        if distance < self.slowdown_radius_m:
            speed = self.cruise_speed_mps * (distance / self.slowdown_radius_m)
        return direction.scale(speed)

"""S3 — the collision shield (spec §16.2 row S3, §16.4, Phase 10).

Uses only the agent's own sensing (spec §17.4's provenance rule: never
simulator ground truth) -- body-frame points from
:mod:`aeris.perception.depth.projection`/:mod:`aeris.perception.lidar.projection`
already land in the body frame via ``T_B_Copt`` (Phase 8's extrinsics),
so this module needs no frame math of its own beyond the one
odom<->body rotation at its public boundary.

The shield is a hard backstop, not a planner: it never *chooses* a
direction (that's :mod:`aeris.autonomy.navigation`'s job) -- it only ever
*reduces* a commanded velocity's magnitude, never increases it, and never
changes its direction (a uniform scalar shrink keeps every sector
constraint the exact inequality spec §16.4 states, ``∀i: v·û_i ≤
v_i,max``, satisfied simultaneously by construction; see
:func:`_uniform_scale_factor`'s docstring for the proof).

**A sector the sensor actually looked at and found nothing within range is
"clear at max range", not "unknown".** The first version of this module
conflated the two (any sector with zero points was marked unknown, i.e.
``v_i,max = 0``) -- since ``v_i,max=0`` for even one sector whose angle is
within 90° of the command direction forces the whole command to zero (the
uniform-scale proof above holds for *any* such sector), this made forward
flight impossible the moment a real depth camera's ~70° FOV left most
sectors unobserved, which is every tick. ``fov_half_angle_rad`` fixes this:
sectors inside the declared FOV default to "clear at max range" absent a
closer point; only sectors genuinely outside the sensor's FOV (lateral,
behind) use the spec's "last-seen clearance" conservative policy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import Vec3
from aeris.core.units import wrap_pi

N_SECTORS = 72  # spec §16.4's own example: "72 x 5 deg"
SECTOR_WIDTH_RAD = 2.0 * math.pi / N_SECTORS


def sector_index(direction_xy: Vec3) -> int:
    """Body-frame horizontal direction -> sector index (0 = FLU +x, increasing CCW)."""
    angle = math.atan2(direction_xy.y, direction_xy.x) % (2.0 * math.pi)
    return int(angle // SECTOR_WIDTH_RAD) % N_SECTORS


def sector_direction(index: int) -> Vec3:
    """The unit direction ``û_i`` (body FLU, horizontal) at a sector's center angle."""
    angle = (index + 0.5) * SECTOR_WIDTH_RAD
    return Vec3(math.cos(angle), math.sin(angle), 0.0)


def fov_sector_indices(half_angle_rad: float) -> frozenset[int]:
    """Sectors whose center angle falls within ``±half_angle_rad`` of forward (angle 0)."""
    indices = set()
    for i in range(N_SECTORS):
        angle = wrap_pi((i + 0.5) * SECTOR_WIDTH_RAD)
        if abs(angle) <= half_angle_rad:
            indices.add(i)
    return frozenset(indices)


def compute_sector_clearances(
    points_body: list[Vec3], *, max_range_m: float, fov_sectors: frozenset[int]
) -> list[float | None]:
    """Bin body-frame points into horizontal sectors by azimuth.

    A sector inside ``fov_sectors`` defaults to ``max_range_m`` (the
    sensor looked there and found nothing within range -- a valid "clear"
    reading), refined downward by any closer point that landed in it.
    A sector outside ``fov_sectors`` is ``None`` (genuinely unobserved
    this tick) regardless of whether a stray point landed in it.
    """
    clearances: list[float | None] = [
        max_range_m if i in fov_sectors else None for i in range(N_SECTORS)
    ]
    for p in points_body:
        r = math.hypot(p.x, p.y)
        if r <= 0.0 or r > max_range_m:
            continue
        idx = sector_index(p)
        if idx not in fov_sectors:
            continue
        current = clearances[idx]
        if current is None or r < current:
            clearances[idx] = r
    return clearances


def sector_v_max(d_i: float, *, a_brake_mps2: float, d_safe_m: float, tau_s: float) -> float:
    """spec §16.4: ``v_i,max = max(0, sqrt(2*a_brake*max(0, d_i-d_safe)) - a_brake*tau)``."""
    return max(0.0, math.sqrt(2.0 * a_brake_mps2 * max(0.0, d_i - d_safe_m)) - a_brake_mps2 * tau_s)


def _relevant_sector_indices(v_body: Vec3, *, fov_sectors: frozenset[int]) -> set[int]:
    """The sectors that actually constrain ``v_body``: the whole (fixed) FOV
    plus the 3 sectors nearest ``v_body``'s own heading.

    Checking the full FOV is always safe -- every FOV sector has real data
    (either an observed range or the "looked and saw nothing" max-range
    default; never a bare unknown), so it can never produce a spurious
    zero. Checking a *cone* around the heading was tried first and found
    fragile: any offset between the cone's center (the heading, which
    varies) and the FOV's center (fixed at 0°) opens a gap band of
    "relevant but never observed" sectors along the cone's far edge,
    which forces a hard zero even for a heading just a few degrees off
    dead-ahead. Checking only the handful of sectors nearest the heading
    itself (rather than a whole cone) avoids this: the dominant sector
    is exactly where the velocity actually points, with two neighbors for
    robustness at a 5°-sector boundary, not a wide swath whose edge can
    drift outside the FOV as the heading changes. A heading pointed
    laterally/backward still lands on sectors outside the FOV -- still
    correctly conservative (unseen -> ``v_i,max = 0``, per spec §16.4's
    default policy) -- since only those 3 extra sectors sit there, not a
    whole cone that might accidentally clip back into the FOV.
    """
    if v_body.x == 0.0 and v_body.y == 0.0:
        return set(fov_sectors)
    dominant = sector_index(v_body)
    near_heading = {(dominant + offset) % N_SECTORS for offset in (-1, 0, 1)}
    return set(fov_sectors) | near_heading


def _uniform_scale_factor(
    v_body: Vec3,
    clearances: list[float],
    *,
    a_brake_mps2: float,
    d_safe_m: float,
    tau_s: float,
    fov_sectors: frozenset[int],
) -> float:
    """The largest ``s in [0, 1]`` such that, for every relevant sector ``i``
    (see :func:`_relevant_sector_indices`), ``(s*v_body)·û_i ≤ v_i,max``.

    For each such sector with ``dot_i = v_body·û_i > 0``, the constraint
    requires ``s ≤ v_i,max/dot_i``. Taking ``s = min(1, min_i(v_i,max/dot_i))``
    satisfies every one of them simultaneously: for the sector achieving
    the minimum, ``s*dot_i = v_i,max`` exactly; for every other relevant
    sector ``j``, ``s ≤ v_j,max/dot_j`` by definition of the minimum, so
    ``s*dot_j ≤ v_j,max`` too. A uniform shrink (not a per-axis clip)
    is what keeps this simple property provable and property-testable.
    """
    scale = 1.0
    for i in _relevant_sector_indices(v_body, fov_sectors=fov_sectors):
        u_i = sector_direction(i)
        dot = v_body.x * u_i.x + v_body.y * u_i.y
        if dot <= 0.0:
            continue
        v_i_max = sector_v_max(
            clearances[i], a_brake_mps2=a_brake_mps2, d_safe_m=d_safe_m, tau_s=tau_s
        )
        candidate = v_i_max / dot
        if candidate < scale:
            scale = candidate
    return max(0.0, scale)


@dataclass(frozen=True, slots=True)
class ShieldConfig:
    a_brake_mps2: float = 2.0
    d_safe_m: float = 0.5
    tau_s: float = 0.3
    max_range_m: float = 15.0
    # Matches the pinned x500_depth's StereoOV7251 horizontal_fov (1.274
    # rad, spec §8.2/Phase 8) halved -- the actual forward depth camera's
    # real coverage, not an arbitrary guess.
    fov_half_angle_rad: float = 0.637


@dataclass(slots=True)
class CollisionShield:
    """Stateful S3 shield: retains each out-of-FOV sector's last-seen
    clearance across ticks for the "unknown sectors" policy (spec §16.4:
    the default is conservative -- limited to the speed at which the
    vehicle can stop within its *last-seen* clearance, ``0.0`` -- fully
    blocked -- for a sector never yet observed at all).
    """

    config: ShieldConfig = field(default_factory=ShieldConfig)
    _last_seen_clearance_m: list[float] = field(default_factory=lambda: [0.0] * N_SECTORS)
    intervention_count: int = 0
    tick_count: int = 0
    _fov_sectors: frozenset[int] = field(default_factory=frozenset)

    def __post_init__(self) -> None:
        self._fov_sectors = fov_sector_indices(self.config.fov_half_angle_rad)

    def sector_clearances_m(self) -> list[float]:
        return list(self._last_seen_clearance_m)

    def _update_sectors(self, points_body: list[Vec3]) -> None:
        observed = compute_sector_clearances(
            points_body, max_range_m=self.config.max_range_m, fov_sectors=self._fov_sectors
        )
        for i, d in enumerate(observed):
            if d is not None:
                self._last_seen_clearance_m[i] = d

    def project(
        self, v_cmd_odom: Vec3, *, orientation_odom_body: Quaternion, points_body: list[Vec3]
    ) -> tuple[Vec3, bool]:
        """Shield a desired ODOM-frame velocity against ``points_body``.

        Returns ``(v_shielded_odom, intervened)``, where ``intervened`` is
        whether the output differs from the input by more than a small
        epsilon (spec §16.4's own intervention-metric definition).
        """
        self.tick_count += 1
        self._update_sectors(points_body)

        v_cmd_body = orientation_odom_body.inverse().rotate(v_cmd_odom)
        v_horizontal_body = Vec3(v_cmd_body.x, v_cmd_body.y, 0.0)
        scale = _uniform_scale_factor(
            v_horizontal_body,
            self._last_seen_clearance_m,
            a_brake_mps2=self.config.a_brake_mps2,
            d_safe_m=self.config.d_safe_m,
            tau_s=self.config.tau_s,
            fov_sectors=self._fov_sectors,
        )
        v_shielded_horizontal_body = v_horizontal_body.scale(scale)
        v_shielded_body = Vec3(
            v_shielded_horizontal_body.x, v_shielded_horizontal_body.y, v_cmd_body.z
        )
        v_shielded_odom = orientation_odom_body.rotate(v_shielded_body)

        intervened = (v_shielded_odom - v_cmd_odom).norm() > 1e-6
        if intervened:
            self.intervention_count += 1
        return v_shielded_odom, intervened

    @property
    def intervention_rate(self) -> float:
        """spec §16.4: "fraction of control ticks where ‖v_cmd - v_shielded‖ > eps"."""
        return self.intervention_count / self.tick_count if self.tick_count else 0.0

"""The transport-agnostic vehicle interface (spec §15).

Autonomy code (from Phase 5 onward) depends only on the protocols and
dataclasses in this module — never on a concrete adapter
(``aeris.vehicle.px4_mavlink``), enforced by an import-linter contract
(``pyproject.toml``). All state here is ENU/FLU (spec §19, ADR-0008); the
one adapter that exists converts NED/FRD at its own boundary and never
leaks it out.

**Safety note (spec §16.1/§16.3):** nothing in this module enforces who
may call :class:`CommandPort`'s methods — that gate is
``aeris.safety.SafetySupervisor``, built in Phase 5. Phase 4 only defines
the interface; it does not yet restrict who holds it.
"""

from __future__ import annotations

from collections.abc import AsyncIterator
from dataclasses import dataclass
from enum import StrEnum, unique
from typing import Protocol, runtime_checkable

from aeris.core.frames import Quaternion, Vec3
from aeris.core.types import Provenance

# --- Enums -------------------------------------------------------------------


@unique
class FlightMode(StrEnum):
    """A coarse, PX4-transport-agnostic view of the vehicle's flight mode.

    Deliberately smaller than PX4's own ``custom_mode`` bitfield — autonomy
    code needs to know "are we in offboard/manual/hold/etc.", not PX4's
    internal encoding. Adapters map PX4's actual mode onto this.
    """

    UNKNOWN = "unknown"
    MANUAL = "manual"
    ALTITUDE = "altitude"
    POSITION = "position"
    OFFBOARD = "offboard"
    HOLD = "hold"
    MISSION = "mission"
    RETURN_TO_LAUNCH = "return_to_launch"
    TAKEOFF = "takeoff"
    LAND = "land"
    ACRO = "acro"
    STABILIZED = "stabilized"


@unique
class LandedState(StrEnum):
    UNKNOWN = "unknown"
    ON_GROUND = "on_ground"
    IN_AIR = "in_air"
    TAKEOFF = "takeoff"
    LANDING = "landing"


@unique
class GpsFixType(StrEnum):
    NO_GPS = "no_gps"
    NO_FIX = "no_fix"
    FIX_2D = "fix_2d"
    FIX_3D = "fix_3d"
    DGPS = "dgps"
    RTK_FLOAT = "rtk_float"
    RTK_FIXED = "rtk_fixed"


@unique
class CommandResultCode(StrEnum):
    ACCEPTED = "accepted"
    TEMPORARILY_REJECTED = "temporarily_rejected"
    DENIED = "denied"
    UNSUPPORTED = "unsupported"
    FAILED = "failed"
    IN_PROGRESS = "in_progress"
    TIMEOUT = "timeout"


# --- State dataclasses ---------------------------------------------------------


@dataclass(frozen=True, slots=True)
class GpsStatus:
    fix_type: GpsFixType
    satellites_visible: int
    eph_m: float  # horizontal position uncertainty
    epv_m: float  # vertical position uncertainty


@dataclass(frozen=True, slots=True)
class BatterySimState:
    """A simulated drain model — spec §41: "labeled as such," never physical energy."""

    remaining_fraction: float  # [0, 1]
    voltage_v: float


@dataclass(frozen=True, slots=True)
class LinkStatus:
    connected: bool
    last_heartbeat_age_s: float


@dataclass(frozen=True, slots=True)
class EkfFlags:
    """A coarse view of PX4 EKF2's estimator health (from ``ESTIMATOR_STATUS``)."""

    raw_flags: int
    pos_horiz_accuracy_m: float
    pos_vert_accuracy_m: float


@dataclass(frozen=True, slots=True)
class VehicleState:
    """A single, agent-legitimate snapshot of the vehicle (spec §15.1).

    Every field here is either a real sensor reading or an *estimate*
    (``source``), never simulator ground truth — spec §17.4's provenance
    rule applies to this dataclass as a whole, not just individual fields,
    since nothing in it is privileged.
    """

    t_sim_s: float
    armed: bool
    flight_mode: FlightMode
    landed_state: LandedState

    pose_odom: Vec3  # ENU, from PX4 EKF2 (spec §19)
    orientation_odom: Quaternion  # q_O_B (ENU/FLU body orientation)
    velocity_odom_mps: Vec3  # ENU
    angular_velocity_body_radps: Vec3  # FLU

    home_odom: Vec3 | None

    gps: GpsStatus
    battery_sim: BatterySimState
    ekf_flags: EkfFlags
    link: LinkStatus

    source: Provenance = Provenance.ESTIMATE


# --- Setpoints -------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class PositionSetpoint:
    """A position + yaw target, in ENU/odom frame (spec §15.1)."""

    position_odom: Vec3
    yaw_rad: float | None = None  # None = hold current yaw


@dataclass(frozen=True, slots=True)
class VelocitySetpoint:
    """A velocity + yaw-rate target.

    ``frame`` selects whether ``velocity`` is expressed in the ENU/odom
    frame or the FLU body frame — both are legitimate offboard inputs
    (spec §15.1: "ENU odom or FLU body frame").
    """

    velocity: Vec3
    yaw_rate_radps: float | None = None
    frame: str = "odom"  # "odom" (ENU) or "body" (FLU)


Setpoint = PositionSetpoint | VelocitySetpoint

# Spec §16.1 rule 1 / §14.3 contract 2: "No learned component outputs
# anything below HighLevelAction, which is one of: body/odom velocity +
# yaw rate; a position/waypoint target; a subgoal for the planner."
# Currently identical to Setpoint -- the third variant (a planner subgoal)
# has no concrete type yet and is added when Phase 12's planner exists.
# `aeris.autonomy` may depend on this type (and nothing else vehicle-side)
# per the import-linter contract of the same name.
HighLevelAction = Setpoint


@dataclass(frozen=True, slots=True)
class Waypoint:
    position_odom: Vec3
    yaw_rad: float | None = None
    acceptance_radius_m: float = 1.0


@dataclass(frozen=True, slots=True)
class VehicleEndpoint:
    """Where to connect. Subject to the hardware guard (spec §16.6)."""

    host: str
    port: int
    simulated: bool = True  # a non-loopback host MUST also set this explicitly


@dataclass(frozen=True, slots=True)
class CommandResult:
    code: CommandResultCode
    message: str = ""

    @property
    def ok(self) -> bool:
        return self.code == CommandResultCode.ACCEPTED


# --- Protocols -------------------------------------------------------------------


@runtime_checkable
class CommandPort(Protocol):
    """The only way to command the vehicle. Spec §14.3 contract 3: only
    ``aeris.safety.SafetySupervisor`` may hold one (enforced starting Phase 5)."""

    async def arm(self) -> CommandResult: ...
    async def disarm(self) -> CommandResult: ...
    async def takeoff(self, altitude_m: float) -> CommandResult: ...
    async def land(self) -> CommandResult: ...
    async def hold(self) -> CommandResult: ...
    async def return_to_launch(self) -> CommandResult: ...

    async def start_offboard(self, initial: Setpoint) -> CommandResult: ...
    async def stop_offboard(self) -> CommandResult: ...

    async def set_position_target(self, sp: PositionSetpoint) -> None: ...
    async def set_velocity_target(self, sp: VelocitySetpoint) -> None: ...
    async def goto_waypoint(self, wp: Waypoint) -> CommandResult: ...

    async def emergency_stop_simulation(self) -> CommandResult: ...


@runtime_checkable
class VehicleInterface(Protocol):
    """What autonomy code depends on. Never a concrete adapter (spec §14.3)."""

    async def connect(self, endpoint: VehicleEndpoint) -> None: ...
    async def disconnect(self) -> None: ...

    async def get_vehicle_state(self) -> VehicleState: ...
    def subscribe_telemetry(self, rate_hz: float) -> AsyncIterator[VehicleState]: ...

    def command_port(self) -> CommandPort: ...

"""A fake ``VehicleInterface``/``CommandPort``, shared across unit tests
that need one (``aeris.safety``, ``aeris.autonomy.mission``, ...).

Not collected by pytest (no ``test_`` prefix). Centralized here per spec
§44.3 ("Mocks are ... centralized (``tests/fixtures/``...)") since more
than one package's tests need the same fake vehicle -- import it as
``from tests.fixtures.vehicle_fakes import FakeVehicle, make_state``
(this resolves via Python's implicit namespace packages; no
``__init__.py`` needed as long as pytest runs from the repo root).
"""

from __future__ import annotations

from collections.abc import AsyncIterator
from typing import Any

from aeris.core.constants import EKF_HEALTHY_REQUIRED_FLAGS
from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import ZERO, Vec3
from aeris.vehicle.interface import (
    BatterySimState,
    CommandResult,
    CommandResultCode,
    EkfFlags,
    FlightMode,
    GpsFixType,
    GpsStatus,
    LandedState,
    LinkStatus,
    PositionSetpoint,
    Setpoint,
    VehicleEndpoint,
    VehicleState,
    VelocitySetpoint,
    Waypoint,
)


def make_state(
    *,
    t_sim_s: float = 0.0,
    armed: bool = False,
    flight_mode: FlightMode = FlightMode.HOLD,
    landed_state: LandedState = LandedState.ON_GROUND,
    pose_odom: Vec3 = ZERO,
    orientation_odom: Quaternion | None = None,
    velocity_odom_mps: Vec3 = ZERO,
    home_odom: Vec3 | None = ZERO,
    ekf_flags_raw: int = EKF_HEALTHY_REQUIRED_FLAGS,
    heartbeat_age_s: float = 0.1,
    link_connected: bool = True,
) -> VehicleState:
    """A healthy-by-default :class:`VehicleState` fake, override what a test needs."""
    return VehicleState(
        t_sim_s=t_sim_s,
        armed=armed,
        flight_mode=flight_mode,
        landed_state=landed_state,
        pose_odom=pose_odom,
        orientation_odom=(
            orientation_odom if orientation_odom is not None else Quaternion.identity()
        ),
        velocity_odom_mps=velocity_odom_mps,
        angular_velocity_body_radps=ZERO,
        home_odom=home_odom,
        gps=GpsStatus(fix_type=GpsFixType.FIX_3D, satellites_visible=10, eph_m=0.5, epv_m=1.0),
        battery_sim=BatterySimState(remaining_fraction=1.0, voltage_v=16.0),
        ekf_flags=EkfFlags(
            raw_flags=ekf_flags_raw, pos_horiz_accuracy_m=0.2, pos_vert_accuracy_m=0.3
        ),
        link=LinkStatus(connected=link_connected, last_heartbeat_age_s=heartbeat_age_s),
    )


class FakeVehicle:
    """Implements both ``VehicleInterface`` and ``CommandPort`` (returns
    itself from ``command_port()``, same as :class:`Px4MavlinkAdapter`).

    ``self.state`` is read live by ``get_vehicle_state()`` -- mutate it
    between calls to simulate telemetry changing. ``self.calls`` records
    every command invocation (name, args) for assertions. ``self.results``
    lets a test force a specific method to return a non-ACCEPTED result.
    """

    def __init__(self, initial_state: VehicleState | None = None) -> None:
        self.state = initial_state if initial_state is not None else make_state()
        self.connected = False
        self.calls: list[tuple[str, tuple[Any, ...]]] = []
        self.results: dict[str, CommandResult] = {}

    def _result(self, name: str) -> CommandResult:
        return self.results.get(name, CommandResult(CommandResultCode.ACCEPTED))

    # --- VehicleInterface ----------------------------------------------------------

    async def connect(self, endpoint: VehicleEndpoint) -> None:
        self.connected = True
        self.calls.append(("connect", (endpoint,)))

    async def disconnect(self) -> None:
        self.connected = False
        self.calls.append(("disconnect", ()))

    async def get_vehicle_state(self) -> VehicleState:
        return self.state

    async def subscribe_telemetry(self, rate_hz: float) -> AsyncIterator[VehicleState]:
        raise NotImplementedError
        yield self.state  # pragma: no cover -- makes this a generator function

    def command_port(self) -> FakeVehicle:
        return self

    # --- CommandPort -----------------------------------------------------------------

    async def arm(self) -> CommandResult:
        self.calls.append(("arm", ()))
        return self._result("arm")

    async def disarm(self) -> CommandResult:
        self.calls.append(("disarm", ()))
        return self._result("disarm")

    async def takeoff(self, altitude_m: float) -> CommandResult:
        self.calls.append(("takeoff", (altitude_m,)))
        return self._result("takeoff")

    async def land(self) -> CommandResult:
        self.calls.append(("land", ()))
        return self._result("land")

    async def hold(self) -> CommandResult:
        self.calls.append(("hold", ()))
        return self._result("hold")

    async def return_to_launch(self) -> CommandResult:
        self.calls.append(("return_to_launch", ()))
        return self._result("return_to_launch")

    async def start_offboard(self, initial: Setpoint) -> CommandResult:
        self.calls.append(("start_offboard", (initial,)))
        return self._result("start_offboard")

    async def stop_offboard(self) -> CommandResult:
        self.calls.append(("stop_offboard", ()))
        return self._result("stop_offboard")

    async def set_position_target(self, sp: PositionSetpoint) -> None:
        self.calls.append(("set_position_target", (sp,)))

    async def set_velocity_target(self, sp: VelocitySetpoint) -> None:
        self.calls.append(("set_velocity_target", (sp,)))

    async def goto_waypoint(self, wp: Waypoint) -> CommandResult:
        self.calls.append(("goto_waypoint", (wp,)))
        return self._result("goto_waypoint")

    async def emergency_stop_simulation(self) -> CommandResult:
        self.calls.append(("emergency_stop_simulation", ()))
        return self._result("emergency_stop_simulation")

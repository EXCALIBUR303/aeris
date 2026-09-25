"""The pymavlink-based PX4 vehicle adapter (spec §15; ADR-0003, finalized here).

Implements both :class:`~aeris.vehicle.interface.VehicleInterface` and
:class:`~aeris.vehicle.interface.CommandPort` (the adapter *is* its own
command port — spec §14.3's "only Safety may hold a CommandPort" is
enforced starting Phase 5, by who is handed the object this class
constructs, not by anything in this file).

**Threading model:** pymavlink's socket API is synchronous. A single
background reader thread owns ``recv_match()`` exclusively — every other
method only ever reads from two lock-protected caches (``_raw``, the
latest message per MAVLink type; ``_command_acks``, the latest
``COMMAND_ACK`` per command id) that the reader thread populates. Nothing
else calls ``recv_match()``, which avoids the classic "two readers racing
on one socket" bug.
"""

from __future__ import annotations

import asyncio
import threading
import time
from collections.abc import AsyncIterator
from typing import Any

from pymavlink import mavutil

from aeris.core.errors import CommandRejectedError, VehicleConnectionError
from aeris.core.logging import get_logger
from aeris.vehicle.hardware_guard import assert_endpoint_allowed
from aeris.vehicle.interface import (
    CommandResult,
    CommandResultCode,
    PositionSetpoint,
    Setpoint,
    VehicleEndpoint,
    VehicleState,
    VelocitySetpoint,
    Waypoint,
)
from aeris.vehicle.px4_mavlink.setpoints import EncodedSetpoint, encode_setpoint
from aeris.vehicle.px4_mavlink.state import build_vehicle_state, t_sim_s_from_local_position

_logger = get_logger(component="vehicle.px4_mavlink")

# MAV_CMD
_MAV_CMD_COMPONENT_ARM_DISARM = 400
_MAV_CMD_NAV_TAKEOFF = 22
_MAV_CMD_NAV_LAND = 21
_MAV_CMD_DO_SET_MODE = 176
_MAV_CMD_NAV_RETURN_TO_LAUNCH = 20

_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
_MAV_RESULT_TO_CODE = {
    0: CommandResultCode.ACCEPTED,
    1: CommandResultCode.TEMPORARILY_REJECTED,
    2: CommandResultCode.DENIED,
    3: CommandResultCode.UNSUPPORTED,
    4: CommandResultCode.FAILED,
    5: CommandResultCode.IN_PROGRESS,
}

# PX4_CUSTOM_MAIN_MODE / PX4_CUSTOM_SUB_MODE_AUTO (see modes.py's docstring
# for the source) used for the mode-switch commands this adapter issues.
_MAIN_AUTO = 4
_MAIN_OFFBOARD = 6
_SUB_AUTO_LOITER = 3
_SUB_AUTO_RTL = 5

_OFFBOARD_STREAM_RATE_HZ = 20.0  # spec §13.4 / constants.AERIS_OFFBOARD_SETPOINT_RATE_HZ
_ACK_POLL_INTERVAL_S = 0.05

# MAVLink message ids requested at a higher rate on connect (spec §51 Phase
# 4: "telemetry rate ... state >= 20 Hz"). The offboard link doesn't stream
# these by default at a guaranteed rate (unlike the GCS link, which PX4's
# own init script sets to 50 Hz explicitly) — verified by reading
# px4-rc.mavlink in Phase 3/4.
_MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32
_MAVLINK_MSG_ID_ATTITUDE_QUATERNION = 31
_MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED = 85  # PX4's own offboard-setpoint echo
_REQUESTED_STREAM_RATE_HZ = 50.0

# PX4's health_and_arming_checks module requires a live "GCS" heartbeat
# stream on a link before it will arm on that link ("Preflight Fail: No
# connection to the GCS", found by reading px4.log during Phase 4 live
# testing) -- a real GCS/companion always heartbeats back, which a
# receive-only client doesn't. 1 Hz matches QGroundControl/MAVSDK.
_HEARTBEAT_SEND_RATE_HZ = 1.0


class Px4MavlinkAdapter:
    """The V1 vehicle adapter. See this module's docstring for the design."""

    def __init__(self) -> None:
        self._conn: mavutil.mavlink_connection | None = None
        self._reader_thread: threading.Thread | None = None
        self._stop_reader = threading.Event()

        self._lock = threading.Lock()
        self._raw: dict[str, Any] = {}
        self._command_acks: dict[int, Any] = {}
        self._last_heartbeat_wall: float | None = None

        self._connected = False
        self._offboard_active = False
        self._setpoint_lock = threading.Lock()
        self._latest_setpoint: EncodedSetpoint | None = None
        self._stream_task: asyncio.Task[None] | None = None
        self._heartbeat_task: asyncio.Task[None] | None = None

    # --- VehicleInterface --------------------------------------------------------

    async def connect(self, endpoint: VehicleEndpoint) -> None:
        assert_endpoint_allowed(endpoint)
        if self._connected:
            raise RuntimeError("already connected")

        conn = mavutil.mavlink_connection(f"udpin:{endpoint.host}:{endpoint.port}")
        self._conn = conn
        self._stop_reader.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="px4-mavlink-reader", daemon=True
        )
        self._reader_thread.start()

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            with self._lock:
                if "HEARTBEAT" in self._raw:
                    break
            await asyncio.sleep(0.05)
        else:
            self._stop_reader.set()
            raise VehicleConnectionError(
                f"no heartbeat from {endpoint.host}:{endpoint.port} within 30s"
            )

        self._connected = True
        self._heartbeat_task = asyncio.create_task(self._send_heartbeats())
        await self._request_message_rates()
        _logger.info("vehicle.connected", host=endpoint.host, port=endpoint.port)

    async def disconnect(self) -> None:
        if not self._connected:
            return
        await self.stop_offboard()
        if self._heartbeat_task is not None:
            self._heartbeat_task.cancel()
            self._heartbeat_task = None
        self._stop_reader.set()
        if self._reader_thread is not None:
            await asyncio.to_thread(self._reader_thread.join, 2.0)
        if self._conn is not None:
            self._conn.close()
        self._connected = False
        _logger.info("vehicle.disconnected")

    async def get_vehicle_state(self) -> VehicleState:
        with self._lock:
            cache = dict(self._raw)
        heartbeat_age = (
            time.monotonic() - self._last_heartbeat_wall
            if self._last_heartbeat_wall is not None
            else float("inf")
        )
        t_sim_s = t_sim_s_from_local_position(cache, fallback=0.0)
        return build_vehicle_state(
            cache, t_sim_s=t_sim_s, last_heartbeat_age_s=heartbeat_age, connected=self._connected
        )

    async def subscribe_telemetry(self, rate_hz: float) -> AsyncIterator[VehicleState]:
        if rate_hz <= 0:
            raise ValueError(f"rate_hz must be positive, got {rate_hz}")
        period_s = 1.0 / rate_hz
        while self._connected:
            yield await self.get_vehicle_state()
            await asyncio.sleep(period_s)

    def command_port(self) -> Px4MavlinkAdapter:
        return self

    # --- CommandPort ---------------------------------------------------------------

    async def arm(self) -> CommandResult:
        return await self._send_command(_MAV_CMD_COMPONENT_ARM_DISARM, 1)

    async def disarm(self) -> CommandResult:
        return await self._send_command(_MAV_CMD_COMPONENT_ARM_DISARM, 0)

    async def takeoff(self, altitude_m: float) -> CommandResult:
        return await self._send_command(_MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude_m)

    async def land(self) -> CommandResult:
        return await self._send_command(_MAV_CMD_NAV_LAND)

    async def hold(self) -> CommandResult:
        return await self._set_px4_mode(_MAIN_AUTO, _SUB_AUTO_LOITER)

    async def return_to_launch(self) -> CommandResult:
        return await self._set_px4_mode(_MAIN_AUTO, _SUB_AUTO_RTL)

    async def start_offboard(self, initial: Setpoint) -> CommandResult:
        with self._setpoint_lock:
            self._latest_setpoint = encode_setpoint(initial)
        if self._stream_task is None or self._stream_task.done():
            self._offboard_active = True
            self._stream_task = asyncio.create_task(self._stream_setpoints())
        # PX4 requires several setpoints to have already arrived before it
        # will accept the OFFBOARD mode switch (spec §8.2's 2 Hz "proof of
        # life" rule; below that, even the *first* switch is rejected).
        await asyncio.sleep(3.0 / _OFFBOARD_STREAM_RATE_HZ)
        return await self._set_px4_mode(_MAIN_OFFBOARD, 0)

    async def stop_offboard(self) -> CommandResult:
        # Leave OFFBOARD *before* stopping the stream — stopping first
        # would trigger PX4's offboard-loss failsafe instead of a clean
        # mode change.
        result = await self.hold()
        self._offboard_active = False
        if self._stream_task is not None:
            self._stream_task.cancel()
            self._stream_task = None
        return result

    async def set_position_target(self, sp: PositionSetpoint) -> None:
        with self._setpoint_lock:
            self._latest_setpoint = encode_setpoint(sp)

    async def set_velocity_target(self, sp: VelocitySetpoint) -> None:
        with self._setpoint_lock:
            self._latest_setpoint = encode_setpoint(sp)

    async def goto_waypoint(self, wp: Waypoint, *, timeout_s: float = 30.0) -> CommandResult:
        """Convenience: a position setpoint + arrival monitor (spec §12).

        A simple, bounded poll loop — proper mission sequencing is Phase
        6's job (spec §51 Phase 6). This exists so Phase 4/5 have
        *something* usable for a single waypoint without waiting on that.
        """
        sp = PositionSetpoint(position_odom=wp.position_odom, yaw_rad=wp.yaw_rad)
        if not self._offboard_active:
            start_result = await self.start_offboard(sp)
            if not start_result.ok:
                return start_result
        else:
            await self.set_position_target(sp)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            state = await self.get_vehicle_state()
            distance = (state.pose_odom - wp.position_odom).norm()
            if distance <= wp.acceptance_radius_m:
                return CommandResult(CommandResultCode.ACCEPTED, "reached waypoint")
            await asyncio.sleep(0.1)
        return CommandResult(
            CommandResultCode.TIMEOUT, f"did not reach waypoint within {timeout_s}s"
        )

    async def emergency_stop_simulation(self) -> CommandResult:
        """Switch to Hold (spec §16.5). The fuller sequence (stop the
        autonomy loop, mark the run ABORTED) is Phase 5's
        ``SafetySupervisor`` — this method only owns the PX4-facing part.
        """
        return await self.hold()

    # --- internals -----------------------------------------------------------------

    def _reader_loop(self) -> None:
        assert self._conn is not None
        while not self._stop_reader.is_set():
            msg = self._conn.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue
            with self._lock:
                self._raw[msg_type] = msg
                if msg_type == "COMMAND_ACK":
                    self._command_acks[msg.command] = msg
            if msg_type == "HEARTBEAT":
                self._last_heartbeat_wall = time.monotonic()

    async def _request_message_rates(self) -> None:
        interval_us = 1_000_000.0 / _REQUESTED_STREAM_RATE_HZ
        for msg_id in (
            _MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            _MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
            _MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,
        ):
            await self._send_command(511, msg_id, interval_us)  # MAV_CMD_SET_MESSAGE_INTERVAL

    async def _send_command(
        self,
        command: int,
        p1: float = 0,
        p2: float = 0,
        p3: float = 0,
        p4: float = 0,
        p5: float = 0,
        p6: float = 0,
        p7: float = 0,
        *,
        timeout_s: float = 5.0,
    ) -> CommandResult:
        conn = self._conn
        if conn is None:
            raise CommandRejectedError("not connected")
        with self._lock:
            self._command_acks.pop(command, None)
        conn.mav.command_long_send(
            conn.target_system, conn.target_component, command, 0, p1, p2, p3, p4, p5, p6, p7
        )

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                ack = self._command_acks.get(command)
            if ack is not None:
                return CommandResult(_MAV_RESULT_TO_CODE.get(ack.result, CommandResultCode.FAILED))
            await asyncio.sleep(_ACK_POLL_INTERVAL_S)
        return CommandResult(
            CommandResultCode.TIMEOUT, f"no ack for command {command} within {timeout_s}s"
        )

    async def _set_px4_mode(self, main_mode: int, sub_mode: int) -> CommandResult:
        return await self._send_command(
            _MAV_CMD_DO_SET_MODE, _MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main_mode, sub_mode
        )

    async def _send_heartbeats(self) -> None:
        conn = self._conn
        assert conn is not None
        period_s = 1.0 / _HEARTBEAT_SEND_RATE_HZ
        try:
            while True:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                await asyncio.sleep(period_s)
        except asyncio.CancelledError:
            pass

    async def _stream_setpoints(self) -> None:
        conn = self._conn
        assert conn is not None
        period_s = 1.0 / _OFFBOARD_STREAM_RATE_HZ
        try:
            while True:
                with self._setpoint_lock:
                    sp = self._latest_setpoint
                if sp is not None:
                    conn.mav.set_position_target_local_ned_send(
                        int(time.monotonic() * 1000) % (2**32),
                        conn.target_system,
                        conn.target_component,
                        sp.coordinate_frame,
                        sp.type_mask,
                        sp.x,
                        sp.y,
                        sp.z,
                        sp.vx,
                        sp.vy,
                        sp.vz,
                        sp.afx,
                        sp.afy,
                        sp.afz,
                        sp.yaw,
                        sp.yaw_rate,
                    )
                await asyncio.sleep(period_s)
        except asyncio.CancelledError:
            pass

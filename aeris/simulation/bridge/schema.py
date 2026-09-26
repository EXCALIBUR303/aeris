"""Sensor Bridge IPC message schema (spec §18.4, ADR-004).

Messages are plain ``dict``s (msgpack-encoded on the wire, spec ADR-004:
"Messages are msgpack with a versioned schema") rather than Pydantic models
— this schema crosses a process boundary between two independently
version-managed Python environments (the bridge process, which needs
Homebrew's ``gz`` Python bindings on its import path, and AERIS's own
``uv``-managed venv), so both sides only need ``msgpack`` installed, not
each other's dependency tree.

Topics (spec §18.4: "GT on a separate namespace"): regular sensor topics
are named directly (``"depth"``, ``"rgb"``, ``"scan"``, ``"camera_info"``,
``"health"``); ground-truth topics are prefixed ``"gt."`` (e.g.
``"gt.pose"``) so a client can allowlist-subscribe to everything except
that prefix (see :mod:`aeris.simulation.bridge.client`).
"""

from __future__ import annotations

from typing import Any, Literal, TypedDict

SCHEMA_VERSION = 1

GT_TOPIC_PREFIX = "gt."

SensorKind = Literal["depth", "rgb", "scan", "camera_info", "clock", "health", "gt_pose"]


class SensorMessage(TypedDict):
    """The envelope every Sensor Bridge message shares (spec §18.2's ``SensorFrame``)."""

    schema_version: int
    kind: SensorKind
    sensor_id: str
    frame_id: str
    t_sim_s: float
    seq: int
    payload: dict[str, Any]


def make_message(
    *,
    kind: SensorKind,
    sensor_id: str,
    frame_id: str,
    t_sim_s: float,
    seq: int,
    payload: dict[str, Any],
) -> SensorMessage:
    return {
        "schema_version": SCHEMA_VERSION,
        "kind": kind,
        "sensor_id": sensor_id,
        "frame_id": frame_id,
        "t_sim_s": t_sim_s,
        "seq": seq,
        "payload": payload,
    }


class HealthEntry(TypedDict):
    """One sensor topic's health, published once per second (spec §18.4)."""

    rate_hz: float
    age_s: float
    count: int


class HealthPayload(TypedDict):
    topics: dict[str, HealthEntry]

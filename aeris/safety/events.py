"""Safety-layer event types (spec §16.3: "Transitions are events ... recorded
in the replay").

Phase 7's ``EventBus``/MCAP recorder don't exist yet, so these events are
only logged (via :mod:`aeris.core.logging`) and kept in the supervisor's
own in-memory log for now — the data shape is what a future
``EventBus``/replay consumes, built here so Phase 7 doesn't have to
retrofit it.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import StrEnum, unique
from typing import Any


@unique
class SafetyEventKind(StrEnum):
    """What kind of thing happened in the safety layer."""

    STATE_TRANSITION = "state_transition"
    COMMAND_REJECTED = "command_rejected"
    WATCHDOG_TRIP = "watchdog_trip"
    INTERVENTION_STARTED = "intervention_started"
    INTERVENTION_CLEARED = "intervention_cleared"
    EMERGENCY_STOP = "emergency_stop"
    STEP_RESPONSE_SAMPLE = "step_response_sample"


@dataclass(frozen=True, slots=True)
class SafetyEvent:
    """One safety-layer event, timestamped in simulation time."""

    t_sim_s: float
    kind: SafetyEventKind
    message: str
    detail: Mapping[str, Any] = field(default_factory=dict)

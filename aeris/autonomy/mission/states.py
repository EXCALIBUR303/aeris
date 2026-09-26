"""The mission executive's state machine (spec §51 Phase 6, patterned
after §32.2's fuller search-and-rescue executive: "IDLE -> PREFLIGHT ->
TAKEOFF -> SEARCH ... -> RETURN -> LAND -> COMPLETE. Also ABORTED and
FAILSAFE from any state." Phase 6 is a plain waypoint mission (no
targets to search for), so ``SEARCH``/``INVESTIGATE`` become a single
``EXECUTING`` state that visits the mission's waypoint list in order --
the exploration-strategy plug-in point Phase 21 adds back is not needed
until it exists.
"""

from __future__ import annotations

from enum import StrEnum, unique


@unique
class MissionState(StrEnum):
    IDLE = "idle"
    PREFLIGHT = "preflight"
    TAKEOFF = "takeoff"
    EXECUTING = "executing"
    RETURN = "return"
    LAND = "land"
    COMPLETE = "complete"
    ABORTED = "aborted"
    FAILSAFE = "failsafe"


_TERMINAL = frozenset({MissionState.COMPLETE, MissionState.ABORTED, MissionState.FAILSAFE})

# IDLE -> PREFLIGHT -> TAKEOFF -> EXECUTING -> RETURN -> LAND -> COMPLETE,
# any non-terminal state -> ABORTED, any state -> FAILSAFE (including a
# self-loop, matching aeris.safety.supervisor's own "any -> FAILSAFE"
# precedent -- a mission already in FAILSAFE can be re-reported as such).
_TRANSITIONS: dict[MissionState, frozenset[MissionState]] = {
    MissionState.IDLE: frozenset({MissionState.PREFLIGHT}),
    MissionState.PREFLIGHT: frozenset({MissionState.TAKEOFF}),
    MissionState.TAKEOFF: frozenset({MissionState.EXECUTING}),
    MissionState.EXECUTING: frozenset({MissionState.RETURN}),
    MissionState.RETURN: frozenset({MissionState.LAND}),
    MissionState.LAND: frozenset({MissionState.COMPLETE}),
    MissionState.COMPLETE: frozenset(),
    MissionState.ABORTED: frozenset(),
    MissionState.FAILSAFE: frozenset(),
}
for _state in list(_TRANSITIONS):
    _extra = {MissionState.FAILSAFE}
    if _state not in _TERMINAL:
        _extra = _extra | {MissionState.ABORTED}
    _TRANSITIONS[_state] = _TRANSITIONS[_state] | _extra


def allowed_transitions(state: MissionState) -> frozenset[MissionState]:
    return _TRANSITIONS[state]


def is_terminal(state: MissionState) -> bool:
    return state in _TERMINAL

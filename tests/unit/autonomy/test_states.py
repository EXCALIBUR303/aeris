from __future__ import annotations

import itertools

import pytest

from aeris.autonomy.mission.states import MissionState, allowed_transitions, is_terminal

ALL_STATES = list(MissionState)


@pytest.mark.parametrize("from_state", ALL_STATES)
@pytest.mark.parametrize("to_state", ALL_STATES)
def test_transition_table_is_internally_consistent(
    from_state: MissionState, to_state: MissionState
) -> None:
    allowed = allowed_transitions(from_state)
    if to_state == MissionState.FAILSAFE:
        assert to_state in allowed  # "any -> FAILSAFE"
    if to_state == MissionState.ABORTED and not is_terminal(from_state):
        assert to_state in allowed  # abort from any non-terminal state


def test_happy_path_sequence_is_linear_and_exact() -> None:
    sequence = [
        MissionState.IDLE,
        MissionState.PREFLIGHT,
        MissionState.TAKEOFF,
        MissionState.EXECUTING,
        MissionState.RETURN,
        MissionState.LAND,
        MissionState.COMPLETE,
    ]
    for a, b in itertools.pairwise(sequence):
        assert b in allowed_transitions(a)


def test_terminal_states_have_no_further_transitions() -> None:
    for state in (MissionState.COMPLETE, MissionState.ABORTED, MissionState.FAILSAFE):
        assert is_terminal(state)
        # Only a self-loop into FAILSAFE is allowed (matches
        # aeris.safety.supervisor's "any -> FAILSAFE" precedent); nothing
        # else, and no path back out.
        assert allowed_transitions(state) == frozenset({MissionState.FAILSAFE})


def test_non_terminal_states_are_not_terminal() -> None:
    for state in (
        MissionState.IDLE,
        MissionState.PREFLIGHT,
        MissionState.TAKEOFF,
        MissionState.EXECUTING,
        MissionState.RETURN,
        MissionState.LAND,
    ):
        assert not is_terminal(state)

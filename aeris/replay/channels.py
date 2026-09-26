"""Replay channel names (spec §39.2). Phase 7 implements L0-L2 -- the
sensor/perception/plan channels (L2-L4) are added by the phases that
first produce that data (perception, mapping, planning).
"""

from __future__ import annotations

MISSION_EVENTS = "/mission/events"
VEHICLE_STATE = "/vehicle/state"
AUTONOMY_ACTION = "/autonomy/action"
SAFETY_EVENTS = "/safety/events"
SAFETY_INTERVENTION = "/safety/intervention"
GT_POSE = "/gt/pose"
GT_CONTACTS = "/gt/contacts"

# Spec §39.2: "ground truth (flagged privileged; displayed only in
# evaluation/replay GT overlays)". Not an import-linter contract (MCAP
# files aren't Python imports) -- aeris.evaluation.episode is the only
# writer of these channels, and any future replay-viewing UI must know to
# treat them specially rather than feed them to an agent.
PRIVILEGED_CHANNELS = frozenset({GT_POSE, GT_CONTACTS})

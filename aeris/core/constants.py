"""Named constants with a source, per spec §14.2 ("no magic numbers").

Add a constant here (never inline) whenever a later phase needs one that
isn't naturally a config value — i.e. it comes from an external spec (PX4,
MAVLink, Gazebo) rather than something AERIS should be able to tune. Always
cite the source.
"""

from __future__ import annotations

# PX4 Offboard mode requires a continuous "proof of life" setpoint stream.
# Source: PX4 docs, "Offboard Mode" (verified Phase 0, 2026-09-25):
# "a continuous 2Hz 'proof of life' signal". AERIS streams at 10x margin
# (spec §13.4 control-loop timing table).
PX4_OFFBOARD_MIN_SETPOINT_RATE_HZ: float = 2.0
AERIS_OFFBOARD_SETPOINT_RATE_HZ: float = 20.0

# Spec §13.4 control-loop timing table.
AERIS_SAFETY_SUPERVISOR_RATE_HZ: float = 20.0
AERIS_LOCAL_NAV_RATE_HZ: float = 10.0
AERIS_MISSION_EXECUTIVE_RATE_HZ: float = 5.0

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

# MAVLink ESTIMATOR_STATUS_FLAGS bits (common.xml — verified against
# pymavlink.dialects.v20.common in Phase 5), used by aeris.safety's EKF
# health gate (spec §16.2 S2: "EKF health gate (PX4 estimator flags)").
# The REQUIRED set matches what Phase 3's `wait_for_ekf_ok` observed a
# genuinely healthy SITL instance report (flags=959, missing only
# POS_VERT_AGL — no rangefinder on this airframe, expected); the FORBIDDEN
# set is PX4's own "something is actively wrong" indicators.
ESTIMATOR_ATTITUDE: int = 1
ESTIMATOR_VELOCITY_HORIZ: int = 2
ESTIMATOR_VELOCITY_VERT: int = 4
ESTIMATOR_POS_HORIZ_REL: int = 8
ESTIMATOR_POS_HORIZ_ABS: int = 16
ESTIMATOR_POS_VERT_ABS: int = 32
ESTIMATOR_GPS_GLITCH: int = 1024
ESTIMATOR_ACCEL_ERROR: int = 2048

EKF_HEALTHY_REQUIRED_FLAGS: int = (
    ESTIMATOR_ATTITUDE
    | ESTIMATOR_VELOCITY_HORIZ
    | ESTIMATOR_VELOCITY_VERT
    | ESTIMATOR_POS_HORIZ_REL
    | ESTIMATOR_POS_HORIZ_ABS
    | ESTIMATOR_POS_VERT_ABS
)
EKF_HEALTHY_FORBIDDEN_FLAGS: int = ESTIMATOR_GPS_GLITCH | ESTIMATOR_ACCEL_ERROR

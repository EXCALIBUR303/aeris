from aeris.core.constants import (
    AERIS_LOCAL_NAV_RATE_HZ,
    AERIS_MISSION_EXECUTIVE_RATE_HZ,
    AERIS_OFFBOARD_SETPOINT_RATE_HZ,
    AERIS_SAFETY_SUPERVISOR_RATE_HZ,
    PX4_OFFBOARD_MIN_SETPOINT_RATE_HZ,
)


def test_aeris_offboard_rate_has_at_least_10x_margin_over_px4_minimum():
    # Spec §13.4: AERIS streams setpoints with a 10x margin over PX4's
    # documented minimum, so a single missed tick never risks the failsafe.
    assert AERIS_OFFBOARD_SETPOINT_RATE_HZ >= 10 * PX4_OFFBOARD_MIN_SETPOINT_RATE_HZ


def test_control_loop_rates_are_positive_and_ordered():
    # Spec §13.4 control-loop timing table: safety supervisor runs fastest,
    # then local nav, then the mission executive — each slower than the last.
    assert (
        0
        < AERIS_MISSION_EXECUTIVE_RATE_HZ
        < AERIS_LOCAL_NAV_RATE_HZ
        < AERIS_SAFETY_SUPERVISOR_RATE_HZ
    )

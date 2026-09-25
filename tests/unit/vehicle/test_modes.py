from aeris.vehicle.interface import FlightMode
from aeris.vehicle.px4_mavlink.modes import decode_px4_custom_mode


def _pack(main_mode: int, sub_mode: int = 0) -> int:
    return (main_mode << 16) | (sub_mode << 24)


def test_manual():
    assert decode_px4_custom_mode(_pack(1)) == FlightMode.MANUAL


def test_altctl():
    assert decode_px4_custom_mode(_pack(2)) == FlightMode.ALTITUDE


def test_posctl():
    assert decode_px4_custom_mode(_pack(3)) == FlightMode.POSITION


def test_acro():
    assert decode_px4_custom_mode(_pack(5)) == FlightMode.ACRO


def test_offboard():
    assert decode_px4_custom_mode(_pack(6)) == FlightMode.OFFBOARD


def test_stabilized():
    assert decode_px4_custom_mode(_pack(7)) == FlightMode.STABILIZED


def test_auto_takeoff():
    assert decode_px4_custom_mode(_pack(4, 2)) == FlightMode.TAKEOFF


def test_auto_loiter_is_hold():
    assert decode_px4_custom_mode(_pack(4, 3)) == FlightMode.HOLD


def test_auto_mission():
    assert decode_px4_custom_mode(_pack(4, 4)) == FlightMode.MISSION


def test_auto_rtl():
    assert decode_px4_custom_mode(_pack(4, 5)) == FlightMode.RETURN_TO_LAUNCH


def test_auto_land():
    assert decode_px4_custom_mode(_pack(4, 6)) == FlightMode.LAND


def test_auto_precland_maps_to_land():
    assert decode_px4_custom_mode(_pack(4, 9)) == FlightMode.LAND


def test_auto_ready_is_unknown():
    assert decode_px4_custom_mode(_pack(4, 1)) == FlightMode.UNKNOWN


def test_auto_with_unrecognized_sub_mode_is_unknown():
    assert decode_px4_custom_mode(_pack(4, 99)) == FlightMode.UNKNOWN


def test_unrecognized_main_mode_is_unknown():
    assert decode_px4_custom_mode(_pack(99)) == FlightMode.UNKNOWN


def test_zero_is_unknown():
    assert decode_px4_custom_mode(0) == FlightMode.UNKNOWN

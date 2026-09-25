import math
from types import SimpleNamespace

import pytest

from aeris.vehicle.interface import FlightMode, GpsFixType, LandedState
from aeris.vehicle.px4_mavlink.state import build_vehicle_state, t_sim_s_from_local_position


def make_cache(**messages):
    return dict(messages)


def test_empty_cache_degrades_gracefully():
    state = build_vehicle_state(make_cache(), t_sim_s=1.0, last_heartbeat_age_s=0.5, connected=True)
    assert state.armed is False
    assert state.flight_mode == FlightMode.UNKNOWN
    assert state.landed_state == LandedState.UNKNOWN
    assert state.home_odom is None
    assert state.gps.fix_type == GpsFixType.NO_GPS
    assert state.t_sim_s == 1.0
    assert state.link.connected is True
    assert state.link.last_heartbeat_age_s == 0.5


def test_armed_bit_decoded():
    hb = SimpleNamespace(base_mode=128 | 4, custom_mode=(6 << 16))
    state = build_vehicle_state(
        make_cache(HEARTBEAT=hb), t_sim_s=0.0, last_heartbeat_age_s=0.0, connected=True
    )
    assert state.armed is True
    assert state.flight_mode == FlightMode.OFFBOARD


def test_disarmed_bit_decoded():
    hb = SimpleNamespace(base_mode=4, custom_mode=(1 << 16))
    state = build_vehicle_state(
        make_cache(HEARTBEAT=hb), t_sim_s=0.0, last_heartbeat_age_s=0.0, connected=True
    )
    assert state.armed is False
    assert state.flight_mode == FlightMode.MANUAL


def test_landed_state_mapping():
    ext = SimpleNamespace(landed_state=2)  # IN_AIR
    state = build_vehicle_state(
        make_cache(EXTENDED_SYS_STATE=ext), t_sim_s=0.0, last_heartbeat_age_s=0.0, connected=True
    )
    assert state.landed_state == LandedState.IN_AIR


def test_local_position_ned_converted_to_enu():
    # NED (north=3, east=1, down=-2) -> ENU (east=1, north=3, up=2)
    lp = SimpleNamespace(time_boot_ms=5000, x=3.0, y=1.0, z=-2.0, vx=0.0, vy=0.0, vz=0.0)
    state = build_vehicle_state(
        make_cache(LOCAL_POSITION_NED=lp), t_sim_s=0.0, last_heartbeat_age_s=0.0, connected=True
    )
    assert state.pose_odom.x == pytest.approx(1.0)
    assert state.pose_odom.y == pytest.approx(3.0)
    assert state.pose_odom.z == pytest.approx(2.0)


def test_local_position_ned_velocity_converted_to_enu():
    lp = SimpleNamespace(time_boot_ms=0, x=0, y=0, z=0, vx=3.0, vy=1.0, vz=-2.0)
    state = build_vehicle_state(
        make_cache(LOCAL_POSITION_NED=lp), t_sim_s=0.0, last_heartbeat_age_s=0.0, connected=True
    )
    assert state.velocity_odom_mps.x == pytest.approx(1.0)
    assert state.velocity_odom_mps.y == pytest.approx(3.0)
    assert state.velocity_odom_mps.z == pytest.approx(2.0)


def test_attitude_quaternion_identity_ned_frd_maps_near_identity_enu_flu():
    # PX4's own NED/FRD identity quaternion (w=1,x=0,y=0,z=0) doesn't mean
    # "facing East" in ENU -- it means "aligned with NED/FRD axes", which
    # is a real, nonzero rotation in ENU/FLU. Just check the round trip
    # produces a unit quaternion (correctness of the conversion itself is
    # covered exhaustively in tests/unit/core/frames/).
    aq = SimpleNamespace(q1=1.0, q2=0.0, q3=0.0, q4=0.0)
    state = build_vehicle_state(
        make_cache(ATTITUDE_QUATERNION=aq), t_sim_s=0.0, last_heartbeat_age_s=0.0, connected=True
    )
    assert state.orientation_odom.norm() == pytest.approx(1.0)


def test_gps_fix_type_and_units():
    gps = SimpleNamespace(fix_type=3, satellites_visible=11, eph=150, epv=250)
    state = build_vehicle_state(
        make_cache(GPS_RAW_INT=gps), t_sim_s=0.0, last_heartbeat_age_s=0.0, connected=True
    )
    assert state.gps.fix_type == GpsFixType.FIX_3D
    assert state.gps.satellites_visible == 11
    assert state.gps.eph_m == pytest.approx(1.5)
    assert state.gps.epv_m == pytest.approx(2.5)


def test_gps_unknown_eph_epv_is_nan():
    gps = SimpleNamespace(fix_type=3, satellites_visible=11, eph=65535, epv=65535)
    state = build_vehicle_state(
        make_cache(GPS_RAW_INT=gps), t_sim_s=0.0, last_heartbeat_age_s=0.0, connected=True
    )
    assert math.isnan(state.gps.eph_m)
    assert math.isnan(state.gps.epv_m)


def test_battery_fields():
    sys_status = SimpleNamespace(battery_remaining=80, voltage_battery=16200)
    state = build_vehicle_state(
        make_cache(SYS_STATUS=sys_status), t_sim_s=0.0, last_heartbeat_age_s=0.0, connected=True
    )
    assert state.battery_sim.remaining_fraction == pytest.approx(0.8)
    assert state.battery_sim.voltage_v == pytest.approx(16.2)


def test_ekf_flags():
    est = SimpleNamespace(flags=959, pos_horiz_accuracy=0.15, pos_vert_accuracy=0.2)
    state = build_vehicle_state(
        make_cache(ESTIMATOR_STATUS=est), t_sim_s=0.0, last_heartbeat_age_s=0.0, connected=True
    )
    assert state.ekf_flags.raw_flags == 959
    assert state.ekf_flags.pos_horiz_accuracy_m == pytest.approx(0.15)


def test_home_position_converted_to_enu():
    home = SimpleNamespace(x=3.0, y=1.0, z=-2.0)
    state = build_vehicle_state(
        make_cache(HOME_POSITION=home), t_sim_s=0.0, last_heartbeat_age_s=0.0, connected=True
    )
    assert state.home_odom is not None
    assert state.home_odom.x == pytest.approx(1.0)
    assert state.home_odom.y == pytest.approx(3.0)
    assert state.home_odom.z == pytest.approx(2.0)


def test_home_position_none_when_not_seen():
    state = build_vehicle_state(make_cache(), t_sim_s=0.0, last_heartbeat_age_s=0.0, connected=True)
    assert state.home_odom is None


def test_t_sim_s_from_local_position():
    lp = SimpleNamespace(time_boot_ms=12345)
    assert t_sim_s_from_local_position(make_cache(LOCAL_POSITION_NED=lp)) == pytest.approx(12.345)


def test_t_sim_s_fallback_when_no_position_yet():
    assert t_sim_s_from_local_position(make_cache(), fallback=7.0) == 7.0

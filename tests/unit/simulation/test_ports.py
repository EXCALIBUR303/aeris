import pytest

from aeris.simulation.launcher.ports import instance_ports


def test_instance_0_matches_px4_rc_mavlink_defaults():
    # Transcribed directly from px4-rc.mavlink with instance=0 (Phase 3).
    p = instance_ports(0)
    assert p.gcs_local == 18570
    assert p.offboard_local == 14580
    assert p.offboard_remote == 14540
    assert p.onboard_payload_local == 14280
    assert p.onboard_payload_remote == 14030
    assert p.onboard_gimbal_local == 13030
    assert p.onboard_gimbal_remote == 13280


def test_instance_offset_is_additive():
    p0 = instance_ports(0)
    p3 = instance_ports(3)
    assert p3.gcs_local == p0.gcs_local + 3
    assert p3.offboard_local == p0.offboard_local + 3
    assert p3.offboard_remote == p0.offboard_remote + 3
    assert p3.onboard_payload_local == p0.onboard_payload_local + 3
    assert p3.onboard_gimbal_local == p0.onboard_gimbal_local + 3


def test_offboard_remote_pins_at_14549_beyond_instance_9():
    # px4-rc.mavlink: "use the same ports for more than 10 instances to
    # avoid port overlaps"
    assert instance_ports(9).offboard_remote == 14549
    assert instance_ports(10).offboard_remote == 14549
    assert instance_ports(50).offboard_remote == 14549


def test_negative_instance_rejected():
    with pytest.raises(ValueError):
        instance_ports(-1)


def test_all_ports_distinct_within_one_instance():
    p = instance_ports(0)
    ports = {
        p.gcs_local,
        p.offboard_local,
        p.offboard_remote,
        p.onboard_payload_local,
        p.onboard_payload_remote,
        p.onboard_gimbal_local,
        p.onboard_gimbal_remote,
    }
    assert len(ports) == 7

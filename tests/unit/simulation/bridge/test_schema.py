"""Unit tests for the Sensor Bridge IPC message schema (spec §18.4, ADR-004)."""

from __future__ import annotations

import msgpack

from aeris.simulation.bridge.schema import GT_TOPIC_PREFIX, SCHEMA_VERSION, make_message


def test_make_message_has_the_documented_envelope() -> None:
    msg = make_message(
        kind="depth",
        sensor_id="front_depth",
        frame_id="front_depth_optical",
        t_sim_s=1.5,
        seq=3,
        payload={"a": 1},
    )
    assert msg["schema_version"] == SCHEMA_VERSION
    assert msg["kind"] == "depth"
    assert msg["sensor_id"] == "front_depth"
    assert msg["frame_id"] == "front_depth_optical"
    assert msg["t_sim_s"] == 1.5
    assert msg["seq"] == 3
    assert msg["payload"] == {"a": 1}


def test_message_round_trips_through_msgpack_with_binary_payload() -> None:
    raw = b"\x00\x01\x02\xff" * 100
    msg = make_message(
        kind="depth",
        sensor_id="front_depth",
        frame_id="front_depth_optical",
        t_sim_s=2.25,
        seq=7,
        payload={"width": 4, "height": 4, "data": raw},
    )

    packed = msgpack.packb(msg, use_bin_type=True)
    unpacked = msgpack.unpackb(packed, raw=False)

    assert unpacked == msg
    assert unpacked["payload"]["data"] == raw


def test_gt_topic_prefix_is_a_distinct_namespace() -> None:
    assert GT_TOPIC_PREFIX == "gt."
    assert not "depth".startswith(GT_TOPIC_PREFIX)
    assert "gt.pose".startswith(GT_TOPIC_PREFIX)

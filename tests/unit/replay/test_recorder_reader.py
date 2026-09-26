from __future__ import annotations

from pathlib import Path

from aeris.replay import channels
from aeris.replay.reader import ReplayReader
from aeris.replay.recorder import ReplayRecorder


def test_round_trip_single_channel(tmp_path: Path) -> None:
    path = tmp_path / "test.mcap"
    with ReplayRecorder(path) as rec:
        rec.write(channels.VEHICLE_STATE, 0.0, {"x": 1.0})
        rec.write(channels.VEHICLE_STATE, 0.5, {"x": 1.5})

    with ReplayReader(path) as reader:
        messages = reader.read_channel(channels.VEHICLE_STATE)
    assert messages == [{"t_sim_s": 0.0, "x": 1.0}, {"t_sim_s": 0.5, "x": 1.5}]


def test_round_trip_multiple_channels(tmp_path: Path) -> None:
    path = tmp_path / "test.mcap"
    with ReplayRecorder(path) as rec:
        rec.write(channels.VEHICLE_STATE, 0.0, {"x": 1.0})
        rec.write(channels.MISSION_EVENTS, 0.1, {"event": "takeoff"})
        rec.write(channels.VEHICLE_STATE, 0.5, {"x": 1.5})

    with ReplayReader(path) as reader:
        assert reader.channel_names() == {channels.VEHICLE_STATE, channels.MISSION_EVENTS}
        all_messages = reader.read_all()
    assert all_messages[channels.VEHICLE_STATE] == [
        {"t_sim_s": 0.0, "x": 1.0},
        {"t_sim_s": 0.5, "x": 1.5},
    ]
    assert all_messages[channels.MISSION_EVENTS] == [{"t_sim_s": 0.1, "event": "takeoff"}]


def test_read_channel_that_was_never_written_is_empty(tmp_path: Path) -> None:
    path = tmp_path / "test.mcap"
    with ReplayRecorder(path) as rec:
        rec.write(channels.VEHICLE_STATE, 0.0, {"x": 1.0})

    with ReplayReader(path) as reader:
        assert reader.read_channel(channels.SAFETY_EVENTS) == []


def test_privileged_channels_are_flagged() -> None:
    assert channels.GT_POSE in channels.PRIVILEGED_CHANNELS
    assert channels.GT_CONTACTS in channels.PRIVILEGED_CHANNELS
    assert channels.VEHICLE_STATE not in channels.PRIVILEGED_CHANNELS


def test_messages_preserve_nested_structure(tmp_path: Path) -> None:
    path = tmp_path / "test.mcap"
    payload = {"pos": [1.0, 2.0, 3.0], "flags": {"armed": True}}
    with ReplayRecorder(path) as rec:
        rec.write(channels.VEHICLE_STATE, 1.0, payload)

    with ReplayReader(path) as reader:
        [message] = reader.read_channel(channels.VEHICLE_STATE)
    assert message["pos"] == [1.0, 2.0, 3.0]
    assert message["flags"] == {"armed": True}

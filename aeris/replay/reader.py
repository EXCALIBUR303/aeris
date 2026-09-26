"""``ReplayReader`` — reads one MCAP file back into per-channel JSON
messages (spec §39.1). Replay is data playback, not re-simulation, so
reading is exact by construction."""

from __future__ import annotations

import json
from pathlib import Path
from typing import IO, Any

from mcap.reader import make_reader


class ReplayReader:
    def __init__(self, path: Path | str) -> None:
        self._file: IO[bytes] = open(path, "rb")  # noqa: SIM115 -- closed in close()
        self._reader = make_reader(self._file)

    def read_channel(self, channel: str) -> list[dict[str, Any]]:
        """All messages on one channel, in ``log_time`` order."""
        return [
            json.loads(message.data.decode("utf-8"))
            for _schema, _chan, message in self._reader.iter_messages(topics=[channel])
        ]

    def read_all(self) -> dict[str, list[dict[str, Any]]]:
        """Every channel present in the file, in ``log_time`` order overall."""
        result: dict[str, list[dict[str, Any]]] = {}
        for _schema, chan, message in self._reader.iter_messages():
            result.setdefault(chan.topic, []).append(json.loads(message.data.decode("utf-8")))
        return result

    def channel_names(self) -> set[str]:
        summary = self._reader.get_summary()
        if summary is None:
            return set()
        return {channel.topic for channel in summary.channels.values()}

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> ReplayReader:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

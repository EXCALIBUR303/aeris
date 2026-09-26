"""``ReplayRecorder`` — writes one MCAP file (spec §39.1, ADR-012).

Each message is a JSON object, keyed by ``t_sim_s`` (spec §39.2's sync
rule) — stored both as the MCAP ``log_time`` (nanoseconds, required by the
container format) and inside the JSON body itself, since replay keys on
simulation time, not wall time or the MCAP container's own clock.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import IO, Any

from mcap.writer import Writer

_SCHEMA_ENCODING = "jsonschema"
_MESSAGE_ENCODING = "json"
_ANY_OBJECT_SCHEMA = b'{"type": "object"}'


class ReplayRecorder:
    """Each channel gets its own schema, registered once on first use --
    currently a permissive "any JSON object" schema per channel (tightened
    once a later phase's per-channel message shape stabilizes)."""

    def __init__(self, path: Path | str) -> None:
        self._file: IO[bytes] = open(path, "wb")  # noqa: SIM115 -- closed in close()
        self._writer = Writer(self._file)
        self._writer.start()
        self._channel_ids: dict[str, int] = {}
        self._sequence: dict[str, int] = {}

    def _channel_id(self, channel: str) -> int:
        if channel not in self._channel_ids:
            schema_id = self._writer.register_schema(
                name=f"aeris{channel.replace('/', '.')}",
                encoding=_SCHEMA_ENCODING,
                data=_ANY_OBJECT_SCHEMA,
            )
            self._channel_ids[channel] = self._writer.register_channel(
                topic=channel, message_encoding=_MESSAGE_ENCODING, schema_id=schema_id
            )
            self._sequence[channel] = 0
        return self._channel_ids[channel]

    def write(self, channel: str, t_sim_s: float, message: dict[str, Any]) -> None:
        channel_id = self._channel_id(channel)
        log_time_ns = max(0, int(t_sim_s * 1e9))
        payload = {"t_sim_s": t_sim_s, **message}
        seq = self._sequence[channel]
        self._sequence[channel] = seq + 1
        self._writer.add_message(
            channel_id,
            log_time=log_time_ns,
            data=json.dumps(payload).encode("utf-8"),
            publish_time=log_time_ns,
            sequence=seq,
        )

    def close(self) -> None:
        # mcap.Writer.finish has no type annotations at all (unlike its
        # other methods), which mypy flags regardless of this module's own
        # strictness -- the one third-party gap in an otherwise-typed lib.
        self._writer.finish()  # type: ignore[no-untyped-call]
        self._file.close()

    def __enter__(self) -> ReplayRecorder:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

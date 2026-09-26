"""``GzBridgeClient`` — the AERIS-venv-side ZeroMQ subscriber (spec §18.4).

Runs entirely in AERIS's normal ``uv``-managed venv (``sim`` extra:
``pyzmq``, ``msgpack``) — never imports ``gz.*`` (spec §14.3 contract 5;
that's confined to :mod:`aeris.simulation.bridge._gz_process`, a separate
OS process this client talks to over ZeroMQ PUB/SUB).
"""

from __future__ import annotations

from types import TracebackType

import msgpack
import zmq

from aeris.simulation.bridge.schema import GT_TOPIC_PREFIX, SensorMessage

_DEFAULT_RCV_TIMEOUT_MS = 1000

# The topic is a prefix of a single-part message (topic + delimiter + msgpack
# payload), not a separate multipart frame. This is the classic ZeroMQ
# PUB/SUB topic-filtering idiom, required here because `zmq.CONFLATE` is
# only well-defined for single-part messages — a multipart `[topic, payload]`
# framing was tried first and confirmed (Phase 8) to silently drop messages
# under CONFLATE, a documented libzmq limitation.
_TOPIC_DELIMITER = b"\x00"


class GroundTruthTopicError(Exception):
    """Raised when code tries to subscribe a regular client to a ``gt.*`` topic.

    Spec §18's validation gate requires "the GT channel is unreachable from
    the agent client" — enforced here rather than only by convention, so a
    typo'd topic name can't silently leak ground truth to agent-facing code.
    """


class GzBridgeClient:
    """Subscribes to one or more Sensor Bridge topics over ZeroMQ SUB.

    ``conflate=True`` (the default) keeps only the latest message per topic
    — the right setting for a control-loop consumer (spec §18.4: "a
    subscriber CONFLATE/HWM setting keeps only the latest frame per sensor
    for control"). Pass ``conflate=False`` for a lossless-ish recorder
    subscription (a large ``RCVHWM`` instead).
    """

    def __init__(
        self,
        endpoint: str,
        topics: list[str],
        *,
        conflate: bool = True,
        rcv_hwm: int = 10,
        rcv_timeout_ms: int = _DEFAULT_RCV_TIMEOUT_MS,
    ) -> None:
        for topic in topics:
            if topic.startswith(GT_TOPIC_PREFIX):
                raise GroundTruthTopicError(
                    f"GzBridgeClient cannot subscribe to ground-truth topic {topic!r} — "
                    f"use a privileged evaluator-only client for '{GT_TOPIC_PREFIX}*' topics"
                )

        self._ctx: zmq.Context[zmq.Socket[bytes]] = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        if conflate:
            self._sock.setsockopt(zmq.CONFLATE, 1)
        else:
            self._sock.setsockopt(zmq.RCVHWM, rcv_hwm)
        self._sock.setsockopt(zmq.RCVTIMEO, rcv_timeout_ms)
        self._sock.connect(endpoint)
        for topic in topics:
            self._sock.setsockopt(zmq.SUBSCRIBE, topic.encode() + _TOPIC_DELIMITER)

    def recv(self) -> SensorMessage | None:
        """Block for one message (up to the configured timeout), or return ``None``."""
        try:
            frame = self._sock.recv()
        except zmq.Again:
            return None
        _topic, message = decode_message(frame)
        return message

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> GzBridgeClient:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()


def encode_message(topic: str, message: SensorMessage) -> bytes:
    """The wire format a publisher sends: ``topic + \\x00 + msgpack_bytes``, one frame.

    :mod:`aeris.simulation.bridge._gz_process` runs in the same Python
    environment as this module (Phase 8 finding: no separate Homebrew-Python
    venv is needed) and imports this function directly, so both sides of
    the wire share one implementation of the framing.
    """
    packed: bytes = msgpack.packb(message, use_bin_type=True)
    return topic.encode() + _TOPIC_DELIMITER + packed


def decode_message(frame: bytes) -> tuple[str, SensorMessage]:
    topic_bytes, _, payload = frame.partition(_TOPIC_DELIMITER)
    message: SensorMessage = msgpack.unpackb(payload, raw=False)
    return topic_bytes.decode(), message

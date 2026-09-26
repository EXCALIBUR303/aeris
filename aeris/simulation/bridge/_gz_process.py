"""The Sensor Bridge process (spec §18.4) — republishes gz sensor topics over ZeroMQ.

**The only AERIS module allowed to import ``gz.*``** (spec §14.3 contract
5, enforced by import-linter). Runs as a standalone OS process — never
imported by other AERIS code — launched by
:func:`aeris.simulation.bridge.launch.start_bridge`, which sets the
``DYLD_LIBRARY_PATH``/``PYTHONPATH`` this needs to import Homebrew's
``gz-transport13``/``gz-msgs10`` Python bindings.

**Phase 8 finding** (documented in ``docs/sensors.md``): those bindings are
ABI-compatible with this project's own ``uv``-managed CPython 3.12
interpreter — no separate Homebrew-Python venv is needed, only
``DYLD_LIBRARY_PATH=/opt/homebrew/lib``, ``PYTHONPATH`` pointed at
Homebrew's ``site-packages``, and the ``protobuf`` package (for
``gz.msgs10``'s ``google.protobuf`` dependency) installed in this venv.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import zmq

from aeris.simulation.bridge.client import encode_message
from aeris.simulation.bridge.schema import HealthEntry, SensorKind, make_message

try:
    from gz.msgs10.camera_info_pb2 import CameraInfo
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node
except ImportError as exc:  # pragma: no cover - only reachable outside the bridge env
    raise RuntimeError(
        "aeris.simulation.bridge._gz_process requires the gz-transport13/gz-msgs10 Python "
        "bindings on sys.path plus DYLD_LIBRARY_PATH set — launch via "
        "aeris.simulation.bridge.launch.start_bridge, not directly (see docs/sensors.md)."
    ) from exc

_HEALTH_PERIOD_S = 1.0


@dataclass(slots=True)
class _TopicHealth:
    count: int = 0
    last_recv_monotonic: float = 0.0
    _rate_window_start: float = 0.0
    _rate_window_count: int = 0

    def record(self, now: float) -> None:
        self.count += 1
        self.last_recv_monotonic = now

    def snapshot_rate_hz(self, now: float) -> float:
        elapsed = now - self._rate_window_start
        rate = (self.count - self._rate_window_count) / elapsed if elapsed > 0 else 0.0
        self._rate_window_start = now
        self._rate_window_count = self.count
        return rate


def _stamp_to_t_sim_s(msg: Image | CameraInfo) -> float:
    return float(msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9)


class SensorBridge:
    """Subscribes to one vehicle instance's gz sensor topics, republishes over ZeroMQ PUB."""

    def __init__(self, *, zmq_endpoint: str) -> None:
        self._pub_ctx: zmq.Context[zmq.Socket[bytes]] = zmq.Context.instance()
        self._pub = self._pub_ctx.socket(zmq.PUB)
        self._pub.setsockopt(zmq.SNDHWM, 10)
        self._pub.bind(zmq_endpoint)
        self._node = Node()
        self._health: dict[str, _TopicHealth] = {}
        self._seq = 0

    def _publish(
        self,
        topic: str,
        *,
        kind: SensorKind,
        sensor_id: str,
        frame_id: str,
        t_sim_s: float,
        payload: dict[str, object],
    ) -> None:
        self._seq += 1
        message = make_message(
            kind=kind,
            sensor_id=sensor_id,
            frame_id=frame_id,
            t_sim_s=t_sim_s,
            seq=self._seq,
            payload=payload,
        )
        self._pub.send(encode_message(topic, message))
        self._health.setdefault(topic, _TopicHealth()).record(time.monotonic())

    def _on_depth(self, msg: Image) -> None:
        self._publish(
            "depth",
            kind="depth",
            sensor_id="front_depth",
            frame_id="front_depth_optical",
            t_sim_s=_stamp_to_t_sim_s(msg),
            payload={"width": msg.width, "height": msg.height, "data": bytes(msg.data)},
        )

    def _on_rgb(self, msg: Image) -> None:
        self._publish(
            "rgb",
            kind="rgb",
            sensor_id="front_rgb",
            frame_id="front_rgb_optical",
            t_sim_s=_stamp_to_t_sim_s(msg),
            payload={
                "width": msg.width,
                "height": msg.height,
                "encoding": "raw_rgb8",
                "data": bytes(msg.data),
            },
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._publish(
            "camera_info",
            kind="camera_info",
            sensor_id="front_depth",
            frame_id="front_depth_optical",
            t_sim_s=_stamp_to_t_sim_s(msg),
            payload={
                "width": msg.width,
                "height": msg.height,
                "intrinsics": list(msg.intrinsics.k),
            },
        )

    def _publish_health(self) -> None:
        now = time.monotonic()
        topics: dict[str, HealthEntry] = {
            topic: {
                "rate_hz": h.snapshot_rate_hz(now),
                "age_s": now - h.last_recv_monotonic,
                "count": h.count,
            }
            for topic, h in self._health.items()
        }
        self._seq += 1
        message = make_message(
            kind="health",
            sensor_id="bridge",
            frame_id="",
            t_sim_s=0.0,
            seq=self._seq,
            payload={"topics": topics},
        )
        self._pub.send(encode_message("health", message))

    def run(
        self,
        *,
        depth_topic: str,
        camera_info_topic: str,
        rgb_topic: str | None,
        duration_s: float | None = None,
    ) -> None:
        self._node.subscribe(Image, depth_topic, self._on_depth)
        self._node.subscribe(CameraInfo, camera_info_topic, self._on_camera_info)
        if rgb_topic:
            self._node.subscribe(Image, rgb_topic, self._on_rgb)

        deadline = None if duration_s is None else time.monotonic() + duration_s
        next_health = time.monotonic() + _HEALTH_PERIOD_S
        while deadline is None or time.monotonic() < deadline:
            time.sleep(0.05)
            if time.monotonic() >= next_health:
                self._publish_health()
                next_health = time.monotonic() + _HEALTH_PERIOD_S


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zmq-endpoint", required=True)
    parser.add_argument("--depth-topic", default="/depth_camera")
    parser.add_argument("--camera-info-topic", default="/camera_info")
    parser.add_argument("--rgb-topic", default=None)
    parser.add_argument("--duration-s", type=float, default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    bridge = SensorBridge(zmq_endpoint=args.zmq_endpoint)
    bridge.run(
        depth_topic=args.depth_topic,
        camera_info_topic=args.camera_info_topic,
        rgb_topic=args.rgb_topic,
        duration_s=args.duration_s,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

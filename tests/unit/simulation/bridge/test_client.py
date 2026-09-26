"""Unit tests for :class:`GzBridgeClient` (spec §18.4, §18's GT-isolation gate).

Uses a plain local ZeroMQ PUB socket as a stand-in publisher — no gz
dependency, so these run in the default ``unit`` tier.
"""

from __future__ import annotations

import time

import pytest
import zmq

from aeris.simulation.bridge.client import GroundTruthTopicError, GzBridgeClient, encode_message
from aeris.simulation.bridge.schema import make_message

_ENDPOINT = "tcp://127.0.0.1:0"  # bind to an ephemeral port, then discover it


@pytest.fixture
def pub_socket():
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUB)
    port = sock.bind_to_random_port("tcp://127.0.0.1")
    yield sock, f"tcp://127.0.0.1:{port}"
    sock.close()


def test_refuses_to_subscribe_to_ground_truth_topics() -> None:
    with pytest.raises(GroundTruthTopicError):
        GzBridgeClient("tcp://127.0.0.1:19000", ["gt.pose"])


def test_refuses_ground_truth_topic_even_mixed_with_regular_ones() -> None:
    with pytest.raises(GroundTruthTopicError):
        GzBridgeClient("tcp://127.0.0.1:19000", ["depth", "gt.pose"])


def test_receives_a_published_message(pub_socket) -> None:
    sock, endpoint = pub_socket
    client = GzBridgeClient(endpoint, ["depth"], rcv_timeout_ms=500)

    sent = make_message(
        kind="depth",
        sensor_id="front_depth",
        frame_id="front_depth_optical",
        t_sim_s=1.0,
        seq=1,
        payload={"x": 1},
    )
    # ZeroMQ's "slow joiner": a PUB can drop messages sent before a SUB's
    # subscription has propagated, so resend until one lands (bounded).
    received = None
    for _ in range(20):
        sock.send(encode_message("depth", sent))
        received = client.recv()
        if received is not None:
            break
        time.sleep(0.05)

    assert received == sent
    client.close()


def test_does_not_receive_messages_on_unsubscribed_topics(pub_socket) -> None:
    sock, endpoint = pub_socket
    client = GzBridgeClient(endpoint, ["depth"], rcv_timeout_ms=300)
    time.sleep(0.2)

    other = make_message(
        kind="rgb",
        sensor_id="front_rgb",
        frame_id="front_rgb_optical",
        t_sim_s=1.0,
        seq=1,
        payload={},
    )
    sock.send(encode_message("rgb", other))

    assert client.recv() is None
    client.close()


def test_context_manager_closes_socket(pub_socket) -> None:
    _sock, endpoint = pub_socket
    with GzBridgeClient(endpoint, ["depth"], rcv_timeout_ms=200) as client:
        assert client.recv() is None  # nothing published; just exercising the timeout path

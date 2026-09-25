import pytest

from aeris.core.errors import UnauthorizedEndpointError
from aeris.vehicle.hardware_guard import assert_endpoint_allowed
from aeris.vehicle.interface import VehicleEndpoint


def test_ipv4_loopback_allowed():
    assert_endpoint_allowed(VehicleEndpoint(host="127.0.0.1", port=14540, simulated=False))


def test_ipv4_loopback_other_than_dot_one_allowed():
    # 127.0.0.0/8 is all loopback, not just 127.0.0.1
    assert_endpoint_allowed(VehicleEndpoint(host="127.5.5.5", port=14540, simulated=False))


def test_ipv6_loopback_allowed():
    assert_endpoint_allowed(VehicleEndpoint(host="::1", port=14540, simulated=False))


def test_localhost_hostname_allowed():
    assert_endpoint_allowed(VehicleEndpoint(host="localhost", port=14540, simulated=False))


def test_non_loopback_with_simulated_true_allowed():
    # spec §12/§16.6: the documented Linux-fallback case (Docker/VM host)
    assert_endpoint_allowed(VehicleEndpoint(host="192.168.64.5", port=14540, simulated=True))


def test_non_loopback_without_simulated_rejected():
    with pytest.raises(UnauthorizedEndpointError):
        assert_endpoint_allowed(VehicleEndpoint(host="192.168.64.5", port=14540, simulated=False))


def test_wildcard_bind_address_is_not_loopback_and_rejected_by_default():
    # 0.0.0.0 accepts traffic from any interface, not just loopback --
    # deliberately NOT treated as safe by default (spec §16.6's guard is
    # meant to actually restrict, not just pattern-match a common default).
    with pytest.raises(UnauthorizedEndpointError):
        assert_endpoint_allowed(VehicleEndpoint(host="0.0.0.0", port=14540, simulated=False))


def test_public_hostname_rejected():
    with pytest.raises(UnauthorizedEndpointError):
        assert_endpoint_allowed(VehicleEndpoint(host="example.com", port=14540, simulated=False))


def test_real_looking_lan_ip_rejected_by_default():
    with pytest.raises(UnauthorizedEndpointError):
        assert_endpoint_allowed(VehicleEndpoint(host="10.0.0.5", port=14540, simulated=False))

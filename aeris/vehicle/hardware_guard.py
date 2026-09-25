"""The hardware guard (spec §16.6): AERIS never connects to a real vehicle.

    "``VehicleEndpoint`` accepts only loopback or configured-simulator
    hosts (``127.0.0.1``, ``::1``, or an explicitly configured
    Linux-fallback sim host tagged ``simulated: true``) ... There is no
    code path or documentation for real-vehicle flight."

Every adapter's ``connect()`` must call :func:`assert_endpoint_allowed`
before opening any connection. This is deliberately a free function, not a
method that could be skipped by a hypothetical alternate adapter — it's
also unit-tested directly (spec §51 Phase 4: "refuses non-loopback
endpoints" is part of the validation gate).
"""

from __future__ import annotations

import ipaddress

from aeris.core.errors import UnauthorizedEndpointError
from aeris.vehicle.interface import VehicleEndpoint

_LOOPBACK_HOSTNAMES = {"localhost"}


def _is_loopback(host: str) -> bool:
    if host in _LOOPBACK_HOSTNAMES:
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False  # not a literal IP; not recognized as loopback


def assert_endpoint_allowed(endpoint: VehicleEndpoint) -> None:
    """Raise :class:`UnauthorizedEndpointError` unless ``endpoint`` is a
    loopback address, or a non-loopback host explicitly marked
    ``simulated=True`` (the documented Linux-fallback case, spec §12/§16.6
    — e.g. a Docker container's or VM's IP for Option B1/B2).

    There is intentionally no other override. A non-loopback,
    non-``simulated`` endpoint is always rejected, regardless of any other
    configuration.
    """
    if _is_loopback(endpoint.host):
        return
    if endpoint.simulated:
        return
    raise UnauthorizedEndpointError(
        f"refusing to connect to {endpoint.host}:{endpoint.port} — not loopback and not "
        f"explicitly marked simulated=True. AERIS never connects to real hardware "
        f"(spec §16.6). If this is a Linux-fallback simulator host (spec §12), "
        f"construct the VehicleEndpoint with simulated=True."
    )

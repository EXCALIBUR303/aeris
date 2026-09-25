"""AERIS Flight safety architecture (spec §16).

``aeris.safety.SafetySupervisor`` is the only package permitted to hold a
:class:`aeris.vehicle.interface.CommandPort` (spec §14.3 contract 3, §16.1
rule 3) — nothing outside this package should ever call
``VehicleInterface.command_port()``. See
:mod:`aeris.safety.supervisor` and ``tests/unit/safety/test_command_port_contract.py``.
"""

from __future__ import annotations

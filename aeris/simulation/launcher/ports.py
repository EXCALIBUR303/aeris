"""Per-instance MAVLink UDP port formulas.

Transcribed from ``ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`` in the
pinned PX4 checkout (read directly, Phase 3) — PX4 computes these itself at
boot from ``$px4_instance``; this module lets AERIS compute the *same*
numbers ahead of time, so a launcher can connect without guessing or
parsing PX4's own boot log.

Do not hand-edit these formulas without re-checking that init script if the
PX4 pin (``configs/versions.lock.yaml``) ever changes — port math has
silently changed between PX4 releases before.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class InstancePorts:
    """The MAVLink UDP ports PX4 opens for one SITL instance.

    All are *local* ports PX4 itself binds (i.e. what an external tool
    connects to), except the ``*_remote`` fields, which are where PX4
    proactively sends packets (relevant for e.g. injecting external-vision
    input in Phase 23 — spec §34.2).
    """

    instance: int

    # GCS link (e.g. QGroundControl) — spec §10.
    gcs_local: int

    # API/Offboard link — what aeris.vehicle adapters connect to (spec §15).
    offboard_local: int
    offboard_remote: int

    # Onboard payload (camera) link.
    onboard_payload_local: int
    onboard_payload_remote: int

    # Onboard gimbal link.
    onboard_gimbal_local: int
    onboard_gimbal_remote: int


def instance_ports(instance: int = 0) -> InstancePorts:
    """Compute :class:`InstancePorts` for a given SITL instance number.

    Matches ``px4-rc.mavlink`` exactly, including its >9-instance special
    case for ``offboard_remote`` (PX4 reuses 14549 beyond instance 9 to
    avoid port overlap; not expected to matter for AERIS's single- or
    few-vehicle profiles, kept for fidelity).
    """
    if instance < 0:
        raise ValueError(f"instance must be >= 0, got {instance}")

    offboard_remote = 14540 + instance if instance <= 9 else 14549

    return InstancePorts(
        instance=instance,
        gcs_local=18570 + instance,
        offboard_local=14580 + instance,
        offboard_remote=offboard_remote,
        onboard_payload_local=14280 + instance,
        onboard_payload_remote=14030 + instance,
        onboard_gimbal_local=13030 + instance,
        onboard_gimbal_remote=13280 + instance,
    )

"""aeris.vehicle — the transport-agnostic vehicle interface and its adapters.

Spec §15. ``aeris.vehicle.interface`` defines what autonomy code sees;
``aeris.vehicle.px4_mavlink`` is the one concrete implementation for V1
(ADR-0003, finalized in Phase 4 — see that module's docstring for why
MAVSDK isn't it).
"""

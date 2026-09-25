"""aeris.vehicle.px4_mavlink — the V1 PX4 vehicle adapter (ADR-0003, finalized Phase 4).

**MAVSDK is not primary, despite ADR-0003's original wording.** Phase 1
found ``mavsdk`` (Python) 4.0.0's native binding segfaults on
``Mavsdk(Configuration(...))`` construction on this machine. Phase 4
re-tested against the same (still-latest) release with no newer point
release available, in both a synchronous and a proper ``asyncio.run``
context — it segfaults identically every time. **pymavlink is therefore
AERIS's primary and only V1 vehicle transport.** No ``aeris.vehicle.px4_mavsdk``
package was built. Revisit if a newer mavsdk release ships (spec §55 does
not require Opus escalation for this — it is exactly the kind of ordinary
environment/dependency problem spec §51's "when not to use Opus" list
names).

This adapter converts PX4's NED/FRD conventions to AERIS's ENU/FLU
(ADR-0008) at this one boundary — see :mod:`aeris.core.frames.conventions`,
which only this package (and its own tests) may import (import-linter
contract).
"""

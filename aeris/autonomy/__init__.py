"""AERIS autonomy engine (spec §13.2): pure Python, runs on sim clock, no
transport code. May depend only on ``aeris.vehicle.interface`` types and
``aeris.safety`` — never a vehicle adapter implementation directly (spec
§14.3 contract 2, import-linter enforced).
"""

from __future__ import annotations

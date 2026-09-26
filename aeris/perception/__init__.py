"""aeris.perception — sensor back-projection (spec §18, §20, Phase 8+).

May not import :mod:`aeris.simulation.groundtruth` (import-linter contract
1, spec §14.3) — perception works only from what the Sensor Bridge
publishes, never simulator ground truth.
"""

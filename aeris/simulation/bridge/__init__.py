"""aeris.simulation.bridge — the Sensor Bridge process + IPC (spec §18.4, ADR-004).

``gz.*`` is imported *only* inside :mod:`aeris.simulation.bridge._gz_process`
(spec §14.3 contract 5) — that module runs as a standalone OS process, not
imported by anything else in AERIS. Everything else here (:mod:`schema`,
:mod:`client`, :mod:`launch`) is a normal, gz-free part of the AERIS venv.
"""

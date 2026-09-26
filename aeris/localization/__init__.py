"""Agent-legitimate pose sources (spec §22): PX4 EKF2 (default V1) and a
synthetic-noise wrapper for sensitivity experiments. Ground truth
(``L-GT``) is deliberately absent here -- it's evaluator-only (spec §17.4)
and this package must not import ``aeris.simulation.groundtruth``
(spec §14.3 contract 1).
"""

"""Common types shared across AERIS.

Zero dependencies on the rest of AERIS, including other ``aeris.core``
modules (import-linter contract: "core.errors and core.types are
dependency-free within aeris.core").
"""

from __future__ import annotations

from enum import StrEnum, unique


@unique
class Provenance(StrEnum):
    """Where a value in an ``AgentObservation`` (or similar) actually came from.

    Spec §17.4: "Every ``AgentObservation`` field carries a ``Provenance``
    ... ``ObservationBuilder`` raises an error on ``ORACLE`` unless the run
    config sets ``oracle_baseline: true``."

    Values:
        SENSOR: a real (possibly noisy) sensor reading — depth, LiDAR, IMU.
        ESTIMATE: a filtered/estimated quantity — PX4 EKF2 pose, SLAM pose.
        MISSION_INPUT: operator-provided knowledge — region names, target
            classes — not derived from the simulator at all.
        ORACLE: simulator ground truth. Legitimate only for the evaluator,
            the world generator, offline label generation, and declared
            ``oracle_baseline`` / ``critic_privileged`` runs (spec §27.8).
    """

    SENSOR = "sensor"
    ESTIMATE = "estimate"
    MISSION_INPUT = "mission_input"
    ORACLE = "oracle"

    @property
    def is_agent_legitimate(self) -> bool:
        """Whether a policy may use a value with this provenance by default."""
        return self is not Provenance.ORACLE


@unique
class Tier(StrEnum):
    """Which simulation tier a run/observation/result came from (spec §17.1)."""

    FASTSIM = "F"
    HIGH_FIDELITY = "H"


@unique
class RunStatus(StrEnum):
    """Terminal status of an experiment run (spec §38.4)."""

    COMPLETED = "completed"
    FAILED = "failed"
    ABORTED = "aborted"
    INVALID = "invalid"

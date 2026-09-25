"""Typed exception hierarchy for AERIS.

Spec §14.2: "No ``except Exception: pass``. Exceptions are typed
(``aeris.core.errors``)." Every AERIS-raised error should be one of these
(or a documented subclass added by the package that needs it), never a bare
``Exception`` or ``ValueError`` reused across unrelated failure modes.

This module has zero dependencies on the rest of AERIS (import-linter
contract: "core.errors and core.types are dependency-free within
aeris.core").
"""

from __future__ import annotations


class AerisError(Exception):
    """Base class for every AERIS-raised exception."""


# --- Configuration ----------------------------------------------------------


class ConfigError(AerisError):
    """A config file, override, or composition is invalid."""


class ConfigNotFoundError(ConfigError):
    """A referenced config file (base or ``extends:`` target) does not exist."""


class ConfigCompositionError(ConfigError):
    """``extends:`` composition or CLI overrides could not be resolved."""


# --- Safety (spec §16) -------------------------------------------------------


class SafetyError(AerisError):
    """Base class for Safety-layer errors. Never silently swallowed."""


class CommandRejectedError(SafetyError):
    """The Safety validator (S1) rejected a command (NaN, out-of-bounds, ...)."""


class UnsafeStateError(SafetyError):
    """A command was attempted from a state the state-gate (S2) disallows."""


class WatchdogTimeoutError(SafetyError):
    """An S4 watchdog (autonomy tick, sensor staleness) tripped."""


# --- Vehicle / transport (spec §15) -----------------------------------------


class VehicleError(AerisError):
    """Base class for vehicle-interface/adapter errors."""


class VehicleConnectionError(VehicleError):
    """Could not connect to, or lost connection to, the vehicle endpoint."""


class UnauthorizedEndpointError(VehicleError):
    """A ``VehicleEndpoint`` failed the hardware guard (spec §16.6).

    Raised when an endpoint is not loopback or an explicitly configured
    simulator host tagged ``simulated: true``. There is intentionally no
    override for this — see spec §16.6, "there is no code path ... for real
    vehicle flight."
    """


# --- Simulation (spec §17) ---------------------------------------------------


class SimulationError(AerisError):
    """Base class for Tier H (PX4 SITL + Gazebo) / Tier F (FastSim) errors."""


class SimulationLaunchError(SimulationError):
    """A simulation profile failed to reach readiness (spec §17.2)."""


class SimulationTierMismatchError(SimulationError):
    """An operation was attempted against the wrong tier (H vs F)."""


class Px4NotFoundError(SimulationError):
    """The configured PX4 checkout doesn't exist or hasn't been built.

    PX4 is an external, pinned dependency (spec §8.3, ADR-0002) — AERIS
    never builds it automatically. See ``docs/mac-setup.md``.
    """


# --- Provenance / ground-truth separation (spec §17.4) -----------------------


class ProvenanceError(AerisError):
    """Base class for ground-truth/observation separation violations."""


class OracleObservationError(ProvenanceError):
    """An ``ORACLE``-provenance field was accessed without ``oracle_baseline: true``.

    This is the runtime enforcement half of the import-linter contract that
    blocks ``aeris.simulation.groundtruth`` imports from learning/autonomy
    code (spec §17.4) — it catches privileged data that reaches an
    ``AgentObservation`` through a path the import contract can't see (e.g.
    a dict merge), not just a direct import.
    """


# --- Experiments / reproducibility (spec §38, §40) ---------------------------


class ExperimentError(AerisError):
    """Base class for experiment/run/manifest errors."""


class PreregistrationRequiredError(ExperimentError):
    """A test-split evaluation was attempted without a committed pre-registration.

    Spec §38.5: "The runner refuses test-split evaluation without a
    committed pre-registration whose hash is stored in the manifest."
    """


class SplitViolationError(ExperimentError):
    """A run tried to use world seeds from a split it isn't authorized for.

    Spec §33.2: "The training entry point refuses to generate worlds from
    val/test ranges. The tuning entry point refuses test ranges."
    """

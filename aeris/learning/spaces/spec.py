"""Observation specs: every field declares its shape, bounds and
:class:`~aeris.core.types.Provenance` (spec §17.4), and the whole spec has a
stable hash that is serialized into every checkpoint -- deployment refuses a
checkpoint whose spec hash doesn't match the builder it's about to feed
(spec §27.6).
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass

import numpy as np
from gymnasium import spaces

from aeris.core.errors import OracleObservationError
from aeris.core.types import Provenance


@dataclass(frozen=True, slots=True)
class FieldSpec:
    name: str
    shape: tuple[int, ...]
    low: float
    high: float
    provenance: Provenance
    note: str = ""


@dataclass(frozen=True, slots=True)
class ObservationSpec:
    name: str
    version: str
    fields: tuple[FieldSpec, ...]

    def canonical(self) -> str:
        return json.dumps(
            {
                "name": self.name,
                "version": self.version,
                "fields": [
                    [f.name, list(f.shape), f.low, f.high, f.provenance.value] for f in self.fields
                ],
            },
            sort_keys=True,
            separators=(",", ":"),
        )

    def hash(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def gym_space(self) -> spaces.Dict:
        return spaces.Dict(
            {
                f.name: spaces.Box(low=f.low, high=f.high, shape=f.shape, dtype=np.float32)
                for f in self.fields
            }
        )

    def check_provenance(self, *, oracle_baseline: bool) -> None:
        """Raise on any ORACLE field unless this run is a declared
        ``oracle_baseline`` (spec §17.4, §27.8)."""
        oracle = [f.name for f in self.fields if not f.provenance.is_agent_legitimate]
        if oracle and not oracle_baseline:
            raise OracleObservationError(
                f"observation spec {self.name!r} has ORACLE fields {oracle} "
                "but the run config does not set oracle_baseline: true"
            )


def verify_checkpoint_spec(expected_hash: str, spec: ObservationSpec) -> None:
    """Deployment-side guard: refuse a checkpoint trained on a different spec."""
    if expected_hash != spec.hash():
        raise ValueError(
            f"checkpoint observation-spec hash {expected_hash[:12]} != builder's "
            f"{spec.hash()[:12]} ({spec.name} {spec.version}) -- refusing to deploy"
        )

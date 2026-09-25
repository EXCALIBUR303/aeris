from aeris.core.types import Provenance, RunStatus, Tier


def test_only_oracle_is_not_agent_legitimate():
    for p in Provenance:
        expected = p is not Provenance.ORACLE
        assert p.is_agent_legitimate is expected


def test_provenance_values_are_stable_strings():
    # These strings appear in manifests/replays; changing them is a spec change.
    assert Provenance.SENSOR == "sensor"
    assert Provenance.ESTIMATE == "estimate"
    assert Provenance.MISSION_INPUT == "mission_input"
    assert Provenance.ORACLE == "oracle"


def test_tier_values():
    assert Tier.FASTSIM == "F"
    assert Tier.HIGH_FIDELITY == "H"


def test_run_status_values():
    assert {s.value for s in RunStatus} == {"completed", "failed", "aborted", "invalid"}

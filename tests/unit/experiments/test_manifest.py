from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path

import pytest

from aeris.core.types import RunStatus, Tier
from aeris.experiments.manifest import (
    RunManifest,
    aeris_version,
    git_info,
    hardware_snapshot,
    make_run_id,
    new_manifest,
    read_manifest,
    software_versions,
    write_manifest,
)


def test_git_info_reports_a_real_sha_in_this_checkout() -> None:
    # This repo IS a git checkout, so git_info() should find something --
    # not asserting a specific SHA (that changes), just that it works.
    sha, _dirty = git_info()
    assert sha is not None
    assert len(sha) == 40  # a full git SHA-1 hex digest


def test_git_info_returns_none_outside_a_git_checkout(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr("aeris.experiments.manifest._REPO_ROOT", tmp_path)
    sha, dirty = git_info()
    assert sha is None
    assert dirty is False


def test_aeris_version_returns_a_string() -> None:
    assert isinstance(aeris_version(), str)
    assert aeris_version() != ""


def test_software_versions_includes_python() -> None:
    versions = software_versions()
    assert "python" in versions
    assert versions["python"][0].isdigit()


def test_hardware_snapshot_includes_machine_and_cpu_count() -> None:
    snapshot = hardware_snapshot()
    assert "machine" in snapshot
    assert "cpu_count" in snapshot
    assert int(snapshot["cpu_count"]) >= 0


def test_make_run_id_format() -> None:
    now = datetime(2026, 9, 26, 12, 30, 45, tzinfo=UTC)
    run_id = make_run_id("square", "abc12345def", now=now)
    assert run_id == "20260926-123045_square_abc12345"


def test_new_manifest_has_mandatory_fields_populated() -> None:
    manifest = new_manifest(
        run_id="test_run",
        task="waypoint_mission",
        method="scripted",
        tier=Tier.HIGH_FIDELITY,
        config_hash="deadbeef",
    )
    assert manifest.run_id == "test_run"
    assert manifest.timestamp_utc  # non-empty
    assert manifest.aeris_version
    assert manifest.config_hash == "deadbeef"
    assert manifest.task == "waypoint_mission"
    assert manifest.method == "scripted"
    assert manifest.tier == Tier.HIGH_FIDELITY
    assert manifest.software_versions
    assert manifest.hardware_snapshot
    assert manifest.status == RunStatus.FAILED  # pessimistic default before finalization


def test_manifest_is_reportable_requires_completed_and_clean() -> None:
    base = new_manifest(run_id="r", task="t", method="m", tier=Tier.HIGH_FIDELITY, config_hash="h")
    dirty_completed = base.model_copy(update={"git_dirty": True, "status": RunStatus.COMPLETED})
    assert not dirty_completed.is_reportable

    clean_failed = base.model_copy(update={"git_dirty": False, "status": RunStatus.FAILED})
    assert not clean_failed.is_reportable

    clean_completed = base.model_copy(update={"git_dirty": False, "status": RunStatus.COMPLETED})
    assert clean_completed.is_reportable


def test_write_then_read_manifest_round_trips(tmp_path: Path) -> None:
    manifest = new_manifest(
        run_id="roundtrip",
        task="waypoint_mission",
        method="scripted",
        tier=Tier.HIGH_FIDELITY,
        config_hash="cafef00d",
        seed=42,
    )
    manifest.status = RunStatus.COMPLETED
    manifest.status_reason = "ok"

    write_manifest(manifest, tmp_path)
    loaded = read_manifest(tmp_path)

    assert loaded == manifest


def test_read_manifest_raises_when_missing(tmp_path: Path) -> None:
    from aeris.core.errors import ConfigNotFoundError

    with pytest.raises(ConfigNotFoundError):
        read_manifest(tmp_path)


def test_manifest_json_is_actually_on_disk(tmp_path: Path) -> None:
    manifest = new_manifest(
        run_id="disk_check", task="t", method="m", tier=Tier.FASTSIM, config_hash="h"
    )
    path = write_manifest(manifest, tmp_path)
    assert path == tmp_path / "manifest.json"
    assert path.is_file()


def test_manifest_is_json_serializable_via_pydantic() -> None:
    manifest = new_manifest(
        run_id="r", task="t", method="m", tier=Tier.HIGH_FIDELITY, config_hash="h"
    )
    dumped = manifest.model_dump_json()
    reloaded = RunManifest.model_validate_json(dumped)
    assert reloaded == manifest

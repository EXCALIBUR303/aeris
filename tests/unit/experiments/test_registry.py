from __future__ import annotations

import sqlite3
from pathlib import Path

from aeris.core.types import RunStatus, Tier
from aeris.experiments.manifest import new_manifest, write_manifest
from aeris.experiments.registry import rebuild_registry, scan_runs


def _make_run(runs_dir: Path, run_id: str, *, status: RunStatus = RunStatus.COMPLETED) -> None:
    run_dir = runs_dir / run_id
    run_dir.mkdir(parents=True)
    manifest = new_manifest(
        run_id=run_id,
        task="waypoint_mission",
        method="scripted",
        tier=Tier.HIGH_FIDELITY,
        config_hash="hash1",
    )
    manifest.status = status
    write_manifest(manifest, run_dir)


def test_scan_runs_empty_directory_returns_empty(tmp_path: Path) -> None:
    assert scan_runs(tmp_path / "nonexistent") == []


def test_scan_runs_finds_every_run_with_a_manifest(tmp_path: Path) -> None:
    runs_dir = tmp_path / "runs"
    _make_run(runs_dir, "run_a")
    _make_run(runs_dir, "run_b")
    manifests = scan_runs(runs_dir)
    assert {m.run_id for m in manifests} == {"run_a", "run_b"}


def test_scan_runs_skips_a_directory_with_no_manifest(tmp_path: Path) -> None:
    runs_dir = tmp_path / "runs"
    _make_run(runs_dir, "run_a")
    (runs_dir / "not_a_run").mkdir(parents=True)
    manifests = scan_runs(runs_dir)
    assert {m.run_id for m in manifests} == {"run_a"}


def test_scan_runs_skips_a_corrupt_manifest(tmp_path: Path) -> None:
    runs_dir = tmp_path / "runs"
    _make_run(runs_dir, "run_a")
    bad_dir = runs_dir / "run_bad"
    bad_dir.mkdir(parents=True)
    (bad_dir / "manifest.json").write_text("{not valid json")
    manifests = scan_runs(runs_dir)
    assert {m.run_id for m in manifests} == {"run_a"}


def test_rebuild_registry_writes_a_queryable_sqlite_db(tmp_path: Path) -> None:
    runs_dir = tmp_path / "runs"
    _make_run(runs_dir, "run_a", status=RunStatus.COMPLETED)
    _make_run(runs_dir, "run_b", status=RunStatus.FAILED)
    db_path = tmp_path / "index.sqlite"

    count = rebuild_registry(runs_dir, db_path)
    assert count == 2

    conn = sqlite3.connect(db_path)
    try:
        rows = conn.execute("SELECT run_id, status FROM runs ORDER BY run_id").fetchall()
    finally:
        conn.close()
    assert rows == [("run_a", "completed"), ("run_b", "failed")]


def test_rebuild_registry_is_idempotent_and_reflects_filesystem_changes(tmp_path: Path) -> None:
    runs_dir = tmp_path / "runs"
    _make_run(runs_dir, "run_a")
    db_path = tmp_path / "index.sqlite"

    assert rebuild_registry(runs_dir, db_path) == 1
    _make_run(runs_dir, "run_b")
    assert rebuild_registry(runs_dir, db_path) == 2  # filesystem is the source of truth

    conn = sqlite3.connect(db_path)
    try:
        run_ids = {row[0] for row in conn.execute("SELECT run_id FROM runs")}
    finally:
        conn.close()
    assert run_ids == {"run_a", "run_b"}

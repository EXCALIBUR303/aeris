"""Run registry (spec §38.1): ``results/index.sqlite``, rebuildable by
scanning run directories -- the filesystem is the source of truth; this
is a queryable cache of it, never the other way around.
"""

from __future__ import annotations

import sqlite3
from pathlib import Path

from aeris.experiments.manifest import RunManifest, read_manifest

_SCHEMA = """
CREATE TABLE runs (
    run_id TEXT PRIMARY KEY,
    experiment_id TEXT,
    timestamp_utc TEXT,
    git_sha TEXT,
    git_dirty INTEGER,
    config_hash TEXT,
    task TEXT,
    method TEXT,
    tier TEXT,
    seed INTEGER,
    status TEXT,
    status_reason TEXT,
    is_reportable INTEGER
)
"""


def scan_runs(runs_dir: Path) -> list[RunManifest]:
    """Every run under ``runs_dir`` with a readable ``manifest.json``, in
    directory-listing order. A run whose manifest can't be parsed is
    skipped, not raised on -- the registry is best-effort over whatever
    the filesystem actually has."""
    if not runs_dir.is_dir():
        return []
    manifests = []
    for run_dir in sorted(p for p in runs_dir.iterdir() if p.is_dir()):
        try:
            manifests.append(read_manifest(run_dir))
        except Exception:
            continue
    return manifests


def rebuild_registry(runs_dir: Path, db_path: Path) -> int:
    """Drop and rebuild ``db_path`` from ``runs_dir``. Returns the number
    of runs indexed."""
    manifests = scan_runs(runs_dir)
    db_path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(db_path)
    try:
        conn.execute("DROP TABLE IF EXISTS runs")
        conn.execute(_SCHEMA)
        conn.executemany(
            "INSERT INTO runs VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
            [
                (
                    m.run_id,
                    m.experiment_id,
                    m.timestamp_utc,
                    m.git_sha,
                    int(m.git_dirty),
                    m.config_hash,
                    m.task,
                    m.method,
                    m.tier.value,
                    m.seed,
                    m.status.value,
                    m.status_reason,
                    int(m.is_reportable),
                )
                for m in manifests
            ],
        )
        conn.commit()
    finally:
        conn.close()
    return len(manifests)

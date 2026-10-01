"""Run manifests (spec §38.2): the mandatory record of what a run *was* —
captured at run start, finalized (status + reason) at run end, and the
single source of provenance every later comparison depends on.
"""

from __future__ import annotations

import contextlib
import os
import platform
import subprocess
import sys
from datetime import UTC, datetime
from pathlib import Path

from pydantic import BaseModel, Field

from aeris.core.errors import ConfigNotFoundError
from aeris.core.types import RunStatus, Tier

_REPO_ROOT = Path(__file__).resolve().parents[2]


def git_info() -> tuple[str | None, bool]:
    """``(git_sha, dirty)`` for the current checkout, or ``(None, False)``
    if this isn't a git checkout at all (never raises)."""
    try:
        sha_proc = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=_REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=5,
        )
        if sha_proc.returncode != 0:
            return None, False
        status_proc = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=_REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=5,
        )
        dirty = bool(status_proc.stdout.strip())
        return sha_proc.stdout.strip(), dirty
    except (OSError, subprocess.SubprocessError):
        return None, False


def aeris_version() -> str:
    try:
        from importlib.metadata import PackageNotFoundError, version

        return version("aeris")
    except PackageNotFoundError:
        return "unknown"


def software_versions() -> dict[str, str]:
    """Spec §38.2: "Software versions (Python, torch, numpy, mavsdk, PX4
    tag/SHA, Gazebo version, QGC if connected, macOS)". Only the ones
    meaningful with what exists as of Phase 7 -- torch/numpy/mavsdk/Gazebo
    versions get added here by whichever later phase first imports them,
    rather than hardcoding a placeholder now."""
    versions = {"python": sys.version.split()[0], "os": platform.platform()}
    try:
        import pymavlink

        versions["pymavlink"] = getattr(pymavlink, "__version__", "unknown")
    except ImportError:
        pass
    from importlib.metadata import PackageNotFoundError, version

    for pkg in ("torch", "numpy", "gymnasium"):  # Phase 14: first phase to train with them
        with contextlib.suppress(PackageNotFoundError):
            versions[pkg] = version(pkg)
    return versions


def hardware_snapshot() -> dict[str, str]:
    return {
        "machine": platform.machine(),
        "processor": platform.processor() or platform.machine(),
        "cpu_count": str(os.cpu_count() or 0),
    }


def make_run_id(name: str, config_hash: str, *, now: datetime | None = None) -> str:
    """Spec §38.1: ``results/runs/<YYYYMMDD-HHMMSS>_<name>_<cfghash8>/``."""
    now = now or datetime.now(UTC)
    return f"{now:%Y%m%d-%H%M%S}_{name}_{config_hash[:8]}"


class RunManifest(BaseModel):
    """Spec §38.2's mandatory fields, adapted to what Phase 7 can actually
    populate honestly -- fields spec lists that need infrastructure from a
    later phase (torch/mps device, checkpoints, PX4/Gazebo version capture)
    stay optional/empty rather than being filled with a placeholder."""

    run_id: str
    experiment_id: str | None = None
    timestamp_utc: str
    git_sha: str | None
    git_dirty: bool
    aeris_version: str
    config_hash: str

    task: str
    method: str
    tier: Tier
    seed: int | None = None
    device: str | None = None

    software_versions: dict[str, str] = Field(default_factory=dict)
    hardware_snapshot: dict[str, str] = Field(default_factory=dict)

    checkpoint_path: str | None = None
    checkpoint_hash: str | None = None

    # Provenance flags (spec §38.2 / §17.4).
    oracle_baseline: bool = False
    critic_privileged: bool = False
    gcs_connected: bool = False

    status: RunStatus = RunStatus.FAILED  # pessimistic default; set explicitly on success
    status_reason: str = ""

    @property
    def is_reportable(self) -> bool:
        """Spec §38.2: "dirty runs are tagged and excluded from final
        reporting." A run must also have actually completed."""
        return not self.git_dirty and self.status == RunStatus.COMPLETED


def new_manifest(
    *,
    run_id: str,
    task: str,
    method: str,
    tier: Tier,
    config_hash: str,
    experiment_id: str | None = None,
    seed: int | None = None,
) -> RunManifest:
    sha, dirty = git_info()
    return RunManifest(
        run_id=run_id,
        experiment_id=experiment_id,
        timestamp_utc=datetime.now(UTC).isoformat(),
        git_sha=sha,
        git_dirty=dirty,
        aeris_version=aeris_version(),
        config_hash=config_hash,
        task=task,
        method=method,
        tier=tier,
        seed=seed,
        software_versions=software_versions(),
        hardware_snapshot=hardware_snapshot(),
    )


def write_manifest(manifest: RunManifest, run_dir: Path) -> Path:
    path = run_dir / "manifest.json"
    path.write_text(manifest.model_dump_json(indent=2))
    return path


def read_manifest(run_dir: Path) -> RunManifest:
    path = run_dir / "manifest.json"
    if not path.is_file():
        raise ConfigNotFoundError(f"no manifest.json in {run_dir}")
    return RunManifest.model_validate_json(path.read_text())

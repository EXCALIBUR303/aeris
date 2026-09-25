"""Resolving where the external PX4 checkout lives, and its build layout.

Spec §8.3 (ADR-0002): PX4 is an external, pinned dependency — "Location
comes from ``AERIS_PX4_DIR`` (default ``~/aeris-deps/PX4-Autopilot``), set
in ``configs/local.yaml`` (gitignored) or the environment."
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

from aeris.core.config import load_yaml
from aeris.core.errors import ConfigNotFoundError, Px4NotFoundError

_ENV_VAR = "AERIS_PX4_DIR"
_DEFAULT_PX4_DIR = "~/aeris-deps/PX4-Autopilot"
_DEFAULT_BUILD_CONFIG = "px4_sitl_default"


def _repo_root() -> Path:
    # aeris/simulation/px4_paths.py -> aeris/simulation -> aeris -> repo root
    return Path(__file__).resolve().parents[2]


def resolve_px4_dir(*, local_config_path: Path | None = None) -> Path:
    """Resolve the PX4-Autopilot checkout directory.

    Precedence (highest first), matching spec §8.3:

    1. the ``AERIS_PX4_DIR`` environment variable
    2. ``px4_dir:`` in ``configs/local.yaml`` (gitignored; see
       ``configs/local.example.yaml``)
    3. the documented default, ``~/aeris-deps/PX4-Autopilot``

    This does **not** check that the directory exists — callers that need
    a built PX4 should use :func:`px4_binary_dir` or :func:`px4_binary_path`,
    which do.
    """
    env_value = os.environ.get(_ENV_VAR)
    if env_value:
        return Path(env_value).expanduser().resolve()

    local_yaml = local_config_path or (_repo_root() / "configs" / "local.yaml")
    try:
        local_config = load_yaml(local_yaml)
    except ConfigNotFoundError:
        local_config = {}
    configured = local_config.get("px4_dir")
    if configured:
        return Path(str(configured)).expanduser().resolve()

    return Path(_DEFAULT_PX4_DIR).expanduser().resolve()


@dataclass(frozen=True, slots=True)
class Px4Layout:
    """Resolved, existence-checked paths into a built PX4 checkout.

    ``build_config`` is PX4's own CMake configuration name — ``px4_sitl_default``
    for every profile AERIS uses (spec never needs ``px4_sitl_test`` etc.).
    """

    px4_dir: Path
    build_config: str

    @property
    def binary_dir(self) -> Path:
        """``<px4_dir>/build/<build_config>`` — PX4's ``PX4_BINARY_DIR``."""
        return self.px4_dir / "build" / self.build_config

    @property
    def binary_path(self) -> Path:
        """The built ``px4`` executable."""
        return self.binary_dir / "bin" / "px4"

    @property
    def rootfs_dir(self) -> Path:
        """PX4's own default working directory (``<binary_dir>/rootfs``).

        PX4's ``main.cpp`` ``chdir()``s here by default regardless of the
        invoking process's cwd (verified by reading
        ``platforms/posix/src/px4/common/main.cpp`` in Phase 3) — AERIS
        relies on this rather than passing an explicit ``-w``, matching the
        exact invocation already validated in Phase 1.
        """
        return self.binary_dir / "rootfs"

    @property
    def log_dir(self) -> Path:
        """Where PX4 writes its own ULog files (``rootfs/log``)."""
        return self.rootfs_dir / "log"

    @property
    def worlds_dir(self) -> Path:
        return self.px4_dir / "Tools" / "simulation" / "gz" / "worlds"

    @property
    def models_dir(self) -> Path:
        return self.px4_dir / "Tools" / "simulation" / "gz" / "models"

    @property
    def gz_plugins_dir(self) -> Path:
        """Built PX4 Gazebo plugins (``OpticalFlowSystem``, ``GstCameraSystem``, ...)."""
        return self.binary_dir / "src" / "modules" / "simulation" / "gz_plugins"

    @property
    def gz_server_config(self) -> Path:
        return self.px4_dir / "src" / "modules" / "simulation" / "gz_bridge" / "server.config"

    def world_sdf(self, world: str) -> Path:
        return self.worlds_dir / f"{world}.sdf"

    def ensure_built(self) -> None:
        """Raise :class:`Px4NotFoundError` if this checkout hasn't been built.

        AERIS never builds PX4 automatically (spec §8.3) — building is a
        one-time manual step per ``docs/mac-setup.md``.
        """
        if not self.px4_dir.is_dir():
            raise Px4NotFoundError(
                f"PX4 checkout not found at {self.px4_dir}. "
                f"See docs/mac-setup.md, or set AERIS_PX4_DIR / configs/local.yaml "
                f"if it lives elsewhere."
            )
        if not self.binary_path.is_file():
            raise Px4NotFoundError(
                f"PX4 is not built at {self.binary_path}. "
                f"Build it first (e.g. `make px4_sitl gz_x500` from {self.px4_dir}) — "
                f"AERIS never builds PX4 automatically (spec §8.3)."
            )
        for path, label in (
            (self.worlds_dir, "worlds directory"),
            (self.models_dir, "models directory"),
            (self.gz_server_config, "Gazebo server config"),
        ):
            if not path.exists():
                raise Px4NotFoundError(f"expected PX4 {label} not found: {path}")


def resolve_px4_layout(
    *, build_config: str = _DEFAULT_BUILD_CONFIG, local_config_path: Path | None = None
) -> Px4Layout:
    """Resolve :class:`Px4Layout` from configuration (spec §8.3 precedence)."""
    return Px4Layout(
        px4_dir=resolve_px4_dir(local_config_path=local_config_path), build_config=build_config
    )

from pathlib import Path

import pytest

from aeris.core.errors import Px4NotFoundError
from aeris.simulation.px4_paths import Px4Layout, resolve_px4_dir, resolve_px4_layout


def write_yaml(path: Path, text: str) -> Path:
    path.write_text(text)
    return path


# --- resolve_px4_dir precedence (spec §8.3) -----------------------------------


def test_env_var_takes_precedence_over_local_yaml(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    write_yaml(tmp_path / "local.yaml", "px4_dir: /from/yaml\n")
    monkeypatch.setenv("AERIS_PX4_DIR", "/from/env")
    resolved = resolve_px4_dir(local_config_path=tmp_path / "local.yaml")
    assert resolved == Path("/from/env").resolve()


def test_local_yaml_used_when_no_env_var(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    monkeypatch.delenv("AERIS_PX4_DIR", raising=False)
    write_yaml(tmp_path / "local.yaml", "px4_dir: /from/yaml\n")
    resolved = resolve_px4_dir(local_config_path=tmp_path / "local.yaml")
    assert resolved == Path("/from/yaml").resolve()


def test_default_used_when_nothing_configured(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    monkeypatch.delenv("AERIS_PX4_DIR", raising=False)
    resolved = resolve_px4_dir(local_config_path=tmp_path / "does_not_exist.yaml")
    assert resolved == Path("~/aeris-deps/PX4-Autopilot").expanduser().resolve()


def test_local_yaml_without_px4_dir_key_falls_back_to_default(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    monkeypatch.delenv("AERIS_PX4_DIR", raising=False)
    write_yaml(tmp_path / "local.yaml", "torch_device: cpu\n")
    resolved = resolve_px4_dir(local_config_path=tmp_path / "local.yaml")
    assert resolved == Path("~/aeris-deps/PX4-Autopilot").expanduser().resolve()


def test_expands_tilde(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    monkeypatch.setenv("AERIS_PX4_DIR", "~/somewhere")
    resolved = resolve_px4_dir(local_config_path=tmp_path / "does_not_exist.yaml")
    assert resolved == Path("~/somewhere").expanduser().resolve()
    assert "~" not in str(resolved)


# --- Px4Layout paths -----------------------------------------------------------


def test_layout_paths_are_computed_correctly(tmp_path: Path):
    layout = Px4Layout(px4_dir=tmp_path, build_config="px4_sitl_default")
    assert layout.binary_dir == tmp_path / "build" / "px4_sitl_default"
    assert layout.binary_path == tmp_path / "build" / "px4_sitl_default" / "bin" / "px4"
    assert layout.rootfs_dir == layout.binary_dir / "rootfs"
    assert layout.log_dir == layout.rootfs_dir / "log"
    assert layout.worlds_dir == tmp_path / "Tools" / "simulation" / "gz" / "worlds"
    assert layout.models_dir == tmp_path / "Tools" / "simulation" / "gz" / "models"
    assert layout.world_sdf("default") == layout.worlds_dir / "default.sdf"


def test_layout_is_frozen(tmp_path: Path):
    layout = Px4Layout(px4_dir=tmp_path, build_config="px4_sitl_default")
    with pytest.raises(AttributeError):
        layout.px4_dir = tmp_path / "other"  # type: ignore[misc]


# --- ensure_built (against a fake, unbuilt checkout) --------------------------


def test_ensure_built_raises_when_px4_dir_missing(tmp_path: Path):
    layout = Px4Layout(px4_dir=tmp_path / "nonexistent", build_config="px4_sitl_default")
    with pytest.raises(Px4NotFoundError, match="checkout not found"):
        layout.ensure_built()


def test_ensure_built_raises_when_binary_missing(tmp_path: Path):
    (tmp_path / "build" / "px4_sitl_default" / "bin").mkdir(parents=True)
    layout = Px4Layout(px4_dir=tmp_path, build_config="px4_sitl_default")
    with pytest.raises(Px4NotFoundError, match="not built"):
        layout.ensure_built()


def test_ensure_built_raises_when_worlds_missing(tmp_path: Path):
    binary_dir = tmp_path / "build" / "px4_sitl_default" / "bin"
    binary_dir.mkdir(parents=True)
    (binary_dir / "px4").write_bytes(b"")
    layout = Px4Layout(px4_dir=tmp_path, build_config="px4_sitl_default")
    with pytest.raises(Px4NotFoundError, match="worlds directory"):
        layout.ensure_built()


# --- against the real, Phase-1-built checkout ---------------------------------


def test_ensure_built_passes_on_the_real_checkout():
    # This machine has PX4 built (Phase 1) — proves the happy path works
    # against the real thing, not just synthetic fixtures. Skips (rather
    # than fails) where that isn't true, e.g. hosted CI (spec §44.2 — CI
    # never has PX4/Gazebo installed).
    layout = resolve_px4_layout()
    if not layout.binary_path.is_file():
        pytest.skip(f"PX4 not built at {layout.binary_path} on this machine")
    layout.ensure_built()  # must not raise

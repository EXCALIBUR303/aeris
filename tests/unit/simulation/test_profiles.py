from pathlib import Path

import pytest
from pydantic import ValidationError

from aeris.core.errors import ConfigCompositionError
from aeris.simulation.launcher.profiles import SimulationProfile, load_profile

REPO_ROOT = Path(__file__).resolve().parents[3]
PROFILES_DIR = REPO_ROOT / "configs" / "simulation"


def write_yaml(path: Path, text: str) -> Path:
    path.write_text(text)
    return path


# --- the real, committed profiles -------------------------------------------


@pytest.mark.parametrize(
    "filename,expected_model",
    [
        ("headless_x500.yaml", "gz_x500"),
        ("headless_x500_depth.yaml", "gz_x500_depth"),
        ("headless_x500_lidar_2d.yaml", "gz_x500_lidar_2d"),
        ("gui_x500.yaml", "gz_x500"),
    ],
)
def test_real_profiles_load_and_validate(filename: str, expected_model: str):
    profile = load_profile(PROFILES_DIR / filename)
    assert profile.model == expected_model
    assert profile.world == "default"


def test_gui_profile_is_not_headless():
    profile = load_profile(PROFILES_DIR / "gui_x500.yaml")
    assert profile.headless is False


def test_headless_profiles_are_headless():
    profile = load_profile(PROFILES_DIR / "headless_x500.yaml")
    assert profile.headless is True


def test_profile_name_defaults_to_filename_stem():
    profile = load_profile(PROFILES_DIR / "headless_x500.yaml")
    assert profile.name == "headless_x500"


# --- validation --------------------------------------------------------------


def test_model_must_start_with_gz_prefix():
    with pytest.raises(ValidationError):
        SimulationProfile(name="bad", model="x500")


def test_speed_factor_must_be_positive():
    with pytest.raises(ValidationError):
        SimulationProfile(name="bad", model="gz_x500", speed_factor=0.0)
    with pytest.raises(ValidationError):
        SimulationProfile(name="bad", model="gz_x500", speed_factor=-1.0)


def test_instance_must_be_nonnegative():
    with pytest.raises(ValidationError):
        SimulationProfile(name="bad", model="gz_x500", instance=-1)


def test_gz_verbose_bounds():
    with pytest.raises(ValidationError):
        SimulationProfile(name="bad", model="gz_x500", gz_verbose=5)
    with pytest.raises(ValidationError):
        SimulationProfile(name="bad", model="gz_x500", gz_verbose=-1)


def test_spawn_pose_accepts_six_floats():
    profile = SimulationProfile(
        name="posed", model="gz_x500", spawn_pose=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
    )
    assert profile.spawn_pose == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)


def test_spawn_pose_rejects_wrong_length():
    with pytest.raises(ValidationError):
        SimulationProfile(name="bad", model="gz_x500", spawn_pose=(1.0, 2.0))


def test_defaults_match_spec_intent():
    profile = SimulationProfile(name="minimal", model="gz_x500")
    assert profile.world == "default"
    assert profile.instance == 0
    assert profile.spawn_pose is None
    assert profile.speed_factor == 1.0
    assert profile.standalone is True
    assert profile.headless is True
    assert profile.render_engine is None


# --- error propagation --------------------------------------------------------


def test_load_profile_missing_model_raises_config_composition_error(tmp_path: Path):
    p = write_yaml(tmp_path / "broken.yaml", "world: default\n")
    with pytest.raises(ConfigCompositionError):
        load_profile(p)


def test_load_profile_composes_extends(tmp_path: Path):
    write_yaml(tmp_path / "base.yaml", "world: default\nheadless: true\n")
    child = write_yaml(tmp_path / "child.yaml", "extends: base.yaml\nmodel: gz_x500_depth\n")
    profile = load_profile(child)
    assert profile.model == "gz_x500_depth"
    assert profile.world == "default"
    assert profile.headless is True

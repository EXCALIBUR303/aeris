from pathlib import Path

import pytest

from aeris.core.errors import ConfigCompositionError
from aeris.simulation.launcher.params import parse_params_file

REPO_ROOT = Path(__file__).resolve().parents[3]
SITL_BASE_PARAMS = REPO_ROOT / "configs" / "vehicle" / "px4_params" / "sitl_base.params"


def write(path: Path, text: str) -> Path:
    path.write_text(text)
    return path


def test_real_sitl_base_params_parses():
    params = parse_params_file(SITL_BASE_PARAMS)
    assert params["COM_RC_IN_MODE"] == 4.0


def test_parses_simple_name_value(tmp_path: Path):
    p = write(tmp_path / "p.params", "MPC_XY_VEL_MAX 5.0\nCOM_RC_IN_MODE 4\n")
    assert parse_params_file(p) == {"MPC_XY_VEL_MAX": 5.0, "COM_RC_IN_MODE": 4.0}


def test_ignores_comments_and_blank_lines(tmp_path: Path):
    p = write(
        tmp_path / "p.params",
        "# a full-line comment\n\nMPC_XY_VEL_MAX 5.0  # inline comment\n\n",
    )
    assert parse_params_file(p) == {"MPC_XY_VEL_MAX": 5.0}


def test_missing_file_raises(tmp_path: Path):
    with pytest.raises(ConfigCompositionError):
        parse_params_file(tmp_path / "nope.params")


def test_malformed_line_raises(tmp_path: Path):
    p = write(tmp_path / "p.params", "NOT_A_VALID_LINE\n")
    with pytest.raises(ConfigCompositionError):
        parse_params_file(p)


def test_too_many_fields_raises(tmp_path: Path):
    p = write(tmp_path / "p.params", "NAME 1 2\n")
    with pytest.raises(ConfigCompositionError):
        parse_params_file(p)


def test_non_numeric_value_raises(tmp_path: Path):
    p = write(tmp_path / "p.params", "NAME not_a_number\n")
    with pytest.raises(ConfigCompositionError):
        parse_params_file(p)


def test_name_over_16_chars_raises(tmp_path: Path):
    p = write(tmp_path / "p.params", "THIS_NAME_IS_WAY_TOO_LONG 1.0\n")
    with pytest.raises(ConfigCompositionError):
        parse_params_file(p)


def test_name_exactly_16_chars_is_allowed(tmp_path: Path):
    name = "A" * 16
    p = write(tmp_path / "p.params", f"{name} 1.0\n")
    assert parse_params_file(p) == {name: 1.0}

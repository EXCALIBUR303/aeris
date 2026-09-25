from pathlib import Path

import pytest

from aeris.core.errors import ConfigCompositionError
from aeris.simulation.launcher.params import (
    _MAV_PARAM_TYPE_INT32,
    _MAV_PARAM_TYPE_REAL32,
    _MAV_PARAM_TYPE_UINT32,
    _decode_param_value,
    _encode_param_value,
    parse_params_file,
)

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


# --- param value wire encoding (spec §44.3: regression tests for the ------------------
# --- Phase 5 finding that PX4 requires an exact MAV_PARAM_TYPE match) -----------------


def test_int32_encode_matches_observed_wire_value():
    # Phase 5 live finding: PX4 echoed int value 1 (COM_RC_IN_MODE's default
    # at the time) back as the float 1.401298464324817e-45 -- exactly
    # struct.pack("<i", 1) reinterpreted as "<f", not a numeric cast.
    assert _encode_param_value(1.0, _MAV_PARAM_TYPE_INT32) == pytest.approx(
        1.401298464324817e-45, abs=1e-50
    )


def test_int32_round_trip_encode_decode():
    for value in [-1000, -1, 0, 1, 3, 4, 60, 1000, 2**20]:
        wire = _encode_param_value(float(value), _MAV_PARAM_TYPE_INT32)
        assert _decode_param_value(wire, _MAV_PARAM_TYPE_INT32) == float(value)


def test_uint32_round_trip_encode_decode():
    for value in [0, 1, 4, 1000, 2**31]:
        wire = _encode_param_value(float(value), _MAV_PARAM_TYPE_UINT32)
        assert _decode_param_value(wire, _MAV_PARAM_TYPE_UINT32) == float(value)


def test_real32_is_passed_through_unchanged():
    for value in [0.0, 1.5, -3.25, 60.0]:
        wire = _encode_param_value(value, _MAV_PARAM_TYPE_REAL32)
        assert wire == value
        assert _decode_param_value(wire, _MAV_PARAM_TYPE_REAL32) == value


def test_int32_and_real32_encodings_are_not_confusable():
    # The exact bug this guards against: encoding an int-typed param as
    # REAL32 (the old, wrong, hardcoded default) produces a wildly
    # different wire value than the correct INT32 encoding.
    int_wire = _encode_param_value(4.0, _MAV_PARAM_TYPE_INT32)
    real_wire = _encode_param_value(4.0, _MAV_PARAM_TYPE_REAL32)
    assert int_wire != real_wire
    assert abs(int_wire) < 1e-40  # a denormalized-float bit pattern, not a real "4.0"

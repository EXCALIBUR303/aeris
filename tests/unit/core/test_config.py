from pathlib import Path

import pytest

from aeris.core.config import (
    apply_overrides,
    canonical_json,
    compose_config,
    config_hash,
    config_hash8,
    deep_merge,
    load_resolved_config,
    load_yaml,
    parse_override,
)
from aeris.core.errors import ConfigCompositionError, ConfigNotFoundError


def write_yaml(path: Path, text: str) -> Path:
    path.write_text(text)
    return path


# --- load_yaml ---------------------------------------------------------------


def test_load_yaml_missing_file_raises(tmp_path: Path):
    with pytest.raises(ConfigNotFoundError):
        load_yaml(tmp_path / "nope.yaml")


def test_load_yaml_empty_file_is_empty_dict(tmp_path: Path):
    p = write_yaml(tmp_path / "empty.yaml", "")
    assert load_yaml(p) == {}


def test_load_yaml_non_mapping_top_level_raises(tmp_path: Path):
    p = write_yaml(tmp_path / "list.yaml", "- 1\n- 2\n")
    with pytest.raises(ConfigCompositionError):
        load_yaml(p)


# --- deep_merge ----------------------------------------------------------------


def test_deep_merge_nested_dicts():
    base = {"a": {"x": 1, "y": 2}, "b": 1}
    override = {"a": {"y": 20, "z": 3}}
    result = deep_merge(base, override)
    assert result == {"a": {"x": 1, "y": 20, "z": 3}, "b": 1}


def test_deep_merge_lists_are_replaced_not_concatenated():
    base = {"sensors": ["imu", "gps"]}
    override = {"sensors": ["depth"]}
    assert deep_merge(base, override) == {"sensors": ["depth"]}


def test_deep_merge_does_not_mutate_inputs():
    base = {"a": {"x": 1}}
    override = {"a": {"y": 2}}
    deep_merge(base, override)
    assert base == {"a": {"x": 1}}
    assert override == {"a": {"y": 2}}


# --- compose_config (extends:) -------------------------------------------------


def test_compose_config_no_extends_returns_own_keys(tmp_path: Path):
    p = write_yaml(tmp_path / "solo.yaml", "seed: 1\nname: solo\n")
    assert compose_config(p) == {"seed": 1, "name": "solo"}


def test_compose_config_single_extends_child_overrides_parent(tmp_path: Path):
    write_yaml(tmp_path / "base.yaml", "seed: 1\nsteps: 100\n")
    child = write_yaml(tmp_path / "child.yaml", "extends: base.yaml\nsteps: 200\n")
    assert compose_config(child) == {"seed": 1, "steps": 200}


def test_compose_config_list_of_extends_merges_in_order(tmp_path: Path):
    write_yaml(tmp_path / "a.yaml", "x: 1\ny: 1\n")
    write_yaml(tmp_path / "b.yaml", "y: 2\nz: 2\n")
    child = write_yaml(tmp_path / "child.yaml", "extends: [a.yaml, b.yaml]\n")
    # b.yaml is listed after a.yaml, so it wins on the shared key 'y'.
    assert compose_config(child) == {"x": 1, "y": 2, "z": 2}


def test_compose_config_multi_level_chain(tmp_path: Path):
    write_yaml(tmp_path / "grandparent.yaml", "tier: smoke\nsteps: 10\n")
    write_yaml(tmp_path / "parent.yaml", "extends: grandparent.yaml\nsteps: 50\nmethod: frontier\n")
    child = write_yaml(tmp_path / "child.yaml", "extends: parent.yaml\nmethod: learned\n")
    assert compose_config(child) == {"tier": "smoke", "steps": 50, "method": "learned"}


def test_compose_config_strips_extends_key_from_result(tmp_path: Path):
    write_yaml(tmp_path / "base.yaml", "a: 1\n")
    child = write_yaml(tmp_path / "child.yaml", "extends: base.yaml\nb: 2\n")
    assert "extends" not in compose_config(child)


def test_compose_config_cycle_raises(tmp_path: Path):
    write_yaml(tmp_path / "a.yaml", "extends: b.yaml\n")
    write_yaml(tmp_path / "b.yaml", "extends: a.yaml\n")
    with pytest.raises(ConfigCompositionError):
        compose_config(tmp_path / "a.yaml")


def test_compose_config_missing_extends_target_raises(tmp_path: Path):
    child = write_yaml(tmp_path / "child.yaml", "extends: missing.yaml\n")
    with pytest.raises(ConfigNotFoundError):
        compose_config(child)


def test_compose_config_invalid_extends_type_raises(tmp_path: Path):
    child = write_yaml(tmp_path / "child.yaml", "extends: 5\n")
    with pytest.raises(ConfigCompositionError):
        compose_config(child)


# --- overrides -----------------------------------------------------------------


def test_parse_override_simple():
    keys, value = parse_override("learning.lr=0.0003")
    assert keys == ["learning", "lr"]
    assert value == pytest.approx(0.0003)


def test_parse_override_types():
    assert parse_override("a=true")[1] is True
    assert parse_override("a=null")[1] is None
    assert parse_override("a=42")[1] == 42
    assert parse_override("a=hello")[1] == "hello"


def test_parse_override_no_equals_raises():
    with pytest.raises(ConfigCompositionError):
        parse_override("no-equals-sign")


def test_parse_override_empty_key_raises():
    with pytest.raises(ConfigCompositionError):
        parse_override("=value")


def test_apply_overrides_sets_nested_key():
    result = apply_overrides({"a": {"b": 1}}, ["a.b=2", "a.c=3"])
    assert result == {"a": {"b": 2, "c": 3}}


def test_apply_overrides_creates_missing_intermediate_dicts():
    result = apply_overrides({}, ["x.y.z=1"])
    assert result == {"x": {"y": {"z": 1}}}


def test_apply_overrides_does_not_mutate_input():
    original = {"a": {"b": 1}}
    apply_overrides(original, ["a.b=2"])
    assert original == {"a": {"b": 1}}


# --- hashing ---------------------------------------------------------------------


def test_config_hash_is_deterministic_regardless_of_key_order():
    c1 = {"a": 1, "b": 2}
    c2 = {"b": 2, "a": 1}
    assert config_hash(c1) == config_hash(c2)


def test_config_hash_changes_with_content():
    assert config_hash({"a": 1}) != config_hash({"a": 2})


def test_config_hash8_is_prefix_of_full_hash():
    c = {"a": 1}
    assert config_hash(c).startswith(config_hash8(c))
    assert len(config_hash8(c)) == 8


def test_canonical_json_is_compact_and_sorted():
    s = canonical_json({"b": 1, "a": 2})
    assert s == '{"a":2,"b":1}'


# --- load_resolved_config (end to end) --------------------------------------------


def test_load_resolved_config_composes_and_overrides(tmp_path: Path):
    write_yaml(tmp_path / "base.yaml", "steps: 100\nseed: 1\n")
    child = write_yaml(tmp_path / "child.yaml", "extends: base.yaml\nsteps: 200\n")
    resolved, h = load_resolved_config(child, overrides=["seed=99"])
    assert resolved == {"steps": 200, "seed": 99}
    assert h == config_hash(resolved)
    assert len(h) == 64  # sha256 hex digest length

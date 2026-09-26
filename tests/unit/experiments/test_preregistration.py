from __future__ import annotations

from pathlib import Path

import pytest

from aeris.core.errors import PreregistrationRequiredError
from aeris.experiments.preregistration import preregistration_path, require_preregistration


def test_non_test_split_needs_no_preregistration(tmp_path: Path) -> None:
    result = require_preregistration("some_exp", "val", preregistration_dir=tmp_path)
    assert result is None
    result = require_preregistration("some_exp", "train", preregistration_dir=tmp_path)
    assert result is None


def test_test_split_refuses_without_a_committed_file(tmp_path: Path) -> None:
    with pytest.raises(PreregistrationRequiredError, match="no committed"):
        require_preregistration("some_exp", "test", preregistration_dir=tmp_path)


def test_test_split_succeeds_with_a_committed_file_and_returns_its_hash(tmp_path: Path) -> None:
    path = preregistration_path("some_exp", preregistration_dir=tmp_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("# Pre-registration\n\nHypothesis: ...\n")

    result = require_preregistration("some_exp", "test", preregistration_dir=tmp_path)

    assert result is not None
    assert len(result) == 64  # sha256 hex digest


def test_hash_changes_if_the_preregistration_file_changes(tmp_path: Path) -> None:
    path = preregistration_path("some_exp", preregistration_dir=tmp_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("version 1")
    hash1 = require_preregistration("some_exp", "test", preregistration_dir=tmp_path)

    path.write_text("version 2")
    hash2 = require_preregistration("some_exp", "test", preregistration_dir=tmp_path)

    assert hash1 != hash2


def test_different_experiments_look_for_different_files(tmp_path: Path) -> None:
    path = preregistration_path("exp_a", preregistration_dir=tmp_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("exp_a's preregistration")

    require_preregistration("exp_a", "test", preregistration_dir=tmp_path)  # succeeds
    with pytest.raises(PreregistrationRequiredError):
        require_preregistration("exp_b", "test", preregistration_dir=tmp_path)  # no file for exp_b

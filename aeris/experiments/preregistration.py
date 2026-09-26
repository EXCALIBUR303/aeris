"""Pre-registration guard (spec §38.5): "The runner refuses test-split
evaluation without a committed pre-registration whose hash is stored in
the manifest."
"""

from __future__ import annotations

import hashlib
from pathlib import Path

from aeris.core.errors import PreregistrationRequiredError

TEST_SPLIT = "test"


def preregistration_path(exp_id: str, *, preregistration_dir: Path) -> Path:
    return preregistration_dir / f"{exp_id}.md"


def require_preregistration(exp_id: str, split: str, *, preregistration_dir: Path) -> str | None:
    """Returns the committed pre-registration file's SHA-256 hash (to be
    stored in the run's manifest) if ``split`` needs one and it exists.

    Returns ``None`` for a non-test split (no pre-registration required).
    Raises :class:`PreregistrationRequiredError` for a test-split
    evaluation with no committed file (spec §38.5).
    """
    if split != TEST_SPLIT:
        return None
    path = preregistration_path(exp_id, preregistration_dir=preregistration_dir)
    if not path.is_file():
        raise PreregistrationRequiredError(
            f"test-split evaluation of {exp_id!r} refused: no committed "
            f"pre-registration at {path} (spec §38.5)"
        )
    return hashlib.sha256(path.read_bytes()).hexdigest()

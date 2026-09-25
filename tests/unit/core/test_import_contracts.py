"""Proves the import-linter contracts declared in pyproject.toml are enforced.

Spec §51 Phase 2 validation gate: "import contracts are enforced (a
deliberate violation in a test fixture is detected)."
"""

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIXTURE_DIR = REPO_ROOT / "tests" / "fixtures" / "import_contract_violation"


def _run_lint_imports(cwd: Path) -> subprocess.CompletedProcess[str]:
    # `import-linter` ships a `lint-imports` console script, not a
    # `python -m importlinter` entry point. Invoke it via the same
    # interpreter's bin directory so this works under `uv run pytest`
    # regardless of PATH.
    lint_imports = Path(sys.executable).with_name("lint-imports")
    return subprocess.run(
        [str(lint_imports)],
        cwd=cwd,
        capture_output=True,
        text=True,
        timeout=60,
    )


def test_real_aeris_contracts_pass():
    result = _run_lint_imports(REPO_ROOT)
    assert result.returncode == 0, (
        f"aeris's own import-linter contracts failed:\n{result.stdout}\n{result.stderr}"
    )


def test_fixture_violation_is_detected():
    result = _run_lint_imports(FIXTURE_DIR)
    assert result.returncode != 0, (
        "the deliberately-broken fixture package should fail import-linter, but it passed:\n"
        f"{result.stdout}\n{result.stderr}"
    )
    assert "broken" in result.stdout.lower()

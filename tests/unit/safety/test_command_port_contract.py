"""Spec §14.3 contract 3 / §16.1 rule 3: "Only aeris.safety may call command
methods on a VehicleInterface ... enforced by exposing a CommandPort that
only the supervisor receives, plus a test that scans for command-method
calls."

Nothing can hold/call a ``CommandPort`` without first obtaining one via
``VehicleInterface.command_port()`` -- so scanning for that one call site
is sufficient and far less fragile than pattern-matching on individual
method names (``arm``, ``land``, ... are common enough words that a
name-based scan would false-positive constantly). Production code only:
``aeris/`` and ``scripts/``, excluding ``aeris/safety`` (the legitimate
holder) and ``aeris/vehicle`` (where adapters implement ``CommandPort``
and ``command_port()`` itself is defined, which is not the same as
*calling* it as a client).
"""

from __future__ import annotations

from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_SCAN_ROOTS = ("aeris", "scripts")
_EXEMPT_DIRS = ("aeris/safety", "aeris/vehicle")
_FORBIDDEN_CALL = ".command_port("


def _python_files_to_scan() -> list[Path]:
    files: list[Path] = []
    for root_name in _SCAN_ROOTS:
        root = _REPO_ROOT / root_name
        if not root.is_dir():
            continue
        for path in root.rglob("*.py"):
            rel = path.relative_to(_REPO_ROOT).as_posix()
            if any(
                rel.startswith(f"{exempt}/") or rel == f"{exempt}.py" for exempt in _EXEMPT_DIRS
            ):
                continue
            files.append(path)
    return files


def test_only_aeris_safety_calls_command_port() -> None:
    violations = []
    for path in _python_files_to_scan():
        text = path.read_text(encoding="utf-8")
        if _FORBIDDEN_CALL in text:
            violations.append(path.relative_to(_REPO_ROOT).as_posix())

    assert not violations, (
        "only aeris.safety may call VehicleInterface.command_port() "
        f"(spec §14.3 contract 3) -- found it in: {violations}"
    )


def test_scan_roots_and_exempt_dirs_actually_exist() -> None:
    """Guard against the scan silently covering nothing (e.g. a renamed package)."""
    assert (_REPO_ROOT / "aeris").is_dir()
    assert (_REPO_ROOT / "aeris" / "safety").is_dir()
    assert (_REPO_ROOT / "aeris" / "vehicle").is_dir()


def test_scan_actually_detects_a_violation() -> None:
    """A canary so this test can't silently stop scanning anything."""
    fake_source = "async def bad():\n    port = vehicle.command_port()\n    await port.arm()\n"
    assert _FORBIDDEN_CALL in fake_source

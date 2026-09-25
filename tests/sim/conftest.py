"""Shared fixtures for `sim`-marked tests (need a real, built PX4 checkout).

Spec §44.1: "sim: requires PX4 SITL + Gazebo running locally" — these tests
are never run in hosted CI (spec §44.2) and are recorded manually in phase
reports with logs instead. They skip cleanly (not fail) wherever PX4 isn't
built, so `pytest` on a machine without the Phase 1 setup doesn't explode.
"""

from __future__ import annotations

from collections.abc import Iterator

import pytest

from aeris.simulation.launcher.launcher import SimulationLauncher
from aeris.simulation.px4_paths import Px4Layout, resolve_px4_layout


@pytest.fixture(scope="session")
def px4_layout() -> Px4Layout:
    layout = resolve_px4_layout()
    if not layout.binary_path.is_file():
        pytest.skip(
            f"PX4 not built at {layout.binary_path} — see docs/mac-setup.md. "
            f"sim-marked tests are not run in hosted CI (spec §44.2)."
        )
    return layout


@pytest.fixture
def launcher(px4_layout: Px4Layout) -> Iterator[SimulationLauncher]:
    launcher = SimulationLauncher(px4_layout)
    yield launcher
    # Belt-and-suspenders cleanup: if a test fails mid-cycle, don't leak
    # processes into the next test.
    launcher.stop()

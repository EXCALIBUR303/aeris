"""Shared fixtures for anything that needs a real, built PX4 checkout.

Used by ``tests/sim/`` (spec §44.1's ``sim`` marker) and ``tests/integration/``
(the ``integration`` marker) — both need PX4 SITL + Gazebo, neither runs in
hosted CI (spec §44.2). Skips cleanly (not fails) wherever PX4 isn't built.
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
            f"sim/integration-marked tests are not run in hosted CI (spec §44.2)."
        )
    return layout


@pytest.fixture
def launcher(px4_layout: Px4Layout) -> Iterator[SimulationLauncher]:
    launcher = SimulationLauncher(px4_layout)
    yield launcher
    # Belt-and-suspenders cleanup: if a test fails mid-cycle, don't leak
    # processes into the next test.
    launcher.stop()

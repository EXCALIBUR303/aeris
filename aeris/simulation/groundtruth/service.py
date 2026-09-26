"""``GroundTruthService``: gz world-frame (ENU, spec §19.1) model poses,
read via the ``gz`` CLI (spec §51 Phase 7: "for P7 this can use a tiny
gz-Python helper" -- implemented here as a subprocess call to `gz topic
-e`, the same tool ``aeris.simulation.launcher.readiness`` already shells
out to, rather than importing ``gz.transport``/``gz.msgs`` directly: only
the future Sensor Bridge process (Phase 8) is allowed to import ``gz.*``
at all (spec §14.3 contract 5), and this module needs to work from the
same venv as everything else in Phase 7).
"""

from __future__ import annotations

import os
import re
import subprocess

from aeris.core.errors import SimulationError
from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.vector import Vec3


def _extract_blocks(text: str, tag: str) -> list[str]:
    """Every top-level ``tag { ... }`` block's inner text, brace-depth
    aware (needed since ``pose { ... }`` blocks nest ``position { ... }``/
    ``orientation { ... }`` blocks using the same ``{``/``}`` tokens)."""
    blocks = []
    marker = f"{tag} {{"
    i = 0
    while True:
        start = text.find(marker, i)
        if start == -1:
            break
        depth = 1
        j = start + len(marker)
        while depth > 0 and j < len(text):
            if text[j] == "{":
                depth += 1
            elif text[j] == "}":
                depth -= 1
            j += 1
        blocks.append(text[start + len(marker) : j - 1])
        i = j
    return blocks


def _extract_scalar(block: str, field: str) -> str | None:
    match = re.search(rf'^\s*{re.escape(field)}:\s*"?([^"\n]+)"?\s*$', block, re.MULTILINE)
    return match.group(1).strip() if match else None


def _extract_vec3(block: str, tag: str) -> Vec3:
    sub_blocks = _extract_blocks(block, tag)
    sub = sub_blocks[0] if sub_blocks else ""

    def field(name: str) -> float:
        raw = _extract_scalar(sub, name)
        return float(raw) if raw is not None else 0.0

    return Vec3(field("x"), field("y"), field("z"))


def _extract_quaternion(block: str, tag: str) -> Quaternion:
    sub_blocks = _extract_blocks(block, tag)
    sub = sub_blocks[0] if sub_blocks else ""

    def field(name: str, default: float) -> float:
        raw = _extract_scalar(sub, name)
        return float(raw) if raw is not None else default

    return Quaternion(field("w", 1.0), field("x", 0.0), field("y", 0.0), field("z", 0.0))


class GroundTruthService:
    """Reads live gz world-frame poses. Every method here returns
    :class:`~aeris.core.types.Provenance.ORACLE`-grade data -- never wire
    this into anything agent-facing."""

    def __init__(self, *, world: str = "default", timeout_s: float = 5.0) -> None:
        self._world = world
        self._timeout_s = timeout_s

    def get_pose(self, model_name: str) -> tuple[Vec3, Quaternion]:
        """``model_name``'s pose (position, orientation) in the Gazebo
        world frame -- ENU (spec §19.1's ``W``), ground truth."""
        env = {**os.environ, "GZ_IP": "127.0.0.1"}
        result = subprocess.run(
            ["gz", "topic", "-e", "-t", f"/world/{self._world}/pose/info", "-n", "1"],
            env=env,
            capture_output=True,
            text=True,
            timeout=self._timeout_s,
        )
        if result.returncode != 0:
            raise SimulationError(f"gz topic read failed (rc={result.returncode}): {result.stderr}")
        for block in _extract_blocks(result.stdout, "pose"):
            name = _extract_scalar(block, "name")
            if name is not None and name.strip('"') == model_name:
                return _extract_vec3(block, "position"), _extract_quaternion(block, "orientation")
        raise SimulationError(
            f"model {model_name!r} not found in /world/{self._world}/pose/info "
            f"(got {len(_extract_blocks(result.stdout, 'pose'))} poses)"
        )

    def get_contacts(self, model_name: str) -> list[str]:
        """Ground-truth contact events involving ``model_name``.

        Always empty as of Phase 7: no contact sensor is subscribed (the
        default world has no obstacles to contact in the first place), and
        the Sensor Bridge's real contact-topic subscription is Phase 8/9's
        job, once ``WorldSpec``-generated obstacles exist to contact.
        """
        return []

#!/usr/bin/env python3
"""Generate ``configs/vehicle/sensors.yaml`` from the pinned vehicle SDF (spec §18.2).

Spec §18.2: "Intrinsics (CameraInfo) and extrinsics (T_base_sensor) come
from configuration that is *generated from the vehicle SDF* (single
source)". This script is that generator — it's a thin wrapper over
:mod:`aeris.core.frames.extrinsics` (spec §14.1: "scripts/ ... thin
wrappers over package code"), re-run whenever the pinned PX4 checkout's
model SDF changes.

Camera intrinsics are computed from the SDF's own ``horizontal_fov`` and
image size via the standard pinhole formula (``fx = width / (2 tan(hfov/2))``,
square pixels, principal point at the image center) — the same formula
gz-sim's own ``Camera`` sensor uses internally to build its
``CameraInfo.intrinsics``, confirmed against a live capture in Phase 8
(see ``docs/sensors.md``).

Usage::

    uv run python scripts/generate_sensors_config.py
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml

from aeris.core.frames.extrinsics import load_sensor_extrinsics
from aeris.simulation.px4_paths import resolve_px4_layout

_MODEL_NAME = "x500_depth"
_SENSORS = {
    "front_rgb": "IMX214",
    "front_depth": "StereoOV7251",
}
_OUTPUT_PATH = Path(__file__).resolve().parents[1] / "configs" / "vehicle" / "sensors.yaml"


def _find_sensor_camera_element(model_sdf_path: Path, sensor_name: str) -> ET.Element:
    """Locate a ``<sensor name=... type="camera|depth_camera">``'s ``<camera>`` child.

    Walks ``<include>``s the same way :mod:`aeris.core.frames.extrinsics`
    does, since (like the base_link/sensor-link poses it resolves) the
    sensor elements themselves live inside the included ``OakD-Lite`` SDF,
    not directly in ``x500_depth/model.sdf``.
    """
    search_dirs = (model_sdf_path.parent.parent,)
    stack = [ET.parse(model_sdf_path).getroot().find("model")]
    while stack:
        model_el = stack.pop()
        if model_el is None:
            continue
        for link_el in model_el.findall("link"):
            for sensor_el in link_el.findall("sensor"):
                if sensor_el.get("name") == sensor_name:
                    camera_el = sensor_el.find("camera")
                    if camera_el is None:
                        raise ValueError(f"sensor {sensor_name!r} has no <camera> element")
                    return camera_el
        for include_el in model_el.findall("include"):
            uri = include_el.findtext("uri")
            if uri is None:
                continue
            name = uri.removeprefix("model://")
            for directory in search_dirs:
                candidate = directory / name / "model.sdf"
                if candidate.is_file():
                    stack.append(ET.parse(candidate).getroot().find("model"))
                    break
    raise ValueError(f"sensor {sensor_name!r} not found under {model_sdf_path}")


def _intrinsics_from_camera_element(camera_el: ET.Element) -> dict[str, float | int]:
    hfov = float(camera_el.findtext("horizontal_fov", "1.0"))
    width = int(camera_el.findtext("image/width", "0"))
    height = int(camera_el.findtext("image/height", "0"))
    fx = width / (2.0 * math.tan(hfov / 2.0))
    return {
        "width": width,
        "height": height,
        "fx": fx,
        "fy": fx,
        "cx": width / 2.0,
        "cy": height / 2.0,
    }


def generate(*, output_path: Path = _OUTPUT_PATH) -> dict[str, object]:
    layout = resolve_px4_layout()
    model_sdf_path = layout.models_dir / _MODEL_NAME / "model.sdf"

    sensors: dict[str, dict[str, object]] = {}
    for sensor_id, sdf_sensor_name in _SENSORS.items():
        extrinsics = load_sensor_extrinsics(model_sdf_path, sensor_name=sdf_sensor_name)
        camera_el = _find_sensor_camera_element(model_sdf_path, sdf_sensor_name)
        intrinsics = _intrinsics_from_camera_element(camera_el)
        sensors[sensor_id] = {
            "sdf_sensor_name": sdf_sensor_name,
            "extrinsics_T_base_sensor": {
                "translation": list(extrinsics.translation),
                "rotation_wxyz": list(extrinsics.rotation),
            },
            "intrinsics": intrinsics,
        }

    doc: dict[str, object] = {
        "_generated_by": "scripts/generate_sensors_config.py",
        "_source_model": str(model_sdf_path),
        "model": _MODEL_NAME,
        "sensors": sensors,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(yaml.safe_dump(doc, sort_keys=False))
    return doc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=_OUTPUT_PATH)
    args = parser.parse_args(argv)
    doc = generate(output_path=args.output)
    print(f"wrote {args.output}")
    print(yaml.safe_dump(doc, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Sensor extrinsics (``T_base_sensor``) generated from a Gazebo model SDF.

Spec §18.2: "extrinsics (``T_base_sensor``) come from configuration that is
generated from the vehicle SDF (single source)". This module is that
generator: it parses a model's ``model.sdf`` (recursively resolving
``<include merge="true">``, which is the only include style any AERIS or
PX4-stock model uses) and composes link/sensor poses to produce the pose of
a named sensor relative to ``base_link``.

Verified against the pinned PX4 checkout in Phase 8 by reading
``Tools/simulation/gz/models/{x500,x500_base,x500_depth,OakD-Lite}/model.sdf``
directly: ``x500_base``'s own top-level ``<pose>`` places ``base_link`` at
``z=.24`` *within the outer model's frame*, so the naive shortcut of reading
only the outermost include's ``<pose>`` would put the camera at the wrong
height — this parser composes the full include chain, including each
included model's own top-level ``<pose>``, exactly as gz-sim's own merge
resolution does.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from xml.etree import ElementTree as ET

from aeris.core.frames.quaternion import Quaternion
from aeris.core.frames.transform import Transform
from aeris.core.frames.vector import Vec3


class ExtrinsicsError(Exception):
    """A model SDF's link/sensor pose chain could not be resolved."""


@dataclass(frozen=True, slots=True)
class _Sensor:
    link_name: str
    local_pose: Transform  # T_link_sensor


@dataclass(slots=True)
class _WalkState:
    links: dict[str, Transform] = field(default_factory=dict)
    sensors: dict[str, _Sensor] = field(default_factory=dict)


def _parse_pose(text: str | None) -> Transform:
    """Parse an SDF ``<pose>x y z roll pitch yaw</pose>`` element.

    SDF poses are fixed-axis (extrinsic) roll-then-pitch-then-yaw, so the
    composed rotation applied to a vector is ``Rz(yaw) @ Ry(pitch) @
    Rx(roll)`` — built here as ``q_yaw.compose(q_pitch).compose(q_roll)``,
    since ``a.compose(b).rotate(v) == a.rotate(b.rotate(v))``.
    """
    if text is None or not text.strip():
        return Transform.identity()
    parts = [float(v) for v in text.split()]
    if len(parts) != 6:
        raise ExtrinsicsError(f"expected 6 pose values 'x y z roll pitch yaw', got: {text!r}")
    x, y, z, roll, pitch, yaw = parts
    q = (
        Quaternion.from_axis_angle(Vec3(0.0, 0.0, 1.0), yaw)
        .compose(Quaternion.from_axis_angle(Vec3(0.0, 1.0, 0.0), pitch))
        .compose(Quaternion.from_axis_angle(Vec3(1.0, 0.0, 0.0), roll))
    )
    return Transform(q, Vec3(x, y, z))


def _resolve_include_uri(uri: str, *, search_dirs: tuple[Path, ...]) -> Path:
    name = uri.removeprefix("model://")
    for directory in search_dirs:
        candidate = directory / name / "model.sdf"
        if candidate.is_file():
            return candidate
    raise ExtrinsicsError(f"could not resolve include uri {uri!r} against {search_dirs}")


def _walk(
    model_el: ET.Element, *, base_pose: Transform, search_dirs: tuple[Path, ...], state: _WalkState
) -> None:
    model_pose = base_pose.compose(_parse_pose(model_el.findtext("pose")))

    for link_el in model_el.findall("link"):
        name = link_el.get("name")
        if name is None:
            continue
        link_pose = model_pose.compose(_parse_pose(link_el.findtext("pose")))
        state.links[name] = link_pose
        for sensor_el in link_el.findall("sensor"):
            sensor_name = sensor_el.get("name")
            if sensor_name is None:
                continue
            state.sensors[sensor_name] = _Sensor(
                link_name=name, local_pose=_parse_pose(sensor_el.findtext("pose"))
            )

    for include_el in model_el.findall("include"):
        uri = include_el.findtext("uri")
        if uri is None:
            continue
        include_pose = model_pose.compose(_parse_pose(include_el.findtext("pose")))
        sdf_path = _resolve_include_uri(uri, search_dirs=search_dirs)
        included_model = ET.parse(sdf_path).getroot().find("model")
        if included_model is None:
            raise ExtrinsicsError(f"no <model> element in included SDF {sdf_path}")
        _walk(included_model, base_pose=include_pose, search_dirs=search_dirs, state=state)


def load_sensor_extrinsics(
    model_sdf_path: Path,
    *,
    sensor_name: str,
    base_link: str = "base_link",
    extra_search_dirs: tuple[Path, ...] = (),
) -> Transform:
    """Return ``T_base_sensor`` for the named ``<sensor>`` in a model SDF.

    ``model_sdf_path`` is the top-level model's ``model.sdf``.
    ``bare-name`` and ``model://name`` include URIs are resolved by looking
    for ``<dir>/<name>/model.sdf`` under ``model_sdf_path``'s own models
    directory (its grandparent — matching every PX4-stock and AERIS model
    layout: ``.../models/<model_name>/model.sdf``) plus ``extra_search_dirs``.
    """
    root_model = ET.parse(model_sdf_path).getroot().find("model")
    if root_model is None:
        raise ExtrinsicsError(f"no <model> element in {model_sdf_path}")

    search_dirs = (model_sdf_path.parent.parent, *extra_search_dirs)
    state = _WalkState()
    _walk(root_model, base_pose=Transform.identity(), search_dirs=search_dirs, state=state)

    if base_link not in state.links:
        raise ExtrinsicsError(
            f"link {base_link!r} not found in {model_sdf_path} (found: {sorted(state.links)})"
        )
    if sensor_name not in state.sensors:
        raise ExtrinsicsError(
            f"sensor {sensor_name!r} not found in {model_sdf_path} (found: {sorted(state.sensors)})"
        )

    sensor = state.sensors[sensor_name]
    if sensor.link_name not in state.links:
        raise ExtrinsicsError(
            f"sensor {sensor_name!r}'s link {sensor.link_name!r} was never resolved"
        )

    t_model_base = state.links[base_link]
    t_model_sensorlink = state.links[sensor.link_name]
    t_base_sensorlink = t_model_base.inverse().compose(t_model_sensorlink)
    return t_base_sensorlink.compose(sensor.local_pose)


def load_link_pose(
    model_sdf_path: Path,
    *,
    link_name: str,
    extra_search_dirs: tuple[Path, ...] = (),
) -> Transform:
    """Return ``T_modelroot_link`` — a named link's pose in the model's own root frame.

    Useful alongside :func:`load_sensor_extrinsics` when a caller also
    needs the model-root-to-``base_link`` offset — e.g. to predict a
    sensor's *world* pose given the model's spawn pose (spawn pose places
    the model root, not ``base_link``, which per the pinned ``x500_base``
    SDF sits at ``z=.24`` within the model root, not at its origin).
    """
    root_model = ET.parse(model_sdf_path).getroot().find("model")
    if root_model is None:
        raise ExtrinsicsError(f"no <model> element in {model_sdf_path}")

    search_dirs = (model_sdf_path.parent.parent, *extra_search_dirs)
    state = _WalkState()
    _walk(root_model, base_pose=Transform.identity(), search_dirs=search_dirs, state=state)

    if link_name not in state.links:
        raise ExtrinsicsError(
            f"link {link_name!r} not found in {model_sdf_path} (found: {sorted(state.links)})"
        )
    return state.links[link_name]

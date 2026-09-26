"""Unit tests for :mod:`aeris.simulation.worlds.sdf` (spec §9.3)."""

from __future__ import annotations

from xml.etree import ElementTree as ET

from aeris.simulation.worlds.sdf import render
from aeris.simulation.worlds.spec import (
    Bounds,
    Box,
    Cylinder,
    SpawnPose,
    WorldFamily,
    WorldSpec,
    WorldSplit,
)


def _spec() -> WorldSpec:
    return WorldSpec(
        name="render_test",
        family=WorldFamily.RUBBLE,
        split=WorldSplit.TRAIN,
        seed=0,
        bounds=Bounds(min_x=-5, min_y=-5, max_x=5, max_y=5),
        boxes=(Box(x=1, y=2, z=0.5, size_x=1, size_y=1, size_z=1),),
        cylinders=(Cylinder(x=-1, y=-2, z=0.5, radius_m=0.3, height_m=1.0),),
        spawn_poses=(SpawnPose(x=0, y=0, z=0.1),),
    )


def test_render_produces_well_formed_xml() -> None:
    xml_text = render(_spec())
    root = ET.fromstring(xml_text)  # raises if malformed
    assert root.tag == "sdf"


def test_render_uses_world_name() -> None:
    xml_text = render(_spec())
    root = ET.fromstring(xml_text)
    world = root.find("world")
    assert world is not None
    assert world.get("name") == "render_test"


def test_render_includes_one_model_per_primitive_plus_ground_plane() -> None:
    xml_text = render(_spec())
    root = ET.fromstring(xml_text)
    models = root.find("world").findall("model")
    names = {m.get("name") for m in models}
    assert "ground_plane" in names
    assert "aeris_box_0" in names
    assert "aeris_cylinder_0" in names
    assert len(models) == 3


def test_render_box_pose_matches_spec() -> None:
    xml_text = render(_spec())
    root = ET.fromstring(xml_text)
    box_model = next(
        m for m in root.find("world").findall("model") if m.get("name") == "aeris_box_0"
    )
    pose_text = box_model.find("pose").text
    assert pose_text.split() == ["1.0", "2.0", "0.5", "0.0", "0.0", "0.0"]


def test_render_includes_required_world_elements() -> None:
    xml_text = render(_spec())
    root = ET.fromstring(xml_text)
    world = root.find("world")
    assert world.find("physics") is not None
    assert world.find("light") is not None
    assert world.find("spherical_coordinates") is not None

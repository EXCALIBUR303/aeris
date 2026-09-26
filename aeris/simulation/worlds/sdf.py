"""``SdfRenderer`` — render a :class:`WorldSpec` into a Gazebo ``.sdf`` world (spec §9.3).

Matches PX4's own ``default.sdf`` boilerplate exactly (physics, gravity,
scene, sun light, spherical coordinates) so a rendered world is a drop-in
replacement for ``"default"`` in :class:`~aeris.simulation.launcher.profiles.SimulationProfile`
-- world-level system plugins (physics, sensors, scene-broadcaster) come
from PX4's ``GZ_SIM_SERVER_CONFIG_PATH``, not from the world SDF itself
(verified by reading ``default.sdf`` directly: it declares none).
"""

from __future__ import annotations

from aeris.simulation.worlds.spec import Box, Cylinder, WorldSpec

_PHYSICS_MAX_STEP_S = 0.004
_PHYSICS_UPDATE_RATE_HZ = 250


def _box_model_sdf(box: Box, index: int) -> str:
    size = f"{box.size_x} {box.size_y} {box.size_z}"
    pose = f"{box.x} {box.y} {box.z} {box.roll_rad} {box.pitch_rad} {box.yaw_rad}"
    return f"""    <model name="aeris_box_{index}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{size}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
        </visual>
      </link>
    </model>"""


def _cylinder_model_sdf(cyl: Cylinder, index: int) -> str:
    pose = f"{cyl.x} {cyl.y} {cyl.z} 0 0 0"
    return f"""    <model name="aeris_cylinder_{index}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{cyl.radius_m}</radius><length>{cyl.height_m}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{cyl.radius_m}</radius><length>{cyl.height_m}</length></cylinder></geometry>
        </visual>
      </link>
    </model>"""


def render(spec: WorldSpec) -> str:
    """Render ``spec`` into a complete ``<sdf>...</sdf>`` world document."""
    ambient = spec.ambient_intensity
    models = [_box_model_sdf(b, i) for i, b in enumerate(spec.boxes)]
    models += [_cylinder_model_sdf(c, i) for i, c in enumerate(spec.cylinders)]
    models_xml = "\n".join(models)

    return f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{spec.name}">
    <physics type="ode">
      <max_step_size>{_PHYSICS_MAX_STEP_S}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>{_PHYSICS_UPDATE_RATE_HZ}</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>{ambient} {ambient} {ambient} 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
{models_xml}
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
      <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""

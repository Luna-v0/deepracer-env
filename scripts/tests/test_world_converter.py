#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Host-runnable tests for the classic-.world -> Jetty-.sdf converter."""
import os
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from world_converter import convert_world_text  # noqa: E402

CLASSIC_WORLD = """\
<?xml version="1.0"?>
<sdf version="1.7">
<world name="reinvent_base">
  <scene><ambient>0.5 0.5 0.5 1.0</ambient><shadows>0</shadows></scene>
  <include>
    <uri>model://sun</uri>
    <pose>0.0 0.0 15.0 0 0 0</pose>
  </include>
  <light type="point" name="Light 1">
    <pose>0 0 15 0 0 0</pose>
    <diffuse>0.225 0.225 0.225 1</diffuse>
  </light>
  <include>
    <uri>model://models/reinvent_base</uri>
    <pose>0 0 0 0 0 0</pose>
    <name>racetrack</name>
  </include>
</world>
</sdf>
"""

# multi-include world (jeremiah-style): two track meshes, both must survive.
MULTI_INCLUDE_WORLD = """\
<?xml version="1.0"?>
<sdf version="1.6">
<world name="multi">
  <include><uri>model://sun</uri></include>
  <include><uri>model://ground_plane</uri></include>
  <include><uri>model://models/reinvent_lines_walls</uri><name>racetrack</name></include>
  <include><uri>model://models/reinvent_grass_asphalt</uri><name>racetrack_1</name></include>
</world>
</sdf>
"""


def _world(sdf_text):
    return ET.fromstring(sdf_text).find("world")


def test_injects_system_plugins_and_physics():
    w = _world(convert_world_text(CLASSIC_WORLD))
    names = {p.get("name") for p in w.findall("plugin")}
    assert "gz::sim::systems::Physics" in names
    assert "gz::sim::systems::UserCommands" in names      # spawn/teleport/step
    assert "gz::sim::systems::SceneBroadcaster" in names  # pose/info + GUI
    assert "gz::sim::systems::Sensors" in names           # camera/lidar
    assert w.find("gravity") is not None
    assert w.find("physics") is not None


def test_classic_sun_replaced_with_native_directional():
    w = _world(convert_world_text(CLASSIC_WORLD))
    suns = [l for l in w.findall("light") if l.get("name") == "sun"]
    assert len(suns) == 1 and suns[0].get("type") == "directional"
    # the classic model://sun include is gone
    assert not any((inc.findtext("uri") or "").endswith("sun")
                   for inc in w.findall("include"))


def test_track_include_and_point_light_preserved():
    w = _world(convert_world_text(CLASSIC_WORLD))
    uris = [inc.findtext("uri") for inc in w.findall("include")]
    assert "model://models/reinvent_base" in uris
    assert any(l.get("name") == "Light 1" for l in w.findall("light"))


def test_multi_include_keeps_both_track_meshes_drops_scaffolding():
    w = _world(convert_world_text(MULTI_INCLUDE_WORLD))
    uris = [inc.findtext("uri") for inc in w.findall("include")]
    assert "model://models/reinvent_lines_walls" in uris
    assert "model://models/reinvent_grass_asphalt" in uris
    assert "model://sun" not in uris
    assert "model://ground_plane" not in uris


def test_output_is_well_formed_and_versioned():
    text = convert_world_text(CLASSIC_WORLD)
    assert text.startswith("<?xml")
    root = ET.fromstring(text)
    assert root.tag == "sdf"
    assert root.get("version") == "1.10"


def test_missing_world_raises():
    with pytest.raises(ValueError):
        convert_world_text("<?xml version='1.0'?><sdf version='1.7'></sdf>")

#!/usr/bin/env python3
#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Convert Gazebo-classic ``.world`` files to Gazebo Jetty ``.sdf`` worlds.

The 127 shipped tracks are near-identical templates: a ``<scene>``, a classic
``model://sun`` include, one or more ``<light>`` elements, and one or more track
``<include>`` blocks. They carry **no physics and no system plugins** — which is
fine for the classic ``gzserver`` (it wires those in itself) but means a raw load
under Gazebo Jetty gives you a static, *uncontrollable* scene: no gravity, and
none of the ``/world/<world>/{create,set_pose,control,pose/info}`` services the
:class:`~deepracer_env.sim_control.backends.ros_gz_backend.RosGzBackend` relies
on. (Those services are provided by the ``UserCommands`` / ``SceneBroadcaster``
system plugins.)

So conversion does three things:

1. **Inject the standard gz system plugins** — ``Physics`` (DART), ``UserCommands``
   (spawn/teleport/step services), ``SceneBroadcaster`` (``pose/info`` + GUI),
   ``Sensors`` (camera/LiDAR rendering), ``Contact`` (collision events) — plus an
   explicit ``<physics>`` step and ``<gravity>``. Without these the car neither
   falls onto the track nor can be driven or read.
2. **Replace the classic ``model://sun`` include** (a Gazebo-classic asset absent
   from Jetty) with a native ``<light type="directional" name="sun">``.
3. **Preserve everything reusable verbatim** — the scene, any ``<light>`` point
   lights, and every track ``<include>`` (so multi-include worlds like
   ``reinvent_base_jeremiah`` keep both meshes). ``model://`` URIs resolve at
   load through ``GZ_SIM_RESOURCE_PATH`` exactly as the classic stack used
   ``GAZEBO_MODEL_PATH``; the mesh ``.dae`` files are renderer-agnostic and need
   no change.

The transform is pure ``xml.etree`` — no ROS, no Gazebo — so it is unit-testable
on any host and runs as a fast batch over the whole ``worlds/`` directory.
"""
from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from typing import List

# Output SDF version. gz-sim 10 (Jetty) reads up to 1.11; 1.10 is broadly safe.
SDF_VERSION = "1.10"

# The standard gz-sim system plugins, copied from the shipped ``empty.sdf`` and
# extended with the Sensors system (needed for camera/LiDAR arenas). These are
# what make a world drivable + controllable under Jetty.
SYSTEM_PLUGINS = """\
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>"""

# Native replacement for the classic ``model://sun`` include.
SUN_LIGHT = """\
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 15 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>"""

# Classic includes that do not carry over: the sun model, and any classic
# ground_plane (Jetty tracks bring their own asphalt mesh + collision).
_DROP_INCLUDE_MODELS = ("sun", "ground_plane")


def _is_dropped_include(include_elem: ET.Element) -> bool:
    """Return True for a ``model://sun`` / ``model://ground_plane`` include."""
    uri = (include_elem.findtext("uri") or "").strip()
    model = uri.rsplit("/", 1)[-1] if uri else ""
    return model in _DROP_INCLUDE_MODELS


def _serialize(elem: ET.Element, indent: str = "    ") -> str:
    """Serialize an element to a string, re-indented under *indent*."""
    raw = ET.tostring(elem, encoding="unicode").strip()
    return "\n".join(indent + line for line in raw.splitlines())


def convert_world_text(world_xml: str) -> str:
    """Convert one classic ``.world`` document to a Jetty ``.sdf`` document.

    Args:
        world_xml: The full text of a classic ``.world`` file.

    Returns:
        The converted Jetty world as an SDF string.

    Raises:
        ValueError: If the input has no ``<world>`` element.
    """
    root = ET.fromstring(world_xml)
    world = root if root.tag == "world" else root.find("world")
    if world is None:
        raise ValueError("no <world> element found")
    world_name = world.get("name", "deepracer_world")

    preserved: List[str] = []
    has_sun = False
    for child in list(world):
        if child.tag == "include":
            if _is_dropped_include(child):
                has_sun = has_sun or ((child.findtext("uri") or "").rstrip("/").endswith("sun"))
                continue
            preserved.append(_serialize(child))
        elif child.tag == "scene":
            preserved.append(_serialize(child))
        elif child.tag == "light":
            # Keep authored point/spot lights verbatim (valid SDF in Jetty).
            preserved.append(_serialize(child))
        elif child.tag in ("physics", "gravity"):
            # Dropped: we inject canonical physics/gravity below to guarantee a
            # consistent, drivable configuration across all 127 worlds.
            continue
        else:
            # Any other authored element (e.g. spherical_coordinates) carries over.
            preserved.append(_serialize(child))

    parts = [
        '<?xml version="1.0" ?>',
        '<sdf version="{}">'.format(SDF_VERSION),
        '  <world name="{}">'.format(world_name),
        SYSTEM_PLUGINS,
        SUN_LIGHT,
    ]
    parts.extend(preserved)
    parts.append("  </world>")
    parts.append("</sdf>")
    return "\n".join(parts) + "\n"


def convert_file(in_path: str, out_path: str) -> None:
    """Convert a single ``.world`` file to ``out_path`` (a ``.sdf``)."""
    with open(in_path, "r") as fh:
        text = fh.read()
    converted = convert_world_text(text)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as fh:
        fh.write(converted)


def convert_dir(in_dir: str, out_dir: str) -> List[str]:
    """Batch-convert every ``*.world`` in *in_dir* to ``*.sdf`` in *out_dir*.

    Returns:
        The list of output paths written.
    """
    written = []
    for name in sorted(os.listdir(in_dir)):
        if not name.endswith(".world"):
            continue
        stem = name[: -len(".world")]
        out_path = os.path.join(out_dir, stem + ".sdf")
        try:
            convert_file(os.path.join(in_dir, name), out_path)
            written.append(out_path)
        except Exception as ex:  # noqa: BLE001
            print("FAILED {}: {}".format(name, ex), file=sys.stderr)
    return written


def _default_dir() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "simulation", "worlds")


def main(argv=None) -> int:
    """CLI entry point. Converts the worlds directory (or a single file)."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--in-dir", default=_default_dir(),
                        help="directory of classic .world files")
    parser.add_argument("--out-dir", default=None,
                        help="output directory for .sdf files (default: same as in-dir)")
    parser.add_argument("--world", default=None,
                        help="convert a single world by name (without extension)")
    args = parser.parse_args(argv)
    out_dir = args.out_dir or args.in_dir

    if args.world:
        in_path = os.path.join(args.in_dir, args.world + ".world")
        out_path = os.path.join(out_dir, args.world + ".sdf")
        convert_file(in_path, out_path)
        print("wrote {}".format(out_path))
        return 0

    written = convert_dir(args.in_dir, out_dir)
    print("converted {} worlds -> {}".format(len(written), out_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

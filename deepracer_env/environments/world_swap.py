'''Runtime track (world) swapping for :class:`DeepRacerEnv`.

This module owns the *Gazebo-side* half of a hot track swap: deleting the
currently-rendered track mesh and spawning a different one, without
restarting the ``gzserver`` process. It is deliberately free of any
:class:`~deepracer_env.track_geom.track_data.TrackData` / controller logic
— that bookkeeping lives in :meth:`DeepRacerEnv.set_world` — so this class
stays a thin, testable wrapper over the standard ``gazebo_ros`` services.

Why spawn via an ``<include>`` wrapper instead of a model SDF path
-----------------------------------------------------------------
The shipped tracks have wildly inconsistent SDF filenames (the default
``reinvent_base`` is ``model.sdf``; ``Vegas_track`` is
``Re_Invent_track_model.sdf``; ``Monaco_building`` is ``Monaco_model.sdf``,
etc.). Guessing the filename breaks on the most common track. Instead we
hand Gazebo the exact ``<include>`` block the ``.world`` file uses::

    <sdf version="1.6">
      <include>
        <uri>model://models/<world></uri>
        <name>racetrack</name>
      </include>
    </sdf>

Gazebo then resolves every mesh/material ``model://`` URI through the
existing ``GAZEBO_MODEL_PATH`` exactly as it does at world-load time, so the
filename inconsistencies never matter. For multi-include worlds (e.g.
``reinvent_base_jeremiah`` loads ``reinvent_lines_walls`` +
``reinvent_grass_asphalt``) we parse the target ``.world`` and replay every
non-``sun`` / non-``ground_plane`` ``<include>`` verbatim.
'''
import logging
import os
import xml.etree.ElementTree as ET

import rospkg
import rospy
from gazebo_msgs.srv import (
    SpawnModel, DeleteModel, GetWorldProperties,
)
from geometry_msgs.msg import Pose

from deepracer_env.log_handler.logger import Logger
from deepracer_env.rospy_wrappers import ServiceProxyWrapper
from deepracer_env.track_geom.constants import (
    SPAWN_SDF_MODEL, DELETE_MODEL, GET_WORLD_PROPERTIES,
    RACETRACK_MODEL_NAME,
)

LOG = Logger(__name__, logging.INFO).get_logger()

_SIM_PKG = "deepracer_simulation_environment"

# Includes that are part of the scene scaffolding, not the track itself.
_NON_TRACK_INCLUDE_MODELS = ("sun", "ground_plane")


class WorldSwapper(object):
    '''Deletes the live track mesh and spawns a different one in-place.

    One instance is created lazily by :class:`DeepRacerEnv`. All Gazebo
    service proxies are created on first use so importing this module never
    requires a live ROS graph.
    '''

    def __init__(self):
        self._spawn_srv = None
        self._delete_srv = None
        self._world_props_srv = None
        rospack = rospkg.RosPack()
        # routes/, models/ and worlds/ are all installed under the package
        # share dir (see simulation/.../CMakeLists.txt).
        self._pkg_path = rospack.get_path(_SIM_PKG)

    # ------------------------------------------------------------------
    # Path / validation helpers
    # ------------------------------------------------------------------

    def route_path(self, world_name):
        '''Absolute path to ``routes/<world>.npy`` (the waypoint file).'''
        return os.path.join(self._pkg_path, "routes", "{}.npy".format(world_name))

    def model_dir(self, world_name):
        '''Absolute path to ``models/<world>/`` (the track mesh model dir).'''
        return os.path.join(self._pkg_path, "models", world_name)

    def world_file(self, world_name):
        '''Absolute path to ``worlds/<world>.world`` (may not exist).'''
        return os.path.join(self._pkg_path, "worlds", "{}.world".format(world_name))

    def validate(self, world_name):
        '''Raise ``ValueError`` if the assets needed to swap to *world_name*
        are missing. Checked *before* anything is deleted so a bad name can
        never leave the world track-less.'''
        route = self.route_path(world_name)
        if not os.path.isfile(route):
            raise ValueError(
                "Cannot swap to world {!r}: route file not found at {}".format(
                    world_name, route))
        model_dir = self.model_dir(world_name)
        if not os.path.isdir(model_dir):
            raise ValueError(
                "Cannot swap to world {!r}: model dir not found at {}".format(
                    world_name, model_dir))

    # ------------------------------------------------------------------
    # Gazebo service plumbing
    # ------------------------------------------------------------------

    def _ensure_services(self):
        if self._spawn_srv is None:
            rospy.wait_for_service(SPAWN_SDF_MODEL, timeout=30.0)
            self._spawn_srv = ServiceProxyWrapper(SPAWN_SDF_MODEL, SpawnModel)
        if self._delete_srv is None:
            rospy.wait_for_service(DELETE_MODEL, timeout=30.0)
            self._delete_srv = ServiceProxyWrapper(DELETE_MODEL, DeleteModel)
        if self._world_props_srv is None:
            rospy.wait_for_service(GET_WORLD_PROPERTIES, timeout=30.0)
            self._world_props_srv = ServiceProxyWrapper(
                GET_WORLD_PROPERTIES, GetWorldProperties)

    def current_model_names(self):
        '''Return the list of model names currently present in Gazebo.'''
        self._ensure_services()
        return list(self._world_props_srv().model_names)

    def _track_model_names(self):
        '''Names of the live track model(s) — anything named ``racetrack``
        (multi-include worlds may suffix it, so match the prefix too).'''
        return [n for n in self.current_model_names()
                if n == RACETRACK_MODEL_NAME or n.startswith(RACETRACK_MODEL_NAME)]

    # ------------------------------------------------------------------
    # SDF construction
    # ------------------------------------------------------------------

    def _include_sdf(self, world_name):
        '''Build the SDF to spawn the new track.

        Prefers replaying the target ``.world`` file's own track
        ``<include>`` blocks (handles multi-include worlds); falls back to a
        single ``model://models/<world>`` include when the world file is
        absent or unparseable.
        '''
        includes = self._parse_world_includes(world_name)
        if not includes:
            includes = [self._default_include_xml(world_name)]
        return '<?xml version="1.0"?>\n<sdf version="1.6">\n{}\n</sdf>\n'.format(
            "\n".join(includes))

    @staticmethod
    def _default_include_xml(world_name):
        return (
            '  <include>\n'
            '    <uri>model://models/{}</uri>\n'
            '    <name>{}</name>\n'
            '  </include>'.format(world_name, RACETRACK_MODEL_NAME)
        )

    def _parse_world_includes(self, world_name):
        '''Return a list of raw ``<include>`` XML strings for the track
        models in ``<world>.world``, skipping sun / ground_plane. Returns an
        empty list if the file is missing or cannot be parsed.'''
        path = self.world_file(world_name)
        if not os.path.isfile(path):
            return []
        try:
            tree = ET.parse(path)
        except ET.ParseError as ex:
            LOG.warning("Could not parse %s, falling back to default "
                        "include wrapper: %s", path, ex)
            return []
        out = []
        for inc in tree.iter("include"):
            uri = inc.findtext("uri", default="")
            model = uri.rsplit("/", 1)[-1] if uri else ""
            if model in _NON_TRACK_INCLUDE_MODELS:
                continue
            out.append("  " + ET.tostring(inc, encoding="unicode").strip())
        return out

    # ------------------------------------------------------------------
    # Public swap primitives (called by DeepRacerEnv.set_world)
    # ------------------------------------------------------------------

    def delete_track(self):
        '''Delete every live track model. Returns the deleted names.'''
        self._ensure_services()
        deleted = []
        for name in self._track_model_names():
            try:
                resp = self._delete_srv(name)
                if getattr(resp, "success", True):
                    deleted.append(name)
                else:
                    LOG.warning("delete_model(%s) reported failure: %s",
                                name, getattr(resp, "status_message", ""))
            except Exception as ex:  # noqa: BLE001 - logged, swap continues
                LOG.warning("delete_model(%s) raised: %s", name, ex)
        LOG.info("WorldSwapper deleted track model(s): %s", deleted)
        return deleted

    def spawn_track(self, world_name):
        '''Spawn *world_name*'s track mesh via the include-wrapper SDF.'''
        self._ensure_services()
        sdf = self._include_sdf(world_name)
        # Tracks are authored at the world origin; the <include> blocks carry
        # their own relative <pose> tags, so spawn the model root at identity.
        # (rospy serialisation rejects a None Pose, hence an explicit one.)
        resp = self._spawn_srv(
            RACETRACK_MODEL_NAME,   # model_name
            sdf,                    # model_xml
            '',                     # robot_namespace
            Pose(),                 # initial_pose (identity; SDF carries poses)
            '',                     # reference_frame
        )
        if not getattr(resp, "success", True):
            raise RuntimeError(
                "spawn_sdf_model for world {!r} failed: {}".format(
                    world_name, getattr(resp, "status_message", "")))
        LOG.info("WorldSwapper spawned track for world %r", world_name)

    def confirm_track_present(self, timeout=10.0):
        '''Block until at least one ``racetrack`` model is registered in
        Gazebo (or *timeout* elapses). Returns ``True`` on success.'''
        deadline = rospy.get_time() + timeout
        while rospy.get_time() < deadline:
            if self._track_model_names():
                return True
            rospy.sleep(0.1)
        return bool(self._track_model_names())

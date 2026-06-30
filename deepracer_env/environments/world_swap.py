'''Runtime track (world) swapping for :class:`DeepRacerEnv`.

This module owns the *Gazebo-side* half of a hot track swap: deleting the
currently-rendered track mesh and spawning a different one, without
restarting the ``gzserver`` process. It is deliberately free of any
:class:`~deepracer_env.track_geom.track_data.TrackData` / controller logic
— that bookkeeping lives in :meth:`DeepRacerEnv.set_world` — so this class
stays a thin, testable wrapper over the simulator seam
(:func:`deepracer_env.runtime.get_sim_control`).

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

Mechanism note (ROS 2 / seam port)
----------------------------------
The original ROS 1 implementation talked to ``gazebo_ros`` directly through
``rospy.ServiceProxy`` for ``SpawnModel`` / ``DeleteModel`` /
``GetWorldProperties``. That plumbing has been replaced by the shared
:class:`~deepracer_env.sim_control.interface.SimControl` seam:
``spawn_entity`` / ``delete_entity`` / ``list_entities``. The seam already
distinguishes "the simulator died" (:class:`SimControlDead`) from "the call
was rejected" (:class:`SimControlError`), which is exactly the
intermittent-gzserver-segfault case this class must turn into a clean,
catchable :class:`WorldSwapError`.
'''
import logging
import os
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from deepracer_env.runtime import get_sim_control
from deepracer_env.sim_control.interface import SimControlError, SimControlDead
from deepracer_env.sim_control.types import Pose, IDENTITY_POSE

from deepracer_env.log_handler.logger import Logger
from deepracer_env.track_geom.constants import RACETRACK_MODEL_NAME

LOG = Logger(__name__, logging.INFO).get_logger()

_SIM_PKG = "deepracer_simulation_environment"


class WorldSwapError(RuntimeError):
    '''Raised when the Gazebo side of a track swap fails unrecoverably — most
    importantly when ``gzserver`` dies mid-swap (an intermittent Gazebo
    segfault on ``delete_model`` of a mesh model). Callers that loop swaps
    (training rotation) should catch this, checkpoint, and restart the sim
    container rather than spinning on a dead simulator.'''

# Includes that are part of the scene scaffolding, not the track itself.
_NON_TRACK_INCLUDE_MODELS = ("sun", "ground_plane")


class WorldSwapper(object):
    '''Deletes the live track mesh and spawns a different one in-place.

    One instance is created lazily by :class:`DeepRacerEnv`. The shared
    :class:`~deepracer_env.sim_control.interface.SimControl` handle is fetched
    on first use so importing this module never requires a live ROS graph.
    '''

    def __init__(self):
        # The sim-control handle is resolved lazily (see :meth:`_sim`) so that
        # constructing a WorldSwapper never spins up a simulator connection.
        self._sim_control = None
        # routes/, models/ and worlds/ are all installed under the package
        # share dir (see simulation/.../CMakeLists.txt). ROS 2's ament index
        # replaces ROS 1's ``rospkg.RosPack().get_path(...)``.
        self._pkg_path = get_package_share_directory(_SIM_PKG)

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
    # Simulator seam plumbing
    # ------------------------------------------------------------------

    def _sim(self):
        '''Return the shared :class:`SimControl`, resolving it on first use.

        Replaces the ROS 1 ``_ensure_services`` lazy-``ServiceProxy`` dance.
        The seam itself is the fail-fast equivalent of a plain ``ServiceProxy``
        (NOT the retry-then-exit ``ServiceProxyWrapper``): a dead simulator
        surfaces as a catchable :class:`SimControlDead`, which a looped caller
        wants so it can recover instead of hanging.'''
        if self._sim_control is None:
            self._sim_control = get_sim_control()
        return self._sim_control

    def gazebo_alive(self, timeout=3.0):
        '''Return True iff the simulator still answers a query.

        Used to turn an intermittent gzserver segfault during a swap into a
        clean, catchable :class:`WorldSwapError` instead of a retry storm. The
        *timeout* arg is kept for API compatibility; liveness is now probed by
        attempting a single ``list_entities()`` through the seam.'''
        try:
            self._sim().list_entities()
            return True
        except Exception:  # noqa: BLE001
            return False

    def current_model_names(self):
        '''Return the list of model names currently present in Gazebo.'''
        return list(self._sim().list_entities())

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
        '''Delete every live track model, then confirm it is gone.

        Raises :class:`WorldSwapError` if the simulator dies during the delete
        (the intermittent Gazebo segfault on mesh ``delete_model``, surfaced by
        the seam as :class:`SimControlDead`) so the caller can recover instead
        of hanging on a dead simulator.'''
        deleted = []
        for name in self._track_model_names():
            try:
                if self._sim().delete_entity(name):
                    deleted.append(name)
                else:
                    LOG.warning("delete_entity(%s) reported failure", name)
            except SimControlDead as ex:
                # The classic symptom of gzserver segfaulting on a mesh delete:
                # the seam reports the simulator is gone. Unrecoverable.
                raise WorldSwapError(
                    "gzserver died while deleting track model {!r} "
                    "(intermittent Gazebo segfault on delete_model): "
                    "{}".format(name, ex))
            except SimControlError as ex:
                # The call was rejected but the simulator may still be alive;
                # double-check liveness before deciding this is fatal.
                if not self.gazebo_alive():
                    raise WorldSwapError(
                        "gzserver died while deleting track model {!r}: "
                        "{}".format(name, ex))
                LOG.warning("delete_entity(%s) raised but gazebo alive: %s",
                            name, ex)
        # The old track must be fully gone before a same-named 'racetrack' is
        # spawned — a lingering duplicate corrupts Gazebo's model list.
        if not self.confirm_track_absent():
            LOG.warning("track model(s) still present after delete: %s",
                        self._track_model_names())
        LOG.info("WorldSwapper deleted track model(s): %s", deleted)
        return deleted

    def spawn_track(self, world_name):
        '''Spawn *world_name*'s track mesh via the include-wrapper SDF.'''
        sdf = self._include_sdf(world_name)
        # Tracks are authored at the world origin; the <include> blocks carry
        # their own relative <pose> tags, so spawn the model root at identity.
        try:
            self._sim().spawn_entity(
                RACETRACK_MODEL_NAME,   # name
                sdf,                    # sdf (identity root; SDF carries poses)
                pose=IDENTITY_POSE,
            )
        except SimControlDead as ex:
            raise WorldSwapError(
                "gzserver died while spawning track for world {!r}: "
                "{}".format(world_name, ex))
        except SimControlError as ex:
            raise RuntimeError(
                "spawn for world {!r} failed: {}".format(world_name, ex))
        LOG.info("WorldSwapper spawned track for world %r", world_name)

    def spawn_track_instance(self, world_name, model_name, offset=(0.0, 0.0)):
        '''Spawn *world_name*'s track as a uniquely-named model whose root sits at
        the world (dx, dy) *offset* — for separated multi-car track instances.
        Each car drives its own track copy far from the others, so cars never see
        or collide with each other. The TrackData for that car is shifted by the
        same offset (``TrackData.create(world_name, offset)``).'''
        # The offset MUST go inside the <include>'s <pose> — the include carries
        # the track's own (origin) pose, which overrides the spawn pose, so
        # passing the offset there alone leaves the mesh at 0,0.
        ox, oy = float(offset[0]), float(offset[1])
        sdf = (
            '<?xml version="1.0"?>\n<sdf version="1.6">\n'
            '  <include>\n'
            '    <uri>model://models/{}</uri>\n'
            '    <name>{}</name>\n'
            '    <pose>{} {} 0 0 0 0</pose>\n'
            '  </include>\n</sdf>\n'.format(world_name, model_name, ox, oy))
        try:
            self._sim().spawn_entity(model_name, sdf, pose=Pose.at(ox, oy))
        except SimControlDead as ex:
            raise WorldSwapError(
                "gzserver died while spawning track instance {!r}: {}".format(
                    model_name, ex))
        except SimControlError as ex:
            raise RuntimeError("spawn track instance {!r} failed: {}".format(
                model_name, ex))
        LOG.info("WorldSwapper spawned track instance %r at offset %s", model_name, offset)

    def confirm_track_present(self, timeout=10.0):
        '''Block (wall-clock) until at least one ``racetrack`` model is
        registered in Gazebo (or *timeout* elapses). Uses ``time`` not
        ``rospy.sleep`` because the sim clock is frozen during the swap.'''
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._track_model_names():
                return True
            time.sleep(0.1)
        return bool(self._track_model_names())

    def confirm_track_absent(self, timeout=10.0):
        '''Block (wall-clock) until no ``racetrack`` model remains in Gazebo
        (or *timeout* elapses). Returns ``True`` once the track is gone.'''
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self._track_model_names():
                return True
            time.sleep(0.1)
        return not self._track_model_names()

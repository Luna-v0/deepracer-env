'''Static-obstacle spawn / respawn / teardown for D1 Object Avoidance.

Uses the standard Gazebo services ``/gazebo/spawn_sdf_model`` and
``/gazebo/delete_model`` (no custom plugin required) and registers each
spawned obstacle with ``TrackData`` so the existing collision detection
and ``get_object_reward_params`` light up automatically.
'''
import logging
import os
import threading
from typing import List, Tuple

import numpy as np
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

from deepracer_env.log_handler.logger import Logger
from deepracer_env.object_avoidance.config import (
    ObjectAvoidanceConfig,
    PLACEMENT_CALLABLE, PLACEMENT_FIXED, PLACEMENT_RANDOM,
)
from deepracer_env.object_avoidance import placement as placement_strategies  # noqa: E402
from deepracer_env.track_geom.constants import (
    ObstacleDimensions, SPAWN_SDF_MODEL, DELETE_MODEL,
)
from deepracer_env.track_geom.utils import euler_to_quaternion


LOG = Logger(__name__, logging.INFO).get_logger()


def _default_sdf_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, 'sdf', 'obstacle_box.sdf')


class ObstacleManager(object):
    '''Owns the lifecycle of D1 static obstacles inside Gazebo + ``TrackData``.

    The manager is environment-scoped: one instance per ``DeepRacerEnv``.
    On ``respawn`` it deletes any previously-spawned obstacles before
    placing the new ones, so the number of obstacles in the world stays
    bounded.
    '''

    def __init__(self, cfg: ObjectAvoidanceConfig, track_data):
        self._cfg = cfg
        self._track_data = track_data
        self._sdf_text = self._read_sdf(cfg.obstacle_sdf_path or _default_sdf_path())
        self._spawned: List[str] = []
        self._lock = threading.Lock()
        self._spawn_srv = None
        self._delete_srv = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def respawn(self, np_random: np.random.Generator) -> List[Tuple[str, Tuple[float, float]]]:
        '''Delete any previously-spawned obstacles, then place ``n_obstacles``
        new ones according to the configured placement strategy.

        Returns a list of ``(name, (x, y))`` tuples for logging.
        '''
        with self._lock:
            self._teardown_locked()
            positions = self._sample_positions(np_random)
            placed: List[Tuple[str, Tuple[float, float]]] = []
            for i, (x, y, yaw) in enumerate(positions):
                name = '{}_{}'.format(self._cfg.name_prefix, i)
                pose = self._make_pose(x, y, yaw)
                try:
                    self._call_spawn(name, pose)
                except Exception as ex:
                    LOG.warning('Failed to spawn %s at (%.2f, %.2f): %s', name, x, y, ex)
                    continue
                # Register with TrackData so the bbox collision + frustum +
                # reward-param machinery treats this obstacle as live.
                self._track_data.initialize_object(
                    name=name,
                    initial_pose=pose,
                    object_dimensions=ObstacleDimensions.BOX_OBSTACLE_DIMENSION,
                )
                self._spawned.append(name)
                placed.append((name, (float(x), float(y))))
            LOG.info('ObstacleManager respawned %d obstacle(s): %s',
                     len(placed), [p[0] for p in placed])
            return placed

    def teardown(self) -> None:
        '''Delete every spawned obstacle from Gazebo + ``TrackData``.

        Safe to call repeatedly; safe to call after Gazebo is gone (failures
        in the ROS service call are swallowed and logged).
        '''
        with self._lock:
            self._teardown_locked()

    @property
    def spawned_names(self) -> List[str]:
        '''Names of currently-spawned obstacles.'''
        with self._lock:
            return list(self._spawned)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _sample_positions(self, np_random):
        if self._cfg.placement == PLACEMENT_FIXED:
            return placement_strategies.fixed(
                self._cfg.n_obstacles, np_random, self._track_data,
                fixed_positions=self._cfg.fixed_positions,
                min_spacing_m=self._cfg.min_spacing_m,
                lane=self._cfg.lane,
            )
        if self._cfg.placement == PLACEMENT_CALLABLE:
            return placement_strategies.from_callable(
                self._cfg.n_obstacles, np_random, self._track_data,
                placement_fn=self._cfg.placement_fn,
                min_spacing_m=self._cfg.min_spacing_m,
                lane=self._cfg.lane,
            )
        if self._cfg.placement == PLACEMENT_RANDOM:
            return placement_strategies.random_on_waypoints(
                self._cfg.n_obstacles, np_random, self._track_data,
                min_spacing_m=self._cfg.min_spacing_m,
                lane=self._cfg.lane,
                max_attempts=self._cfg.max_placement_attempts,
            )
        raise ValueError('Unknown placement strategy: {!r}'.format(self._cfg.placement))

    def _teardown_locked(self):
        if not self._spawned:
            return
        for name in self._spawned:
            try:
                self._call_delete(name)
            except Exception as ex:
                LOG.warning('Failed to delete %s from Gazebo: %s', name, ex)
            self._track_data.remove_object(name)
        self._spawned = []

    # Half-height of the bundled obstacle_box.sdf (0.40 m tall) so the box
    # rests on the track surface (z=0). If a user supplies a custom SDF with
    # a different height, override via ObjectAvoidanceConfig.obstacle_sdf_path
    # AND make sure that SDF positions its visual/collision around z=0.
    _BOX_HALF_HEIGHT = 0.20

    @staticmethod
    def _make_pose(x: float, y: float, yaw: float) -> Pose:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = ObstacleManager._BOX_HALF_HEIGHT
        q = euler_to_quaternion(yaw=float(yaw))
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    @staticmethod
    def _read_sdf(path: str) -> str:
        with open(path, 'r') as fh:
            return fh.read()

    def _ensure_services(self):
        if self._spawn_srv is None:
            rospy.wait_for_service(SPAWN_SDF_MODEL, timeout=30.0)
            self._spawn_srv = rospy.ServiceProxy(SPAWN_SDF_MODEL, SpawnModel)
        if self._delete_srv is None:
            rospy.wait_for_service(DELETE_MODEL, timeout=30.0)
            self._delete_srv = rospy.ServiceProxy(DELETE_MODEL, DeleteModel)

    def _call_spawn(self, name: str, pose: Pose):
        self._ensure_services()
        return self._spawn_srv(
            model_name=name,
            model_xml=self._sdf_text,
            robot_namespace='',
            initial_pose=pose,
            reference_frame='',
        )

    def _call_delete(self, name: str):
        self._ensure_services()
        return self._delete_srv(model_name=name)

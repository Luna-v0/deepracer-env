'''Gymnasium environment for the AWS DeepRacer Gazebo simulation.

Minimal usage
-------------
Pass a reward function and the environment builds everything else::

    from deepracer_env.environments.deepracer_env import DeepRacerEnv

    def my_reward(params: dict) -> float:
        return float(params['progress']) * float(params['speed']) / 4.0

    env = DeepRacerEnv(reward_fn=my_reward)
    obs, info = env.reset()
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())

Customising sensors
-------------------
Pass a list of ``Input`` value strings::

    from deepracer_env.sensors.constants import Input

    env = DeepRacerEnv(
        reward_fn=my_reward,
        sensors=[Input.CAMERA.value, Input.LIDAR.value],
    )

Overriding controller parameters
---------------------------------
Provide a partial ``config`` dict with any :class:`~deepracer_env.agent_ctrl.constants.ConfigParams`
*value* keys you want to change; the rest keep their defaults::

    import deepracer_env.agent_ctrl.constants as ctrl_const

    env = DeepRacerEnv(
        reward_fn=my_reward,
        config={ctrl_const.ConfigParams.NUMBER_OF_RESETS.value: 0},
    )

Full control
------------
Build an :class:`~deepracer_env.agents.agent.Agent` yourself and pass it directly.
In this case ``reward_fn``, ``sensors``, and ``config`` are ignored::

    from deepracer_env.agents.agent import Agent
    from deepracer_env.sensors.composite_sensor import CompositeSensor
    from deepracer_env.agent_ctrl.rollout_agent_ctrl import RolloutCtrl

    sensor = CompositeSensor()
    ctrl   = RolloutCtrl(my_config, my_metrics, is_training=True)
    env    = DeepRacerEnv(agent=Agent(sensor, ctrl))

Action space
------------
A continuous ``Box(2,)`` vector: ``[steering_angle_deg, speed_m_s]``.

* ``steering_angle_deg`` ∈ [−30, 30] — positive turns the car left.
* ``speed_m_s`` ∈ [0.1, 4.0]  — forward speed in m/s.

Observation space
-----------------
A ``gymnasium.spaces.Dict`` whose keys are the active sensor ``Input.value``
strings (e.g. ``"CAMERA"``, ``"LIDAR"``).
Each value is a ``Box`` matching the sensor's output shape.
'''
import logging
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import gymnasium

import deepracer_env.agent_ctrl.constants as ctrl_const
from deepracer_env.environments.constants import (
    LINK_NAMES,
    VELOCITY_TOPICS,
    STEERING_TOPICS,
)
from deepracer_env.sensors.constants import Input
from deepracer_env.constants import SIMAPP_VERSION_5
from deepracer_env.log_handler.logger import Logger

LOG = Logger(__name__, logging.INFO).get_logger()

# ---------------------------------------------------------------------------
# Default action space: [steering_angle_deg, speed_m_s]
# ---------------------------------------------------------------------------
DEFAULT_ACTION_SPACE: gymnasium.spaces.Box = gymnasium.spaces.Box(
    low=np.array([-30.0, 0.1], dtype=np.float32),
    high=np.array([30.0, 4.0], dtype=np.float32),
    dtype=np.float32,
)

# ---------------------------------------------------------------------------
# Default sensors
# ---------------------------------------------------------------------------
DEFAULT_SENSORS: List[str] = [Input.CAMERA.value]

# ---------------------------------------------------------------------------
# Default controller config
# ---------------------------------------------------------------------------
_DEFAULT_CTRL_CONFIG: Dict[str, Any] = {
    ctrl_const.ConfigParams.AGENT_NAME.value:               'racecar',
    ctrl_const.ConfigParams.LINK_NAME_LIST.value:           LINK_NAMES,
    ctrl_const.ConfigParams.VELOCITY_LIST.value:            VELOCITY_TOPICS,
    ctrl_const.ConfigParams.STEERING_LIST.value:            STEERING_TOPICS,
    ctrl_const.ConfigParams.ACTION_SPACE.value:             DEFAULT_ACTION_SPACE,
    ctrl_const.ConfigParams.VERSION.value:                  SIMAPP_VERSION_5,
    ctrl_const.ConfigParams.IS_CONTINUOUS.value:            True,
    ctrl_const.ConfigParams.NUMBER_OF_RESETS.value:         5,
    ctrl_const.ConfigParams.PENALTY_SECONDS.value:          2.0,
    ctrl_const.ConfigParams.NUMBER_OF_TRIALS.value:         1000,
    ctrl_const.ConfigParams.RACE_TYPE.value:                'TIME_TRIAL',
    ctrl_const.ConfigParams.COLLISION_PENALTY.value:        2.0,
    ctrl_const.ConfigParams.OFF_TRACK_PENALTY.value:        2.0,
    ctrl_const.ConfigParams.IMMOBILIZED_PENALTY.value:      2.0,
    ctrl_const.ConfigParams.REVERSE_PENALTY.value:          2.0,
    ctrl_const.ConfigParams.CHANGE_START.value:             True,
    ctrl_const.ConfigParams.ALT_DIR.value:                  False,
    ctrl_const.ConfigParams.ROUND_ROBIN_ADVANCE_DIST.value: 0.05,
    ctrl_const.ConfigParams.START_POSITION_OFFSET.value:    0.0,
    ctrl_const.ConfigParams.DONE_CONDITION.value:           any,
    ctrl_const.ConfigParams.PARK_POSITIONS.value:           [(0.0, 0.0, 0.0)],
}


class _NoopMetrics:
    '''No-op metrics implementation used when no metrics backend is provided.'''
    def reset(self):                         pass
    def append_episode_metrics(self, **kw):  pass
    def upload_episode_metrics(self):        pass
    def upload_step_metrics(self, _):        pass
    def update_mp4_video_metrics(self, _):   pass


def _build_agent(
    reward_fn: Callable[[dict], float],
    sensors: List[str],
    config: Optional[Dict[str, Any]],
    is_training: bool,
    extra_ctrl_config: Optional[Dict[str, Any]] = None,
):
    '''Build a default :class:`~deepracer_env.agents.agent.Agent` from its component parts.

    This is the wiring that happens inside ROS/Gazebo — hidden from the user
    unless they need to customise it.
    '''
    # Initialise the ROS node if one hasn't been started yet.
    import rospy
    if not rospy.core.is_initialized():
        rospy.init_node('deepracer_env', anonymous=True)

    # Deferred imports keep the top-level import cost low and avoid
    # circular dependencies at module load time.
    from deepracer_env.agents.agent import Agent
    from deepracer_env.sensors.composite_sensor import CompositeSensor
    from deepracer_env.sensors.sensors_rollout import SensorFactory
    from deepracer_env.agent_ctrl.rollout_agent_ctrl import RolloutCtrl

    racecar_name = 'racecar'

    # Build composite sensor
    composite_sensor = CompositeSensor()
    for sensor_type in sensors:
        composite_sensor.add_sensor(
            SensorFactory.create_sensor(racecar_name, sensor_type, {})
        )

    # Merge user overrides into the default config. Order matters:
    #   defaults < env-level (object_avoidance, etc.) < user `config` overrides
    ctrl_config = {**_DEFAULT_CTRL_CONFIG}
    ctrl_config[ctrl_const.ConfigParams.REWARD.value] = reward_fn
    if extra_ctrl_config:
        ctrl_config.update(extra_ctrl_config)
    if config:
        ctrl_config.update(config)

    ctrl = RolloutCtrl(ctrl_config, metrics=_NoopMetrics(), is_training=is_training)
    return Agent(composite_sensor, ctrl)


class DeepRacerEnv(gymnasium.Env):
    '''Gymnasium-compatible environment wrapping the DeepRacer Gazebo simulator.

    See the module docstring for usage examples.

    Args:
        reward_fn: Callable ``(params: dict) -> float``.  Required unless
            ``agent`` is provided.  ``params`` contains all keys from
            :class:`~deepracer_env.agent_ctrl.constants.RewardParam`.
        sensors: Active sensor types as ``Input.value`` strings.
            Defaults to :data:`DEFAULT_SENSORS` (``["CAMERA"]``).
        config: Partial override dict for the controller.  Keys are the
            string *values* of :class:`~deepracer_env.agent_ctrl.constants.ConfigParams`.
            Any key not present keeps its default value.
        agent: Fully-initialised :class:`~deepracer_env.agents.agent.Agent`.
            When provided, ``reward_fn``, ``sensors``, and ``config`` are
            ignored — you are in full control.
        action_space: Override the action space.  Defaults to
            :data:`DEFAULT_ACTION_SPACE`.
        is_training: Passed to the default controller; controls whether
            the start position advances between episodes.  Ignored when
            ``agent`` is provided explicitly.
    '''

    metadata = {'render_modes': []}

    def __init__(
        self,
        reward_fn: Optional[Callable[[dict], float]] = None,
        sensors: Optional[List[str]] = None,
        config: Optional[Dict[str, Any]] = None,
        agent=None,
        action_space: Optional[gymnasium.spaces.Space] = None,
        is_training: bool = True,
        object_avoidance: Optional[Any] = None,
    ) -> None:
        super().__init__()

        # Surface OA config knobs into the controller config dict before
        # building the agent so RolloutCtrl and CrashResetRule see them at
        # construction time.
        self._oa_cfg = object_avoidance
        extra_ctrl_config: Dict[str, Any] = {}
        if self._oa_cfg is not None:
            extra_ctrl_config[ctrl_const.ConfigParams.OBJECT_AVOIDANCE_ENABLED.value] = \
                bool(self._oa_cfg.enabled)
            extra_ctrl_config[ctrl_const.ConfigParams.TERMINATE_ON_COLLISION.value] = \
                bool(self._oa_cfg.terminate_on_collision)

        if agent is not None:
            self._agent = agent
        else:
            if reward_fn is None:
                raise ValueError(
                    'Provide either reward_fn (to use the default agent) '
                    'or agent (to supply your own).'
                )
            self._agent = _build_agent(
                reward_fn=reward_fn,
                sensors=sensors if sensors is not None else DEFAULT_SENSORS,
                config=config,
                is_training=is_training,
                extra_ctrl_config=extra_ctrl_config,
            )

        # Construct the ObstacleManager *after* the agent so the TrackData
        # singleton is already initialised by the controller.
        self._obstacle_manager = None
        if self._oa_cfg is not None and self._oa_cfg.enabled:
            from deepracer_env.object_avoidance import ObstacleManager
            from deepracer_env.track_geom.track_data import TrackData
            self._obstacle_manager = ObstacleManager(
                self._oa_cfg, TrackData.get_instance())

        self.action_space: gymnasium.spaces.Space = (
            action_space if action_space is not None else DEFAULT_ACTION_SPACE
        )
        obs_space = self._agent.get_observation_space()
        if obs_space is None:
            raise ValueError(
                'Agent has no sensor configured — cannot determine observation_space.'
            )
        self.observation_space: gymnasium.spaces.Dict = obs_space

        # Store last step info for diagnostics
        self._last_step_info: Dict[str, Any] = {}

        # Runtime world-swap state. The WorldSwapper (Gazebo spawn/delete
        # plumbing) is created lazily on the first set_world() call so that
        # importing / constructing the env never requires a live ROS graph
        # beyond what the agent already needs. set_world() is only valid
        # between episodes, so it is gated on at least one reset() having run.
        self._world_swapper = None
        self._has_reset: bool = False
        self._pause_srv = None
        self._unpause_srv = None

    # ------------------------------------------------------------------
    # Core gymnasium.Env interface
    # ------------------------------------------------------------------

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None,
    ) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        '''Reset the environment to the beginning of a new episode.

        Returns:
            observation (dict): Initial sensor observation.
            info (dict): Auxiliary diagnostic information (empty by default).
        '''
        super().reset(seed=seed)
        LOG.debug('DeepRacerEnv.reset() called')
        # Respawn obstacles *before* the agent reset so the controller's
        # start-pose computation (which scans TrackData.object_poses) sees
        # the new layout.
        placed = []
        if self._obstacle_manager is not None:
            placed = self._obstacle_manager.respawn(self.np_random)
        obs = self._agent.reset_agent()
        info: Dict[str, Any] = {
            'objects_location': [list(xy) for _, xy in placed],
            'is_crashed': False,
        }
        self._last_step_info = info
        self._has_reset = True
        return obs, info

    def step(
        self, action: np.ndarray
    ) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
        '''Run one environment step.

        Publishes *action* to Gazebo via ROS, waits for the next sensor
        observation, and returns the standard gymnasium 5-tuple.

        Args:
            action (np.ndarray): ``[steering_angle_deg, speed_m_s]``.

        Returns:
            observation (dict): Sensor observation after the action.
            reward (float): Scalar reward from the reward function.
            terminated (bool): ``True`` if the episode ended naturally
                (e.g. lap complete, off-track, time-up).
            truncated (bool): Always ``False`` (no external time limit
                beyond what the reset-rules manager handles).
            info (dict): Step metrics forwarded from the controller.
        '''
        # 1. Publish command to Gazebo
        self._agent.send_action(action)
        # 2. Advance internal state (read car position, compute metrics)
        agents_info_map = self._agent.update_agent(action)
        # 3. Evaluate the action (compute reward, termination)
        obs, reward, done = self._agent.judge_action(action, agents_info_map)
        # Surface the always-populated object / crash flags into info so
        # wrappers (D2, D3) don't have to dig into the controller.
        info: Dict[str, Any] = dict(agents_info_map) if isinstance(agents_info_map, dict) else {}
        ctrl = getattr(self._agent, 'ctrl', None)
        reward_params = getattr(ctrl, 'reward_params', None) if ctrl is not None else None
        if reward_params is not None:
            info['objects_location'] = list(reward_params.get('objects_location', []))
            info['is_crashed'] = bool(reward_params.get('is_crashed', False))
            info['is_offtrack'] = bool(reward_params.get('is_offtrack', False))
            info['closest_objects'] = list(reward_params.get('closest_objects', [-1, -1]))
        self._last_step_info = info
        return obs, float(reward), bool(done), False, info

    def close(self) -> None:
        '''Stop the car and release resources.'''
        LOG.info('DeepRacerEnv.close() — setting car speed to zero')
        try:
            self._agent.send_action(np.array([0.0, 0.0], dtype=np.float32))
        except Exception:
            pass
        if self._obstacle_manager is not None:
            try:
                self._obstacle_manager.teardown()
            except Exception as ex:
                LOG.warning('Obstacle teardown failed during close(): %s', ex)

    def render(self) -> None:  # type: ignore[override]
        '''Rendering is handled externally by Gazebo / RViz.'''
        pass

    # ------------------------------------------------------------------
    # Runtime world swap
    # ------------------------------------------------------------------

    def set_world(self, world_name: str) -> None:
        '''Swap the rendered track to *world_name* at runtime, without
        restarting Gazebo.

        **Between-episodes contract.** This may only be called *between*
        episodes: after :meth:`reset` has run at least once and after the
        previous episode has terminated (i.e. before the next :meth:`reset`).
        It is **not** safe to call mid-episode — it deletes and respawns the
        track model, rebuilds the :class:`TrackData` geometry the reward and
        reset rules depend on, and teleports the car. Calling it before the
        first :meth:`reset` raises :class:`RuntimeError`.

        The swap, in order (all under ``pause_physics`` so the car never
        free-falls onto a missing track):

        1. Validate the target world's assets exist.
        2. Pause physics.
        3. Tear down object-avoidance obstacles against the *old* ``TrackData``.
        4. Delete the live ``racetrack`` model(s).
        5. Spawn the new track via an ``<include>`` wrapper SDF (Gazebo
           resolves the mesh paths, sidestepping SDF-filename inconsistencies).
        6. Rebuild ``TrackData`` + ``FrustumManager`` and rebind every cached
           reference (controller, reset rules, obstacle manager).
        7. Reset the car onto the new start line.
        8. Unpause physics, drain stale sensor frames, wait for a fresh one.

        Idempotent: ``set_world(current_world)`` performs a clean rebuild
        rather than crashing.

        Args:
            world_name: Target world (e.g. ``"arctic_pro"``). Must have both
                ``routes/<world>.npy`` and ``models/<world>/`` installed in the
                ``deepracer_simulation_environment`` package.

        Raises:
            RuntimeError: if called before the first :meth:`reset`.
            ValueError: if the target world's assets are missing.
        '''
        import rospy
        from std_srvs.srv import Empty
        from deepracer_env.environments.world_swap import WorldSwapper
        from deepracer_env.rospy_wrappers import ServiceProxyWrapper
        from deepracer_env.track_geom.constants import (
            PAUSE_PHYSICS, UNPAUSE_PHYSICS,
        )
        from deepracer_env.track_geom.track_data import TrackData
        from deepracer_env.cameras.frustum_manager import FrustumManager

        if not self._has_reset:
            raise RuntimeError(
                'set_world() is only valid between episodes; call reset() at '
                'least once before swapping the world.')

        if self._world_swapper is None:
            self._world_swapper = WorldSwapper()
        swapper = self._world_swapper

        # Fail fast on a bad world name BEFORE touching Gazebo, so a typo can
        # never leave the simulation track-less.
        swapper.validate(world_name)

        current_world = rospy.get_param('WORLD_NAME', None)
        LOG.info('set_world(%r): swapping from %r', world_name, current_world)

        # Lazily create the pause/unpause proxies.
        if self._pause_srv is None:
            rospy.wait_for_service(PAUSE_PHYSICS, timeout=30.0)
            self._pause_srv = ServiceProxyWrapper(PAUSE_PHYSICS, Empty)
        if self._unpause_srv is None:
            rospy.wait_for_service(UNPAUSE_PHYSICS, timeout=30.0)
            self._unpause_srv = ServiceProxyWrapper(UNPAUSE_PHYSICS, Empty)

        try:
            self._pause_srv()

            # 1. Tear down OA obstacles while the OLD TrackData is still live
            #    (teardown both deletes the Gazebo models and unregisters them
            #    from the current — correct — TrackData).
            if self._obstacle_manager is not None:
                try:
                    self._obstacle_manager.teardown()
                except Exception as ex:  # noqa: BLE001
                    LOG.warning('Obstacle teardown during set_world failed: %s', ex)

            # 2. Delete the live track, spawn the new one.
            swapper.delete_track()
            swapper.spawn_track(world_name)
            if not swapper.confirm_track_present():
                raise RuntimeError(
                    'set_world(%r): new track model did not appear in Gazebo'
                    % world_name)

            # 3. Rebuild track geometry. Clearing the singleton + WORLD_NAME and
            #    re-getting forces a fresh load of routes/<world>.npy.
            TrackData._instance_ = None
            rospy.set_param('WORLD_NAME', world_name)
            TrackData.get_instance()
            # FrustumManager caches camera-frustum geometry keyed on the old
            # track; drop it so object_in_camera recomputes for the new world.
            FrustumManager._instance_ = None

            # 4. Rebind cached references. The controller holds the old
            #    TrackData + derived start geometry and re-registers the agent;
            #    the reset rules are rebuilt inside reset_track_data().
            self._agent.reset_track_data()

            # 5. Rebind the obstacle manager to the new TrackData. There is no
            #    setter, so reconstruct it preserving the original config. The
            #    next reset() will respawn obstacles on the NEW track.
            if self._oa_cfg is not None and self._oa_cfg.enabled:
                from deepracer_env.object_avoidance import ObstacleManager
                self._obstacle_manager = ObstacleManager(
                    self._oa_cfg, TrackData.get_instance())

            # 6. Put the car on the new start line (reuses the controller's
            #    existing blocking SetModelState reset path).
            self._agent.ctrl.reset_agent()
        finally:
            # Always unpause, even if the swap blew up half-way, so the sim is
            # never left frozen.
            try:
                self._unpause_srv()
            except Exception as ex:  # noqa: BLE001
                LOG.error('Failed to unpause physics after set_world: %s', ex)

        # 7. Discard frames buffered against the old track and block for a
        #    fresh one so the next observation reflects the new world.
        self._agent.drain_sensors()
        LOG.info('set_world(%r): swap complete', world_name)

    # ------------------------------------------------------------------
    # Convenience helpers
    # ------------------------------------------------------------------

    @property
    def agent(self):
        '''The underlying :class:`~deepracer_env.agents.agent.Agent` instance.'''
        return self._agent

    @property
    def last_step_info(self) -> Dict[str, Any]:
        '''Diagnostic info dict from the most recent :meth:`step` call.'''
        return self._last_step_info

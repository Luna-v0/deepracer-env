'''Register the canonical SafetyDeepRacer-* IDs with gymnasium and (when
available) safety_gymnasium.

Two IDs are registered:

* ``SafetyDeepRacer-OffTrack-v0`` — safety-0; cost on off-track. Inner
  ``DeepRacerEnv`` has Object Avoidance disabled by default.
* ``SafetyDeepRacer-Collision-v0`` — safety-1; cost on collision (and
  optionally off-track, depending on ``combine``). Inner env has
  ``ObjectAvoidanceConfig(terminate_on_collision=False)`` so the
  per-step crash signal stays alive.

Both IDs accept any kwargs that :class:`DeepRacerEnv` accepts (``reward_fn``,
``sensors``, ``config``, ...) and any kwarg specific to
:class:`SafetyDeepRacerEnv` prefixed with ``safety_``.
'''
import logging
from typing import Any, Dict

from deepracer_env.log_handler.logger import Logger

LOG = Logger(__name__, logging.INFO).get_logger()


_OFFTRACK_ID = 'SafetyDeepRacer-OffTrack-v0'
_COLLISION_ID = 'SafetyDeepRacer-Collision-v0'


def _split_safety_kwargs(kwargs: Dict[str, Any]) -> Dict[str, Any]:
    '''Pop every ``safety_<name>`` kwarg into ``<name>`` for the wrapper.'''
    out: Dict[str, Any] = {}
    for k in list(kwargs):
        if k.startswith('safety_'):
            out[k[len('safety_'):]] = kwargs.pop(k)
    return out


def _make_offtrack(**kwargs) -> Any:
    from deepracer_env.environments.deepracer_env import DeepRacerEnv
    from deepracer_env.safety.safety_env import SafetyDeepRacerEnv
    safety_kwargs = _split_safety_kwargs(kwargs)
    safety_kwargs.setdefault('level', 'safety-0')
    return SafetyDeepRacerEnv(DeepRacerEnv(**kwargs), **safety_kwargs)


def _make_collision(**kwargs) -> Any:
    from deepracer_env.environments.deepracer_env import DeepRacerEnv
    from deepracer_env.object_avoidance import ObjectAvoidanceConfig
    from deepracer_env.safety.safety_env import SafetyDeepRacerEnv
    safety_kwargs = _split_safety_kwargs(kwargs)
    safety_kwargs.setdefault('level', 'safety-1')
    safety_kwargs.setdefault('combine', 'max')
    # Default to enabled OA with terminate_on_collision=False so the cost
    # signal is hot every step the bbox overlaps.
    if 'object_avoidance' not in kwargs:
        kwargs['object_avoidance'] = ObjectAvoidanceConfig(
            enabled=True, terminate_on_collision=False)
    return SafetyDeepRacerEnv(DeepRacerEnv(**kwargs), **safety_kwargs)


def register_safety_envs() -> None:
    '''Register both safety IDs with gymnasium (and safety_gymnasium if
    importable). Safe to call repeatedly — duplicate registrations are
    swallowed.'''
    import gymnasium

    def _maybe_register(reg_fn, env_id, entry_point):
        try:
            reg_fn(id=env_id, entry_point=entry_point)
        except Exception as ex:
            # Most likely already registered. Log at DEBUG to avoid noise.
            LOG.debug('register %s skipped: %s', env_id, ex)

    _maybe_register(gymnasium.register, _OFFTRACK_ID,
                    'deepracer_env.safety.registration:_make_offtrack')
    _maybe_register(gymnasium.register, _COLLISION_ID,
                    'deepracer_env.safety.registration:_make_collision')

    try:
        import safety_gymnasium  # noqa: F401
        _maybe_register(safety_gymnasium.register, _OFFTRACK_ID,
                        'deepracer_env.safety.registration:_make_offtrack')
        _maybe_register(safety_gymnasium.register, _COLLISION_ID,
                        'deepracer_env.safety.registration:_make_collision')
    except ImportError:
        LOG.debug('safety_gymnasium not installed — skipping that registry.')

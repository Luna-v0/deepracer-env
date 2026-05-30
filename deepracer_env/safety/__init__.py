'''Safety-Gymnasium compatibility for DeepRacer (D3).

Public API
----------
* :class:`SafetyDeepRacerEnv` — wraps :class:`DeepRacerEnv` and returns the
  6-tuple ``(obs, reward, cost, terminated, truncated, info)`` used by
  constrained-RL algorithms.
* :class:`SafetyToGymnasium` — convert a 6-tuple env to the standard
  5-tuple shape (with ``info['cost']`` carrying the cost stream).
* :class:`GymnasiumToSafety` — inverse adapter for envs that publish cost
  through ``info['cost']``.
* :func:`register_safety_envs` — register the canonical safety_gymnasium
  IDs (``SafetyDeepRacer-OffTrack-v0`` and
  ``SafetyDeepRacer-Collision-v0``). Called automatically when this
  module is imported.

See ``plans/03-safety-gymnasium.md`` for the design.
'''
from deepracer_env.safety.safety_env import SafetyDeepRacerEnv
from deepracer_env.safety.adapters import SafetyToGymnasium, GymnasiumToSafety
from deepracer_env.safety.levels import CostLevel, COMBINE_MODES
from deepracer_env.safety.registration import register_safety_envs

# Best-effort eager registration so ``gym.make("SafetyDeepRacer-OffTrack-v0")``
# works without the user having to import the module explicitly. Swallow
# import errors so the rest of the package stays usable in test
# environments that don't have safety_gymnasium installed yet.
try:
    register_safety_envs()
except Exception:
    pass

__all__ = [
    'SafetyDeepRacerEnv',
    'SafetyToGymnasium', 'GymnasiumToSafety',
    'CostLevel', 'COMBINE_MODES',
    'register_safety_envs',
]

'''Adapters between the 6-tuple safety_gymnasium API and standard
5-tuple gymnasium API.

Use :class:`SafetyToGymnasium` to feed a 6-tuple env into algorithms that
only know the 5-tuple shape (e.g. Stable-Baselines3 PPO) while still
exposing the cost stream through ``info['cost']``. Use
:class:`GymnasiumToSafety` for the inverse — wrap a 5-tuple env that
publishes cost via ``info['cost']`` and surface it as the 6-tuple a
constrained-RL framework expects.
'''
from typing import Any, Dict

import gymnasium as gym


class SafetyToGymnasium(gym.Wrapper):
    '''Convert a 6-tuple ``SafetyDeepRacerEnv`` into a 5-tuple env.

    ``step`` returns ``(obs, reward, terminated, truncated, info)`` with
    the cost stashed under ``info['cost']`` (and a running
    ``info['episode_cost']`` for convenience).
    '''

    def __init__(self, env: gym.Env) -> None:
        super().__init__(env)
        self._episode_cost = 0.0

    def reset(self, **kwargs):
        self._episode_cost = 0.0
        out = self.env.reset(**kwargs)
        obs, info = out
        info = dict(info) if isinstance(info, dict) else {}
        info.setdefault('cost', 0.0)
        info['episode_cost'] = 0.0
        return obs, info

    def step(self, action):
        out = self.env.step(action)
        if len(out) != 6:
            raise TypeError(
                'SafetyToGymnasium expects a 6-tuple env (got {}-tuple). '
                'Wrap a SafetyDeepRacerEnv, not a plain DeepRacerEnv.'
                .format(len(out))
            )
        obs, reward, cost, terminated, truncated, info = out
        info = dict(info) if isinstance(info, dict) else {}
        info['cost'] = float(cost)
        self._episode_cost += float(cost)
        info['episode_cost'] = self._episode_cost
        return obs, float(reward), bool(terminated), bool(truncated), info


class GymnasiumToSafety(gym.Wrapper):
    '''Convert a 5-tuple env that publishes cost via ``info['cost']``
    (e.g. a `SafetyToGymnasium`-wrapped env round-tripping back) into the
    6-tuple safety_gymnasium shape. Missing ``info['cost']`` defaults to
    0 with a single warning per env instance.
    '''

    def __init__(self, env: gym.Env) -> None:
        super().__init__(env)
        self._warned_missing_cost = False

    def reset(self, **kwargs):
        return self.env.reset(**kwargs)

    def step(self, action):
        out = self.env.step(action)
        if len(out) != 5:
            raise TypeError(
                'GymnasiumToSafety expects a 5-tuple env (got {}-tuple).'
                .format(len(out))
            )
        obs, reward, terminated, truncated, info = out
        info = dict(info) if isinstance(info, dict) else {}
        cost = info.get('cost')
        if cost is None:
            if not self._warned_missing_cost:
                import warnings
                warnings.warn(
                    "GymnasiumToSafety: inner env did not set info['cost']; "
                    'falling back to 0. (Future steps silent.)',
                    stacklevel=2,
                )
                self._warned_missing_cost = True
            cost = 0.0
        return obs, float(reward), float(cost), bool(terminated), bool(truncated), info

'''SafetyDeepRacerEnv — 6-tuple wrapper for DeepRacer (D3).

Returns ``(obs, reward, cost, terminated, truncated, info)`` for
constrained-RL algorithms (CPO, PPO-Lag, etc.). The cost stream is
derived from the always-populated ``is_offtrack`` / ``is_crashed`` keys
that D1 added to the info dict.
'''
import logging
import warnings
from typing import Any, Dict, Optional

import gymnasium as gym

from deepracer_env.log_handler.logger import Logger
from deepracer_env.safety.levels import (
    COMPOSITE_TERMS, CostLevel,
    validate_combine, validate_level, validate_weights,
)


LOG = Logger(__name__, logging.INFO).get_logger()


class SafetyDeepRacerEnv(gym.Wrapper):
    '''Wraps a :class:`DeepRacerEnv` (or any 5-tuple env that puts
    ``is_offtrack`` / ``is_crashed`` in ``info``) and emits a 6-tuple
    ``(obs, reward, cost, terminated, truncated, info)``.

    Args:
        env: The inner gymnasium env.
        level: ``"safety-0"`` (off-track only), ``"safety-1"`` (collision +
            optional off-track), or ``"safety-2"`` (composite weighted sum;
            requires ``weights``).
        combine: How to combine the crash and off-track signals at
            ``safety-1``: ``"max"`` (default), ``"add"``, or
            ``"crash-only"`` (ignore off-track entirely).
        terminate_on_cost: If True, the wrapper sets ``terminated=True`` on
            any step where ``cost > 0``. Off by default — constrained-RL
            algorithms typically prefer the cost signal alive over the
            full trajectory.
        weights: Required for ``safety-2``. Dict mapping any subset of
            ``("offtrack", "crash", "near_collision", "steering_jerk")``
            to non-negative floats. No defaults are provided.
        near_collision_threshold_m: Distance to the closest object below
            which the ``near_collision`` cost component fires (only used
            for ``safety-2`` when ``weights["near_collision"]`` is set).
    '''

    def __init__(
        self,
        env: gym.Env,
        *,
        level: str = CostLevel.SAFETY_0.value,
        combine: str = 'max',
        terminate_on_cost: bool = False,
        weights: Optional[Dict[str, float]] = None,
        near_collision_threshold_m: float = 0.5,
    ) -> None:
        super().__init__(env)
        self._level = validate_level(level)
        self._combine = validate_combine(combine)
        self._terminate_on_cost = bool(terminate_on_cost)
        self._weights: Dict[str, float] = {}
        if self._level == CostLevel.SAFETY_2:
            self._weights = validate_weights(weights)
        elif weights is not None:
            LOG.warning('weights= ignored at level %s (only safety-2 uses it).',
                        self._level.value)
        self._near_threshold = float(near_collision_threshold_m)
        self._prev_steering: Optional[float] = None
        self._warn_about_terminate_on_collision()

    # ------------------------------------------------------------------
    # gymnasium.Wrapper interface
    # ------------------------------------------------------------------

    def reset(self, **kwargs):
        self._prev_steering = None
        obs, info = self.env.reset(**kwargs)
        info = dict(info) if isinstance(info, dict) else {}
        info['cost'] = 0.0
        return obs, info

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        info = dict(info) if isinstance(info, dict) else {}
        cost = self._compute_cost(info, action)
        info['cost'] = float(cost)
        if self._terminate_on_cost and cost > 0.0:
            terminated = True
        # Track steering for the next step's jerk computation.
        try:
            self._prev_steering = float(action[0])
        except (TypeError, IndexError):
            self._prev_steering = None
        return obs, float(reward), float(cost), bool(terminated), bool(truncated), info

    # ------------------------------------------------------------------
    # Public accessors (useful for adapters / debuggers)
    # ------------------------------------------------------------------

    @property
    def level(self) -> CostLevel:
        return self._level

    @property
    def combine(self) -> str:
        return self._combine

    @property
    def weights(self) -> Dict[str, float]:
        return dict(self._weights)

    # ------------------------------------------------------------------
    # Cost computation
    # ------------------------------------------------------------------

    def _compute_cost(self, info: Dict[str, Any], action) -> float:
        if self._level == CostLevel.SAFETY_0:
            return 1.0 if info.get('is_offtrack') else 0.0
        if self._level == CostLevel.SAFETY_1:
            crash = 1.0 if info.get('is_crashed') else 0.0
            if self._combine == 'crash-only':
                return crash
            offtrack = 1.0 if info.get('is_offtrack') else 0.0
            if self._combine == 'add':
                return crash + offtrack
            # default 'max'
            return max(crash, offtrack)
        # safety-2 — composite weighted sum
        terms = self._compute_composite_terms(info, action)
        return float(sum(self._weights.get(k, 0.0) * v for k, v in terms.items()))

    def _compute_composite_terms(self, info: Dict[str, Any], action) -> Dict[str, float]:
        terms = {k: 0.0 for k in COMPOSITE_TERMS}
        terms['offtrack'] = 1.0 if info.get('is_offtrack') else 0.0
        terms['crash'] = 1.0 if info.get('is_crashed') else 0.0
        # Near-collision: scaled distance-to-nearest-object in [0, 1].
        # 1.0 when the closest object is within `_near_threshold`,
        # 0 when there are no objects nearby.
        distances = info.get('objects_distance') or []
        if distances and 'near_collision' in self._weights:
            try:
                d_min = min(abs(float(d)) for d in distances)
                if d_min < self._near_threshold:
                    terms['near_collision'] = 1.0 - (d_min / self._near_threshold)
            except (TypeError, ValueError):
                pass
        # Steering-jerk: normalised |Δsteering| in [0, 1] (action space
        # bounds ±30°, so the worst-case jerk is 60° per step).
        if 'steering_jerk' in self._weights and self._prev_steering is not None:
            try:
                jerk = abs(float(action[0]) - self._prev_steering) / 60.0
                terms['steering_jerk'] = min(max(jerk, 0.0), 1.0)
            except (TypeError, IndexError):
                pass
        return terms

    # ------------------------------------------------------------------
    # Misc
    # ------------------------------------------------------------------

    def _warn_about_terminate_on_collision(self) -> None:
        '''Emit a warning if the inner env terminates on collision while
        we're at a cost level that wants the signal alive.'''
        if self._level == CostLevel.SAFETY_0:
            return
        inner = self.env
        cfg = getattr(inner, '_oa_cfg', None)
        if cfg is None or not getattr(cfg, 'enabled', False):
            return
        if getattr(cfg, 'terminate_on_collision', True):
            warnings.warn(
                'SafetyDeepRacerEnv at level={} sees terminate_on_collision=True '
                'on the inner DeepRacerEnv — the cost signal will only fire once '
                'per episode (the last step). Set '
                'ObjectAvoidanceConfig(terminate_on_collision=False) to keep '
                'the per-step crash cost alive across the full trajectory.'
                .format(self._level.value),
                stacklevel=3,
            )

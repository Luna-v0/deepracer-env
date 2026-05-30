'''Cost-level configuration for SafetyDeepRacerEnv (D3).

A "level" picks which info keys feed the cost stream:

* ``safety-0`` — off-track only (``is_offtrack``).
* ``safety-1`` — collision (``is_crashed``); combines with off-track via
  ``combine`` mode. Requires D1 Object Avoidance to be live to ever fire.
* ``safety-2`` — composite weighted sum (off-track + crash + near-collision +
  steering-jerk). The user must supply ``weights={...}`` explicitly —
  there are no defaults (planning decision Q7).
'''
from enum import Enum
from typing import Dict


class CostLevel(str, Enum):
    SAFETY_0 = 'safety-0'
    SAFETY_1 = 'safety-1'
    SAFETY_2 = 'safety-2'


# Valid values for ``combine`` (safety-1 and safety-2).
COMBINE_MODES = ('add', 'max', 'crash-only')


# Composite cost terms recognised by ``safety-2``. The user-supplied
# ``weights`` dict must use a subset of these keys; unknown keys raise.
COMPOSITE_TERMS = ('offtrack', 'crash', 'near_collision', 'steering_jerk')


def validate_level(level: str) -> CostLevel:
    try:
        return CostLevel(level)
    except ValueError as exc:
        raise ValueError(
            'Unknown cost level {!r}. Expected one of {}.'.format(
                level, [m.value for m in CostLevel])
        ) from exc


def validate_combine(combine: str) -> str:
    if combine not in COMBINE_MODES:
        raise ValueError(
            'Unknown combine mode {!r}. Expected one of {}.'.format(
                combine, list(COMBINE_MODES))
        )
    return combine


def validate_weights(weights: Dict[str, float]) -> Dict[str, float]:
    '''Check that ``weights`` is well-formed for ``safety-2``.

    Raises with a clear message if any term is unknown or non-numeric.
    Returns the same dict (unchanged) so the caller can keep a reference.
    '''
    if not isinstance(weights, dict) or not weights:
        raise ValueError(
            'safety-2 requires weights={"offtrack": ..., "crash": ...} — '
            'see plans/03-safety-gymnasium.md (decision Q7). '
            'No default weights are provided.'
        )
    bad = [k for k in weights if k not in COMPOSITE_TERMS]
    if bad:
        raise ValueError(
            'Unknown safety-2 cost term(s) {}. Allowed: {}.'.format(
                bad, list(COMPOSITE_TERMS))
        )
    for k, v in weights.items():
        try:
            float(v)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                'safety-2 weight {!r} is not numeric: {!r}'.format(k, v)
            ) from exc
    return weights

'''Object Avoidance (D1) — static obstacle spawn/teardown for DeepRacer.

See ``plans/01-object-avoidance.md`` for the design.
'''
from deepracer_env.object_avoidance.config import ObjectAvoidanceConfig
# Import the placement submodule *before* obstacle_manager so the latter's
# ``from . import placement`` lookup finds a fully-initialised module.
from deepracer_env.object_avoidance import placement
from deepracer_env.object_avoidance.obstacle_manager import ObstacleManager

__all__ = ['ObjectAvoidanceConfig', 'ObstacleManager', 'placement']

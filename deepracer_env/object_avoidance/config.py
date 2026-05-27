'''Configuration dataclass for the Object Avoidance feature (D1).'''
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple


# Placement strategy identifiers — see ``placement.py``.
PLACEMENT_RANDOM = 'random_on_waypoints'
PLACEMENT_FIXED = 'fixed'
PLACEMENT_CALLABLE = 'callable'

# Lane identifiers — selects where the obstacle is placed in the track cross-section.
LANE_ANY = 'any'
LANE_INNER = 'inner'
LANE_OUTER = 'outer'
LANE_CENTER = 'center'


@dataclass
class ObjectAvoidanceConfig:
    '''Configuration for the static-obstacle Object Avoidance feature.

    Default ``terminate_on_collision=True`` is faithful to the AWS DeepRacer
    Object Avoidance race mode. D3 (Safety-Gymnasium) sets this to ``False``
    so the per-step ``is_crashed`` cost signal stays alive across the full
    trajectory.
    '''
    enabled: bool = True
    n_obstacles: int = 2
    placement: str = PLACEMENT_RANDOM
    fixed_positions: Optional[List[Tuple[float, float]]] = None
    placement_fn: Optional[Callable] = None
    min_spacing_m: float = 2.0
    lane: str = LANE_ANY
    terminate_on_collision: bool = True
    seed: Optional[int] = None
    # Path to the SDF used for the obstacle model. Defaults to the bundled
    # ``sdf/obstacle_box.sdf`` resolved relative to this package.
    obstacle_sdf_path: Optional[str] = None
    # Prefix used for spawned model names. Must contain "obstacle" so the
    # existing mercy-reset / off-track logic in RolloutCtrl picks them up.
    name_prefix: str = 'obstacle'
    # Max placement-rejection attempts per obstacle before giving up.
    max_placement_attempts: int = 200

'''Placement strategies for static obstacles (D1).

Every strategy is a callable with the signature

    fn(n, np_random, track_data, *, min_spacing_m, lane) -> List[(x, y, yaw)]

so user code can drop in a custom strategy via
``ObjectAvoidanceConfig(placement='callable', placement_fn=fn)``.
'''
import math
from typing import Callable, List, Optional, Tuple

import numpy as np
from shapely.geometry import Point

from deepracer_env.object_avoidance.config import (
    LANE_ANY, LANE_CENTER, LANE_INNER, LANE_OUTER,
)


Position = Tuple[float, float, float]  # (x, y, yaw)


def _yaw_at(track_data, x: float, y: float) -> float:
    '''Tangent yaw of the centerline at the projection of (x, y).'''
    d = track_data.center_line.project(Point(x, y))
    return track_data.center_line.interpolate_yaw(d, normalized=False)


def _project_to_lane(track_data, lane_id: str, x: float, y: float,
                     np_random: np.random.Generator) -> Tuple[float, float]:
    '''Move (x, y) onto the requested lane and return the new (x, y).

    ``lane_id='any'`` chooses inner or outer per call, ``'center'`` returns
    the input unchanged.
    '''
    pt = Point(x, y)
    if lane_id == LANE_CENTER:
        return x, y
    if lane_id == LANE_INNER:
        lane = track_data.inner_lane
    elif lane_id == LANE_OUTER:
        lane = track_data.outer_lane
    else:  # LANE_ANY
        lane = track_data.inner_lane if np_random.integers(0, 2) == 0 \
            else track_data.outer_lane
    proj = lane.interpolate(lane.project(pt))
    return float(proj.x), float(proj.y)


def random_on_waypoints(n: int, np_random: np.random.Generator, track_data,
                        *, min_spacing_m: float, lane: str,
                        max_attempts: int = 200) -> List[Position]:
    '''Sample ``n`` waypoint indices spread by ``min_spacing_m`` along the
    centerline, then offset each onto the requested ``lane``.

    Spacing is enforced via rejection sampling over the centerline arc
    length. The wrap-around distance is taken into account for loop tracks.
    '''
    waypoints = list(track_data.center_line.coords)
    n_wp = len(waypoints)
    track_length = float(track_data.get_track_length())

    chosen_indices: List[int] = []
    chosen_dists: List[float] = []
    attempts = 0
    budget = max(max_attempts, n * 50)
    while len(chosen_indices) < n and attempts < budget:
        attempts += 1
        idx = int(np_random.integers(0, n_wp))
        cx, cy = waypoints[idx]
        d = float(track_data.center_line.project(Point(cx, cy)))
        too_close = False
        for other in chosen_dists:
            gap = abs(d - other)
            wrap_gap = min(gap, track_length - gap) if track_data.is_loop else gap
            if wrap_gap < min_spacing_m:
                too_close = True
                break
        if not too_close:
            chosen_indices.append(idx)
            chosen_dists.append(d)

    if len(chosen_indices) < n:
        raise RuntimeError(
            'random_on_waypoints could not place {} obstacles with '
            'min_spacing_m={:.2f} (track length {:.2f}m) after {} attempts. '
            'Reduce n_obstacles or min_spacing_m.'
            .format(n, min_spacing_m, track_length, attempts)
        )

    positions: List[Position] = []
    for idx in chosen_indices:
        cx, cy = waypoints[idx]
        yaw = _yaw_at(track_data, cx, cy)
        x, y = _project_to_lane(track_data, lane, cx, cy, np_random)
        positions.append((x, y, yaw))
    return positions


def fixed(n: int, np_random: np.random.Generator, track_data,
          *, fixed_positions: List[Tuple[float, float]],
          min_spacing_m: float, lane: str) -> List[Position]:
    '''Return ``fixed_positions`` verbatim with yaw read from the centerline
    tangent at each point. ``n`` must match ``len(fixed_positions)``.
    '''
    if fixed_positions is None or len(fixed_positions) != n:
        raise ValueError(
            'placement="fixed" requires fixed_positions of length n_obstacles '
            '(got n={}, len(fixed_positions)={}).'
            .format(n, 0 if fixed_positions is None else len(fixed_positions))
        )
    return [(float(x), float(y), _yaw_at(track_data, x, y))
            for x, y in fixed_positions]


def from_callable(n: int, np_random: np.random.Generator, track_data,
                  *, placement_fn: Callable, min_spacing_m: float,
                  lane: str) -> List[Position]:
    '''Defer to a user-supplied ``placement_fn(np_random, track_data)``.

    The callable is expected to return a list of ``(x, y, yaw)`` tuples of
    length ``n``. We do not validate spacing for user-supplied placements.
    '''
    if placement_fn is None:
        raise ValueError('placement="callable" requires placement_fn=...')
    result = placement_fn(np_random, track_data)
    out: List[Position] = []
    for entry in result:
        if len(entry) == 2:
            x, y = entry
            yaw = _yaw_at(track_data, x, y)
        else:
            x, y, yaw = entry
        out.append((float(x), float(y), float(yaw)))
    if len(out) != n:
        raise ValueError(
            'placement_fn returned {} positions, expected {}.'
            .format(len(out), n)
        )
    return out

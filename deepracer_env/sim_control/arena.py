#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Tiled multi-arena layout: *N* decoupled cars in one shared simulator.

The requirement
---------------
Run a *dynamic* number of car instances that **share one Gazebo process** yet
are **fully decoupled** from one another — each with its *own track* and its
*own domain randomisation*, and an *own* episode lifecycle (reset, reward,
termination) that never perturbs a neighbour.

The design: spatial tiling
--------------------------
Lay each arena's track at a distinct world offset on a square grid, far enough
apart (``spacing`` metres) that cars can neither see nor collide with each
other. Arena *i* is the tuple::

    Arena(index=i,
          car_name="car_i",
          track_name=<the track assigned to i>,        # may differ per arena
          track_entity_name="racetrack_i",
          origin=Vec3(dx_i, dy_i, 0),                  # its grid offset
          dr_seed=base_seed + i)                       # independent randomisation

What this module is responsible for — and what it is *not*
---------------------------------------------------------
This is **pure geometry and bookkeeping**; it imports no ROS and no simulator.
It computes the grid, names the entities, seeds per-arena randomisation, and
converts poses between the shared world frame and each arena's local frame. The
actual spawning/teleporting/stepping is done by a :class:`SimControl` backend,
which consumes :class:`Arena` objects. That separation is what keeps the
decoupling testable on a laptop with no Gazebo.

Why a *local frame* matters
---------------------------
Track geometry (progress, ``distance_from_center``, ``closest_waypoints``) is
defined in the track's own coordinates, loaded from ``routes/<track>.npy``.
Because every arena hosts a *possibly different* track translated to a different
offset, the reward for car *i* must be computed in arena *i*'s **local** frame:
read the car's world pose, subtract the arena origin, then evaluate against that
arena's (un-offset) track. :meth:`ArenaLayout.to_local` is exactly that
subtraction. (The legacy code instead pre-offset each car's ``TrackData`` copy;
both are equivalent — this keeps a single canonical, un-shifted track per arena
and moves the cheap transform to read-time.)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from deepracer_env.sim_control.types import Pose, Quaternion, Vec3


# Default grid spacing between adjacent arenas, in metres. Large enough that the
# biggest shipped track plus its camera far-clip cannot reach a neighbour. This
# is the legacy ``MultiAgentDeepRacerEnv`` default, preserved for parity.
DEFAULT_ARENA_SPACING_M = 300.0


@dataclass(frozen=True)
class Arena:
    """One decoupled racing instance within the shared world.

    Attributes:
        index: Zero-based arena index.
        car_name: Namespace/entity name of this arena's car (e.g. ``"car_0"``).
            Drives every per-car topic (``/car_0/camera/...``) and the car
            entity name.
        track_name: The track this arena runs (the ``routes/<name>.npy`` /
            ``models/<name>`` key). Different arenas may run different tracks.
        track_entity_name: The Gazebo entity name of this arena's spawned track
            mesh (e.g. ``"racetrack_0"``), unique so deletes/recolours target
            exactly one arena.
        origin: World-frame offset of this arena's track origin.
        dr_seed: Independent seed for this arena's domain randomisation, so two
            arenas running the same track still randomise differently.
    """

    index: int
    car_name: str
    track_name: str
    track_entity_name: str
    origin: Vec3 = field(default_factory=Vec3)
    dr_seed: int = 0


class ArenaLayout:
    """Computes and holds the :class:`Arena` set for a shared-simulator run.

    This is the single source of truth for "how many cars, on which tracks, at
    which offsets, with which seeds". Construct it once per environment; pass the
    resulting :class:`Arena` objects to the :class:`SimControl` backend to spawn
    tracks and to the env to wire per-car observation/reward.

    Example:
        >>> layout = ArenaLayout(n_arenas=3, tracks=["reinvent_base",
        ...                                          "Bowtie_track",
        ...                                          "reinvent_base"])
        >>> [a.track_entity_name for a in layout.arenas]
        ['racetrack_0', 'racetrack_1', 'racetrack_2']
        >>> layout.arenas[0].origin            # first arena always at the origin
        Vec3(x=0.0, y=0.0, z=0.0)
    """

    def __init__(
        self,
        n_arenas: int,
        tracks: Sequence[str],
        *,
        spacing: float = DEFAULT_ARENA_SPACING_M,
        base_seed: int = 0,
        car_name_fmt: str = "car_{index}",
        track_entity_fmt: str = "racetrack_{index}",
    ) -> None:
        """Build a layout for *n_arenas* cars.

        Args:
            n_arenas: Number of decoupled cars (``>= 1``). Dynamic — the same
                code path serves 1 or 64 cars.
            tracks: Track name per arena. If a single name is given it is
                broadcast to every arena (the "same track, independent DR" case);
                otherwise ``len(tracks)`` must equal *n_arenas* (the
                "different track per arena" case).
            spacing: Grid spacing in metres between adjacent arena origins.
            base_seed: Arena *i* gets ``base_seed + i`` as its DR seed.
            car_name_fmt: ``str.format`` template for the car name; receives
                ``index``.
            track_entity_fmt: Template for the track entity name; receives
                ``index``.

        Raises:
            ValueError: If *n_arenas* < 1, or ``tracks`` is neither length-1 nor
                length-*n_arenas*.
        """
        if n_arenas < 1:
            raise ValueError("n_arenas must be >= 1, got {}".format(n_arenas))
        tracks = list(tracks)
        if len(tracks) == 1:
            tracks = tracks * n_arenas
        if len(tracks) != n_arenas:
            raise ValueError(
                "tracks must have length 1 or n_arenas ({}), got {}".format(
                    n_arenas, len(tracks)))

        self._spacing = float(spacing)
        offsets = self.grid_offsets(n_arenas, self._spacing)
        self._arenas: List[Arena] = [
            Arena(
                index=i,
                car_name=car_name_fmt.format(index=i),
                track_name=tracks[i],
                track_entity_name=track_entity_fmt.format(index=i),
                origin=Vec3(dx, dy, 0.0),
                dr_seed=base_seed + i,
            )
            for i, (dx, dy) in enumerate(offsets)
        ]

    # -- accessors -------------------------------------------------------------

    @property
    def arenas(self) -> List[Arena]:
        """The list of :class:`Arena` objects, in index order."""
        return list(self._arenas)

    def __len__(self) -> int:
        return len(self._arenas)

    def __iter__(self):
        return iter(self._arenas)

    def get(self, index: int) -> Arena:
        """Return arena *index*."""
        return self._arenas[index]

    # -- geometry --------------------------------------------------------------

    @staticmethod
    def grid_offsets(n: int, spacing: float) -> List[Tuple[float, float]]:
        """Square-grid ``(dx, dy)`` offsets, arena 0 pinned at the origin.

        Arenas fill a row-major ``ceil(sqrt(n)) x ceil(sqrt(n))`` grid so the
        bounding box stays compact (minimising how far the camera-free cars sit
        from the rendered region). Arena 0 is always ``(0, 0)`` because the
        car-0 track is the one the world file loads at world-load time; the
        others are spawned around it.

        Args:
            n: Number of arenas.
            spacing: Distance between adjacent grid cells, in metres.

        Returns:
            A list of ``n`` ``(dx, dy)`` tuples, ``[(0.0, 0.0), ...]``.
        """
        cols = int(math.ceil(math.sqrt(n))) if n > 0 else 0
        return [
            ((i % cols) * spacing, (i // cols) * spacing)
            for i in range(n)
        ]

    def to_local(self, arena: Arena, world_pose: Pose) -> Pose:
        """Express a *world*-frame pose in *arena*'s local (track) frame.

        Subtracts the arena origin; orientation is unchanged because arenas are
        pure translations of one another (tracks are pre-rotated in their mesh).
        Use this to evaluate the reward for a car against its un-offset track.

        Args:
            arena: The arena whose local frame to use.
            world_pose: A pose in the shared world frame.

        Returns:
            The same pose relative to ``arena.origin``.
        """
        return Pose(world_pose.position - arena.origin, world_pose.orientation)

    def to_world(self, arena: Arena, local_pose: Pose) -> Pose:
        """Express an *arena*-local pose in the shared world frame.

        Inverse of :meth:`to_local`. Use this to turn a track-relative start
        pose (from ``routes/<track>.npy``) into the world pose to teleport the
        car to.

        Args:
            arena: The arena whose local frame the pose is given in.
            local_pose: A pose relative to ``arena.origin``.

        Returns:
            The pose in the shared world frame.
        """
        return Pose(local_pose.position + arena.origin, local_pose.orientation)

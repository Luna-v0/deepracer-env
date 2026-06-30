#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Backend-agnostic value objects exchanged across the simulator seam.

These dataclasses are the *lingua franca* of :mod:`deepracer_env.sim_control`.
The environment, the reset rules, and the domain randomizers speak only these
types; the concrete backends (``simulation_interfaces`` or ``ros_gz``) translate
them to/from the corresponding ROS 2 / Gazebo message classes at the very edge.

Keeping the contract in plain Python dataclasses — rather than ``geometry_msgs``
— buys three things:

* **Testability.** Pose math and arena geometry can be unit-tested on a host
  with no ROS installed (none of this module imports ``rclpy``).
* **ABI insulation.** A change in the underlying message ABI (ROS 1 ``Pose`` vs
  ROS 2 ``Pose`` vs ``gz.msgs.Pose``) is absorbed inside one backend, not
  scattered across 99 call sites.
* **Intent.** ``EntityState`` says "a pose and a twist for one named thing",
  which is exactly the vocabulary the reset/reward code already uses — far
  clearer than the eight-service zoo it replaces.

All fields use SI units and the right-handed, Z-up world frame Gazebo uses.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Tuple


@dataclass(frozen=True)
class Vec3:
    """A 3-D vector / point in metres (world frame, Z-up)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def as_tuple(self) -> Tuple[float, float, float]:
        """Return ``(x, y, z)`` — handy for numpy / message construction."""
        return (self.x, self.y, self.z)


@dataclass(frozen=True)
class Quaternion:
    """An orientation as a quaternion. Defaults to the identity rotation."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    @staticmethod
    def from_yaw(yaw_rad: float) -> "Quaternion":
        """Build a quaternion for a planar (Z-axis) rotation.

        The DeepRacer car only ever spawns/teleports with a heading, so a
        yaw-only constructor covers every reset/placement call site.

        Args:
            yaw_rad: Heading in radians, CCW about +Z.

        Returns:
            The unit quaternion representing that heading.
        """
        half = 0.5 * yaw_rad
        return Quaternion(0.0, 0.0, math.sin(half), math.cos(half))

    @property
    def yaw(self) -> float:
        """The Z-axis (heading) component of this orientation, in radians."""
        siny = 2.0 * (self.w * self.z + self.x * self.y)
        cosy = 1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        return math.atan2(siny, cosy)


@dataclass(frozen=True)
class Pose:
    """A rigid-body pose: a position and an orientation."""

    position: Vec3 = field(default_factory=Vec3)
    orientation: Quaternion = field(default_factory=Quaternion)

    @staticmethod
    def at(x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> "Pose":
        """Convenience constructor from planar coordinates plus a heading."""
        return Pose(Vec3(x, y, z), Quaternion.from_yaw(yaw))


@dataclass(frozen=True)
class Twist:
    """A spatial velocity: linear (m/s) and angular (rad/s) components."""

    linear: Vec3 = field(default_factory=Vec3)
    angular: Vec3 = field(default_factory=Vec3)


@dataclass(frozen=True)
class EntityState:
    """The full kinematic state of one named simulation entity.

    This is the unit the reset/reward path reads (``get_entity_state``) and the
    unit a teleport writes (``set_entity_state``). It is the seam's replacement
    for ``gazebo_msgs/ModelState`` and ``gazebo_msgs/LinkState`` alike.
    """

    pose: Pose = field(default_factory=Pose)
    twist: Twist = field(default_factory=Twist)


@dataclass(frozen=True)
class ColorRGBA:
    """An RGBA colour with channels in ``[0, 1]`` (the Gazebo material range)."""

    r: float = 0.0
    g: float = 0.0
    b: float = 0.0
    a: float = 1.0


# Identity pose, exported as a module constant so call sites can spawn a track
# at the world origin without constructing a throwaway object each time.
IDENTITY_POSE = Pose()

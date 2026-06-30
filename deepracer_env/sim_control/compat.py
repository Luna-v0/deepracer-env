#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Gazebo-classic message shims + seam converters.

The ROS 1 stack passed ``gazebo_msgs/ModelState`` and ``gazebo_msgs/LinkState``
objects around the tracker layer, and tracker getters returned
``gazebo_msgs/GetModelStateResponse``-shaped objects (``.success`` / ``.pose`` /
``.twist``). Gazebo Jetty has no ``gazebo_msgs`` (ros_gz uses different types),
so this module provides tiny drop-in stand-ins plus converters to the
backend-agnostic :mod:`deepracer_env.sim_control.types`.

Note that ``geometry_msgs`` (``Pose`` / ``Twist`` / ``Point`` / ``Quaternion``)
*does* exist on ROS 2, so the shims hold real ``geometry_msgs`` poses/twists —
exactly what the legacy controller and reward code already mutate. Only the
``ModelState`` / ``LinkState`` envelopes and the response shape are re-created
here. This module imports ``geometry_msgs`` and is therefore in-container only.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from geometry_msgs.msg import Pose as RosPose
from geometry_msgs.msg import Twist as RosTwist

from deepracer_env.sim_control.types import (
    EntityState,
    Pose,
    Quaternion,
    Twist,
    Vec3,
)


def _identity_pose() -> RosPose:
    p = RosPose()
    p.orientation.w = 1.0  # valid identity quaternion (ROS default w=0 is invalid)
    return p


@dataclass
class ModelState:
    """Drop-in for ``gazebo_msgs/ModelState`` (model_name + pose/twist)."""

    model_name: str = ""
    pose: RosPose = field(default_factory=_identity_pose)
    twist: RosTwist = field(default_factory=RosTwist)
    reference_frame: str = "world"


@dataclass
class LinkState:
    """Drop-in for ``gazebo_msgs/LinkState`` (link_name + pose/twist)."""

    link_name: str = ""
    pose: RosPose = field(default_factory=_identity_pose)
    twist: RosTwist = field(default_factory=RosTwist)
    reference_frame: str = "world"


@dataclass
class StateResponse:
    """Drop-in for ``gazebo_msgs/GetModelStateResponse``."""

    success: bool = True
    status_message: str = ""
    pose: RosPose = field(default_factory=_identity_pose)
    twist: RosTwist = field(default_factory=RosTwist)


@dataclass
class LinkStateResponse:
    """Drop-in for ``gazebo_msgs/GetLinkStateResponse`` (carries a ``link_state``)."""

    success: bool = True
    status_message: str = ""
    link_state: LinkState = field(default_factory=LinkState)


# -- geometry_msgs <-> seam conversion ---------------------------------------

def ros_to_seam_pose(p: RosPose) -> Pose:
    """Convert a ``geometry_msgs/Pose`` to a seam :class:`Pose`."""
    return Pose(
        Vec3(p.position.x, p.position.y, p.position.z),
        Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
    )


def seam_to_ros_pose(p: Pose) -> RosPose:
    """Convert a seam :class:`Pose` to a ``geometry_msgs/Pose``."""
    rp = RosPose()
    rp.position.x, rp.position.y, rp.position.z = p.position.as_tuple()
    rp.orientation.x = p.orientation.x
    rp.orientation.y = p.orientation.y
    rp.orientation.z = p.orientation.z
    rp.orientation.w = p.orientation.w
    return rp


def seam_to_ros_twist(t: Twist) -> RosTwist:
    """Convert a seam :class:`Twist` to a ``geometry_msgs/Twist``."""
    rt = RosTwist()
    rt.linear.x, rt.linear.y, rt.linear.z = t.linear.as_tuple()
    rt.angular.x, rt.angular.y, rt.angular.z = t.angular.as_tuple()
    return rt


def ros_to_seam_twist(t: RosTwist) -> Twist:
    """Convert a ``geometry_msgs/Twist`` to a seam :class:`Twist`."""
    return Twist(Vec3(t.linear.x, t.linear.y, t.linear.z),
                 Vec3(t.angular.x, t.angular.y, t.angular.z))


def to_entity_state(state) -> EntityState:
    """Convert a :class:`ModelState`/:class:`LinkState` to a seam EntityState."""
    return EntityState(ros_to_seam_pose(state.pose), ros_to_seam_twist(state.twist))


def state_response(entity_state: EntityState, success: bool = True,
                   message: str = "") -> StateResponse:
    """Build a getter :class:`StateResponse` from a seam EntityState."""
    return StateResponse(
        success=success, status_message=message,
        pose=seam_to_ros_pose(entity_state.pose),
        twist=seam_to_ros_twist(entity_state.twist),
    )


def link_state_response(link_name: str, entity_state: EntityState,
                        success: bool = True, message: str = "") -> LinkStateResponse:
    """Build a :class:`LinkStateResponse` (``.link_state``) from an EntityState."""
    return LinkStateResponse(
        success=success, status_message=message,
        link_state=LinkState(
            link_name=link_name,
            pose=seam_to_ros_pose(entity_state.pose),
            twist=seam_to_ros_twist(entity_state.twist)),
    )
